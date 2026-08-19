#include "system/LimitSafetyManager.h"

#include <Arduino.h>

#include "config/SystemConfig.h"

namespace plotter {

namespace {
LimitSafetyUpdate emptyUpdate() {
    return {false, 0U, 0U, 0U, 0.0f, 0.0f};
}
} // namespace

LimitSafetyManager::LimitSafetyManager(LimitSwitch& leftLimit, LimitSwitch& rightLimit, 
                                       LimitSwitch& bottomLimit, LimitSwitch& topLimit, 
                                       GCodeController& gCodeController,
                                       PlotterSystem& plotterSystem)
    : leftLimit_(leftLimit), rightLimit_(rightLimit), bottomLimit_(bottomLimit), topLimit_(topLimit),
      gCodeController_(gCodeController), 
      plotterSystem_(plotterSystem), 
      recoverableLimitMask_(0U),
      expectedLimitMask_(0U), 
      pendingInterruptMask_(0U),
      lastResetBlockingLimitMask_(0U), lastSafetyCheckMs_(0UL) {
}

void LimitSafetyManager::begin() {
    recoverableLimitMask_ = 0U;
    expectedLimitMask_ = 0U;
    pendingInterruptMask_ = 0U;
    lastResetBlockingLimitMask_ = 0U;
    lastSafetyCheckMs_ = 0UL;
    gCodeController_.setExpectedLimitMask(0U);
}

void LimitSafetyManager::updateSwitches() {
    leftLimit_.update();
    rightLimit_.update();
    bottomLimit_.update();
    topLimit_.update();
}

LimitSafetyUpdate LimitSafetyManager::handleLimitInterrupts(uint8_t interruptMask) {
    if (interruptMask == 0U || !gCodeController_.systemStarted()) {
        return emptyUpdate();
    }

    const PlotterState state = plotterSystem_.state();

    if (state == PlotterState::HOMING || state == PlotterState::FAULT) {
        return emptyUpdate();
    }

    const uint8_t unexpectedMask =
        static_cast<uint8_t>(interruptMask & static_cast<uint8_t>(~expectedLimitMask_));

    if (unexpectedMask == 0U) {
        return emptyUpdate();
    }

    pendingInterruptMask_ |= unexpectedMask;
    return emptyUpdate();
}

LimitSafetyUpdate LimitSafetyManager::update() {
    LimitSafetyUpdate result = emptyUpdate();

    if (!gCodeController_.systemStarted()) {
        return result;
    }

    const PlotterState state = plotterSystem_.state();

    if (state == PlotterState::HOMING || state == PlotterState::FAULT) {
        return result;
    }

    result.releasedExpectedMask = clearReleasedExpectedLimits();

    const uint8_t confirmedInterruptMask = collectConfirmedInterrupts();

    if (confirmedInterruptMask != 0U) {
        return enterUnexpectedLimitFault(confirmedInterruptMask);
    }

    const unsigned long currentTimeMs = millis();

    if ((currentTimeMs - lastSafetyCheckMs_) < SystemConfig::LIMIT_SAFETY_CHECK_INTERVAL_MS) {
        return result;
    }

    lastSafetyCheckMs_ = currentTimeMs;

    const uint8_t unexpectedMask = unexpectedPressedMask();

    if (unexpectedMask == 0U) {
        return result;
    }

    return enterUnexpectedLimitFault(unexpectedMask);
}

uint8_t LimitSafetyManager::collectConfirmedInterrupts() {
    const uint8_t masks[] = {LEFT_LIMIT_MASK, RIGHT_LIMIT_MASK, BOTTOM_LIMIT_MASK, TOP_LIMIT_MASK};
    LimitSwitch* switches[] = {&leftLimit_, &rightLimit_, &bottomLimit_, &topLimit_};
    uint8_t confirmedMask = 0U;

    for (uint8_t index = 0; index < 4U; ++index) {
        const uint8_t mask = masks[index];

        if ((pendingInterruptMask_ & mask) == 0U || switches[index]->isInterruptVerificationPending()) {
            continue;
        }

        pendingInterruptMask_ &= static_cast<uint8_t>(~mask);

        if (switches[index]->isPressed() && (expectedLimitMask_ & mask) == 0U) {
            confirmedMask |= mask;
        }
    }

    return confirmedMask;
}

LimitSafetyUpdate LimitSafetyManager::enterUnexpectedLimitFault(uint8_t unexpectedMask) {
    LimitSafetyUpdate result = emptyUpdate();

    const uint8_t boundaryMask = classifyRecoverableBoundaryLimits(unexpectedMask);
    const Converter::CartesianDisplacement position = gCodeController_.currentCartesianPosition();

    const FSMResult faultResult = plotterSystem_.reportFault(FaultCode::UNEXPECTED_LIMIT);

    if (!faultResult.accepted) {
        return result;
    }

    recoverableLimitMask_ = boundaryMask;
    result.faultEntered = true;
    result.pressedMask = unexpectedMask;
    result.recoverableMask = boundaryMask;
    result.positionXMm = position.xMm;
    result.positionYMm = position.yMm;
    return result;
}

bool LimitSafetyManager::faultResetAllowed() {
    const uint8_t acknowledgedMask = static_cast<uint8_t>(expectedLimitMask_ | recoverableLimitMask_);

    lastResetBlockingLimitMask_ = static_cast<uint8_t>(pressedMask() & static_cast<uint8_t>(~acknowledgedMask));

    return lastResetBlockingLimitMask_ == 0U;
}

void LimitSafetyManager::armRecoverableLimitsAfterReset() {
    expectedLimitMask_ |= static_cast<uint8_t>(recoverableLimitMask_ & pressedMask());
    recoverableLimitMask_ = 0U;
    lastResetBlockingLimitMask_ = 0U;
    gCodeController_.setExpectedLimitMask(expectedLimitMask_);
}

uint8_t LimitSafetyManager::armPressedLimitsAfterHoming(uint8_t eligibleMask) {
    expectedLimitMask_ = static_cast<uint8_t>(pressedMask() & eligibleMask);
    recoverableLimitMask_ = 0U;
    pendingInterruptMask_ = 0U;
    lastResetBlockingLimitMask_ = 0U;
    gCodeController_.setExpectedLimitMask(expectedLimitMask_);
    return expectedLimitMask_;
}

void LimitSafetyManager::clearForHoming() {
    recoverableLimitMask_ = 0U;
    expectedLimitMask_ = 0U;
    pendingInterruptMask_ = 0U;
    lastResetBlockingLimitMask_ = 0U;
    gCodeController_.setExpectedLimitMask(0U);
}

void LimitSafetyManager::clearRecoverableFaultAttribution() {
    recoverableLimitMask_ = 0U;
    pendingInterruptMask_ = 0U;
    lastResetBlockingLimitMask_ = 0U;
}

uint8_t LimitSafetyManager::pressedMask() const {
    uint8_t mask = 0U;

    if (leftLimit_.isPressed()) {
        mask |= LEFT_LIMIT_MASK;
    }
    if (rightLimit_.isPressed()) {
        mask |= RIGHT_LIMIT_MASK;
    }
    if (bottomLimit_.isPressed()) {
        mask |= BOTTOM_LIMIT_MASK;
    }
    if (topLimit_.isPressed()) {
        mask |= TOP_LIMIT_MASK;
    }

    return mask;
}

uint8_t LimitSafetyManager::expectedMask() const {
    return expectedLimitMask_;
}

uint8_t LimitSafetyManager::recoverableMask() const {
    return recoverableLimitMask_;
}

uint8_t LimitSafetyManager::resetBlockingMask() const {
    return lastResetBlockingLimitMask_;
}

uint8_t LimitSafetyManager::unexpectedPressedMask() const {
    return static_cast<uint8_t>(pressedMask() & static_cast<uint8_t>(~expectedLimitMask_));
}

uint8_t LimitSafetyManager::classifyRecoverableBoundaryLimits(uint8_t candidateMask) const {
    if (!gCodeController_.safetyLimitsLoaded()) {
        return 0U;
    }

    const Converter::CartesianDisplacement position = gCodeController_.currentCartesianPosition();
    const GCodeSafetyLimits limits = gCodeController_.safetyLimits();
    const float tolerance =
        SystemConfig::LIMIT_BOUNDARY_TOLERANCE_MM > 0.0f ? SystemConfig::LIMIT_BOUNDARY_TOLERANCE_MM : 0.0f;

    uint8_t recoverableMask = 0U;

    if ((candidateMask & LEFT_LIMIT_MASK) != 0U && position.xMm <= limits.xMinimumMm + tolerance) {
        recoverableMask |= LEFT_LIMIT_MASK;
    }

    if ((candidateMask & RIGHT_LIMIT_MASK) != 0U && position.xMm >= limits.xMaximumMm - tolerance) {
        recoverableMask |= RIGHT_LIMIT_MASK;
    }

    if ((candidateMask & BOTTOM_LIMIT_MASK) != 0U && position.yMm <= limits.yMinimumMm + tolerance) {
        recoverableMask |= BOTTOM_LIMIT_MASK;
    }

    if ((candidateMask & TOP_LIMIT_MASK) != 0U && position.yMm >= limits.yMaximumMm - tolerance) {
        recoverableMask |= TOP_LIMIT_MASK;
    }

    return recoverableMask;
}

uint8_t LimitSafetyManager::clearReleasedExpectedLimits() {
    const uint8_t releasedMask = static_cast<uint8_t>(expectedLimitMask_ & static_cast<uint8_t>(~pressedMask()));

    if (releasedMask != 0U) {
        expectedLimitMask_ &= static_cast<uint8_t>(~releasedMask);
        gCodeController_.setExpectedLimitMask(expectedLimitMask_);
    }

    return releasedMask;
}

} // namespace plotter
