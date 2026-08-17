#include "control/HomingController.h"

#include <Arduino.h>

namespace {
float absoluteValue(float value) {
    return value < 0.0f ? -value : value;
}

int16_t commandForVelocitySign(float velocity, uint8_t pwm) {
    if (velocity > 0.0f) {
        return static_cast<int16_t>(pwm);
    }

    if (velocity < 0.0f) {
        return -static_cast<int16_t>(pwm);
    }

    return 0;
}
} // namespace

HomingController::HomingController(Encoder& encoderA, Encoder& encoderB, 
                                   MotorDriver& motorA, MotorDriver& motorB,
                                   const Converter& converter, 
                                   LimitSwitch& xMinSwitch, LimitSwitch& xMaxSwitch,
                                   LimitSwitch& yMinSwitch, LimitSwitch& yMaxSwitch, 
                                   const HomingConfig& config)
    : encoderA_(encoderA), encoderB_(encoderB), 
      motorA_(motorA), motorB_(motorB), 
      converter_(converter),
      limitSwitches_{&xMinSwitch, &xMaxSwitch, &yMinSwitch, &yMaxSwitch}, 
      config_(config),
      result_{{0, 0}, {0, 0}, false, false}, 
      firstContactCounts_{0, 0}, releaseCounts_{0, 0}, 
      stage_(HomingStage::IDLE),
      phase_(HomingPhase::IDLE), 
      expectedSwitch_(ExpectedSwitch::NONE), 
      fault_(HomingFault::NONE), 
      overallStartMs_(0),
      phaseStartMs_(0), 
      allowedPressedMask_(0), 
      active_(false) {
}

void HomingController::begin() {
    stopMotors();

    result_ = {{0, 0}, {0, 0}, false, false};
    firstContactCounts_ = {0, 0};
    releaseCounts_ = {0, 0};

    stage_ = HomingStage::IDLE;
    phase_ = HomingPhase::IDLE;
    expectedSwitch_ = ExpectedSwitch::NONE;
    fault_ = HomingFault::NONE;

    overallStartMs_ = 0;
    phaseStartMs_ = 0;
    allowedPressedMask_ = 0;
    active_ = false;

    clearSwitchEvents();
}

bool HomingController::start() {
    stopMotors();

    result_ = {{0, 0}, {0, 0}, false, false};
    fault_ = HomingFault::NONE;
    active_ = false;

    clearSwitchEvents();
    updateAllSwitches();

    if (!configurationIsValid()) {
        fail(HomingFault::INVALID_CONFIGURATION);
        return false;
    }

    if (hasContradictoryLimits()) {
        fail(HomingFault::CONTRADICTORY_LIMITS);
        return false;
    }

    overallStartMs_ = millis();
    active_ = true;

    // Home directly to each origin. The max switches remain active safety
    // inputs but are no longer visited for travel calibration.
    beginTarget(HomingStage::X_ORIGIN);
    return true;
}

void HomingController::update() {
    if (!active_) {
        return;
    }

    updateAllSwitches();

    if (hasContradictoryLimits()) {
        fail(HomingFault::CONTRADICTORY_LIMITS);
        return;
    }

    if ((millis() - overallStartMs_) >= config_.overallTimeoutMs) {
        fail(HomingFault::TIMEOUT);
        return;
    }

    if (hasUnexpectedLimit()) {
        fail(HomingFault::WRONG_LIMIT);
        return;
    }

    switch (phase_) {
    case HomingPhase::COARSE_APPROACH:
        if (switchFor(expectedSwitch_).isPressed()) {
            stopMotors();
            firstContactCounts_ = Encoder::getCountPair(encoderA_, encoderB_);
            setPhase(HomingPhase::CONTACT_PAUSE);
        } else if (phaseTimedOut(config_.searchTimeoutMs)) {
            fail(HomingFault::TIMEOUT);
        } else {
            driveTowardTarget(config_.coarseApproachPwm);
        }
        break;

    case HomingPhase::CONTACT_PAUSE:
        stopMotors();

        if ((millis() - phaseStartMs_) >= config_.contactPauseMs) {
            setPhase(HomingPhase::BACKOFF);
        }
        break;

    case HomingPhase::BACKOFF:
        if (phaseTimedOut(config_.backoffTimeoutMs)) {
            fail(HomingFault::TIMEOUT);
            break;
        }

        if (switchFor(expectedSwitch_).isReleased() && distanceFromFirstContactMm() >= config_.backoffDistanceMm) {
            stopMotors();
            setPhase(HomingPhase::FINE_APPROACH);
        } else {
            driveAwayFromTarget(config_.backoffPwm);
        }
        break;

    case HomingPhase::FINE_APPROACH:
        if (switchFor(expectedSwitch_).isPressed()) {
            stopMotors();
            setPhase(HomingPhase::FINE_CONTACT_PAUSE);
        } else if (phaseTimedOut(config_.searchTimeoutMs)) {
            fail(HomingFault::TIMEOUT);
        } else {
            driveTowardTarget(config_.fineApproachPwm);
        }
        break;

    case HomingPhase::FINE_CONTACT_PAUSE:
        stopMotors();

        // A released switch here means the fine contact did not remain
        // stable during the pause. Approach it again rather than using
        // an uncontrolled rebound as the reference position.
        if (switchFor(expectedSwitch_).isReleased()) {
            switchFor(expectedSwitch_).consumeReleasedEvent();
            setPhase(HomingPhase::FINE_APPROACH);
        } else if ((millis() - phaseStartMs_) >= config_.fineContactPauseMs) {
            // Discard any stale release event from earlier bounce. The
            // switch is confirmed pressed at the start of FINAL_RELEASE,
            // so the next event must be its new falling edge.
            switchFor(expectedSwitch_).consumeReleasedEvent();
            setPhase(HomingPhase::FINAL_RELEASE);
        }
        break;

    case HomingPhase::FINAL_RELEASE:
        if (switchFor(expectedSwitch_).consumeReleasedEvent()) {
            stopMotors();

            // Capture both motor-space counts in the same update that
            // accepts the debounced active-high falling edge.
            releaseCounts_ = Encoder::getCountPair(encoderA_, encoderB_);
            setPhase(HomingPhase::RECORD_POSITION);
        } else if (phaseTimedOut(config_.finalReleaseTimeoutMs)) {
            fail(HomingFault::TIMEOUT);
        } else {
            driveAwayFromTarget(config_.finalReleasePwm);
        }
        break;

    case HomingPhase::RECORD_POSITION:
        recordCurrentTarget();
        advanceAfterOriginRecorded();
        break;

    case HomingPhase::IDLE:
    case HomingPhase::COMPLETE:
    case HomingPhase::ABORTED:
        stopMotors();
        break;
    }
}

void HomingController::stop() {
    stopMotors();
    active_ = false;

    if (stage_ != HomingStage::COMPLETE && stage_ != HomingStage::ABORTED) {
        stage_ = HomingStage::IDLE;
        phase_ = HomingPhase::IDLE;
        expectedSwitch_ = ExpectedSwitch::NONE;
    }
}

bool HomingController::isActive() const {
    return active_;
}

bool HomingController::isComplete() const {
    return stage_ == HomingStage::COMPLETE;
}

bool HomingController::hasFault() const {
    return fault_ != HomingFault::NONE;
}

HomingStage HomingController::stage() const {
    return stage_;
}

HomingPhase HomingController::phase() const {
    return phase_;
}

ExpectedSwitch HomingController::expectedSwitch() const {
    return expectedSwitch_;
}

HomingFault HomingController::fault() const {
    return fault_;
}

HomingResult HomingController::result() const {
    return result_;
}

void HomingController::beginTarget(HomingStage nextStage) {
    stage_ = nextStage;

    switch (stage_) {
    case HomingStage::X_ORIGIN:
        expectedSwitch_ = ExpectedSwitch::X_MIN;
        break;

    case HomingStage::Y_ORIGIN:
        expectedSwitch_ = ExpectedSwitch::Y_MIN;
        break;

    default:
        expectedSwitch_ = ExpectedSwitch::NONE;
        break;
    }

    clearSwitchEvents();

    allowedPressedMask_ = 0;

    for (uint8_t index = 0; index < LIMIT_SWITCH_COUNT; ++index) {
        const ExpectedSwitch current = static_cast<ExpectedSwitch>(index);

        if (current != expectedSwitch_ && limitSwitches_[index]->isPressed()) {
            allowedPressedMask_ |= static_cast<uint8_t>(1U << index);
        }
    }

    setPhase(HomingPhase::COARSE_APPROACH);
}

void HomingController::setPhase(HomingPhase nextPhase) {
    phase_ = nextPhase;
    phaseStartMs_ = millis();
}

void HomingController::advanceAfterOriginRecorded() {
    switch (stage_) {
    case HomingStage::X_ORIGIN:
        beginTarget(HomingStage::Y_ORIGIN);
        break;

    case HomingStage::Y_ORIGIN:
        stopMotors();

        // Both physical axes are now at their origin switches.
        // Reset both motor-space counts in one critical section.
        Encoder::zeroCountPair(encoderA_, encoderB_);

        stage_ = HomingStage::COMPLETE;
        phase_ = HomingPhase::COMPLETE;
        expectedSwitch_ = ExpectedSwitch::NONE;
        active_ = false;
        clearSwitchEvents();
        break;

    default:
        fail(HomingFault::INVALID_CONFIGURATION);
        break;
    }
}

void HomingController::updateAllSwitches() {
    for (uint8_t index = 0; index < LIMIT_SWITCH_COUNT; ++index) {
        limitSwitches_[index]->update();
    }
}

void HomingController::clearSwitchEvents() {
    for (uint8_t index = 0; index < LIMIT_SWITCH_COUNT; ++index) {
        limitSwitches_[index]->consumePressedEvent();
        limitSwitches_[index]->consumeReleasedEvent();
        limitSwitches_[index]->consumeRejectedInterruptEvent();
    }
}

bool HomingController::hasContradictoryLimits() const {
    const bool xContradiction = limitSwitches_[static_cast<uint8_t>(ExpectedSwitch::X_MIN)]->isPressed() &&
                                limitSwitches_[static_cast<uint8_t>(ExpectedSwitch::X_MAX)]->isPressed();

    const bool yContradiction = limitSwitches_[static_cast<uint8_t>(ExpectedSwitch::Y_MIN)]->isPressed() &&
                                limitSwitches_[static_cast<uint8_t>(ExpectedSwitch::Y_MAX)]->isPressed();

    return xContradiction || yContradiction;
}

bool HomingController::hasUnexpectedLimit() {
    for (uint8_t index = 0; index < LIMIT_SWITCH_COUNT; ++index) {
        const ExpectedSwitch current = static_cast<ExpectedSwitch>(index);

        if (current == expectedSwitch_) {
            continue;
        }

        const uint8_t bit = static_cast<uint8_t>(1U << index);

        if ((allowedPressedMask_ & bit) != 0U) {
            if (limitSwitches_[index]->isReleased()) {
                allowedPressedMask_ &= static_cast<uint8_t>(~bit);
            }

            continue;
        }

        if (limitSwitches_[index]->isPressed()) {
            return true;
        }
    }

    return false;
}

LimitSwitch& HomingController::switchFor(ExpectedSwitch expected) {
    return *limitSwitches_[static_cast<uint8_t>(expected)];
}

int8_t HomingController::targetXDirection() const {
    if (stage_ == HomingStage::X_ORIGIN) {
        return -1;
    }

    return 0;
}

int8_t HomingController::targetYDirection() const {
    if (stage_ == HomingStage::Y_ORIGIN) {
        return -1;
    }

    return 0;
}

void HomingController::driveTowardTarget(uint8_t pwm) {
    driveCartesian(targetXDirection(), targetYDirection(), pwm);
}

void HomingController::driveAwayFromTarget(uint8_t pwm) {
    driveCartesian(-targetXDirection(), -targetYDirection(), pwm);
}

void HomingController::driveCartesian(int8_t xDirection, int8_t yDirection, uint8_t pwm) {
    const Converter::MotorReference reference = converter_.cartesianToMotorReference(
        0.0f, 0.0f, static_cast<float>(xDirection), static_cast<float>(yDirection));

    motorA_.setOutput(commandForVelocitySign(reference.aVelocityCountsPerSecond, pwm));

    motorB_.setOutput(commandForVelocitySign(reference.bVelocityCountsPerSecond, pwm));
}

void HomingController::stopMotors() {
    motorA_.stop();
    motorB_.stop();
}

float HomingController::distanceFromFirstContactMm() const {
    const Encoder::CountPair current = Encoder::getCountPair(encoderA_, encoderB_);

    const Converter::CartesianDisplacement displacement =
        converter_.motorToCartesianDisplacement(static_cast<float>(current.countA - firstContactCounts_.countA),
                                                static_cast<float>(current.countB - firstContactCounts_.countB));

    if (stage_ == HomingStage::X_ORIGIN) {
        return absoluteValue(displacement.xMm);
    }

    return absoluteValue(displacement.yMm);
}

void HomingController::recordCurrentTarget() {
    switch (stage_) {
    case HomingStage::X_ORIGIN:
        result_.xOrigin = releaseCounts_;
        result_.xValid = true;
        break;

    case HomingStage::Y_ORIGIN:
        result_.yOrigin = releaseCounts_;
        result_.yValid = true;
        break;

    default:
        fail(HomingFault::INVALID_CONFIGURATION);
        break;
    }
}

bool HomingController::phaseTimedOut(unsigned long timeoutMs) const {
    return (millis() - phaseStartMs_) >= timeoutMs;
}

bool HomingController::configurationIsValid() const {
    return config_.coarseApproachPwm > 0 && config_.backoffPwm > 0 && config_.fineApproachPwm > 0 &&
           config_.finalReleasePwm > 0 && config_.backoffDistanceMm > 0.0f && config_.searchTimeoutMs > 0 &&
           config_.backoffTimeoutMs > 0 && config_.finalReleaseTimeoutMs > 0 && config_.overallTimeoutMs > 0;
}

void HomingController::fail(HomingFault fault) {
    stopMotors();

    fault_ = fault;
    stage_ = HomingStage::ABORTED;
    phase_ = HomingPhase::ABORTED;
    expectedSwitch_ = ExpectedSwitch::NONE;
    active_ = false;
}
