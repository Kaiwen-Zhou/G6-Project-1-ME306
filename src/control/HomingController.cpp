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
                                   const HomingConfig& config,
                                   bool debounceLimitInterrupts)
    : encoderA_(encoderA), encoderB_(encoderB), 
      motorA_(motorA), motorB_(motorB), 
      converter_(converter),
      limitSwitches_{&xMinSwitch, &xMaxSwitch, &yMinSwitch, &yMaxSwitch}, 
      config_(config),
      result_{{0, 0}, {0, 0}, false, false}, 
      firstContactCounts_{0, 0}, releaseCounts_{0, 0}, targetStartCounts_{0, 0},
      stage_(HomingStage::IDLE),
      phase_(HomingPhase::IDLE), 
      expectedSwitch_(ExpectedSwitch::NONE), 
      fault_(HomingFault::NONE), 
      overallStartMs_(0),
      phaseStartMs_(0), 
      allowedPressedMask_(0), 
      interruptVerificationMask_(0),
      debounceLimitInterrupts_(debounceLimitInterrupts),
      active_(false) {
}

void HomingController::begin() {
    stopMotors();

    result_ = {{0, 0}, {0, 0}, false, false};
    firstContactCounts_ = {0, 0};
    releaseCounts_ = {0, 0};
    targetStartCounts_ = {0, 0};

    stage_ = HomingStage::IDLE;
    phase_ = HomingPhase::IDLE;
    expectedSwitch_ = ExpectedSwitch::NONE;
    fault_ = HomingFault::NONE;

    overallStartMs_ = 0;
    phaseStartMs_ = 0;
    allowedPressedMask_ = 0;
    interruptVerificationMask_ = 0;
    active_ = false;

    clearSwitchEvents();
}

bool HomingController::start() {
    stopMotors();

    result_ = {{0, 0}, {0, 0}, false, false};
    fault_ = HomingFault::NONE;
    interruptVerificationMask_ = 0;
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

    uint8_t acceptedInterruptMask = 0U;

    if (debounceLimitInterrupts_) {
        if (interruptVerificationPending()) {
            stopMotors();
            return;
        }
    } else {
        acceptedInterruptMask = interruptVerificationMask_;
        interruptVerificationMask_ = 0U;
    }

    if (hasContradictoryLimits(acceptedInterruptMask)) {
        fail(HomingFault::CONTRADICTORY_LIMITS);
        return;
    }

    if ((millis() - overallStartMs_) >= config_.overallTimeoutMs) {
        fail(HomingFault::TIMEOUT);
        return;
    }

    if (hasUnexpectedLimit(acceptedInterruptMask)) {
        fail(HomingFault::WRONG_LIMIT);
        return;
    }

    switch (phase_) {
    case HomingPhase::COARSE_APPROACH:
        if (switchTriggered(expectedSwitch_, acceptedInterruptMask)) {
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
        if (switchTriggered(expectedSwitch_, acceptedInterruptMask)) {
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

void HomingController::notifyLimitInterrupt(uint8_t interruptMask) {
    if (!active_ || interruptMask == 0U) {
        return;
    }

    interruptVerificationMask_ |= interruptMask;
    stopMotors();
}

void HomingController::stop() {
    stopMotors();
    interruptVerificationMask_ = 0;
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
    interruptVerificationMask_ = 0;
    targetStartCounts_ = Encoder::getCountPair(encoderA_, encoderB_);

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

bool HomingController::interruptVerificationPending() {
    for (uint8_t index = 0; index < LIMIT_SWITCH_COUNT; ++index) {
        const uint8_t bit = static_cast<uint8_t>(1U << index);

        if ((interruptVerificationMask_ & bit) == 0U) {
            continue;
        }

        if (!limitSwitches_[index]->isInterruptVerificationPending()) {
            interruptVerificationMask_ &= static_cast<uint8_t>(~bit);
        }
    }

    return interruptVerificationMask_ != 0U;
}

bool HomingController::switchTriggered(ExpectedSwitch expected, uint8_t interruptMask) const {
    if (expected == ExpectedSwitch::NONE) {
        return false;
    }

    const uint8_t index = static_cast<uint8_t>(expected);
    const uint8_t bit = static_cast<uint8_t>(1U << index);
    return limitSwitches_[index]->isPressed() || (interruptMask & bit) != 0U;
}

bool HomingController::hasContradictoryLimits(uint8_t interruptMask) const {
    const bool xContradiction = switchTriggered(ExpectedSwitch::X_MIN, interruptMask) &&
                                switchTriggered(ExpectedSwitch::X_MAX, interruptMask);

    const bool yContradiction = switchTriggered(ExpectedSwitch::Y_MIN, interruptMask) &&
                                switchTriggered(ExpectedSwitch::Y_MAX, interruptMask);

    return xContradiction || yContradiction;
}

bool HomingController::hasUnexpectedLimit(uint8_t interruptMask) {
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

        if (limitSwitches_[index]->isPressed() || (interruptMask & bit) != 0U) {
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
    const Converter::MotorReference baseReference = converter_.cartesianToMotorReference(
        0.0f, 0.0f, static_cast<float>(xDirection), static_cast<float>(yDirection));

    int16_t motorACommand = commandForVelocitySign(baseReference.aVelocityCountsPerSecond, pwm);
    int16_t motorBCommand = commandForVelocitySign(baseReference.bVelocityCountsPerSecond, pwm);

    if (config_.straightnessCorrectionEnabled && ((xDirection == 0) != (yDirection == 0))) {
        const float crossErrorMm = crossAxisErrorMm(xDirection, yDirection);
        const uint8_t correctionPwm = straightnessCorrectionPwm(crossErrorMm, pwm);

        if (correctionPwm > 0U) {
            float correctionXDirection = 0.0f;
            float correctionYDirection = 0.0f;

            // Correct only the axis perpendicular to the commanded homing
            // direction. The Converter supplies the corresponding A/B signs,
            // including the configured coordinate-sign mapping.
            if (xDirection != 0) {
                correctionYDirection = crossErrorMm > 0.0f ? -1.0f : 1.0f;
            } else {
                correctionXDirection = crossErrorMm > 0.0f ? -1.0f : 1.0f;
            }

            const Converter::MotorReference correctionReference = converter_.cartesianToMotorReference(
                0.0f, 0.0f, correctionXDirection, correctionYDirection);

            motorACommand += commandForVelocitySign(correctionReference.aVelocityCountsPerSecond, correctionPwm);
            motorBCommand += commandForVelocitySign(correctionReference.bVelocityCountsPerSecond, correctionPwm);
        }
    }

    motorA_.setOutput(motorACommand);

    motorB_.setOutput(motorBCommand);
}

float HomingController::crossAxisErrorMm(int8_t xDirection, int8_t yDirection) const {
    const Encoder::CountPair current = Encoder::getCountPair(encoderA_, encoderB_);

    const Converter::CartesianDisplacement displacement =
        converter_.motorToCartesianDisplacement(static_cast<float>(current.countA - targetStartCounts_.countA),
                                                static_cast<float>(current.countB - targetStartCounts_.countB));

    if (xDirection != 0 && yDirection == 0) {
        return displacement.yMm;
    }

    if (yDirection != 0 && xDirection == 0) {
        return displacement.xMm;
    }

    return 0.0f;
}

uint8_t HomingController::straightnessCorrectionPwm(float crossAxisErrorMm, uint8_t basePwm) const {
    float errorMagnitudeMm = absoluteValue(crossAxisErrorMm);

    if (errorMagnitudeMm <= config_.straightnessDeadbandMm) {
        return 0U;
    }

    errorMagnitudeMm -= config_.straightnessDeadbandMm;

    float correctionPwm = config_.straightnessKpPwmPerMm * errorMagnitudeMm;

    if (correctionPwm > static_cast<float>(config_.straightnessMaximumCorrectionPwm)) {
        correctionPwm = static_cast<float>(config_.straightnessMaximumCorrectionPwm);
    }

    // The correction may slow one motor to zero, but it must not reverse a
    // motor against the current homing direction.
    if (correctionPwm > static_cast<float>(basePwm)) {
        correctionPwm = static_cast<float>(basePwm);
    }

    return static_cast<uint8_t>(correctionPwm + 0.5f);
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
    const bool motionConfigurationValid =
        config_.coarseApproachPwm > 0 && config_.backoffPwm > 0 && config_.fineApproachPwm > 0 &&
        config_.finalReleasePwm > 0 && config_.backoffDistanceMm > 0.0f && config_.searchTimeoutMs > 0 &&
        config_.backoffTimeoutMs > 0 && config_.finalReleaseTimeoutMs > 0 && config_.overallTimeoutMs > 0;

    const bool straightnessConfigurationValid =
        !config_.straightnessCorrectionEnabled ||
        (config_.straightnessKpPwmPerMm > 0.0f && config_.straightnessMaximumCorrectionPwm > 0U &&
         config_.straightnessDeadbandMm >= 0.0f);

    return motionConfigurationValid && straightnessConfigurationValid;
}

void HomingController::fail(HomingFault fault) {
    stopMotors();

    fault_ = fault;
    stage_ = HomingStage::ABORTED;
    phase_ = HomingPhase::ABORTED;
    expectedSwitch_ = ExpectedSwitch::NONE;
    interruptVerificationMask_ = 0;
    active_ = false;
}
