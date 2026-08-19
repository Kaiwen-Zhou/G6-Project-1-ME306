#include "control/AxisController.h"

#include <Arduino.h>

#include "config/SystemConfig.h"

namespace {
constexpr float MICROSECONDS_TO_SECONDS = 0.000001f;

constexpr float REFERENCE_VELOCITY_EPSILON = 0.01f;
constexpr float PERCENT_TO_FRACTION = 0.01f;

float makeNonNegative(float value) {
    return value < 0.0f ? -value : value;
}

float calculateBasePwmScale(float remainingDistanceFraction) {
    if (remainingDistanceFraction <= 0.0f) {
        return 0.0f;
    }

    if (remainingDistanceFraction > 1.0f) {
        remainingDistanceFraction = 1.0f;
    }

    float taperStartFraction =
        SystemConfig::MOTION_BASE_PWM_TAPER_START_REMAINING_PERCENT * PERCENT_TO_FRACTION;

    if (taperStartFraction > 1.0f) {
        taperStartFraction = 1.0f;
    }

    if (taperStartFraction <= 0.0f || remainingDistanceFraction >= taperStartFraction) {
        return 1.0f;
    }

    return remainingDistanceFraction / taperStartFraction;
}
} // namespace

AxisController::AxisController(PIDController& pidController, MotorDriver& motor, float positionTolerance)
    : legacyEncoder_(nullptr), 
      pidController_(pidController), 
      motor_(motor),
      positionTolerance_(makeNonNegative(positionTolerance)), 
      legacyUpdateIntervalMicros_(0),
      legacyLastUpdateMicros_(0), 
      referencePosition_(0.0f), 
      referenceVelocity_(0.0f),
      remainingDistanceFraction_(0.0f),
      movementDirection_(0),
      currentPosition_(0),
      trackingError_(0.0f), 
      active_(false) {
}

AxisController::AxisController(Encoder& encoder, PIDController& pidController, MotorDriver& motor,
                               int32_t positionTolerance, unsigned long updateIntervalMicros)
    : legacyEncoder_(&encoder), 
      pidController_(pidController), 
      motor_(motor),
      positionTolerance_(makeNonNegative(static_cast<float>(positionTolerance))),
      legacyUpdateIntervalMicros_(updateIntervalMicros), 
      legacyLastUpdateMicros_(0), 
      referencePosition_(0.0f),
      referenceVelocity_(0.0f),
      remainingDistanceFraction_(0.0f),
      movementDirection_(0),
      currentPosition_(0), 
      trackingError_(0.0f), 
      active_(false) {
}

void AxisController::begin() {
    motor_.begin();
    motor_.stop();
    pidController_.reset();

    active_ = false;
    referenceVelocity_ = 0.0f;
    remainingDistanceFraction_ = 0.0f;
    movementDirection_ = 0;
    legacyLastUpdateMicros_ = 0;

    if (legacyEncoder_ != nullptr) {
        currentPosition_ = legacyEncoder_->getCount();
    }

    referencePosition_ = static_cast<float>(currentPosition_);

    trackingError_ = 0.0f;
}

void AxisController::startTracking(int32_t currentPosition) {
    motor_.stop();
    pidController_.reset();

    currentPosition_ = currentPosition;

    // Begin by holding the current motor-space position.
    // The coordinator can then supply the first trajectory reference.
    referencePosition_ = static_cast<float>(currentPosition);

    referenceVelocity_ = 0.0f;
    remainingDistanceFraction_ = 0.0f;
    movementDirection_ = 0;
    trackingError_ = 0.0f;

    active_ = true;
}

void AxisController::startTracking() {
    int32_t currentPosition = currentPosition_;

    if (legacyEncoder_ != nullptr) {
        currentPosition = legacyEncoder_->getCount();
    }

    startTracking(currentPosition);

    // Timing is retained only for the temporary old update() path.
    legacyLastUpdateMicros_ = micros();
}

void AxisController::setReference(float referencePosition, float referenceVelocityCountsPerSecond) {
    setReference(referencePosition, referenceVelocityCountsPerSecond, 0.0f);
}

void AxisController::setReference(float referencePosition, float referenceVelocityCountsPerSecond,
                                  float remainingDistanceFraction) {
    referencePosition_ = referencePosition;
    referenceVelocity_ = referenceVelocityCountsPerSecond;
    remainingDistanceFraction_ = remainingDistanceFraction;

    if (referenceVelocity_ > REFERENCE_VELOCITY_EPSILON) {
        movementDirection_ = 1;
    } else if (referenceVelocity_ < -REFERENCE_VELOCITY_EPSILON) {
        movementDirection_ = -1;
    }

    trackingError_ = referencePosition_ - static_cast<float>(currentPosition_);
}

void AxisController::setReferencePosition(int32_t referencePosition) {
    setReference(static_cast<float>(referencePosition), 0.0f, 0.0f);
}

void AxisController::update(int32_t currentPosition, float timeStepSeconds) {
    currentPosition_ = currentPosition;

    trackingError_ = referencePosition_ - static_cast<float>(currentPosition_);

    if (!active_) {
        return;
    }

    // Invalid timing must not change the motor command or
    // accumulate the PID integral.
    if (timeStepSeconds <= 0.0f) {
        return;
    }

    float controllerOutput =
        pidController_.update(trackingError_, referenceVelocity_, timeStepSeconds);

    float basePwm =
        SystemConfig::MOTION_BASE_PWM * calculateBasePwmScale(remainingDistanceFraction_);

    // The normal path-based taper can reduce the base PWM below the motor's
    // static-friction threshold near the destination. While the motor is still
    // outside tolerance and still behind the target in the planned movement
    // direction, keep a small configurable amount of base PWM available.
    //
    // If the motor overshoots, stillBehindTarget becomes false immediately, so
    // this endpoint minimum does not oppose reverse braking correction.
    const bool stillBehindTarget =
        (movementDirection_ > 0 && trackingError_ > positionTolerance_) ||
        (movementDirection_ < 0 && trackingError_ < -positionTolerance_);

    if (stillBehindTarget &&
        basePwm < SystemConfig::MOTION_ENDPOINT_MINIMUM_BASE_PWM) {
        basePwm = SystemConfig::MOTION_ENDPOINT_MINIMUM_BASE_PWM;
    }

    if (movementDirection_ > 0) {
        // Static-friction compensation is added only while the controller is
        // still requesting motion in the planned direction. It must not weaken
        // a requested reverse braking correction.
        if (controllerOutput >= 0.0f) {
            controllerOutput += basePwm;
        } else if (controllerOutput < -SystemConfig::MOTION_MAXIMUM_REVERSE_CORRECTION_PWM) {
            controllerOutput = -SystemConfig::MOTION_MAXIMUM_REVERSE_CORRECTION_PWM;
        }
    } else if (movementDirection_ < 0) {
        if (controllerOutput <= 0.0f) {
            controllerOutput -= basePwm;
        } else if (controllerOutput > SystemConfig::MOTION_MAXIMUM_REVERSE_CORRECTION_PWM) {
            controllerOutput = SystemConfig::MOTION_MAXIMUM_REVERSE_CORRECTION_PWM;
        }
    }

    motor_.setOutput(convertToMotorCommand(controllerOutput));
}

void AxisController::update() {
    if (!active_ || legacyEncoder_ == nullptr) {
        return;
    }

    const unsigned long currentTimeMicros = micros();

    const unsigned long elapsedMicros = currentTimeMicros - legacyLastUpdateMicros_;

    if (elapsedMicros < legacyUpdateIntervalMicros_) {
        return;
    }

    if (elapsedMicros == 0) {
        return;
    }

    legacyLastUpdateMicros_ = currentTimeMicros;

    const float timeStepSeconds = static_cast<float>(elapsedMicros) * MICROSECONDS_TO_SECONDS;

    update(legacyEncoder_->getCount(), timeStepSeconds);
}

void AxisController::stop() {
    motor_.stop();
    pidController_.reset();

    referenceVelocity_ = 0.0f;
    remainingDistanceFraction_ = 0.0f;
    movementDirection_ = 0;
    active_ = false;
}

void AxisController::reset(int32_t currentPosition) {
    stop();

    currentPosition_ = currentPosition;

    referencePosition_ = static_cast<float>(currentPosition);

    referenceVelocity_ = 0.0f;
    remainingDistanceFraction_ = 0.0f;
    movementDirection_ = 0;
    trackingError_ = 0.0f;

    legacyLastUpdateMicros_ = 0;
}

void AxisController::reset() {
    int32_t currentPosition = currentPosition_;

    if (legacyEncoder_ != nullptr) {
        currentPosition = legacyEncoder_->getCount();
    }

    reset(currentPosition);
}

bool AxisController::isActive() const {
    return active_;
}

bool AxisController::isWithinTolerance() const {
    return isErrorWithinTolerance(trackingError_);
}

AxisTelemetry AxisController::getTelemetry() const {
    return AxisTelemetry{referencePosition_,  referenceVelocity_,
                         currentPosition_,    trackingError_,
                         motor_.getOutput(),  pidController_.getIntegralOutput(),
                         isWithinTolerance(), active_};
}

bool AxisController::isErrorWithinTolerance(float error) const {
    if (error < 0.0f) {
        error = -error;
    }

    return error <= positionTolerance_;
}

int16_t AxisController::convertToMotorCommand(float controllerOutput) const {
    // Defensive limits before conversion to int16_t.
    if (controllerOutput >= 32767.0f) {
        return 32767;
    }

    if (controllerOutput <= -32768.0f) {
        return -32768;
    }

    // Round to the nearest integer PWM command.
    if (controllerOutput >= 0.0f) {
        return static_cast<int16_t>(controllerOutput + 0.5f);
    }

    return static_cast<int16_t>(controllerOutput - 0.5f);
}
