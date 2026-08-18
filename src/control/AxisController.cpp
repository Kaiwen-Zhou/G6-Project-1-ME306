#include "control/AxisController.h"

#include <Arduino.h>

namespace {
constexpr float MICROSECONDS_TO_SECONDS = 0.000001f;

constexpr float REFERENCE_VELOCITY_EPSILON = 0.01f;

// Static-friction compensation.
constexpr float STATIC_FEEDFORWARD_PWM = 70.0f;

// Static feedforward reaches its full value at this reference speed.
// Below it, the added PWM ramps proportionally with trajectory speed.
constexpr float STATIC_FEEDFORWARD_RAMP_COUNTS_PER_SECOND = 50.0f;

// Maximum PWM allowed opposite to the current trajectory direction.
constexpr float MAXIMUM_REVERSE_BRAKING_PWM = 40.0f;

float makeNonNegative(float value) {
    return value < 0.0f ? -value : value;
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
    referencePosition_ = referencePosition;
    referenceVelocity_ = referenceVelocityCountsPerSecond;

    trackingError_ = referencePosition_ - static_cast<float>(currentPosition_);
}

void AxisController::setReferencePosition(int32_t referencePosition) {
    setReference(static_cast<float>(referencePosition), 0.0f);
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

    if (referenceVelocity_ > REFERENCE_VELOCITY_EPSILON) {
        float staticFeedforwardScale =
            referenceVelocity_ / STATIC_FEEDFORWARD_RAMP_COUNTS_PER_SECOND;

        if (staticFeedforwardScale > 1.0f) {
            staticFeedforwardScale = 1.0f;
        }

        controllerOutput += STATIC_FEEDFORWARD_PWM * staticFeedforwardScale;

        // Allow limited reverse braking, but never more than the configured PWM.
        if (controllerOutput < -MAXIMUM_REVERSE_BRAKING_PWM) {
            controllerOutput = -MAXIMUM_REVERSE_BRAKING_PWM;
        }
    } else if (referenceVelocity_ < -REFERENCE_VELOCITY_EPSILON) {
        float staticFeedforwardScale =
            -referenceVelocity_ / STATIC_FEEDFORWARD_RAMP_COUNTS_PER_SECOND;

        if (staticFeedforwardScale > 1.0f) {
            staticFeedforwardScale = 1.0f;
        }

        controllerOutput -= STATIC_FEEDFORWARD_PWM * staticFeedforwardScale;

        // Allow limited reverse braking, but never more than the configured PWM.
        if (controllerOutput > MAXIMUM_REVERSE_BRAKING_PWM) {
            controllerOutput = MAXIMUM_REVERSE_BRAKING_PWM;
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
    active_ = false;
}

void AxisController::reset(int32_t currentPosition) {
    stop();

    currentPosition_ = currentPosition;

    referencePosition_ = static_cast<float>(currentPosition);

    referenceVelocity_ = 0.0f;
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