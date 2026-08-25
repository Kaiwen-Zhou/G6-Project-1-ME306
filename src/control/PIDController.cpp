#include "control/PIDController.h"

namespace {
float clampValue(float value, float minimumValue, float maximumValue) {
    if (value < minimumValue) {
        return minimumValue;
    }

    if (value > maximumValue) {
        return maximumValue;
    }

    return value;
}

void orderLimits(float& minimumValue, float& maximumValue) {
    if (minimumValue > maximumValue) {
        const float temporaryValue = minimumValue;
        minimumValue = maximumValue;
        maximumValue = temporaryValue;
    }
}

float moveTowardZero(float value, float amount) {
    if (amount <= 0.0f) {
        return value;
    }

    if (value > 0.0f) {
        value -= amount;
        return value < 0.0f ? 0.0f : value;
    }

    if (value < 0.0f) {
        value += amount;
        return value > 0.0f ? 0.0f : value;
    }

    return 0.0f;
}
} // namespace

PIDController::PIDController(float proportionalGain, float integralGain, float minimumOutput, float maximumOutput,
                             float minimumIntegralOutput, float maximumIntegralOutput,
                             float velocityFeedforwardGain)
    : proportionalGain_(proportionalGain),
      integralGain_(integralGain),
      velocityFeedforwardGain_(velocityFeedforwardGain),
      integralOutput_(0.0f),
      minimumOutput_(minimumOutput),
      maximumOutput_(maximumOutput),
      minimumIntegralOutput_(minimumIntegralOutput),
      maximumIntegralOutput_(maximumIntegralOutput) {
    orderLimits(minimumOutput_, maximumOutput_);
    orderLimits(minimumIntegralOutput_, maximumIntegralOutput_);
}

float PIDController::update(float error, float targetVelocityCountsPerSecond, float timeStepSeconds) {
    return update(error, targetVelocityCountsPerSecond, timeStepSeconds, 0, 0.0f);
}

float PIDController::update(float error, float targetVelocityCountsPerSecond, float timeStepSeconds,
                            int8_t blockedIntegralDirection,
                            float integralBleedRatePerSecond) {
    const float proportionalOutput = proportionalGain_ * error;

    const float feedforwardOutput = velocityFeedforwardGain_ * targetVelocityCountsPerSecond;

    if (timeStepSeconds > 0.0f) {
        // Endpoint policy: gently remove only stored integral that is still
        // helping the blocked/original movement direction.
        //
        // Integral already helping the opposite correction direction is left
        // untouched, so braking and overshoot recovery retain integral action.
        const bool storedIntegralInBlockedDirection =
            (blockedIntegralDirection > 0 && integralOutput_ > 0.0f) ||
            (blockedIntegralDirection < 0 && integralOutput_ < 0.0f);

        if (storedIntegralInBlockedDirection && integralBleedRatePerSecond > 0.0f) {
            integralOutput_ = moveTowardZero(integralOutput_, integralBleedRatePerSecond * timeStepSeconds);
        }

        const float integralChange = integralGain_ * error * timeStepSeconds;

        const bool integralChangePushesBlockedDirection =
            (blockedIntegralDirection > 0 && integralChange > 0.0f) ||
            (blockedIntegralDirection < 0 && integralChange < 0.0f);

        // During endpoint deceleration, do not rebuild integral in the
        // original movement direction. Integral change in the opposite
        // direction is still allowed immediately.
        if (!integralChangePushesBlockedDirection) {
            const float candidateIntegralOutput =
                clampValue(integralOutput_ + integralChange, minimumIntegralOutput_, maximumIntegralOutput_);

            const float candidateControllerOutput = proportionalOutput + candidateIntegralOutput + feedforwardOutput;

            // Existing conditional-integration anti-windup remains active:
            // do not accumulate more integral when it would push an already
            // saturated PID output farther into saturation.
            const bool pushesFurtherAboveMaximum =
                candidateControllerOutput > maximumOutput_ && integralChange > 0.0f;

            const bool pushesFurtherBelowMinimum =
                candidateControllerOutput < minimumOutput_ && integralChange < 0.0f;

            if (!pushesFurtherAboveMaximum && !pushesFurtherBelowMinimum) {
                integralOutput_ = candidateIntegralOutput;
            }
        }
    }

    const float controllerOutput = proportionalOutput + integralOutput_ + feedforwardOutput;

    return clampValue(controllerOutput, minimumOutput_, maximumOutput_);
}

float PIDController::update(float error, float timeStepSeconds) {
    return update(error, 0.0f, timeStepSeconds);
}

void PIDController::reset() {
    integralOutput_ = 0.0f;
}

void PIDController::setGains(float proportionalGain, float integralGain) {
    proportionalGain_ = proportionalGain;
    integralGain_ = integralGain;
}

void PIDController::setVelocityFeedforwardGain(float velocityFeedforwardGain) {
    velocityFeedforwardGain_ = velocityFeedforwardGain;
}

void PIDController::setOutputLimits(float minimumOutput, float maximumOutput) {
    orderLimits(minimumOutput, maximumOutput);

    minimumOutput_ = minimumOutput;
    maximumOutput_ = maximumOutput;
}

void PIDController::setIntegralLimits(float minimumIntegralOutput, float maximumIntegralOutput) {
    orderLimits(minimumIntegralOutput, maximumIntegralOutput);

    minimumIntegralOutput_ = minimumIntegralOutput;
    maximumIntegralOutput_ = maximumIntegralOutput;

    integralOutput_ = clampValue(integralOutput_, minimumIntegralOutput_, maximumIntegralOutput_);
}

float PIDController::getProportionalGain() const {
    return proportionalGain_;
}

float PIDController::getIntegralGain() const {
    return integralGain_;
}

float PIDController::getVelocityFeedforwardGain() const {
    return velocityFeedforwardGain_;
}

float PIDController::getIntegralOutput() const {
    return integralOutput_;
}
