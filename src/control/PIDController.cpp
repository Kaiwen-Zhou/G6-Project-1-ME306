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
} // namespace

PIDController::PIDController(float proportionalGain, float integralGain, 
                             float minimumOutput, float maximumOutput,
                             float minimumIntegralOutput, float maximumIntegralOutput, 
                             float velocityFeedforwardGain)
    : proportionalGain_(proportionalGain), integralGain_(integralGain), velocityFeedforwardGain_(velocityFeedforwardGain), 
      integralOutput_(0.0f), 
      minimumOutput_(minimumOutput),
      maximumOutput_(maximumOutput), 
      minimumIntegralOutput_(minimumIntegralOutput),
      maximumIntegralOutput_(maximumIntegralOutput) {
    orderLimits(minimumOutput_, maximumOutput_);
    orderLimits(minimumIntegralOutput_, maximumIntegralOutput_);
}

float PIDController::update(float error, float targetVelocityCountsPerSecond, float timeStepSeconds) {
    const float proportionalOutput = proportionalGain_ * error;

    const float feedforwardOutput = velocityFeedforwardGain_ * targetVelocityCountsPerSecond;

    if (timeStepSeconds > 0.0f) {
        const float integralChange = integralGain_ * error * timeStepSeconds;

        const float candidateIntegralOutput =
            clampValue(integralOutput_ + integralChange, minimumIntegralOutput_, maximumIntegralOutput_);

        const float candidateControllerOutput = proportionalOutput + candidateIntegralOutput + feedforwardOutput;

        // Conditional integration anti-windup:
        // do not accumulate more integral when it would push an
        // already saturated output further into saturation.
        const bool pushesFurtherAboveMaximum = candidateControllerOutput > maximumOutput_ && integralChange > 0.0f;

        const bool pushesFurtherBelowMinimum = candidateControllerOutput < minimumOutput_ && integralChange < 0.0f;

        if (!pushesFurtherAboveMaximum && !pushesFurtherBelowMinimum) {
            integralOutput_ = candidateIntegralOutput;
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