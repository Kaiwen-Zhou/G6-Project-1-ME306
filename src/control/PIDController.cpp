#include "control/PIDController.h"

PIDController::PIDController(float proportionalGain,
                             float integralGain,
                             float minimumOutput,
                             float maximumOutput,
                             float minimumIntegralOutput,
                             float maximumIntegralOutput)
    : proportionalGain_(proportionalGain),
      integralGain_(integralGain),
      integralOutput_(0.0f),
      minimumOutput_(minimumOutput),
      maximumOutput_(maximumOutput),
      minimumIntegralOutput_(minimumIntegralOutput),
      maximumIntegralOutput_(maximumIntegralOutput)
{
}

float PIDController::update(float error, float timeStepSeconds)
{
    const float proportionalOutput = proportionalGain_ * error;

    float proposedIntegralOutput = integralOutput_;

    // I(k) = I(k-1) + Ki * error(k) * dt
    // A non-positive dt skips integration but still allows proportional output.
    if (timeStepSeconds > 0.0f)
    {
        proposedIntegralOutput +=
            integralGain_ * error * timeStepSeconds;
    }

    // Limit the stored integral contribution.
    if (proposedIntegralOutput > maximumIntegralOutput_)
    {
        proposedIntegralOutput = maximumIntegralOutput_;
    }
    else if (proposedIntegralOutput < minimumIntegralOutput_)
    {
        proposedIntegralOutput = minimumIntegralOutput_;
    }

    const float proposedOutput =
        proportionalOutput + proposedIntegralOutput;

    const float integralChange =
        proposedIntegralOutput - integralOutput_;

    // Conditional integration anti-windup:
    // block an integral change only when it would push a saturated output
    // farther into saturation. A change in the opposite direction is accepted,
    // so the integral can unwind after the error reverses.
    const bool pushesFurtherAboveMaximum =
        proposedOutput > maximumOutput_ && integralChange > 0.0f;

    const bool pushesFurtherBelowMinimum =
        proposedOutput < minimumOutput_ && integralChange < 0.0f;

    if (!pushesFurtherAboveMaximum && !pushesFurtherBelowMinimum)
    {
        integralOutput_ = proposedIntegralOutput;
    }

    float output = proportionalOutput + integralOutput_;

    // Limit the final command sent to the next control module.
    if (output > maximumOutput_)
    {
        output = maximumOutput_;
    }
    else if (output < minimumOutput_)
    {
        output = minimumOutput_;
    }

    return output;
}

void PIDController::reset()
{
    integralOutput_ = 0.0f;
}

void PIDController::setGains(float proportionalGain, float integralGain)
{
    proportionalGain_ = proportionalGain;
    integralGain_ = integralGain;
}

void PIDController::setOutputLimits(float minimumOutput,
                                    float maximumOutput)
{
    minimumOutput_ = minimumOutput;
    maximumOutput_ = maximumOutput;
}

void PIDController::setIntegralLimits(float minimumIntegralOutput,
                                      float maximumIntegralOutput)
{
    minimumIntegralOutput_ = minimumIntegralOutput;
    maximumIntegralOutput_ = maximumIntegralOutput;

    // Keep the existing state inside the newly configured limits.
    if (integralOutput_ > maximumIntegralOutput_)
    {
        integralOutput_ = maximumIntegralOutput_;
    }
    else if (integralOutput_ < minimumIntegralOutput_)
    {
        integralOutput_ = minimumIntegralOutput_;
    }
}

float PIDController::getProportionalGain() const
{
    return proportionalGain_;
}

float PIDController::getIntegralGain() const
{
    return integralGain_;
}

float PIDController::getIntegralOutput() const
{
    return integralOutput_;
}