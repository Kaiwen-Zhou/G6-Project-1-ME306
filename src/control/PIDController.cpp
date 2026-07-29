#include "control/PIDController.h"

void initialisePI(
    PIController &controller,
    float kp,
    float ki,
    float minimumOutput,
    float maximumOutput
)
{
    controller.kp = kp;
    controller.ki = ki;
    controller.integralError = 0.0f;
    controller.minimumOutput = minimumOutput;
    controller.maximumOutput = maximumOutput;
}

void resetPI(PIController &controller)
{
    controller.integralError = 0.0f;
}

float updatePI(
    PIController &controller,
    float setpoint,
    float measurement,
    float timeStepSeconds
)
{
    if (timeStepSeconds <= 0.0f)
    {
        return 0.0f;
    }

    float error = setpoint - measurement;

    // Discrete approximation of the integral:
    // integral(k) = integral(k-1) + error(k) * dt
    float proposedIntegral =
        controller.integralError + error * timeStepSeconds;

    float proportionalOutput = controller.kp * error;
    float integralOutput = controller.ki * proposedIntegral;
    float proposedOutput = proportionalOutput + integralOutput;

    // Integral clamping from Lecture 3:
    // if the actuator is saturated, do not store the new integral value.
    if (proposedOutput > controller.maximumOutput)
    {
        return controller.maximumOutput;
    }

    if (proposedOutput < controller.minimumOutput)
    {
        return controller.minimumOutput;
    }

    controller.integralError = proposedIntegral;
    return proposedOutput;
}