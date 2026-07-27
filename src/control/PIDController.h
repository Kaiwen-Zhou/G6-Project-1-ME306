#ifndef PI_CONTROLLER_H
#define PI_CONTROLLER_H

// All values are kept in one simple structure so that each motor can have
// its own gains and its own integral history.
struct PIController
{
    float kp;
    float ki;
    float integralError;
    float minimumOutput;
    float maximumOutput;
};

// Set the gains and output limits, then clear the stored integral.
void initialisePI(
    PIController &controller,
    float kp,
    float ki,
    float minimumOutput,
    float maximumOutput
);

// Clear the integral term. Call this after homing, M999, or a fault.
void resetPI(PIController &controller);

// Calculate one PI control update.
//
// setpoint and measurement must use the same unit, normally encoder counts.
// timeStepSeconds is the fixed control period in seconds.
// The return value is a signed motor command, normally from -255 to +255.
float updatePI(
    PIController &controller,
    float setpoint,
    float measurement,
    float timeStepSeconds
);

#endif
