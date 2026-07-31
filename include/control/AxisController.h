#pragma once

#include <stdint.h>

#include "control/PIDController.h"
#include "hardware/Encoder.h"
#include "hardware/MotorDriver.h"

// Basic values that can be printed or read by the system layer.
struct AxisTelemetry
{
    int32_t targetPosition;
    int32_t currentPosition;
    int32_t positionError;
    int16_t motorOutput;
    float integralOutput;
    bool withinTolerance;
    bool active;
    bool complete;
};

// Controls one motor using one encoder and one PIDController.
// X-Y kinematics, limit switches, homing, G-code and FSM events are handled
// by the higher-level system code.
class AxisController
{
public:
    AxisController(Encoder& encoder,
                   PIDController& pidController,
                   MotorDriver& motor,
                   int32_t positionTolerance,
                   unsigned long updateIntervalMicros,
                   uint8_t requiredSettledUpdates = 1);

    // Initialise the motor and leave the axis stopped.
    void begin();

    // Start a non-blocking move to an absolute encoder-count target.
    void setTargetPosition(int32_t targetPosition);

    // Call repeatedly from loop(). A control calculation is performed only
    // after the configured update interval has elapsed.
    void update();

    // Stop the motor, cancel the move and clear the PID integral state.
    void stop();

    // Return to an idle state and use the current position as the target.
    void reset();

    bool isActive() const;
    bool isComplete() const;
    AxisTelemetry getTelemetry() const;

private:
    bool isWithinTolerance(int32_t error) const;
    void finishMove();

    Encoder& encoder_;
    PIDController& pidController_;
    MotorDriver& motor_;

    int32_t positionTolerance_;
    unsigned long updateIntervalMicros_;
    uint8_t requiredSettledUpdates_;

    int32_t targetPosition_;
    unsigned long lastUpdateMicros_;
    uint8_t settledUpdateCount_;
    bool active_;
    bool complete_;
};