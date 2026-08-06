#pragma once

#include <stdint.h>

#include "control/PIDController.h"
#include "hardware/Encoder.h"
#include "hardware/MotorDriver.h"

// Motor-space telemetry for one A/B control channel.
// Cartesian X/Y telemetry is calculated by XYCoordinator.
struct AxisTelemetry
{
    int32_t referencePosition;
    int32_t currentPosition;
    int32_t trackingError;
    int16_t motorOutput;
    float integralOutput;
    bool withinTolerance;
    bool active;
};

// Tracks one motor-space encoder-count reference using one motor,
// one encoder and one PIDController.
//
// Each instance controls either motor coordinate A or B.
// Cartesian conversion, trajectory generation, homing and complete-move
// detection are handled by higher-level system modules.
class AxisController
{
public:
    AxisController(Encoder& encoder,
                   PIDController& pidController,
                   MotorDriver& motor,
                   int32_t positionTolerance,
                   unsigned long updateIntervalMicros);

    // Initialise the motor and leave the controller stopped.
    void begin();

    // Start reference tracking and reset the PID state once.
    void startTracking();

    // Update the encoder-count reference without resetting the PID.
    // This function does not activate an inactive controller.
    void setReferencePosition(int32_t referencePosition);

    // Run one non-blocking control update when the configured interval
    // has elapsed.
    void update();

    // Stop the motor, deactivate tracking and clear the PID state.
    void stop();

    // Stop tracking and use the current encoder position as the reference.
    void reset();

    bool isActive() const;
    bool isWithinTolerance() const;
    AxisTelemetry getTelemetry() const;

private:
    bool isErrorWithinTolerance(int32_t error) const;

    Encoder& encoder_;
    PIDController& pidController_;
    MotorDriver& motor_;

    int32_t positionTolerance_;
    unsigned long updateIntervalMicros_;

    int32_t referencePosition_;
    unsigned long lastUpdateMicros_;
    bool active_;
};
