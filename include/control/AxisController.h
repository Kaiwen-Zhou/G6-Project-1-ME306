#pragma once

#include <stdint.h>

#include "control/PIDController.h"
#include "hardware/Encoder.h"
#include "hardware/MotorDriver.h"

/**
 * Closed-loop controller for one A/B motor-space axis.
 *
 * Start tracking from an encoder position, supply position and velocity
 * references, and call update() with the latest count and shared control-loop
 * timestep. The controller combines PI output, velocity feedforward, endpoint
 * compensation, and the MotorDriver output limit.
 */

// Motor-space telemetry for one A/B control channel.
// Cartesian telemetry is calculated by XYCoordinator.
struct AxisTelemetry {
        float referencePosition;
        float referenceVelocity;
        int32_t currentPosition;
        float trackingError;
        int16_t motorOutput;
        float integralOutput;
        bool withinTolerance;
        bool active;
};

class AxisController {
    public:
        // Coordinated-control constructor used by XYCoordinator. Encoder
        // readings and control timing are supplied externally.
        AxisController(PIDController& pidController, MotorDriver& motor, float positionTolerance);

        // Legacy self-timed constructor retained for standalone callers.
        AxisController(Encoder& encoder, PIDController& pidController, MotorDriver& motor,
                       int32_t positionTolerance, unsigned long updateIntervalMicros);

        void begin();

        // Begin tracking from a synchronised encoder snapshot.
        void startTracking(int32_t currentPosition);

        // Legacy interface that reads the configured encoder internally.
        void startTracking();

        // Set the continuously changing motor-space trajectory reference.
        // remainingDistanceFraction is shared by both axes and refers to the
        // remaining Cartesian path length: 1.0 at the start and 0.0 at the target.
        void setReference(float referencePosition, float referenceVelocityCountsPerSecond);

        void setReference(float referencePosition, float referenceVelocityCountsPerSecond,
                          float remainingDistanceFraction);

        // Fixed-position compatibility interface.
        void setReferencePosition(int32_t referencePosition);

        // Execute exactly one control calculation.
        //
        // currentPosition comes from the synchronised A/B encoder snapshot.
        // timeStepSeconds is shared by both motor controllers.
        void update(int32_t currentPosition, float timeStepSeconds);

        // Legacy internally timed interface.
        void update();

        void stop();

        // Reset using a supplied synchronised encoder position.
        void reset(int32_t currentPosition);

        // Legacy interface that reads the configured encoder internally.
        void reset();

        bool isActive() const;
        bool isWithinTolerance() const;
        AxisTelemetry getTelemetry() const;

    private:
        bool isErrorWithinTolerance(float error) const;
        int16_t convertToMotorCommand(float controllerOutput) const;

        // Used only by the legacy compatibility interfaces.
        Encoder* legacyEncoder_;

        PIDController& pidController_;
        MotorDriver& motor_;

        float positionTolerance_;

        // Used only by the legacy update() interface.
        unsigned long legacyUpdateIntervalMicros_;
        unsigned long legacyLastUpdateMicros_;

        float referencePosition_;
        float referenceVelocity_;
        float remainingDistanceFraction_;
        int8_t movementDirection_;
        int32_t currentPosition_;
        float trackingError_;

        bool active_;
};
