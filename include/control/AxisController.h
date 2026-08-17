#pragma once

#include <stdint.h>

#include "control/PIDController.h"
#include "hardware/Encoder.h"
#include "hardware/MotorDriver.h"

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
        // New constructor used by XYCoordinator.
        // Encoder readings and control timing are supplied externally.
        AxisController(PIDController& pidController, MotorDriver& motor, float positionTolerance);

        // Temporary compatibility constructor for the current PlotterSystem.
        // Remove after XYCoordinator is integrated.
        AxisController(Encoder& encoder, PIDController& pidController, MotorDriver& motor, 
                       int32_t positionTolerance, unsigned long updateIntervalMicros);

        void begin();

        // New interface: begin tracking from a synchronised encoder snapshot.
        void startTracking(int32_t currentPosition);

        // Temporary compatibility interface.
        void startTracking();

        // Set the continuously changing motor-space trajectory reference.
        void setReference(float referencePosition, float referenceVelocityCountsPerSecond);

        // Temporary fixed-position compatibility interface.
        void setReferencePosition(int32_t referencePosition);

        // New interface: execute exactly one control calculation.
        //
        // currentPosition comes from the synchronised A/B encoder snapshot.
        // timeStepSeconds is shared by both motor controllers.
        void update(int32_t currentPosition, float timeStepSeconds);

        // Temporary internally timed compatibility interface.
        void update();

        void stop();

        // Reset using a supplied synchronised encoder position.
        void reset(int32_t currentPosition);

        // Temporary compatibility interface.
        void reset();

        bool isActive() const;
        bool isWithinTolerance() const;
        AxisTelemetry getTelemetry() const;

    private:
        bool isErrorWithinTolerance(float error) const;
        int16_t convertToMotorCommand(float controllerOutput) const;

        // Used only by the temporary compatibility interfaces.
        Encoder* legacyEncoder_;

        PIDController& pidController_;
        MotorDriver& motor_;

        float positionTolerance_;

        // Used only by the temporary update() interface.
        unsigned long legacyUpdateIntervalMicros_;
        unsigned long legacyLastUpdateMicros_;

        float referencePosition_;
        float referenceVelocity_;
        int32_t currentPosition_;
        float trackingError_;

        bool active_;
};
