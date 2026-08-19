/*
Cartesian trajectory displacement
        ↓ Converter
relative A/B displacement
        ↓ add moveStartCountA/B
absolute A/B reference
        ↓ AxisController
motor output
*/
#pragma once

#include <stdint.h>

#include "control/AxisController.h"
#include "control/Converter.h"
#include "hardware/Encoder.h"

// Combined Cartesian and motor-space telemetry for one move.
struct XYCoordinatorTelemetry {
        // Cartesian trajectory reference relative to the
        // beginning of the current move.
        float referenceXDisplacementMm;
        float referenceYDisplacementMm;
        float referenceXVelocityMmPerSecond;
        float referenceYVelocityMmPerSecond;

        // Cartesian displacement calculated from the latest
        // synchronised A/B encoder snapshot.
        float actualXDisplacementMm;
        float actualYDisplacementMm;

        AxisTelemetry motorA;
        AxisTelemetry motorB;

        bool active;
};

// Coordinates two motor-space AxisController objects.
//
// The trajectory reference supplied to this class is Cartesian X/Y,
// relative to the start of the current move. Converter changes this
// into A/B motor-space references.
//
// This class owns the shared control timing and supplies both axes
// with the same encoder snapshot time and the same dt.
class XYCoordinator {
    public:
        XYCoordinator(Encoder& encoderA, Encoder& encoderB, AxisController& axisA, AxisController& axisB,
                      const Converter& converter, unsigned long controlIntervalMicros);

        // Initialise both motor-space controllers.
        // Does not start a move or read the encoders.
        void begin();

        // Start a new move using one synchronised A/B encoder snapshot.
        // The current counts become the origin of the move.
        void startMove();

        // Set Cartesian reference relative to the start of this move.
        //
        // Displacement units: mm
        // Velocity units: mm/s
        void setCartesianReference(float xDisplacementMm, float yDisplacementMm, float xVelocityMmPerSecond,
                                   float yVelocityMmPerSecond);

        void setCartesianReference(float xDisplacementMm, float yDisplacementMm, float xVelocityMmPerSecond,
                                   float yVelocityMmPerSecond, float remainingDistanceFraction);

        // Run at most one coordinated A/B control update when the
        // configured control interval has elapsed.
        void update();

        // Immediately stop and deactivate both motor controllers.
        void stop();

        // Stop both controllers and use the current synchronised
        // encoder counts as the new held position.
        void reset();

        bool isActive() const;

        // Intended for use only after the trajectory planner reports
        // that it has reached the end of the trajectory.
        bool areAxesWithinTolerance() const;

        XYCoordinatorTelemetry getTelemetry() const;

    private:
        Encoder& encoderA_;
        Encoder& encoderB_;

        AxisController& axisA_;
        AxisController& axisB_;

        const Converter& converter_;

        unsigned long controlIntervalMicros_;
        unsigned long lastUpdateMicros_;

        int32_t moveStartCountA_;
        int32_t moveStartCountB_;

        Encoder::CountPair latestCounts_;

        float referenceXDisplacementMm_;
        float referenceYDisplacementMm_;
        float referenceXVelocityMmPerSecond_;
        float referenceYVelocityMmPerSecond_;

        bool active_;
};
