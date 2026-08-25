#pragma once

#include <stdint.h>

/**
 * Converts between Cartesian X/Y motion and coupled A/B motor space.
 *
 * Construct the converter with each motor's millimetres-per-count scale and
 * coordinate sign. cartesianToMotorReference() returns displacement relative
 * to the current move; XYCoordinator adds the move-start encoder counts to
 * form absolute controller references.
 */
class Converter {
    public:
        struct MotorReference {
                // Relative to the start of the current G1 move.
                float aDisplacementCounts;
                float bDisplacementCounts;

                float aVelocityCountsPerSecond;
                float bVelocityCountsPerSecond;
        };

        struct CartesianDisplacement {
                float xMm;
                float yMm;
        };

        Converter(float motorAMmPerCount,
                  float motorBMmPerCount,
                  int8_t motorACoordinateSign = 1,
                  int8_t motorBCoordinateSign = 1);

        MotorReference cartesianToMotorReference(float xDisplacementMm, float yDisplacementMm,
                                                 float xVelocityMmPerSecond, float yVelocityMmPerSecond) const;

        CartesianDisplacement motorToCartesianDisplacement(float aDisplacementCounts, float bDisplacementCounts) const;

    private:
        float motorAMmPerCount_;
        float motorBMmPerCount_;

        float motorACoordinateSign_;
        float motorBCoordinateSign_;
};
