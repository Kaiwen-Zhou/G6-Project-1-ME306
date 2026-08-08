/* Note: Converter returns current move's relative A/B displacement
    XYCoordinator will need to add
    absoluteAReference =
        startCountA + motorReference.aDisplacementCounts;

    absoluteBReference =
        startCountB + motorReference.bDisplacementCounts;
*/

#pragma once

#include <stdint.h>

class Converter
{
public:
    struct MotorReference
    {
        // Relative to the start of the current G1 move.
        float aDisplacementCounts;
        float bDisplacementCounts;

        float aVelocityCountsPerSecond;
        float bVelocityCountsPerSecond;
    };

    struct CartesianDisplacement
    {
        float xMm;
        float yMm;
    };

    Converter(float motorAMmPerCount,
              float motorBMmPerCount,
              int8_t motorACoordinateSign = 1,
              int8_t motorBCoordinateSign = 1);

    MotorReference cartesianToMotorReference(
        float xDisplacementMm,
        float yDisplacementMm,
        float xVelocityMmPerSecond,
        float yVelocityMmPerSecond) const;

    CartesianDisplacement motorToCartesianDisplacement(
        float aDisplacementCounts,
        float bDisplacementCounts) const;

private:
    float motorAMmPerCount_;
    float motorBMmPerCount_;

    float motorACoordinateSign_;
    float motorBCoordinateSign_;
};