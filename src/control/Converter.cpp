#include "control/Converter.h"

Converter::Converter(float motorAMmPerCount,
                     float motorBMmPerCount,
                     int8_t motorACoordinateSign,
                     int8_t motorBCoordinateSign)
    : motorAMmPerCount_(motorAMmPerCount),
      motorBMmPerCount_(motorBMmPerCount),
      motorACoordinateSign_(motorACoordinateSign < 0 ? -1.0f : 1.0f),
      motorBCoordinateSign_(motorBCoordinateSign < 0 ? -1.0f : 1.0f)
{
}

Converter::MotorReference
Converter::cartesianToMotorReference(
    float xDisplacementMm,
    float yDisplacementMm,
    float xVelocityMmPerSecond,
    float yVelocityMmPerSecond) const
{
    // Physical X and Y axes are swapped relative to the original
    // software Cartesian convention.
    //
    // Original:
    // A = X + Y
    // B = X - Y
    //
    // After swapping physical X/Y:
    // A = Y + X
    // B = Y - X

    const float aDisplacementMm =
        -yDisplacementMm + xDisplacementMm;

    const float bDisplacementMm =
        -yDisplacementMm - xDisplacementMm;

    const float aVelocityMmPerSecond =
        -yVelocityMmPerSecond + xVelocityMmPerSecond;

    const float bVelocityMmPerSecond =
        -yVelocityMmPerSecond - xVelocityMmPerSecond;

    const float signedAMmPerCount =
        motorAMmPerCount_ * motorACoordinateSign_;

    const float signedBMmPerCount =
        motorBMmPerCount_ * motorBCoordinateSign_;

    return {
        aDisplacementMm / signedAMmPerCount,
        bDisplacementMm / signedBMmPerCount,
        aVelocityMmPerSecond / signedAMmPerCount,
        bVelocityMmPerSecond / signedBMmPerCount
    };
}

Converter::CartesianDisplacement
Converter::motorToCartesianDisplacement(
    float aDisplacementCounts,
    float bDisplacementCounts) const
{
    const float aDisplacementMm =
        aDisplacementCounts *
        motorAMmPerCount_ *
        motorACoordinateSign_;

    const float bDisplacementMm =
        bDisplacementCounts *
        motorBMmPerCount_ *
        motorBCoordinateSign_;

    // Original physical reconstruction:
    //
    // physicalX = (A + B) / 2
    // physicalY = (A - B) / 2
    //
    // Software X/Y are swapped relative to the physical axes.

    const float physicalXDisplacementMm =
        0.5f * (aDisplacementMm + bDisplacementMm);

    const float physicalYDisplacementMm =
        0.5f * (aDisplacementMm - bDisplacementMm);

    return {
        physicalYDisplacementMm,  // software X
        -physicalXDisplacementMm   // software Y
    };
}