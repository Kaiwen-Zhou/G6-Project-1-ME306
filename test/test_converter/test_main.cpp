#include <unity.h>

#include "control/Converter.h"

namespace
{
constexpr float A_MM_PER_COUNT = 0.01f;
constexpr float B_MM_PER_COUNT = 0.02f;
constexpr float FLOAT_TOLERANCE = 0.001f;

Converter converter(
    A_MM_PER_COUNT,
    B_MM_PER_COUNT,
    1,
    1);
}

void setUp()
{
}

void tearDown()
{
}

void testPureXMovement()
{
    const Converter::MotorReference result =
        converter.cartesianToMotorReference(
            10.0f,  // X displacement
            0.0f,   // Y displacement
            0.0f,
            0.0f);

    // A displacement = X + Y = 10 mm
    // B displacement = X - Y = 10 mm
    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        1000.0f,
        result.aDisplacementCounts);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        500.0f,
        result.bDisplacementCounts);
}

void testPureYMovement()
{
    const Converter::MotorReference result =
        converter.cartesianToMotorReference(
            0.0f,
            10.0f,
            0.0f,
            0.0f);

    // A displacement = 10 mm
    // B displacement = -10 mm
    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        1000.0f,
        result.aDisplacementCounts);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        -500.0f,
        result.bDisplacementCounts);
}

void testDiagonalMovement()
{
    const Converter::MotorReference result =
        converter.cartesianToMotorReference(
            10.0f,
            10.0f,
            0.0f,
            0.0f);

    // A displacement = 20 mm
    // B displacement = 0 mm
    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        2000.0f,
        result.aDisplacementCounts);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        0.0f,
        result.bDisplacementCounts);
}

void testVelocityConversion()
{
    const Converter::MotorReference result =
        converter.cartesianToMotorReference(
            0.0f,
            0.0f,
            2.0f,  // X velocity
            1.0f); // Y velocity

    // A velocity = 2 + 1 = 3 mm/s
    // B velocity = 2 - 1 = 1 mm/s
    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        300.0f,
        result.aVelocityCountsPerSecond);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        50.0f,
        result.bVelocityCountsPerSecond);
}

void testRoundTripConversion()
{
    const Converter::MotorReference motorReference =
        converter.cartesianToMotorReference(
            10.0f,
            5.0f,
            0.0f,
            0.0f);

    const Converter::CartesianDisplacement recovered =
        converter.motorToCartesianDisplacement(
            motorReference.aDisplacementCounts,
            motorReference.bDisplacementCounts);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        10.0f,
        recovered.xMm);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        5.0f,
        recovered.yMm);
}

int main(int argc, char** argv)
{
    UNITY_BEGIN();

    RUN_TEST(testPureXMovement);
    RUN_TEST(testPureYMovement);
    RUN_TEST(testDiagonalMovement);
    RUN_TEST(testVelocityConversion);
    RUN_TEST(testRoundTripConversion);

    return UNITY_END();
}