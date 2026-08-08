#include <unity.h>

#include "control/PIDController.h"

namespace
{
constexpr float FLOAT_TOLERANCE = 0.0001f;
}

void setUp()
{
}

void tearDown()
{
}

void testProportionalAndFeedforwardAreAdded()
{
    PIDController controller(
        2.0f,    // Kp
        0.0f,    // Ki
        -255.0f,
        255.0f,
        -100.0f,
        100.0f,
        0.5f);   // Kv

    const float output =
        controller.update(
            10.0f,  // position error
            20.0f,  // target velocity
            0.01f);

    // P  = 2 * 10 = 20
    // FF = 0.5 * 20 = 10
    // Total = 30
    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        30.0f,
        output);
}

void testIntegralAccumulatesUsingTimeStep()
{
    PIDController controller(
        0.0f,
        2.0f,
        -255.0f,
        255.0f,
        -100.0f,
        100.0f);

    float output =
        controller.update(
            5.0f,
            0.0f,
            0.1f);

    // I = 2 * 5 * 0.1 = 1
    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        1.0f,
        output);

    output =
        controller.update(
            5.0f,
            0.0f,
            0.1f);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        2.0f,
        output);
}

void testIntegralContributionIsLimited()
{
    PIDController controller(
        0.0f,
        10.0f,
        -255.0f,
        255.0f,
        -3.0f,
        3.0f);

    const float output =
        controller.update(
            10.0f,
            0.0f,
            1.0f);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        3.0f,
        output);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        3.0f,
        controller.getIntegralOutput());
}

void testFinalOutputIsLimited()
{
    PIDController controller(
        100.0f,
        0.0f,
        -255.0f,
        255.0f,
        -100.0f,
        100.0f);

    const float positiveOutput =
        controller.update(
            10.0f,
            0.0f,
            0.01f);

    const float negativeOutput =
        controller.update(
            -10.0f,
            0.0f,
            0.01f);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        255.0f,
        positiveOutput);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        -255.0f,
        negativeOutput);
}

void testAntiWindupStopsFurtherIntegration()
{
    PIDController controller(
        300.0f,
        10.0f,
        -255.0f,
        255.0f,
        -100.0f,
        100.0f);

    const float output =
        controller.update(
            1.0f,
            0.0f,
            1.0f);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        255.0f,
        output);

    // P alone has saturated the output.
    // The positive integral must not continue accumulating.
    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        0.0f,
        controller.getIntegralOutput());
}

void testResetClearsIntegralOutput()
{
    PIDController controller(
        0.0f,
        2.0f,
        -255.0f,
        255.0f,
        -100.0f,
        100.0f);

    controller.update(
        5.0f,
        0.0f,
        0.1f);

    controller.reset();

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        0.0f,
        controller.getIntegralOutput());
}

void testOldUpdateInterfaceUsesZeroVelocity()
{
    PIDController controller(
        2.0f,
        0.0f,
        -255.0f,
        255.0f,
        -100.0f,
        100.0f,
        0.5f);

    // Old two-argument interface:
    // update(error, dt)
    const float output =
        controller.update(
            10.0f,
            0.01f);

    // Feedforward must be zero.
    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        20.0f,
        output);
}

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();

    RUN_TEST(testProportionalAndFeedforwardAreAdded);
    RUN_TEST(testIntegralAccumulatesUsingTimeStep);
    RUN_TEST(testIntegralContributionIsLimited);
    RUN_TEST(testFinalOutputIsLimited);
    RUN_TEST(testAntiWindupStopsFurtherIntegration);
    RUN_TEST(testResetClearsIntegralOutput);
    RUN_TEST(testOldUpdateInterfaceUsesZeroVelocity);

    return UNITY_END();
}