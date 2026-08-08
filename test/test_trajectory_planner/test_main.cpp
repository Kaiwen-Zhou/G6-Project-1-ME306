#include <unity.h>

#include "system/TrajectoryPlanner.h"

using plotter::TrajectoryPlanner;
using plotter::TrajectoryReference;

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

void testInvalidMotionLimitsAreRejected()
{
    TrajectoryPlanner planner;

    TEST_ASSERT_FALSE(
        planner.startMove(
            0.0f,
            0.0f,
            10.0f,
            0.0f,
            0.0f,
            10.0f));

    TEST_ASSERT_FALSE(planner.isActive());
    TEST_ASSERT_FALSE(planner.isComplete());

    TEST_ASSERT_FALSE(
        planner.startMove(
            0.0f,
            0.0f,
            10.0f,
            0.0f,
            600.0f,
            0.0f));

    TEST_ASSERT_FALSE(planner.isActive());
    TEST_ASSERT_FALSE(planner.isComplete());
}

void testZeroDistanceMoveCompletesAtZeroDisplacement()
{
    TrajectoryPlanner planner;

    TEST_ASSERT_TRUE(
        planner.startMove(
            12.0f,
            8.0f,
            12.0f,
            8.0f,
            600.0f,
            10.0f));

    const TrajectoryReference reference =
        planner.update(1.0f);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        0.0f,
        reference.xDisplacementMm);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        0.0f,
        reference.yDisplacementMm);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        0.0f,
        reference.xVelocityMmPerSecond);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        0.0f,
        reference.yVelocityMmPerSecond);

    TEST_ASSERT_TRUE(reference.complete);
    TEST_ASSERT_FALSE(planner.isActive());
    TEST_ASSERT_TRUE(planner.isComplete());
}

void testTrapezoidalProfileUsesMoveRelativeDisplacement()
{
    TrajectoryPlanner planner;

    // Absolute Cartesian move:
    // start  = (10, 20)
    // target = (30, 20)
    //
    // Relative output must be:
    // start = (0, 0)
    // end   = (20, 0)
    //
    // Feedrate = 600 mm/min = 10 mm/s
    // Acceleration = 10 mm/s^2
    //
    // Acceleration time = 1 s
    // Cruise time       = 1 s
    // Total time        = 3 s
    TEST_ASSERT_TRUE(
        planner.startMove(
            10.0f,
            20.0f,
            30.0f,
            20.0f,
            600.0f,
            10.0f));

    TrajectoryReference reference =
        planner.update(0.0f);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        0.0f,
        reference.xDisplacementMm);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        0.0f,
        reference.yDisplacementMm);

    // t = 0.5 s:
    // s = 0.5 * 10 * 0.5^2 = 1.25 mm
    // v = 10 * 0.5 = 5 mm/s
    reference = planner.update(0.5f);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        1.25f,
        reference.xDisplacementMm);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        5.0f,
        reference.xVelocityMmPerSecond);

    // t = 1.0 s: end of acceleration.
    reference = planner.update(0.5f);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        5.0f,
        reference.xDisplacementMm);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        10.0f,
        reference.xVelocityMmPerSecond);

    // t = 2.0 s: end of cruise.
    reference = planner.update(1.0f);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        15.0f,
        reference.xDisplacementMm);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        10.0f,
        reference.xVelocityMmPerSecond);

    // t = 2.5 s: halfway through deceleration.
    reference = planner.update(0.5f);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        18.75f,
        reference.xDisplacementMm);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        5.0f,
        reference.xVelocityMmPerSecond);

    // t = 3.0 s: exact final relative displacement.
    reference = planner.update(0.5f);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        20.0f,
        reference.xDisplacementMm);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        0.0f,
        reference.yDisplacementMm);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        0.0f,
        reference.xVelocityMmPerSecond);

    TEST_ASSERT_TRUE(reference.complete);
    TEST_ASSERT_FALSE(planner.isActive());
    TEST_ASSERT_TRUE(planner.isComplete());
}

void testShortDiagonalMoveUsesTriangularProfile()
{
    TrajectoryPlanner planner;

    // Move displacement = (6, 8), path length = 10 mm.
    //
    // Requested velocity is deliberately too high, so the planner
    // must generate a triangular profile.
    TEST_ASSERT_TRUE(
        planner.startMove(
            -4.0f,
            7.0f,
            2.0f,
            15.0f,
            6000.0f,
            10.0f));

    // At t = 0.5 s:
    // path displacement = 1.25 mm
    // path velocity = 5 mm/s
    //
    // Unit direction = (0.6, 0.8)
    TrajectoryReference reference =
        planner.update(0.5f);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        0.75f,
        reference.xDisplacementMm);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        1.0f,
        reference.yDisplacementMm);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        3.0f,
        reference.xVelocityMmPerSecond);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        4.0f,
        reference.yVelocityMmPerSecond);

    // Total triangular-profile duration is 2 seconds.
    reference = planner.update(1.5f);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        6.0f,
        reference.xDisplacementMm);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        8.0f,
        reference.yDisplacementMm);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        0.0f,
        reference.xVelocityMmPerSecond);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        0.0f,
        reference.yVelocityMmPerSecond);

    TEST_ASSERT_TRUE(reference.complete);
}

void testNonPositiveTimeStepLeavesReferenceUnchanged()
{
    TrajectoryPlanner planner;

    TEST_ASSERT_TRUE(
        planner.startMove(
            0.0f,
            0.0f,
            20.0f,
            0.0f,
            600.0f,
            10.0f));

    const TrajectoryReference before =
        planner.update(0.5f);

    const TrajectoryReference afterZero =
        planner.update(0.0f);

    const TrajectoryReference afterNegative =
        planner.update(-1.0f);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        before.xDisplacementMm,
        afterZero.xDisplacementMm);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        before.xDisplacementMm,
        afterNegative.xDisplacementMm);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        before.xVelocityMmPerSecond,
        afterNegative.xVelocityMmPerSecond);

    TEST_ASSERT_TRUE(planner.isActive());
}

void testStopHoldsLatestDisplacementAndClearsVelocity()
{
    TrajectoryPlanner planner;

    TEST_ASSERT_TRUE(
        planner.startMove(
            0.0f,
            0.0f,
            20.0f,
            0.0f,
            600.0f,
            10.0f));

    const TrajectoryReference beforeStop =
        planner.update(0.5f);

    planner.stop();

    const TrajectoryReference afterStop =
        planner.update(10.0f);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        beforeStop.xDisplacementMm,
        afterStop.xDisplacementMm);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        beforeStop.yDisplacementMm,
        afterStop.yDisplacementMm);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        0.0f,
        afterStop.xVelocityMmPerSecond);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        0.0f,
        afterStop.yVelocityMmPerSecond);

    TEST_ASSERT_FALSE(afterStop.complete);
    TEST_ASSERT_FALSE(planner.isActive());
    TEST_ASSERT_FALSE(planner.isComplete());
}

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();

    RUN_TEST(testInvalidMotionLimitsAreRejected);
    RUN_TEST(testZeroDistanceMoveCompletesAtZeroDisplacement);
    RUN_TEST(testTrapezoidalProfileUsesMoveRelativeDisplacement);
    RUN_TEST(testShortDiagonalMoveUsesTriangularProfile);
    RUN_TEST(testNonPositiveTimeStepLeavesReferenceUnchanged);
    RUN_TEST(testStopHoldsLatestDisplacementAndClearsVelocity);

    return UNITY_END();
}