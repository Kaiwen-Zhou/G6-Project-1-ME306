#include <unity.h>

#include "control/AxisController.h"
#include "control/Converter.h"
#include "control/PIDController.h"
#include "control/XYCoordinator.h"
#include "hardware/Encoder.h"
#include "hardware/MotorDriver.h"
#include "system/PlotterSystem.h"
#include "system/TrajectoryPlanner.h"

namespace TestHardware
{
void reset();

void setMicros(unsigned long value);

void setEncoderCounts(
    int32_t countA,
    int32_t countB);

unsigned int getCountPairReadCount();
}

namespace
{

constexpr float FLOAT_TOLERANCE = 0.0001f;

struct SystemFixture
{
    Encoder encoderA;
    Encoder encoderB;

    PIDController pidA;
    PIDController pidB;

    MotorDriver motorA;
    MotorDriver motorB;

    AxisController axisA;
    AxisController axisB;

    Converter converter;
    XYCoordinator coordinator;

    plotter::TrajectoryPlanner planner;
    plotter::PlotterSystem system;

    SystemFixture()
        : encoderA(true),
          encoderB(false),
          pidA(
              10.0f, 0.0f,
              -255.0f, 255.0f,
              -100.0f, 100.0f),
          pidB(
              10.0f, 0.0f,
              -255.0f, 255.0f,
              -100.0f, 100.0f),
          motorA(1, 2),
          motorB(3, 4),
          axisA(
              pidA,
              motorA,
              0.25f),
          axisB(
              pidB,
              motorB,
              0.25f),
          converter(
              1.0f,
              1.0f),
          coordinator(
              encoderA,
              encoderB,
              axisA,
              axisB,
              converter,
              1000UL),
          planner(),
          system(
              axisA,
              axisB,
              coordinator,
              planner)
    {
    }

    void beginAndCompleteStartupHoming()
    {
        TestHardware::setMicros(0UL);
        TestHardware::setEncoderCounts(0, 0);

        system.begin();

        TEST_ASSERT_EQUAL_INT(
            static_cast<int>(plotter::PlotterState::HOMING),
            static_cast<int>(system.state()));

        const plotter::FSMResult result =
            system.reportHomingComplete();

        TEST_ASSERT_TRUE(result.accepted);

        TEST_ASSERT_EQUAL_INT(
            static_cast<int>(plotter::PlotterState::IDLE),
            static_cast<int>(system.state()));
    }
};

}  // namespace

void setUp()
{
    TestHardware::reset();
}

void tearDown()
{
}

void testTrajectoryReferenceFlowsIntoBothMotorAxes()
{
    SystemFixture fixture;
    fixture.beginAndCompleteStartupHoming();

    const plotter::FSMResult result =
        fixture.system.requestMove(
            10.0f,
            0.0f,
            600.0f,
            10.0f);

    TEST_ASSERT_TRUE(result.accepted);

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(plotter::PlotterState::MOVING),
        static_cast<int>(fixture.system.state()));

    // At t = 0.5 s:
    // path displacement = 1.25 mm.
    //
    // Pure X:
    // A = X + Y = 1.25 counts
    // B = X - Y = 1.25 counts
    TestHardware::setMicros(500000UL);
    fixture.system.update();

    const AxisTelemetry telemetryA =
        fixture.axisA.getTelemetry();

    const AxisTelemetry telemetryB =
        fixture.axisB.getTelemetry();

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        1.25f,
        telemetryA.referencePosition);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        1.25f,
        telemetryB.referencePosition);

    TEST_ASSERT_TRUE(
        fixture.coordinator.isActive());

    // Reaching an intermediate reference must not finish the move.
    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(plotter::PlotterState::MOVING),
        static_cast<int>(fixture.system.state()));
}

void testMoveCompletesOnlyAfterTrajectoryAndSettlingTime()
{
    SystemFixture fixture;
    fixture.beginAndCompleteStartupHoming();

    TEST_ASSERT_TRUE(
        fixture.system.requestMove(
            10.0f,
            0.0f,
            600.0f,
            10.0f)
            .accepted);

    // At t = 2 s the triangular trajectory reaches 10 mm.
    // Pure X movement produces A = 10 and B = 10.
    TestHardware::setEncoderCounts(10, 10);
    TestHardware::setMicros(2000000UL);

    fixture.system.update();

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(plotter::PlotterState::MOVING),
        static_cast<int>(fixture.system.state()));

    // Only 49 ms settled: still moving.
    TestHardware::setMicros(2049000UL);
    fixture.system.update();

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(plotter::PlotterState::MOVING),
        static_cast<int>(fixture.system.state()));

    // 50 ms continuously settled: move completes.
    TestHardware::setMicros(2050000UL);
    fixture.system.update();

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(plotter::PlotterState::IDLE),
        static_cast<int>(fixture.system.state()));

    TEST_ASSERT_FALSE(
        fixture.coordinator.isActive());

    TEST_ASSERT_EQUAL_INT16(
        0,
        fixture.motorA.getOutput());

    TEST_ASSERT_EQUAL_INT16(
        0,
        fixture.motorB.getOutput());
}

void testOneAxisOutsideTolerancePreventsCompletion()
{
    SystemFixture fixture;
    fixture.beginAndCompleteStartupHoming();

    TEST_ASSERT_TRUE(
        fixture.system.requestMove(
            10.0f,
            0.0f,
            600.0f,
            10.0f)
            .accepted);

    // A reaches its final reference, B does not.
    TestHardware::setEncoderCounts(10, 9);
    TestHardware::setMicros(2000000UL);
    fixture.system.update();

    TestHardware::setMicros(2100000UL);
    fixture.system.update();

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(plotter::PlotterState::MOVING),
        static_cast<int>(fixture.system.state()));

    // B now reaches the final reference. Settling begins here.
    TestHardware::setEncoderCounts(10, 10);
    TestHardware::setMicros(2101000UL);
    fixture.system.update();

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(plotter::PlotterState::MOVING),
        static_cast<int>(fixture.system.state()));

    TestHardware::setMicros(2151000UL);
    fixture.system.update();

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(plotter::PlotterState::IDLE),
        static_cast<int>(fixture.system.state()));
}

void testFaultStopsPlannerCoordinatorAndMotors()
{
    SystemFixture fixture;
    fixture.beginAndCompleteStartupHoming();

    TEST_ASSERT_TRUE(
        fixture.system.requestMove(
            10.0f,
            0.0f,
            600.0f,
            10.0f)
            .accepted);

    TestHardware::setMicros(500000UL);
    fixture.system.update();

    TEST_ASSERT_NOT_EQUAL(
        0,
        fixture.motorA.getOutput());

    TEST_ASSERT_NOT_EQUAL(
        0,
        fixture.motorB.getOutput());

    const plotter::FSMResult faultResult =
        fixture.system.reportFault(
            plotter::FaultCode::UNEXPECTED_LIMIT);

    TEST_ASSERT_TRUE(faultResult.accepted);

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(plotter::PlotterState::FAULT),
        static_cast<int>(fixture.system.state()));

    TEST_ASSERT_FALSE(
        fixture.planner.isActive());

    TEST_ASSERT_FALSE(
        fixture.coordinator.isActive());

    TEST_ASSERT_FALSE(
        fixture.axisA.isActive());

    TEST_ASSERT_FALSE(
        fixture.axisB.isActive());

    TEST_ASSERT_EQUAL_INT16(
        0,
        fixture.motorA.getOutput());

    TEST_ASSERT_EQUAL_INT16(
        0,
        fixture.motorB.getOutput());
}

void testInvalidMotionParametersAreRejectedBeforeMoving()
{
    SystemFixture fixture;
    fixture.beginAndCompleteStartupHoming();

    const plotter::FSMResult result =
        fixture.system.requestMove(
            10.0f,
            0.0f,
            0.0f,
            10.0f);

    TEST_ASSERT_FALSE(result.accepted);

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(
            plotter::RejectReason::
                INVALID_MOTION_PARAMETERS),
        static_cast<int>(result.rejectReason));

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(plotter::PlotterState::IDLE),
        static_cast<int>(fixture.system.state()));

    TEST_ASSERT_FALSE(
        fixture.coordinator.isActive());

    TEST_ASSERT_FALSE(
        fixture.planner.isActive());
}

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();

    RUN_TEST(
        testTrajectoryReferenceFlowsIntoBothMotorAxes);

    RUN_TEST(
        testMoveCompletesOnlyAfterTrajectoryAndSettlingTime);

    RUN_TEST(
        testOneAxisOutsideTolerancePreventsCompletion);

    RUN_TEST(
        testFaultStopsPlannerCoordinatorAndMotors);

    RUN_TEST(
        testInvalidMotionParametersAreRejectedBeforeMoving);

    return UNITY_END();
}