#include <unity.h>

#include "control/AxisController.h"
#include "control/Converter.h"
#include "control/HomingController.h"
#include "control/PIDController.h"
#include "control/XYCoordinator.h"
#include "hardware/Encoder.h"
#include "hardware/LimitSwitch.h"
#include "hardware/MotorDriver.h"
#include "system/PlotterSystem.h"
#include "system/TrajectoryPlanner.h"

namespace TestHardware
{
void reset();

void setMillis(unsigned long value);
void setMicros(unsigned long value);
void advanceMillis(unsigned long change);

void setEncoderCounts(
    int32_t countA,
    int32_t countB);

void setSwitchPressed(
    uint8_t pin,
    bool pressed);
}

namespace
{
constexpr uint8_t X_MIN_PIN = 2;
constexpr uint8_t X_MAX_PIN = 3;
constexpr uint8_t Y_MIN_PIN = 18;
constexpr uint8_t Y_MAX_PIN = 19;

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

    LimitSwitch xMinSwitch;
    LimitSwitch xMaxSwitch;
    LimitSwitch yMinSwitch;
    LimitSwitch yMaxSwitch;

    HomingConfig homingConfig;
    HomingController homingController;

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
          xMinSwitch(X_MIN_PIN, INPUT, 1UL),
          xMaxSwitch(X_MAX_PIN, INPUT, 1UL),
          yMinSwitch(Y_MIN_PIN, INPUT, 1UL),
          yMaxSwitch(Y_MAX_PIN, INPUT, 1UL),
          homingConfig{
              60,
              40,
              20,
              10,
              2.0f,
              2UL,
              2UL,
              100UL,
              20UL,
              20UL,
              1000UL},
          homingController(
              encoderA,
              encoderB,
              motorA,
              motorB,
              converter,
              xMinSwitch,
              xMaxSwitch,
              yMinSwitch,
              yMaxSwitch,
              homingConfig),
          system(
              axisA,
              axisB,
              coordinator,
              planner,
              homingController)
    {
    }

    void beginSystem()
    {
        TestHardware::setMillis(0UL);
        TestHardware::setEncoderCounts(0, 0);

        TestHardware::setSwitchPressed(X_MIN_PIN, false);
        TestHardware::setSwitchPressed(X_MAX_PIN, false);
        TestHardware::setSwitchPressed(Y_MIN_PIN, false);
        TestHardware::setSwitchPressed(Y_MAX_PIN, false);

        xMinSwitch.begin();
        xMaxSwitch.begin();
        yMinSwitch.begin();
        yMaxSwitch.begin();

        system.begin();

        TEST_ASSERT_EQUAL_INT(
            static_cast<int>(plotter::PlotterState::HOMING),
            static_cast<int>(system.state()));

        TEST_ASSERT_TRUE(homingController.isActive());
    }

    void updateAfter(
        unsigned long milliseconds = 1UL)
    {
        TestHardware::advanceMillis(milliseconds);
        system.update();
    }

    void setDebouncedSwitch(
        uint8_t pin,
        bool pressed,
        int32_t countA,
        int32_t countB)
    {
        TestHardware::setEncoderCounts(
            countA,
            countB);

        TestHardware::setSwitchPressed(
            pin,
            pressed);

        system.update();
        updateAfter(1UL);
    }

    void completeCurrentEnd(
        uint8_t pin,
        int32_t firstContactA,
        int32_t firstContactB,
        int32_t backoffA,
        int32_t backoffB,
        int32_t fineContactA,
        int32_t fineContactB,
        int32_t releaseA,
        int32_t releaseB)
    {
        setDebouncedSwitch(
            pin,
            true,
            firstContactA,
            firstContactB);

        TEST_ASSERT_EQUAL_INT(
            static_cast<int>(HomingPhase::CONTACT_PAUSE),
            static_cast<int>(homingController.phase()));

        updateAfter(homingConfig.contactPauseMs);
        updateAfter(1UL);

        setDebouncedSwitch(
            pin,
            false,
            backoffA,
            backoffB);

        TEST_ASSERT_EQUAL_INT(
            static_cast<int>(HomingPhase::FINE_APPROACH),
            static_cast<int>(homingController.phase()));

        updateAfter(1UL);

        setDebouncedSwitch(
            pin,
            true,
            fineContactA,
            fineContactB);

        TEST_ASSERT_EQUAL_INT(
            static_cast<int>(HomingPhase::FINE_CONTACT_PAUSE),
            static_cast<int>(homingController.phase()));

        TEST_ASSERT_EQUAL_INT16(0, motorA.getOutput());
        TEST_ASSERT_EQUAL_INT16(0, motorB.getOutput());

        updateAfter(homingConfig.fineContactPauseMs);

        TEST_ASSERT_EQUAL_INT(
            static_cast<int>(HomingPhase::FINAL_RELEASE),
            static_cast<int>(homingController.phase()));

        updateAfter(1UL);

        setDebouncedSwitch(
            pin,
            false,
            releaseA,
            releaseB);

        TEST_ASSERT_EQUAL_INT(
            static_cast<int>(HomingPhase::RECORD_POSITION),
            static_cast<int>(homingController.phase()));

        // RECORD_POSITION advances on the next non-blocking update.
        updateAfter(1UL);
    }

    void completeStartupHoming()
    {
        beginSystem();

        updateAfter(1UL);

        completeCurrentEnd(
            X_MAX_PIN,
            100, 100,
            98, 98,
            99, 99,
            98, 98);

        TEST_ASSERT_EQUAL_INT(
            static_cast<int>(HomingStage::X_ORIGIN),
            static_cast<int>(homingController.stage()));

        completeCurrentEnd(
            X_MIN_PIN,
            0, 0,
            2, 2,
            1, 1,
            2, 2);

        TEST_ASSERT_EQUAL_INT(
            static_cast<int>(HomingStage::Y_MAX),
            static_cast<int>(homingController.stage()));

        completeCurrentEnd(
            Y_MAX_PIN,
            101, -99,
            99, -97,
            100, -98,
            99, -97);

        TEST_ASSERT_EQUAL_INT(
            static_cast<int>(HomingStage::Y_ORIGIN),
            static_cast<int>(homingController.stage()));

        completeCurrentEnd(
            Y_MIN_PIN,
            1, 1,
            3, -1,
            2, 0,
            3, -1);

        TEST_ASSERT_EQUAL_INT(
            static_cast<int>(plotter::PlotterState::IDLE),
            static_cast<int>(system.state()));

        TEST_ASSERT_TRUE(system.machineZeroKnown());
        TEST_ASSERT_TRUE(homingController.isComplete());

        TEST_ASSERT_EQUAL_INT32(0, encoderA.getCount());
        TEST_ASSERT_EQUAL_INT32(0, encoderB.getCount());

        TEST_ASSERT_FALSE(coordinator.isActive());
        TEST_ASSERT_FALSE(axisA.isActive());
        TEST_ASSERT_FALSE(axisB.isActive());

        // Give the motion tests the same time origin as before integration.
        TestHardware::setMicros(0UL);
    }
};
}

void setUp()
{
    TestHardware::reset();
}

void tearDown()
{
}

void testStartupHomingRunsToCompletionWithoutManualFSMEvent()
{
    SystemFixture fixture;
    fixture.completeStartupHoming();

    const HomingResult result =
        fixture.homingController.result();

    TEST_ASSERT_TRUE(result.xValid);
    TEST_ASSERT_TRUE(result.yValid);

        TEST_ASSERT_FLOAT_WITHIN(
            FLOAT_TOLERANCE,
            96.0f,
            result.xTravelMm);

        TEST_ASSERT_FLOAT_WITHIN(
            FLOAT_TOLERANCE,
            96.0f,
            result.yTravelMm);
}

void testHomingFaultMapsIntoPlotterFaultAndStopsMotors()
{
    SystemFixture fixture;
    fixture.beginSystem();

    fixture.updateAfter(1UL);

    TEST_ASSERT_NOT_EQUAL(0, fixture.motorA.getOutput());
    TEST_ASSERT_NOT_EQUAL(0, fixture.motorB.getOutput());

    // X_MIN is unexpected while startup homing is searching for X_MAX.
    fixture.setDebouncedSwitch(
        X_MIN_PIN,
        true,
        -10,
        -10);

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(plotter::PlotterState::FAULT),
        static_cast<int>(fixture.system.state()));

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(plotter::FaultCode::WRONG_HOMING_LIMIT),
        static_cast<int>(fixture.system.activeFault()));

    TEST_ASSERT_FALSE(fixture.homingController.isActive());
    TEST_ASSERT_EQUAL_INT16(0, fixture.motorA.getOutput());
    TEST_ASSERT_EQUAL_INT16(0, fixture.motorB.getOutput());
}

void testTrajectoryReferenceFlowsIntoBothMotorAxes()
{
    SystemFixture fixture;
    fixture.completeStartupHoming();

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

    // At t = 0.5 s, triangular-profile displacement is 1.25 mm.
    // Pure X maps to A = 1.25 and B = 1.25 counts in this fixture.
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

    TEST_ASSERT_TRUE(fixture.coordinator.isActive());

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(plotter::PlotterState::MOVING),
        static_cast<int>(fixture.system.state()));
}

void testMoveCompletesOnlyAfterTrajectoryAndSettlingTime()
{
    SystemFixture fixture;
    fixture.completeStartupHoming();

    TEST_ASSERT_TRUE(
        fixture.system.requestMove(
            10.0f,
            0.0f,
            600.0f,
            10.0f)
            .accepted);

    TestHardware::setEncoderCounts(10, 10);
    TestHardware::setMicros(2000000UL);

    fixture.system.update();

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(plotter::PlotterState::MOVING),
        static_cast<int>(fixture.system.state()));

    TestHardware::setMicros(2049000UL);
    fixture.system.update();

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(plotter::PlotterState::MOVING),
        static_cast<int>(fixture.system.state()));

    TestHardware::setMicros(2050000UL);
    fixture.system.update();

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(plotter::PlotterState::IDLE),
        static_cast<int>(fixture.system.state()));

    TEST_ASSERT_FALSE(fixture.coordinator.isActive());
    TEST_ASSERT_EQUAL_INT16(0, fixture.motorA.getOutput());
    TEST_ASSERT_EQUAL_INT16(0, fixture.motorB.getOutput());
}

void testOneAxisOutsideTolerancePreventsCompletion()
{
    SystemFixture fixture;
    fixture.completeStartupHoming();

    TEST_ASSERT_TRUE(
        fixture.system.requestMove(
            10.0f,
            0.0f,
            600.0f,
            10.0f)
            .accepted);

    TestHardware::setEncoderCounts(10, 9);
    TestHardware::setMicros(2000000UL);
    fixture.system.update();

    TestHardware::setMicros(2100000UL);
    fixture.system.update();

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(plotter::PlotterState::MOVING),
        static_cast<int>(fixture.system.state()));

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

void testFaultStopsPlannerCoordinatorHomingAndMotors()
{
    SystemFixture fixture;
    fixture.completeStartupHoming();

    TEST_ASSERT_TRUE(
        fixture.system.requestMove(
            10.0f,
            0.0f,
            600.0f,
            10.0f)
            .accepted);

    TestHardware::setMicros(500000UL);
    fixture.system.update();

    TEST_ASSERT_NOT_EQUAL(0, fixture.motorA.getOutput());
    TEST_ASSERT_NOT_EQUAL(0, fixture.motorB.getOutput());

    const plotter::FSMResult faultResult =
        fixture.system.reportFault(
            plotter::FaultCode::UNEXPECTED_LIMIT);

    TEST_ASSERT_TRUE(faultResult.accepted);

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(plotter::PlotterState::FAULT),
        static_cast<int>(fixture.system.state()));

    TEST_ASSERT_FALSE(fixture.planner.isActive());
    TEST_ASSERT_FALSE(fixture.coordinator.isActive());
    TEST_ASSERT_FALSE(fixture.homingController.isActive());
    TEST_ASSERT_FALSE(fixture.axisA.isActive());
    TEST_ASSERT_FALSE(fixture.axisB.isActive());

    TEST_ASSERT_EQUAL_INT16(0, fixture.motorA.getOutput());
    TEST_ASSERT_EQUAL_INT16(0, fixture.motorB.getOutput());
}

void testInvalidMotionParametersAreRejectedBeforeMoving()
{
    SystemFixture fixture;
    fixture.completeStartupHoming();

    const plotter::FSMResult result =
        fixture.system.requestMove(
            10.0f,
            0.0f,
            0.0f,
            10.0f);

    TEST_ASSERT_FALSE(result.accepted);

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(
            plotter::RejectReason::INVALID_MOTION_PARAMETERS),
        static_cast<int>(result.rejectReason));

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(plotter::PlotterState::IDLE),
        static_cast<int>(fixture.system.state()));

    TEST_ASSERT_FALSE(fixture.coordinator.isActive());
    TEST_ASSERT_FALSE(fixture.planner.isActive());
}

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();

    RUN_TEST(
        testStartupHomingRunsToCompletionWithoutManualFSMEvent);

    RUN_TEST(
        testHomingFaultMapsIntoPlotterFaultAndStopsMotors);

    RUN_TEST(
        testTrajectoryReferenceFlowsIntoBothMotorAxes);

    RUN_TEST(
        testMoveCompletesOnlyAfterTrajectoryAndSettlingTime);

    RUN_TEST(
        testOneAxisOutsideTolerancePreventsCompletion);

    RUN_TEST(
        testFaultStopsPlannerCoordinatorHomingAndMotors);

    RUN_TEST(
        testInvalidMotionParametersAreRejectedBeforeMoving);

    return UNITY_END();
}
