#include <unity.h>

#include "control/AxisController.h"
#include "control/Converter.h"
#include "control/PIDController.h"
#include "control/XYCoordinator.h"
#include "hardware/Encoder.h"
#include "hardware/MotorDriver.h"

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
}

void setUp()
{
    TestHardware::reset();
}

void tearDown()
{
}

void testStartMoveUsesOneSynchronizedSnapshot()
{
    Encoder encoderA(true);
    Encoder encoderB(false);

    PIDController pidA(
        0.0f, 0.0f,
        -255.0f, 255.0f,
        -100.0f, 100.0f);

    PIDController pidB(
        0.0f, 0.0f,
        -255.0f, 255.0f,
        -100.0f, 100.0f);

    MotorDriver motorA(1, 2);
    MotorDriver motorB(3, 4);

    AxisController axisA(
        pidA,
        motorA,
        1.0f);

    AxisController axisB(
        pidB,
        motorB,
        1.0f);

    Converter converter(
        1.0f,
        1.0f);

    XYCoordinator coordinator(
        encoderA,
        encoderB,
        axisA,
        axisB,
        converter,
        1000UL);

    coordinator.begin();

    TestHardware::setEncoderCounts(
        100,
        200);

    TestHardware::setMicros(5000UL);

    coordinator.startMove();

    TEST_ASSERT_EQUAL_UINT(
        1,
        TestHardware::getCountPairReadCount());

    const XYCoordinatorTelemetry telemetry =
        coordinator.getTelemetry();

    TEST_ASSERT_TRUE(telemetry.active);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        100.0f,
        telemetry.motorA.referencePosition);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        200.0f,
        telemetry.motorB.referencePosition);

    TEST_ASSERT_EQUAL_INT32(
        100,
        telemetry.motorA.currentPosition);

    TEST_ASSERT_EQUAL_INT32(
        200,
        telemetry.motorB.currentPosition);

    TEST_ASSERT_EQUAL_INT16(
        0,
        telemetry.motorA.motorOutput);

    TEST_ASSERT_EQUAL_INT16(
        0,
        telemetry.motorB.motorOutput);
}

void testCartesianReferenceBecomesAbsoluteABReference()
{
    Encoder encoderA(true);
    Encoder encoderB(false);

    PIDController pidA(
        0.0f, 0.0f,
        -255.0f, 255.0f,
        -100.0f, 100.0f);

    PIDController pidB(
        0.0f, 0.0f,
        -255.0f, 255.0f,
        -100.0f, 100.0f);

    MotorDriver motorA(1, 2);
    MotorDriver motorB(3, 4);

    AxisController axisA(
        pidA,
        motorA,
        1.0f);

    AxisController axisB(
        pidB,
        motorB,
        1.0f);

    // 1 mm per count makes the expected values easy to verify.
    Converter converter(
        1.0f,
        1.0f);

    XYCoordinator coordinator(
        encoderA,
        encoderB,
        axisA,
        axisB,
        converter,
        1000UL);

    coordinator.begin();

    TestHardware::setEncoderCounts(
        100,
        200);

    coordinator.startMove();

    coordinator.setCartesianReference(
        10.0f,  // X displacement
        4.0f,   // Y displacement
        3.0f,   // X velocity
        1.0f);  // Y velocity

    const XYCoordinatorTelemetry telemetry =
        coordinator.getTelemetry();

    // A = X + Y = 14 counts
    // B = X - Y = 6 counts
    //
    // Absolute references:
    // A = 100 + 14 = 114
    // B = 200 + 6  = 206
    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        114.0f,
        telemetry.motorA.referencePosition);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        206.0f,
        telemetry.motorB.referencePosition);

    // A velocity = X velocity + Y velocity = 4
    // B velocity = X velocity - Y velocity = 2
    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        4.0f,
        telemetry.motorA.referenceVelocity);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        2.0f,
        telemetry.motorB.referenceVelocity);
}

void testUpdateUsesOneSnapshotAndOneSharedTimeStep()
{
    Encoder encoderA(true);
    Encoder encoderB(false);

    PIDController pidA(
        0.0f,
        100.0f,
        -255.0f,
        255.0f,
        -100.0f,
        100.0f);

    PIDController pidB(
        0.0f,
        100.0f,
        -255.0f,
        255.0f,
        -100.0f,
        100.0f);

    MotorDriver motorA(1, 2);
    MotorDriver motorB(3, 4);

    AxisController axisA(
        pidA,
        motorA,
        1.0f);

    AxisController axisB(
        pidB,
        motorB,
        1.0f);

    Converter converter(
        1.0f,
        1.0f);

    XYCoordinator coordinator(
        encoderA,
        encoderB,
        axisA,
        axisB,
        converter,
        1000UL);

    coordinator.begin();

    TestHardware::setEncoderCounts(0, 0);
    TestHardware::setMicros(10000UL);

    coordinator.startMove();

    // X = 15, Y = -5 gives:
    // A reference = 10
    // B reference = 20
    coordinator.setCartesianReference(
        15.0f,
        -5.0f,
        0.0f,
        0.0f);

    // Only 500 us elapsed: no control cycle and no new snapshot.
    TestHardware::setMicros(10500UL);
    coordinator.update();

    TEST_ASSERT_EQUAL_UINT(
        1,
        TestHardware::getCountPairReadCount());

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        0.0f,
        pidA.getIntegralOutput());

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        0.0f,
        pidB.getIntegralOutput());

    // Exactly 1000 us = 0.001 s has now elapsed.
    TestHardware::setMicros(11000UL);
    coordinator.update();

    // One snapshot at startMove and one for this control cycle.
    TEST_ASSERT_EQUAL_UINT(
        2,
        TestHardware::getCountPairReadCount());

    // A integral:
    // 100 * 10 counts * 0.001 s = 1
    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        1.0f,
        pidA.getIntegralOutput());

    // B integral:
    // 100 * 20 counts * 0.001 s = 2
    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        2.0f,
        pidB.getIntegralOutput());

    TEST_ASSERT_EQUAL_INT16(
        1,
        motorA.getOutput());

    TEST_ASSERT_EQUAL_INT16(
        2,
        motorB.getOutput());
}

void testTelemetryConvertsLatestCountsBackToXY()
{
    Encoder encoderA(true);
    Encoder encoderB(false);

    PIDController pidA(
        0.0f, 0.0f,
        -255.0f, 255.0f,
        -100.0f, 100.0f);

    PIDController pidB(
        0.0f, 0.0f,
        -255.0f, 255.0f,
        -100.0f, 100.0f);

    MotorDriver motorA(1, 2);
    MotorDriver motorB(3, 4);

    AxisController axisA(
        pidA,
        motorA,
        1.0f);

    AxisController axisB(
        pidB,
        motorB,
        1.0f);

    Converter converter(
        1.0f,
        1.0f);

    XYCoordinator coordinator(
        encoderA,
        encoderB,
        axisA,
        axisB,
        converter,
        1000UL);

    coordinator.begin();

    TestHardware::setEncoderCounts(
        100,
        200);

    TestHardware::setMicros(1000UL);
    coordinator.startMove();

    // Relative motor displacement:
    // A = 14, B = 6
    //
    // X = (A + B) / 2 = 10
    // Y = (A - B) / 2 = 4
    TestHardware::setEncoderCounts(
        114,
        206);

    TestHardware::setMicros(2000UL);
    coordinator.update();

    const XYCoordinatorTelemetry telemetry =
        coordinator.getTelemetry();

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        10.0f,
        telemetry.actualXDisplacementMm);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        4.0f,
        telemetry.actualYDisplacementMm);
}

void testStopStopsAndDeactivatesBothAxes()
{
    Encoder encoderA(true);
    Encoder encoderB(false);

    PIDController pidA(
        10.0f, 0.0f,
        -255.0f, 255.0f,
        -100.0f, 100.0f);

    PIDController pidB(
        10.0f, 0.0f,
        -255.0f, 255.0f,
        -100.0f, 100.0f);

    MotorDriver motorA(1, 2);
    MotorDriver motorB(3, 4);

    AxisController axisA(
        pidA,
        motorA,
        1.0f);

    AxisController axisB(
        pidB,
        motorB,
        1.0f);

    Converter converter(
        1.0f,
        1.0f);

    XYCoordinator coordinator(
        encoderA,
        encoderB,
        axisA,
        axisB,
        converter,
        1000UL);

    coordinator.begin();

    TestHardware::setEncoderCounts(0, 0);
    TestHardware::setMicros(1000UL);

    coordinator.startMove();

    coordinator.setCartesianReference(
        10.0f,
        0.0f,
        0.0f,
        0.0f);

    TestHardware::setMicros(2000UL);
    coordinator.update();

    TEST_ASSERT_NOT_EQUAL(
        0,
        motorA.getOutput());

    TEST_ASSERT_NOT_EQUAL(
        0,
        motorB.getOutput());

    coordinator.stop();

    TEST_ASSERT_FALSE(
        coordinator.isActive());

    TEST_ASSERT_FALSE(
        axisA.isActive());

    TEST_ASSERT_FALSE(
        axisB.isActive());

    TEST_ASSERT_EQUAL_INT16(
        0,
        motorA.getOutput());

    TEST_ASSERT_EQUAL_INT16(
        0,
        motorB.getOutput());
}

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();

    RUN_TEST(testStartMoveUsesOneSynchronizedSnapshot);
    RUN_TEST(testCartesianReferenceBecomesAbsoluteABReference);
    RUN_TEST(testUpdateUsesOneSnapshotAndOneSharedTimeStep);
    RUN_TEST(testTelemetryConvertsLatestCountsBackToXY);
    RUN_TEST(testStopStopsAndDeactivatesBothAxes);

    return UNITY_END();
}