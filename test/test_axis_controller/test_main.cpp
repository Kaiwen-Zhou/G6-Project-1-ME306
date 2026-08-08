#include <unity.h>

#include "control/AxisController.h"
#include "control/PIDController.h"
#include "hardware/MotorDriver.h"

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

void testStartTrackingHoldsCurrentPosition()
{
    PIDController pid(
        1.0f, 0.0f,
        -255.0f, 255.0f,
        -100.0f, 100.0f);

    MotorDriver motor(1, 2);

    AxisController axis(
        pid,
        motor,
        2.0f);

    axis.begin();
    axis.startTracking(120);

    const AxisTelemetry telemetry =
        axis.getTelemetry();

    TEST_ASSERT_TRUE(telemetry.active);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        120.0f,
        telemetry.referencePosition);

    TEST_ASSERT_EQUAL_INT32(
        120,
        telemetry.currentPosition);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        0.0f,
        telemetry.trackingError);

    TEST_ASSERT_EQUAL_INT16(
        0,
        telemetry.motorOutput);
}

void testPositionFeedbackAndFeedforwardAreApplied()
{
    PIDController pid(
        2.0f,    // Kp
        0.0f,    // Ki
        -255.0f, 255.0f,
        -100.0f, 100.0f,
        0.5f);   // Kv

    MotorDriver motor(1, 2);

    AxisController axis(
        pid,
        motor,
        2.0f);

    axis.begin();
    axis.startTracking(0);

    axis.setReference(
        10.0f,
        20.0f);

    axis.update(
        4,
        0.01f);

    // Error = 10 - 4 = 6 counts
    // P = 2 * 6 = 12
    // FF = 0.5 * 20 = 10
    // Motor output = 22
    TEST_ASSERT_EQUAL_INT16(
        22,
        motor.getOutput());
}

void testFractionalReferenceIsPreserved()
{
    PIDController pid(
        10.0f, 0.0f,
        -255.0f, 255.0f,
        -100.0f, 100.0f);

    MotorDriver motor(1, 2);

    AxisController axis(
        pid,
        motor,
        1.0f);

    axis.begin();
    axis.startTracking(0);

    axis.setReference(
        10.5f,
        0.0f);

    axis.update(
        10,
        0.01f);

    const AxisTelemetry telemetry =
        axis.getTelemetry();

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        10.5f,
        telemetry.referencePosition);

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        0.5f,
        telemetry.trackingError);

    TEST_ASSERT_EQUAL_INT16(
        5,
        telemetry.motorOutput);
}

void testNonPositiveTimeStepDoesNotDriveMotor()
{
    PIDController pid(
        10.0f, 5.0f,
        -255.0f, 255.0f,
        -100.0f, 100.0f);

    MotorDriver motor(1, 2);

    AxisController axis(
        pid,
        motor,
        1.0f);

    axis.begin();
    axis.startTracking(0);
    axis.setReference(10.0f, 0.0f);

    axis.update(0, 0.0f);

    TEST_ASSERT_EQUAL_INT16(
        0,
        motor.getOutput());

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        0.0f,
        pid.getIntegralOutput());
}

void testToleranceUsesFloatTrackingError()
{
    PIDController pid(
        0.0f, 0.0f,
        -255.0f, 255.0f,
        -100.0f, 100.0f);

    MotorDriver motor(1, 2);

    AxisController axis(
        pid,
        motor,
        2.0f);

    axis.begin();
    axis.startTracking(0);

    axis.setReference(10.5f, 0.0f);
    axis.update(9, 0.01f);

    // Error = 1.5 counts.
    TEST_ASSERT_TRUE(
        axis.isWithinTolerance());

    axis.setReference(12.0f, 0.0f);

    // Stored current position remains 9:
    // error = 3 counts.
    TEST_ASSERT_FALSE(
        axis.isWithinTolerance());
}

void testInactiveControllerDoesNotDriveMotor()
{
    PIDController pid(
        10.0f, 0.0f,
        -255.0f, 255.0f,
        -100.0f, 100.0f);

    MotorDriver motor(1, 2);

    AxisController axis(
        pid,
        motor,
        1.0f);

    axis.begin();
    axis.setReference(20.0f, 0.0f);
    axis.update(0, 0.01f);

    TEST_ASSERT_EQUAL_INT16(
        0,
        motor.getOutput());
}

void testStopClearsMotorAndIntegral()
{
    PIDController pid(
        0.0f, 10.0f,
        -255.0f, 255.0f,
        -100.0f, 100.0f);

    MotorDriver motor(1, 2);

    AxisController axis(
        pid,
        motor,
        1.0f);

    axis.begin();
    axis.startTracking(0);
    axis.setReference(10.0f, 0.0f);
    axis.update(0, 0.1f);

    TEST_ASSERT_NOT_EQUAL(
        0,
        motor.getOutput());

    axis.stop();

    TEST_ASSERT_EQUAL_INT16(
        0,
        motor.getOutput());

    TEST_ASSERT_FALSE(
        axis.isActive());

    TEST_ASSERT_FLOAT_WITHIN(
        FLOAT_TOLERANCE,
        0.0f,
        pid.getIntegralOutput());
}

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();

    RUN_TEST(testStartTrackingHoldsCurrentPosition);
    RUN_TEST(testPositionFeedbackAndFeedforwardAreApplied);
    RUN_TEST(testFractionalReferenceIsPreserved);
    RUN_TEST(testNonPositiveTimeStepDoesNotDriveMotor);
    RUN_TEST(testToleranceUsesFloatTrackingError);
    RUN_TEST(testInactiveControllerDoesNotDriveMotor);
    RUN_TEST(testStopClearsMotorAndIntegral);

    return UNITY_END();
}