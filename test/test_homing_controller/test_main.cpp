#include <unity.h>

#include "control/Converter.h"
#include "control/HomingController.h"
#include "hardware/Encoder.h"
#include "hardware/LimitSwitch.h"
#include "hardware/MotorDriver.h"

namespace TestHardware
{
void reset();
void setMillis(unsigned long value);
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

struct HomingFixture
{
    Encoder encoderA;
    Encoder encoderB;

    MotorDriver motorA;
    MotorDriver motorB;

    Converter converter;

    LimitSwitch xMinSwitch;
    LimitSwitch xMaxSwitch;
    LimitSwitch yMinSwitch;
    LimitSwitch yMaxSwitch;

    HomingConfig config;
    HomingController controller;

    HomingFixture()
        : encoderA(true),
          encoderB(false),
          motorA(1, 2),
          motorB(3, 4),
          converter(1.0f, 1.0f),
          xMinSwitch(X_MIN_PIN, INPUT, 1UL),
          xMaxSwitch(X_MAX_PIN, INPUT, 1UL),
          yMinSwitch(Y_MIN_PIN, INPUT, 1UL),
          yMaxSwitch(Y_MAX_PIN, INPUT, 1UL),
          config{
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
          controller(
              encoderA,
              encoderB,
              motorA,
              motorB,
              converter,
              xMinSwitch,
              xMaxSwitch,
              yMinSwitch,
              yMaxSwitch,
              config)
    {
    }

    void beginSwitchesAndController()
    {
        xMinSwitch.begin();
        xMaxSwitch.begin();
        yMinSwitch.begin();
        yMaxSwitch.begin();

        motorA.begin();
        motorB.begin();
        controller.begin();
    }
};

void updateAfter(
    HomingFixture& fixture,
    unsigned long milliseconds = 1UL)
{
    TestHardware::advanceMillis(milliseconds);
    fixture.controller.update();
}

void setDebouncedSwitch(
    HomingFixture& fixture,
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

    // First observation starts the 1 ms debounce interval.
    fixture.controller.update();

    // Second observation accepts the stable active-high level.
    updateAfter(fixture, 1UL);
}

void finishContactAndEnterBackoff(
    HomingFixture& fixture)
{
    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(HomingPhase::CONTACT_PAUSE),
        static_cast<int>(fixture.controller.phase()));

    TEST_ASSERT_EQUAL_INT16(
        0,
        fixture.motorA.getOutput());

    TEST_ASSERT_EQUAL_INT16(
        0,
        fixture.motorB.getOutput());

    updateAfter(
        fixture,
        fixture.config.contactPauseMs);

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(HomingPhase::BACKOFF),
        static_cast<int>(fixture.controller.phase()));
}

void finishFineContactAtRelease(
    HomingFixture& fixture,
    uint8_t pin,
    int32_t releaseCountA,
    int32_t releaseCountB,
    int16_t expectedMotorA,
    int16_t expectedMotorB)
{
    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(HomingPhase::FINE_CONTACT_PAUSE),
        static_cast<int>(fixture.controller.phase()));

    TEST_ASSERT_EQUAL_INT16(0, fixture.motorA.getOutput());
    TEST_ASSERT_EQUAL_INT16(0, fixture.motorB.getOutput());

    updateAfter(
        fixture,
        fixture.config.fineContactPauseMs);

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(HomingPhase::FINAL_RELEASE),
        static_cast<int>(fixture.controller.phase()));

    updateAfter(fixture);

    TEST_ASSERT_EQUAL_INT16(
        expectedMotorA,
        fixture.motorA.getOutput());

    TEST_ASSERT_EQUAL_INT16(
        expectedMotorB,
        fixture.motorB.getOutput());

    setDebouncedSwitch(
        fixture,
        pin,
        false,
        releaseCountA,
        releaseCountB);

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(HomingPhase::RECORD_POSITION),
        static_cast<int>(fixture.controller.phase()));

    TEST_ASSERT_EQUAL_INT16(0, fixture.motorA.getOutput());
    TEST_ASSERT_EQUAL_INT16(0, fixture.motorB.getOutput());

    // The synchronized falling-edge snapshot is stored in the result and
    // the next target begins on the following non-blocking update.
    updateAfter(fixture);
}
}

void setUp()
{
    TestHardware::reset();
}

void tearDown()
{
}

void testCompleteSequenceUsesCorrectDirectionsAndZerosBothEncoders()
{
    HomingFixture fixture;
    fixture.beginSwitchesAndController();

    TEST_ASSERT_TRUE(
        fixture.controller.start());

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(HomingStage::X_MAX),
        static_cast<int>(fixture.controller.stage()));

    // X max: positive X maps to positive A and positive B.
    updateAfter(fixture);

    TEST_ASSERT_EQUAL_INT16(60, fixture.motorA.getOutput());
    TEST_ASSERT_EQUAL_INT16(60, fixture.motorB.getOutput());

    setDebouncedSwitch(
        fixture,
        X_MAX_PIN,
        true,
        100,
        100);

    finishContactAndEnterBackoff(fixture);
    updateAfter(fixture);

    TEST_ASSERT_EQUAL_INT16(-40, fixture.motorA.getOutput());
    TEST_ASSERT_EQUAL_INT16(-40, fixture.motorB.getOutput());

    // Releasing the switch alone is insufficient; the measured backoff
    // distance must also reach 2 mm.
    setDebouncedSwitch(
        fixture,
        X_MAX_PIN,
        false,
        99,
        99);

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(HomingPhase::BACKOFF),
        static_cast<int>(fixture.controller.phase()));

    TestHardware::setEncoderCounts(98, 98);
    updateAfter(fixture);

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(HomingPhase::FINE_APPROACH),
        static_cast<int>(fixture.controller.phase()));

    updateAfter(fixture);

    TEST_ASSERT_EQUAL_INT16(20, fixture.motorA.getOutput());
    TEST_ASSERT_EQUAL_INT16(20, fixture.motorB.getOutput());

    setDebouncedSwitch(
        fixture,
        X_MAX_PIN,
        true,
        99,
        99);

    finishFineContactAtRelease(
        fixture,
        X_MAX_PIN,
        98,
        98,
        -10,
        -10);

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(HomingStage::X_ORIGIN),
        static_cast<int>(fixture.controller.stage()));

    updateAfter(fixture);

    TEST_ASSERT_EQUAL_INT16(-60, fixture.motorA.getOutput());
    TEST_ASSERT_EQUAL_INT16(-60, fixture.motorB.getOutput());

    setDebouncedSwitch(
        fixture,
        X_MIN_PIN,
        true,
        0,
        0);

    finishContactAndEnterBackoff(fixture);
    updateAfter(fixture);

    TEST_ASSERT_EQUAL_INT16(40, fixture.motorA.getOutput());
    TEST_ASSERT_EQUAL_INT16(40, fixture.motorB.getOutput());

    setDebouncedSwitch(
        fixture,
        X_MIN_PIN,
        false,
        2,
        2);

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(HomingPhase::FINE_APPROACH),
        static_cast<int>(fixture.controller.phase()));

    updateAfter(fixture);

    TEST_ASSERT_EQUAL_INT16(-20, fixture.motorA.getOutput());
    TEST_ASSERT_EQUAL_INT16(-20, fixture.motorB.getOutput());

    setDebouncedSwitch(
        fixture,
        X_MIN_PIN,
        true,
        1,
        1);

    finishFineContactAtRelease(
        fixture,
        X_MIN_PIN,
        2,
        2,
        10,
        10);

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(HomingStage::Y_MAX),
        static_cast<int>(fixture.controller.stage()));

    // Positive Y maps to positive A and negative B.
    updateAfter(fixture);

    TEST_ASSERT_EQUAL_INT16(60, fixture.motorA.getOutput());
    TEST_ASSERT_EQUAL_INT16(-60, fixture.motorB.getOutput());

    setDebouncedSwitch(
        fixture,
        Y_MAX_PIN,
        true,
        101,
        -99);

    finishContactAndEnterBackoff(fixture);
    updateAfter(fixture);

    TEST_ASSERT_EQUAL_INT16(-40, fixture.motorA.getOutput());
    TEST_ASSERT_EQUAL_INT16(40, fixture.motorB.getOutput());

    setDebouncedSwitch(
        fixture,
        Y_MAX_PIN,
        false,
        99,
        -97);

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(HomingPhase::FINE_APPROACH),
        static_cast<int>(fixture.controller.phase()));

    updateAfter(fixture);

    TEST_ASSERT_EQUAL_INT16(20, fixture.motorA.getOutput());
    TEST_ASSERT_EQUAL_INT16(-20, fixture.motorB.getOutput());

    setDebouncedSwitch(
        fixture,
        Y_MAX_PIN,
        true,
        100,
        -98);

    finishFineContactAtRelease(
        fixture,
        Y_MAX_PIN,
        99,
        -97,
        -10,
        10);

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(HomingStage::Y_ORIGIN),
        static_cast<int>(fixture.controller.stage()));

    updateAfter(fixture);

    TEST_ASSERT_EQUAL_INT16(-60, fixture.motorA.getOutput());
    TEST_ASSERT_EQUAL_INT16(60, fixture.motorB.getOutput());

    setDebouncedSwitch(
        fixture,
        Y_MIN_PIN,
        true,
        1,
        1);

    finishContactAndEnterBackoff(fixture);
    updateAfter(fixture);

    TEST_ASSERT_EQUAL_INT16(40, fixture.motorA.getOutput());
    TEST_ASSERT_EQUAL_INT16(-40, fixture.motorB.getOutput());

    setDebouncedSwitch(
        fixture,
        Y_MIN_PIN,
        false,
        3,
        -1);

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(HomingPhase::FINE_APPROACH),
        static_cast<int>(fixture.controller.phase()));

    updateAfter(fixture);

    TEST_ASSERT_EQUAL_INT16(-20, fixture.motorA.getOutput());
    TEST_ASSERT_EQUAL_INT16(20, fixture.motorB.getOutput());

    setDebouncedSwitch(
        fixture,
        Y_MIN_PIN,
        true,
        2,
        0);

    finishFineContactAtRelease(
        fixture,
        Y_MIN_PIN,
        3,
        -1,
        10,
        -10);

    TEST_ASSERT_TRUE(fixture.controller.isComplete());
    TEST_ASSERT_FALSE(fixture.controller.isActive());
    TEST_ASSERT_FALSE(fixture.controller.hasFault());

    TEST_ASSERT_EQUAL_INT16(0, fixture.motorA.getOutput());
    TEST_ASSERT_EQUAL_INT16(0, fixture.motorB.getOutput());

    TEST_ASSERT_EQUAL_INT32(0, fixture.encoderA.getCount());
    TEST_ASSERT_EQUAL_INT32(0, fixture.encoderB.getCount());

    const HomingResult result =
        fixture.controller.result();

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

void testFineContactSnapshotComesFromReleasedEdge()
{
    HomingFixture fixture;
    fixture.beginSwitchesAndController();

    TEST_ASSERT_TRUE(fixture.controller.start());

    setDebouncedSwitch(
        fixture,
        X_MAX_PIN,
        true,
        100,
        100);

    finishContactAndEnterBackoff(fixture);

    setDebouncedSwitch(
        fixture,
        X_MAX_PIN,
        false,
        98,
        98);

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(HomingPhase::FINE_APPROACH),
        static_cast<int>(fixture.controller.phase()));

    setDebouncedSwitch(
        fixture,
        X_MAX_PIN,
        true,
        99,
        99);

    // Pressing during the fine approach only begins the pause. It must not
    // become the recorded endpoint.
    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(HomingPhase::FINE_CONTACT_PAUSE),
        static_cast<int>(fixture.controller.phase()));

    TEST_ASSERT_EQUAL_INT32(
        0,
        fixture.controller.result().xMaximum.countA);

    updateAfter(
        fixture,
        fixture.config.fineContactPauseMs);

    updateAfter(fixture);

    TEST_ASSERT_EQUAL_INT16(-10, fixture.motorA.getOutput());
    TEST_ASSERT_EQUAL_INT16(-10, fixture.motorB.getOutput());

    setDebouncedSwitch(
        fixture,
        X_MAX_PIN,
        false,
        98,
        98);

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(HomingPhase::RECORD_POSITION),
        static_cast<int>(fixture.controller.phase()));

    updateAfter(fixture);

    const HomingResult result = fixture.controller.result();

    TEST_ASSERT_EQUAL_INT32(98, result.xMaximum.countA);
    TEST_ASSERT_EQUAL_INT32(98, result.xMaximum.countB);
}

void testFinalReleaseTimeoutAbortsIfSwitchStaysPressed()
{
    HomingFixture fixture;
    fixture.beginSwitchesAndController();

    TEST_ASSERT_TRUE(fixture.controller.start());

    setDebouncedSwitch(
        fixture,
        X_MAX_PIN,
        true,
        100,
        100);

    finishContactAndEnterBackoff(fixture);

    setDebouncedSwitch(
        fixture,
        X_MAX_PIN,
        false,
        98,
        98);

    setDebouncedSwitch(
        fixture,
        X_MAX_PIN,
        true,
        99,
        99);

    updateAfter(
        fixture,
        fixture.config.fineContactPauseMs);

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(HomingPhase::FINAL_RELEASE),
        static_cast<int>(fixture.controller.phase()));

    updateAfter(
        fixture,
        fixture.config.finalReleaseTimeoutMs);

    TEST_ASSERT_TRUE(fixture.controller.hasFault());

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(HomingFault::TIMEOUT),
        static_cast<int>(fixture.controller.fault()));

    TEST_ASSERT_EQUAL_INT16(0, fixture.motorA.getOutput());
    TEST_ASSERT_EQUAL_INT16(0, fixture.motorB.getOutput());
}

void testUnexpectedLimitAbortsAndStopsBothMotors()
{
    HomingFixture fixture;
    fixture.beginSwitchesAndController();

    TEST_ASSERT_TRUE(fixture.controller.start());
    updateAfter(fixture);

    TEST_ASSERT_NOT_EQUAL(0, fixture.motorA.getOutput());
    TEST_ASSERT_NOT_EQUAL(0, fixture.motorB.getOutput());

    // X_MIN is wrong while the controller is searching for X_MAX.
    setDebouncedSwitch(
        fixture,
        X_MIN_PIN,
        true,
        -10,
        -10);

    TEST_ASSERT_TRUE(fixture.controller.hasFault());

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(HomingFault::WRONG_LIMIT),
        static_cast<int>(fixture.controller.fault()));

    TEST_ASSERT_EQUAL_INT16(0, fixture.motorA.getOutput());
    TEST_ASSERT_EQUAL_INT16(0, fixture.motorB.getOutput());
}

void testContradictoryLimitsPreventHomingStart()
{
    TestHardware::setSwitchPressed(X_MIN_PIN, true);
    TestHardware::setSwitchPressed(X_MAX_PIN, true);

    HomingFixture fixture;
    fixture.beginSwitchesAndController();

    TEST_ASSERT_FALSE(fixture.controller.start());

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(HomingFault::CONTRADICTORY_LIMITS),
        static_cast<int>(fixture.controller.fault()));

    TEST_ASSERT_FALSE(fixture.controller.isActive());
    TEST_ASSERT_EQUAL_INT16(0, fixture.motorA.getOutput());
    TEST_ASSERT_EQUAL_INT16(0, fixture.motorB.getOutput());
}

void testSearchTimeoutAbortsAndStopsBothMotors()
{
    HomingFixture fixture;
    fixture.beginSwitchesAndController();

    TEST_ASSERT_TRUE(fixture.controller.start());

    updateAfter(
        fixture,
        fixture.config.searchTimeoutMs);

    TEST_ASSERT_TRUE(fixture.controller.hasFault());

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(HomingFault::TIMEOUT),
        static_cast<int>(fixture.controller.fault()));

    TEST_ASSERT_EQUAL_INT16(0, fixture.motorA.getOutput());
    TEST_ASSERT_EQUAL_INT16(0, fixture.motorB.getOutput());
}

void testInvalidConfigurationPreventsHomingStart()
{
    HomingFixture fixture;
    fixture.config.backoffDistanceMm = 0.0f;

    // HomingController stores its own config copy, so create the controller
    // under test after changing the invalid value.
    HomingController invalidController(
        fixture.encoderA,
        fixture.encoderB,
        fixture.motorA,
        fixture.motorB,
        fixture.converter,
        fixture.xMinSwitch,
        fixture.xMaxSwitch,
        fixture.yMinSwitch,
        fixture.yMaxSwitch,
        fixture.config);

    fixture.xMinSwitch.begin();
    fixture.xMaxSwitch.begin();
    fixture.yMinSwitch.begin();
    fixture.yMaxSwitch.begin();

    invalidController.begin();

    TEST_ASSERT_FALSE(invalidController.start());

    TEST_ASSERT_EQUAL_INT(
        static_cast<int>(HomingFault::INVALID_CONFIGURATION),
        static_cast<int>(invalidController.fault()));
}

int main(int argc, char** argv)
{
    (void)argc;
    (void)argv;

    UNITY_BEGIN();

    RUN_TEST(
        testCompleteSequenceUsesCorrectDirectionsAndZerosBothEncoders);

    RUN_TEST(
        testFineContactSnapshotComesFromReleasedEdge);

    RUN_TEST(
        testFinalReleaseTimeoutAbortsIfSwitchStaysPressed);

    RUN_TEST(
        testUnexpectedLimitAbortsAndStopsBothMotors);

    RUN_TEST(
        testContradictoryLimitsPreventHomingStart);

    RUN_TEST(
        testSearchTimeoutAbortsAndStopsBothMotors);

    RUN_TEST(
        testInvalidConfigurationPreventsHomingStart);

    return UNITY_END();
}