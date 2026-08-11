#include <Arduino.h>
#include <avr/interrupt.h>
#include <stdint.h>
#include <string.h>

#include "config/PinConfig.h"
#include "config/SystemConfig.h"
#include "control/AxisController.h"
#include "control/Converter.h"
#include "control/HomingController.h"
#include "control/PIDController.h"
#include "control/XYCoordinator.h"
#include "hardware/Encoder.h"
#include "hardware/LimitSwitch.h"
#include "hardware/MotorDriver.h"
#include "system/PlotterFSM.h"
#include "system/PlotterSystem.h"
#include "system/TrajectoryPlanner.h"

namespace
{
// -----------------------------------------------------------------------------
// High-power hardware bring-up settings
// -----------------------------------------------------------------------------

// Closed-loop controller values for initial physical testing.
// Final values still require proper tuning on the real mechanism.
constexpr float INITIAL_PROPORTIONAL_GAIN = 5.0f;
constexpr float INITIAL_INTEGRAL_GAIN = 0.0f;
constexpr float INITIAL_VELOCITY_FEEDFORWARD_GAIN = 0.0f;

// Allow the controller to use the full Arduino PWM range.
constexpr float CONTROLLER_MINIMUM_OUTPUT = -255.0f;
constexpr float CONTROLLER_MAXIMUM_OUTPUT = 255.0f;

// Integral is currently disabled because Ki = 0,
// but the limits are also opened for later tuning.
constexpr float CONTROLLER_MINIMUM_INTEGRAL_OUTPUT = -255.0f;
constexpr float CONTROLLER_MAXIMUM_INTEGRAL_OUTPUT = 255.0f;

// Approximately 0.21 mm with the current 14 mm pulley/count conversion.
constexpr float INITIAL_POSITION_TOLERANCE_COUNTS = 20.0f;

// Full MotorDriver output range.
constexpr uint8_t BRINGUP_MOTOR_OUTPUT_LIMIT = 255;

// Full-power manual motor pulse.
constexpr uint8_t MANUAL_TEST_PWM = 255;

// Long enough to make physical motion obvious.
constexpr unsigned long MANUAL_TEST_DURATION_MS = 1000UL;

constexpr size_t COMMAND_BUFFER_SIZE = 20U;

// Closed-loop move-test settings.
// These control trajectory speed/acceleration, not the PWM limit.
constexpr float MOVE_TEST_FEEDRATE_MM_PER_MINUTE = 120.0f;
constexpr float MOVE_TEST_ACCELERATION_MM_PER_SECOND_SQUARED = 10.0f;
constexpr float MOVE_TEST_POSITION_EPSILON_MM = 0.05f;
constexpr float MOVE_TEST_MINIMUM_TRAVEL_MM = 5.0f;
constexpr unsigned long MOVE_TELEMETRY_INTERVAL_MS = 500UL;

struct FractionalTestPoint
{
    float xFraction;
    float yFraction;
    const char* name;
};

const FractionalTestPoint TEST_POINT_1 = {0.20f, 0.20f, "P1"};
const FractionalTestPoint TEST_POINT_2 = {0.80f, 0.20f, "P2"};
const FractionalTestPoint TEST_POINT_3 = {0.80f, 0.80f, "P3"};
const FractionalTestPoint TEST_POINT_4 = {0.20f, 0.80f, "P4"};
const FractionalTestPoint TEST_POINT_CENTRE = {0.50f, 0.50f, "CENTER"};

const FractionalTestPoint MOVE_TEST_ROUTE[] = {
    TEST_POINT_1,
    TEST_POINT_2,
    TEST_POINT_3,
    TEST_POINT_4,
    TEST_POINT_1,
    TEST_POINT_3,
    TEST_POINT_4,
    TEST_POINT_2,
    TEST_POINT_CENTRE
};

constexpr size_t MOVE_TEST_ROUTE_POINT_COUNT =
    sizeof(MOVE_TEST_ROUTE) / sizeof(MOVE_TEST_ROUTE[0]);

// -----------------------------------------------------------------------------
// Hardware and control objects - declared in dependency order
// -----------------------------------------------------------------------------

Encoder encoderA(true);
Encoder encoderB(false);

LimitSwitch leftLimitSwitch(
    PinConfig::LIMIT_SWITCH_LEFT_PIN,
    SystemConfig::LIMIT_SWITCH_INPUT_MODE,
    SystemConfig::LIMIT_SWITCH_DEBOUNCE_MS);

LimitSwitch rightLimitSwitch(
    PinConfig::LIMIT_SWITCH_RIGHT_PIN,
    SystemConfig::LIMIT_SWITCH_INPUT_MODE,
    SystemConfig::LIMIT_SWITCH_DEBOUNCE_MS);

LimitSwitch bottomLimitSwitch(
    PinConfig::LIMIT_SWITCH_BOTTOM_PIN,
    SystemConfig::LIMIT_SWITCH_INPUT_MODE,
    SystemConfig::LIMIT_SWITCH_DEBOUNCE_MS);

LimitSwitch topLimitSwitch(
    PinConfig::LIMIT_SWITCH_TOP_PIN,
    SystemConfig::LIMIT_SWITCH_INPUT_MODE,
    SystemConfig::LIMIT_SWITCH_DEBOUNCE_MS);

MotorDriver motorA(
    PinConfig::MOTOR_1_DIRECTION_PIN,
    PinConfig::MOTOR_1_PWM_PIN,
    SystemConfig::MOTOR_1_DIRECTION_INVERTED,
    BRINGUP_MOTOR_OUTPUT_LIMIT);

MotorDriver motorB(
    PinConfig::MOTOR_2_DIRECTION_PIN,
    PinConfig::MOTOR_2_PWM_PIN,
    SystemConfig::MOTOR_2_DIRECTION_INVERTED,
    BRINGUP_MOTOR_OUTPUT_LIMIT);

PIDController pidA(
    INITIAL_PROPORTIONAL_GAIN,
    INITIAL_INTEGRAL_GAIN,
    CONTROLLER_MINIMUM_OUTPUT,
    CONTROLLER_MAXIMUM_OUTPUT,
    CONTROLLER_MINIMUM_INTEGRAL_OUTPUT,
    CONTROLLER_MAXIMUM_INTEGRAL_OUTPUT,
    INITIAL_VELOCITY_FEEDFORWARD_GAIN);

PIDController pidB(
    INITIAL_PROPORTIONAL_GAIN,
    INITIAL_INTEGRAL_GAIN,
    CONTROLLER_MINIMUM_OUTPUT,
    CONTROLLER_MAXIMUM_OUTPUT,
    CONTROLLER_MINIMUM_INTEGRAL_OUTPUT,
    CONTROLLER_MAXIMUM_INTEGRAL_OUTPUT,
    INITIAL_VELOCITY_FEEDFORWARD_GAIN);

AxisController axisA(
    pidA,
    motorA,
    INITIAL_POSITION_TOLERANCE_COUNTS);

AxisController axisB(
    pidB,
    motorB,
    INITIAL_POSITION_TOLERANCE_COUNTS);

Converter converter(
    SystemConfig::MOTOR_A_MM_PER_COUNT,
    SystemConfig::MOTOR_B_MM_PER_COUNT,
    SystemConfig::MOTOR_A_COORDINATE_SIGN,
    SystemConfig::MOTOR_B_COORDINATE_SIGN);

XYCoordinator xyCoordinator(
    encoderA,
    encoderB,
    axisA,
    axisB,
    converter,
    SystemConfig::MOTION_CONTROL_PERIOD_MICROS);

const HomingConfig homingConfig = {
    SystemConfig::HOMING_COARSE_APPROACH_PWM,
    SystemConfig::HOMING_BACKOFF_PWM,
    SystemConfig::HOMING_FINE_APPROACH_PWM,
    SystemConfig::HOMING_FINAL_RELEASE_PWM,
    SystemConfig::HOMING_BACKOFF_DISTANCE_MM,
    SystemConfig::HOMING_CONTACT_PAUSE_MS,
    SystemConfig::HOMING_FINE_CONTACT_PAUSE_MS,
    SystemConfig::HOMING_SEARCH_TIMEOUT_MS,
    SystemConfig::HOMING_BACKOFF_TIMEOUT_MS,
    SystemConfig::HOMING_FINAL_RELEASE_TIMEOUT_MS,
    SystemConfig::HOMING_OVERALL_TIMEOUT_MS
};

HomingController homingController(
    encoderA,
    encoderB,
    motorA,
    motorB,
    converter,
    leftLimitSwitch,
    rightLimitSwitch,
    bottomLimitSwitch,
    topLimitSwitch,
    homingConfig);

plotter::TrajectoryPlanner trajectoryPlanner;

plotter::PlotterSystem plotterSystem(
    axisA,
    axisB,
    xyCoordinator,
    trajectoryPlanner,
    homingController);

// -----------------------------------------------------------------------------
// Bring-up console state
// -----------------------------------------------------------------------------

char commandBuffer[COMMAND_BUFFER_SIZE] = {};
size_t commandLength = 0U;

bool systemStarted = false;

bool manualPulseActive = false;
unsigned long manualPulseStartMs = 0UL;

// If manual open-loop motion is performed after homing,
// the previous machine coordinate reference is no longer trustworthy.
bool manualMotionSinceHoming = false;

bool moveTestRouteActive = false;
size_t nextMoveTestRoutePoint = 0U;

bool activeMoveTargetValid = false;
float activeMoveTargetXMm = 0.0f;
float activeMoveTargetYMm = 0.0f;

unsigned long lastMoveTelemetryMs = 0UL;

bool lastLeftPressed = false;
bool lastRightPressed = false;
bool lastBottomPressed = false;
bool lastTopPressed = false;

plotter::PlotterState lastPlotterState = plotter::PlotterState::IDLE;
HomingStage lastHomingStage = HomingStage::IDLE;
HomingPhase lastHomingPhase = HomingPhase::IDLE;

// -----------------------------------------------------------------------------
// Limit-switch external-interrupt callbacks
// -----------------------------------------------------------------------------

void onLeftLimitSwitchInterrupt()
{
    leftLimitSwitch.notifyFromISR();
}

void onRightLimitSwitchInterrupt()
{
    rightLimitSwitch.notifyFromISR();
}

void onBottomLimitSwitchInterrupt()
{
    bottomLimitSwitch.notifyFromISR();
}

void onTopLimitSwitchInterrupt()
{
    topLimitSwitch.notifyFromISR();
}

// -----------------------------------------------------------------------------
// Print helpers
// -----------------------------------------------------------------------------

void printPressedState(bool pressed)
{
    Serial.print(pressed ? F("PRESSED") : F("released"));
}

void printPlotterState(plotter::PlotterState state)
{
    switch (state)
    {
        case plotter::PlotterState::IDLE:
            Serial.print(F("IDLE"));
            break;

        case plotter::PlotterState::HOMING:
            Serial.print(F("HOMING"));
            break;

        case plotter::PlotterState::MOVING:
            Serial.print(F("MOVING"));
            break;

        case plotter::PlotterState::FAULT:
            Serial.print(F("FAULT"));
            break;
    }
}

void printHomingStage(HomingStage stage)
{
    switch (stage)
    {
        case HomingStage::IDLE:
            Serial.print(F("IDLE"));
            break;

        case HomingStage::X_MAX:
            Serial.print(F("X_MAX"));
            break;

        case HomingStage::X_ORIGIN:
            Serial.print(F("X_ORIGIN"));
            break;

        case HomingStage::Y_MAX:
            Serial.print(F("Y_MAX"));
            break;

        case HomingStage::Y_ORIGIN:
            Serial.print(F("Y_ORIGIN"));
            break;

        case HomingStage::COMPLETE:
            Serial.print(F("COMPLETE"));
            break;

        case HomingStage::ABORTED:
            Serial.print(F("ABORTED"));
            break;
    }
}

void printHomingPhase(HomingPhase phase)
{
    switch (phase)
    {
        case HomingPhase::IDLE:
            Serial.print(F("IDLE"));
            break;

        case HomingPhase::COARSE_APPROACH:
            Serial.print(F("COARSE_APPROACH"));
            break;

        case HomingPhase::CONTACT_PAUSE:
            Serial.print(F("CONTACT_PAUSE"));
            break;

        case HomingPhase::BACKOFF:
            Serial.print(F("BACKOFF"));
            break;

        case HomingPhase::FINE_APPROACH:
            Serial.print(F("FINE_APPROACH"));
            break;

        case HomingPhase::FINE_CONTACT_PAUSE:
            Serial.print(F("FINE_CONTACT_PAUSE"));
            break;

        case HomingPhase::FINAL_RELEASE:
            Serial.print(F("FINAL_RELEASE"));
            break;

        case HomingPhase::RECORD_POSITION:
            Serial.print(F("RECORD_POSITION"));
            break;

        case HomingPhase::COMPLETE:
            Serial.print(F("COMPLETE"));
            break;

        case HomingPhase::ABORTED:
            Serial.print(F("ABORTED"));
            break;
    }
}

void printFaultCode(plotter::FaultCode fault)
{
    switch (fault)
    {
        case plotter::FaultCode::NONE:
            Serial.print(F("NONE"));
            break;

        case plotter::FaultCode::UNEXPECTED_LIMIT:
            Serial.print(F("UNEXPECTED_LIMIT"));
            break;

        case plotter::FaultCode::WRONG_HOMING_LIMIT:
            Serial.print(F("WRONG_HOMING_LIMIT"));
            break;

        case plotter::FaultCode::CONTRADICTORY_LIMITS:
            Serial.print(F("CONTRADICTORY_LIMITS"));
            break;

        case plotter::FaultCode::HOMING_TIMEOUT:
            Serial.print(F("HOMING_TIMEOUT"));
            break;

        case plotter::FaultCode::MOVE_TIMEOUT:
            Serial.print(F("MOVE_TIMEOUT"));
            break;

        case plotter::FaultCode::ENCODER_NO_MOTION:
            Serial.print(F("ENCODER_NO_MOTION"));
            break;

        case plotter::FaultCode::POSITION_OUT_OF_RANGE:
            Serial.print(F("POSITION_OUT_OF_RANGE"));
            break;

        case plotter::FaultCode::INTERNAL_ERROR:
            Serial.print(F("INTERNAL_ERROR"));
            break;
    }
}

void printRejectReason(plotter::RejectReason reason)
{
    switch (reason)
    {
        case plotter::RejectReason::NONE:
            Serial.print(F("NONE"));
            break;

        case plotter::RejectReason::BUSY:
            Serial.print(F("BUSY"));
            break;

        case plotter::RejectReason::MACHINE_ZERO_UNKNOWN:
            Serial.print(F("MACHINE_ZERO_UNKNOWN"));
            break;

        case plotter::RejectReason::FAULT_ACTIVE:
            Serial.print(F("FAULT_ACTIVE"));
            break;

        case plotter::RejectReason::UNEXPECTED_EVENT:
            Serial.print(F("UNEXPECTED_EVENT"));
            break;

        case plotter::RejectReason::INVALID_MOTION_PARAMETERS:
            Serial.print(F("INVALID_MOTION_PARAMETERS"));
            break;
    }
}

void printEncoderCounts()
{
    const Encoder::CountPair counts =
        Encoder::getCountPair(encoderA, encoderB);

    Serial.print(F("A="));
    Serial.print(counts.countA);

    Serial.print(F(" B="));
    Serial.print(counts.countB);
}

void printSwitchStates()
{
    Serial.print(F("LEFT="));
    printPressedState(leftLimitSwitch.isPressed());

    Serial.print(F(" RIGHT="));
    printPressedState(rightLimitSwitch.isPressed());

    Serial.print(F(" BOTTOM="));
    printPressedState(bottomLimitSwitch.isPressed());

    Serial.print(F(" TOP="));
    printPressedState(topLimitSwitch.isPressed());
}

void printStatus()
{
    Serial.print(F("STATUS state="));

    if (systemStarted)
    {
        printPlotterState(plotterSystem.state());
    }
    else
    {
        Serial.print(F("BRINGUP"));
    }

    Serial.print(F(" | "));
    printSwitchStates();

    Serial.print(F(" | "));
    printEncoderCounts();

    Serial.print(F(" | motorA="));
    Serial.print(motorA.getOutput());

    Serial.print(F(" motorB="));
    Serial.print(motorB.getOutput());

    if (manualMotionSinceHoming)
    {
        Serial.print(F(" | REHOME_REQUIRED"));
    }

    Serial.println();
}

void printHelp()
{
    Serial.println(F("Commands (send with CR or LF):"));
    Serial.println(F("  STATUS  - switches, encoders, motors and FSM"));
    Serial.println(F("  A+ A-   - 1000 ms FULL-PWM Motor A pulse"));
    Serial.println(F("  B+ B-   - 1000 ms FULL-PWM Motor B pulse"));
    Serial.println(F("  X+ X-   - 1000 ms FULL-PWM Cartesian X pulse"));
    Serial.println(F("  Y+ Y-   - 1000 ms FULL-PWM Cartesian Y pulse"));
    Serial.println(F("  HOME/G28- initialise the system and start homing"));
    Serial.println(F("  P1..P4  - move to an interior test point"));
    Serial.println(F("  CENTER  - move to the measured travel centre"));
    Serial.println(F("  ROUTE   - run horizontal, vertical and diagonal lines"));
    Serial.println(F("  STOP    - stop and enter FAULT if system is active"));
    Serial.println(F("  RESET   - clear fault after all limits are released"));
    Serial.println(F("Manual pulses are allowed before HOME or while system is IDLE."));
    Serial.println(F("Manual motion after HOME requires HOME again before closed-loop moves."));
}

// -----------------------------------------------------------------------------
// Switch and manual-pulse handling
// -----------------------------------------------------------------------------

void updateAllLimitSwitches()
{
    leftLimitSwitch.update();
    rightLimitSwitch.update();
    bottomLimitSwitch.update();
    topLimitSwitch.update();
}

bool anyLimitSwitchPressed()
{
    return leftLimitSwitch.isPressed() ||
           rightLimitSwitch.isPressed() ||
           bottomLimitSwitch.isPressed() ||
           topLimitSwitch.isPressed();
}

bool contradictoryLimitSwitchesPressed()
{
    return
        (leftLimitSwitch.isPressed() && rightLimitSwitch.isPressed()) ||
        (bottomLimitSwitch.isPressed() && topLimitSwitch.isPressed());
}

void stopManualPulse()
{
    motorA.stop();
    motorB.stop();

    manualPulseActive = false;
}

void startManualPulse(
    int16_t motorACommand,
    int16_t motorBCommand)
{
    // Manual open-loop motion is permitted before HOME,
    // or after system startup only while the FSM is IDLE.
    if (systemStarted &&
        plotterSystem.state() != plotter::PlotterState::IDLE)
    {
        Serial.println(
            F("REJECTED: manual pulse requires IDLE state."));
        return;
    }

    // NOTE:
    // Manual bring-up commands intentionally ignore limit switch states.
    // This is for hardware direction/power testing only.

    stopManualPulse();

    motorA.setOutput(motorACommand);
    motorB.setOutput(motorBCommand);

    manualPulseStartMs = millis();
    manualPulseActive = true;

    if (systemStarted &&
        plotterSystem.machineZeroKnown())
    {
        manualMotionSinceHoming = true;
    }

    Serial.print(F("PULSE motorA="));
    Serial.print(motorA.getOutput());

    Serial.print(F(" motorB="));
    Serial.println(motorB.getOutput());

    if (manualMotionSinceHoming)
    {
        Serial.println(
            F("WARNING: manual motion performed after HOME; run HOME again before closed-loop moves."));
    }
}

int16_t commandForVelocitySign(float velocity)
{
    if (velocity > 0.0f)
    {
        return static_cast<int16_t>(MANUAL_TEST_PWM);
    }

    if (velocity < 0.0f)
    {
        return -static_cast<int16_t>(MANUAL_TEST_PWM);
    }

    return 0;
}

void startCartesianPulse(
    int8_t xDirection,
    int8_t yDirection)
{
    const Converter::MotorReference reference =
        converter.cartesianToMotorReference(
            0.0f,
            0.0f,
            static_cast<float>(xDirection),
            static_cast<float>(yDirection));

    startManualPulse(
        commandForVelocitySign(
            reference.aVelocityCountsPerSecond),
        commandForVelocitySign(
            reference.bVelocityCountsPerSecond));
}

void updateManualPulse()
{
    if (!manualPulseActive)
    {
        return;
    }

    // NOTE:
    // Manual bring-up pulses intentionally ignore limit switches.
    // The pulse ends only when MANUAL_TEST_DURATION_MS expires.

    if ((millis() - manualPulseStartMs) >=
        MANUAL_TEST_DURATION_MS)
    {
        stopManualPulse();

        Serial.print(F("PULSE COMPLETE: "));
        printEncoderCounts();
        Serial.println();
    }
}

void reportSwitchChange(
    const __FlashStringHelper* name,
    bool currentPressed,
    bool& previousPressed)
{
    if (currentPressed == previousPressed)
    {
        return;
    }

    previousPressed = currentPressed;

    Serial.print(F("SWITCH "));
    Serial.print(name);
    Serial.print(F(" -> "));
    printPressedState(currentPressed);
    Serial.println();
}

void monitorSwitchChanges()
{
    reportSwitchChange(
        F("LEFT"),
        leftLimitSwitch.isPressed(),
        lastLeftPressed);

    reportSwitchChange(
        F("RIGHT"),
        rightLimitSwitch.isPressed(),
        lastRightPressed);

    reportSwitchChange(
        F("BOTTOM"),
        bottomLimitSwitch.isPressed(),
        lastBottomPressed);

    reportSwitchChange(
        F("TOP"),
        topLimitSwitch.isPressed(),
        lastTopPressed);
}

// -----------------------------------------------------------------------------
// Closed-loop Cartesian move tests
// -----------------------------------------------------------------------------

enum class MoveTestRequestResult : uint8_t
{
    STARTED,
    ALREADY_AT_TARGET,
    REJECTED
};

float absoluteValue(float value)
{
    return value < 0.0f ? -value : value;
}

Converter::CartesianDisplacement currentCartesianPosition()
{
    const Encoder::CountPair counts =
        Encoder::getCountPair(
            encoderA,
            encoderB);

    return converter.motorToCartesianDisplacement(
        static_cast<float>(counts.countA),
        static_cast<float>(counts.countB));
}

bool moveTestIsReady()
{
    if (!systemStarted)
    {
        Serial.println(F("REJECTED: run HOME first."));
        return false;
    }

    if (plotterSystem.state() !=
        plotter::PlotterState::IDLE)
    {
        Serial.println(
            F("REJECTED: system is not IDLE."));
        return false;
    }

    if (!plotterSystem.machineZeroKnown())
    {
        Serial.println(
            F("REJECTED: machine zero is unknown; run HOME."));
        return false;
    }

    if (manualMotionSinceHoming)
    {
        Serial.println(
            F("REJECTED: manual motion occurred after HOME; run HOME again."));
        return false;
    }

    if (anyLimitSwitchPressed())
    {
        Serial.println(
            F("REJECTED: release all limit switches first."));
        return false;
    }

    const HomingResult homingResult =
        homingController.result();

    if (!homingResult.xValid ||
        !homingResult.yValid ||
        homingResult.xTravelMm <
            MOVE_TEST_MINIMUM_TRAVEL_MM ||
        homingResult.yTravelMm <
            MOVE_TEST_MINIMUM_TRAVEL_MM)
    {
        Serial.println(
            F("REJECTED: measured X/Y travel is invalid."));
        return false;
    }

    return true;
}

MoveTestRequestResult requestMoveToTestPoint(
    const FractionalTestPoint& point)
{
    if (!moveTestIsReady())
    {
        return MoveTestRequestResult::REJECTED;
    }

    const HomingResult homingResult =
        homingController.result();

    const Converter::CartesianDisplacement current =
        currentCartesianPosition();

    const float targetXMm =
        homingResult.xTravelMm * point.xFraction;

    const float targetYMm =
        homingResult.yTravelMm * point.yFraction;

    const float deltaXMm =
        targetXMm - current.xMm;

    const float deltaYMm =
        targetYMm - current.yMm;

    if (absoluteValue(deltaXMm) <=
            MOVE_TEST_POSITION_EPSILON_MM &&
        absoluteValue(deltaYMm) <=
            MOVE_TEST_POSITION_EPSILON_MM)
    {
        Serial.print(F("MOVE "));
        Serial.print(point.name);
        Serial.println(
            F(" skipped: already at target."));

        return MoveTestRequestResult::ALREADY_AT_TARGET;
    }

    const plotter::FSMResult request =
        plotterSystem.requestMove(
            deltaXMm,
            deltaYMm,
            MOVE_TEST_FEEDRATE_MM_PER_MINUTE,
            MOVE_TEST_ACCELERATION_MM_PER_SECOND_SQUARED);

    if (!request.accepted)
    {
        Serial.print(F("MOVE "));
        Serial.print(point.name);

        Serial.print(F(" rejected: "));
        printRejectReason(request.rejectReason);

        Serial.println();

        return MoveTestRequestResult::REJECTED;
    }

    activeMoveTargetXMm = targetXMm;
    activeMoveTargetYMm = targetYMm;
    activeMoveTargetValid = true;

    lastMoveTelemetryMs = millis();

    Serial.print(F("MOVE "));
    Serial.print(point.name);

    Serial.print(F(" accepted: current=("));
    Serial.print(current.xMm, 2);

    Serial.print(F(", "));
    Serial.print(current.yMm, 2);

    Serial.print(F(") target=("));
    Serial.print(targetXMm, 2);

    Serial.print(F(", "));
    Serial.print(targetYMm, 2);

    Serial.print(F(") delta=("));
    Serial.print(deltaXMm, 2);

    Serial.print(F(", "));
    Serial.print(deltaYMm, 2);

    Serial.println(F(") mm"));

    return MoveTestRequestResult::STARTED;
}

void requestNextRoutePoint()
{
    while (moveTestRouteActive &&
           nextMoveTestRoutePoint <
               MOVE_TEST_ROUTE_POINT_COUNT)
    {
        const FractionalTestPoint& point =
            MOVE_TEST_ROUTE[
                nextMoveTestRoutePoint];

        ++nextMoveTestRoutePoint;

        const MoveTestRequestResult result =
            requestMoveToTestPoint(point);

        if (result ==
            MoveTestRequestResult::STARTED)
        {
            return;
        }

        if (result ==
            MoveTestRequestResult::REJECTED)
        {
            moveTestRouteActive = false;

            Serial.println(
                F("ROUTE cancelled."));

            return;
        }
    }

    if (moveTestRouteActive)
    {
        moveTestRouteActive = false;

        Serial.println(
            F("ROUTE COMPLETE."));
    }
}

void startSingleMoveTest(
    const FractionalTestPoint& point)
{
    moveTestRouteActive = false;
    nextMoveTestRoutePoint = 0U;

    requestMoveToTestPoint(point);
}

void startMoveTestRoute()
{
    if (!moveTestIsReady())
    {
        return;
    }

    moveTestRouteActive = true;
    nextMoveTestRoutePoint = 0U;

    Serial.println(
        F("ROUTE started; STOP/M112 cancels and enters FAULT."));

    requestNextRoutePoint();
}

void monitorMoveTelemetry()
{
    if (!systemStarted ||
        plotterSystem.state() !=
            plotter::PlotterState::MOVING ||
        !activeMoveTargetValid)
    {
        return;
    }

    const unsigned long currentTimeMs =
        millis();

    if ((currentTimeMs -
         lastMoveTelemetryMs) <
        MOVE_TELEMETRY_INTERVAL_MS)
    {
        return;
    }

    lastMoveTelemetryMs =
        currentTimeMs;

    const Converter::CartesianDisplacement current =
        currentCartesianPosition();

    const XYCoordinatorTelemetry telemetry =
        xyCoordinator.getTelemetry();

    Serial.print(F("MOVE x="));
    Serial.print(current.xMm, 2);

    Serial.print(F("/"));
    Serial.print(activeMoveTargetXMm, 2);

    Serial.print(F(" y="));
    Serial.print(current.yMm, 2);

    Serial.print(F("/"));
    Serial.print(activeMoveTargetYMm, 2);

    Serial.print(F(" | errA="));
    Serial.print(
        telemetry.motorA.trackingError,
        1);

    Serial.print(F(" errB="));
    Serial.print(
        telemetry.motorB.trackingError,
        1);

    Serial.print(F(" outA="));
    Serial.print(
        telemetry.motorA.motorOutput);

    Serial.print(F(" outB="));
    Serial.println(
        telemetry.motorB.motorOutput);
}

// -----------------------------------------------------------------------------
// System monitoring and command handling
// -----------------------------------------------------------------------------

void printHomingResult()
{
    const HomingResult result =
        homingController.result();

    Serial.print(
        F("HOMING RESULT xTravelMm="));

    Serial.print(
        result.xTravelMm,
        3);

    Serial.print(
        F(" yTravelMm="));

    Serial.print(
        result.yTravelMm,
        3);

    Serial.print(
        F(" xValid="));

    Serial.print(
        result.xValid
            ? F("yes")
            : F("no"));

    Serial.print(
        F(" yValid="));

    Serial.println(
        result.yValid
            ? F("yes")
            : F("no"));
}

void monitorSystemChanges()
{
    if (!systemStarted)
    {
        return;
    }

    const plotter::PlotterState currentState =
        plotterSystem.state();

    if (currentState != lastPlotterState)
    {
        const plotter::PlotterState previousState =
            lastPlotterState;

        lastPlotterState =
            currentState;

        Serial.print(F("FSM -> "));
        printPlotterState(currentState);

        if (currentState ==
            plotter::PlotterState::FAULT)
        {
            moveTestRouteActive = false;
            activeMoveTargetValid = false;

            Serial.print(F(" code="));

            printFaultCode(
                plotterSystem.activeFault());
        }

        Serial.println();

        if (currentState ==
                plotter::PlotterState::IDLE &&
            previousState ==
                plotter::PlotterState::HOMING &&
            plotterSystem.machineZeroKnown())
        {
            moveTestRouteActive = false;
            activeMoveTargetValid = false;

            // A successful HOME restores a valid
            // coordinate reference after any manual motion.
            manualMotionSinceHoming = false;

            printHomingResult();
        }

        if (currentState ==
                plotter::PlotterState::IDLE &&
            previousState ==
                plotter::PlotterState::MOVING)
        {
            const Converter::CartesianDisplacement current =
                currentCartesianPosition();

            Serial.print(
                F("MOVE COMPLETE: actual=("));

            Serial.print(
                current.xMm,
                2);

            Serial.print(F(", "));

            Serial.print(
                current.yMm,
                2);

            Serial.println(
                F(") mm"));

            activeMoveTargetValid = false;

            if (moveTestRouteActive)
            {
                requestNextRoutePoint();
            }
        }
    }

    const HomingStage currentStage =
        homingController.stage();

    const HomingPhase currentPhase =
        homingController.phase();

    if (currentStage != lastHomingStage ||
        currentPhase != lastHomingPhase)
    {
        lastHomingStage =
            currentStage;

        lastHomingPhase =
            currentPhase;

        Serial.print(
            F("HOMING stage="));

        printHomingStage(currentStage);

        Serial.print(
            F(" phase="));

        printHomingPhase(currentPhase);

        Serial.print(F(" | "));

        printEncoderCounts();

        Serial.println();
    }
}

void startOrRequestHoming()
{
    stopManualPulse();

    moveTestRouteActive = false;
    activeMoveTargetValid = false;

    if (contradictoryLimitSwitchesPressed())
    {
        Serial.println(
            F("REJECTED: contradictory X or Y limit inputs."));
        return;
    }

    if (!systemStarted)
    {
        plotterSystem.begin();

        systemStarted = true;

        lastPlotterState =
            plotter::PlotterState::IDLE;

        lastHomingStage =
            HomingStage::IDLE;

        lastHomingPhase =
            HomingPhase::IDLE;

        Serial.println(
            F("System initialised; startup homing requested."));

        monitorSystemChanges();
        return;
    }

    const plotter::FSMResult result =
        plotterSystem.requestHoming();

    Serial.println(
        result.accepted
            ? F("Homing request accepted.")
            : F("Homing request rejected by FSM."));
}

void emergencyStop()
{
    stopManualPulse();

    moveTestRouteActive = false;
    activeMoveTargetValid = false;

    if (!systemStarted)
    {
        Serial.println(F("Stopped."));
        return;
    }

    if (plotterSystem.state() ==
            plotter::PlotterState::HOMING ||
        plotterSystem.state() ==
            plotter::PlotterState::MOVING)
    {
        plotterSystem.reportFault(
            plotter::FaultCode::INTERNAL_ERROR);

        Serial.println(
            F("STOP accepted; system entered FAULT."));
    }
    else
    {
        Serial.println(
            F("System already stopped."));
    }
}

void resetSystemFault()
{
    if (!systemStarted)
    {
        Serial.println(
            F("REJECTED: system has not been started."));
        return;
    }

    if (anyLimitSwitchPressed())
    {
        Serial.println(
            F("REJECTED: release all limit switches before RESET."));
        return;
    }

    const plotter::FSMResult result =
        plotterSystem.resetFault();

    if (!result.accepted)
    {
        Serial.println(
            F("Fault reset rejected by FSM."));
        return;
    }

    if (manualMotionSinceHoming)
    {
        Serial.println(
            F("Fault reset accepted. Manual motion invalidated the previous position; run HOME."));
        return;
    }

    if (plotterSystem.machineZeroKnown())
    {
        Serial.println(
            F("Fault reset accepted. Machine zero retained; move commands are available."));
    }
    else
    {
        Serial.println(
            F("Fault reset accepted. Machine zero unknown; run HOME before move."));
    }
}

void executeCommand(const char* command)
{
    if (strcmp(command, "HELP") == 0 ||
        strcmp(command, "?") == 0)
    {
        printHelp();
    }
    else if (strcmp(command, "STATUS") == 0)
    {
        printStatus();
    }
    else if (strcmp(command, "A+") == 0)
    {
        startManualPulse(
            MANUAL_TEST_PWM,
            0);
    }
    else if (strcmp(command, "A-") == 0)
    {
        startManualPulse(
            -static_cast<int16_t>(
                MANUAL_TEST_PWM),
            0);
    }
    else if (strcmp(command, "B+") == 0)
    {
        startManualPulse(
            0,
            MANUAL_TEST_PWM);
    }
    else if (strcmp(command, "B-") == 0)
    {
        startManualPulse(
            0,
            -static_cast<int16_t>(
                MANUAL_TEST_PWM));
    }
    else if (strcmp(command, "X+") == 0)
    {
        startCartesianPulse(
            1,
            0);
    }
    else if (strcmp(command, "X-") == 0)
    {
        startCartesianPulse(
            -1,
            0);
    }
    else if (strcmp(command, "Y+") == 0)
    {
        startCartesianPulse(
            0,
            1);
    }
    else if (strcmp(command, "Y-") == 0)
    {
        startCartesianPulse(
            0,
            -1);
    }
    else if (strcmp(command, "HOME") == 0 ||
             strcmp(command, "G28") == 0)
    {
        startOrRequestHoming();
    }
    else if (strcmp(command, "P1") == 0)
    {
        startSingleMoveTest(
            TEST_POINT_1);
    }
    else if (strcmp(command, "P2") == 0)
    {
        startSingleMoveTest(
            TEST_POINT_2);
    }
    else if (strcmp(command, "P3") == 0)
    {
        startSingleMoveTest(
            TEST_POINT_3);
    }
    else if (strcmp(command, "P4") == 0)
    {
        startSingleMoveTest(
            TEST_POINT_4);
    }
    else if (strcmp(command, "CENTER") == 0 ||
             strcmp(command, "CENTRE") == 0)
    {
        startSingleMoveTest(
            TEST_POINT_CENTRE);
    }
    else if (strcmp(command, "ROUTE") == 0)
    {
        startMoveTestRoute();
    }
    else if (strcmp(command, "STOP") == 0 ||
             strcmp(command, "M112") == 0)
    {
        emergencyStop();
    }
    else if (strcmp(command, "RESET") == 0 ||
             strcmp(command, "M999") == 0)
    {
        resetSystemFault();
    }
    else
    {
        Serial.print(
            F("Unknown command: "));

        Serial.println(command);
    }
}

char toUpperAscii(char value)
{
    if (value >= 'a' &&
        value <= 'z')
    {
        return static_cast<char>(
            value - 'a' + 'A');
    }

    return value;
}

void pollSerialCommands()
{
    while (Serial.available() > 0)
    {
        const char received =
            static_cast<char>(
                Serial.read());

        if (received == '\r' ||
            received == '\n')
        {
            if (commandLength > 0U)
            {
                commandBuffer[
                    commandLength] = '\0';

                executeCommand(
                    commandBuffer);

                commandLength = 0U;
            }

            continue;
        }

        if (received == ' ' ||
            received == '\t')
        {
            continue;
        }

        if (commandLength <
            COMMAND_BUFFER_SIZE - 1U)
        {
            commandBuffer[
                commandLength] =
                    toUpperAscii(
                        received);

            ++commandLength;
        }
        else
        {
            commandLength = 0U;

            Serial.println(
                F("Command too long; buffer cleared."));
        }
    }
}

}  // namespace

// -----------------------------------------------------------------------------
// Encoder pin-change interrupts
// -----------------------------------------------------------------------------

// Encoder A:
// D68 / A14 = PCINT22 = PCINT2_vect
ISR(PCINT2_vect)
{
    encoderA.update();
}

// Encoder B:
// D52 = PCINT1 = PCINT0_vect
ISR(PCINT0_vect)
{
    encoderB.update();
}

// -----------------------------------------------------------------------------
// Arduino setup / loop
// -----------------------------------------------------------------------------

void setup()
{
    Serial.begin(
        SystemConfig::SERIAL_BAUD_RATE);

    // Required for pre-homing manual motor tests.
    motorA.begin();
    motorB.begin();

    Encoder::zeroCountPair(
        encoderA,
        encoderB);

    // Active-high limit switches.
    leftLimitSwitch.begin(
        onLeftLimitSwitchInterrupt);

    rightLimitSwitch.begin(
        onRightLimitSwitchInterrupt);

    bottomLimitSwitch.begin(
        onBottomLimitSwitchInterrupt);

    topLimitSwitch.begin(
        onTopLimitSwitchInterrupt);

    lastLeftPressed =
        leftLimitSwitch.isPressed();

    lastRightPressed =
        rightLimitSwitch.isPressed();

    lastBottomPressed =
        bottomLimitSwitch.isPressed();

    lastTopPressed =
        topLimitSwitch.isPressed();

    Serial.println();
    Serial.println(
        F("ME306 plotter HIGH-POWER hardware bring-up console"));

    Serial.println(
        F("Active-high limits; automatic homing is DISARMED until HOME/G28."));

    Serial.print(
        F("Motor command limit: "));

    Serial.println(
        BRINGUP_MOTOR_OUTPUT_LIMIT);

    Serial.print(
        F("Manual test PWM: "));

    Serial.println(
        MANUAL_TEST_PWM);

    Serial.print(
        F("Manual pulse duration ms: "));

    Serial.println(
        MANUAL_TEST_DURATION_MS);

    Serial.print(
        F("Closed-loop output range: "));

    Serial.print(
        CONTROLLER_MINIMUM_OUTPUT,
        0);

    Serial.print(F(" to "));

    Serial.println(
        CONTROLLER_MAXIMUM_OUTPUT,
        0);

    Serial.print(
        F("Initial Kp: "));

    Serial.println(
        INITIAL_PROPORTIONAL_GAIN,
        2);

    printHelp();
    printStatus();

    if (anyLimitSwitchPressed())
    {
        Serial.println(
            F("WARNING: one or more switches are active at startup."));
    }

    Serial.print(F("PCMSK2 = 0x"));
    Serial.println(PCMSK2, HEX);

    Serial.print(F("PCMSK0 = 0x"));
    Serial.println(PCMSK0, HEX);
}

void loop()
{
    pollSerialCommands();

    if (!systemStarted)
    {
        updateAllLimitSwitches();
    }
    else
    {
        // HomingController owns switch updates while HOMING.
        //
        // At IDLE/MOVING, main keeps switch debouncing alive.
        if (plotterSystem.state() !=
            plotter::PlotterState::HOMING)
        {
            updateAllLimitSwitches();

            if (plotterSystem.state() ==
                    plotter::PlotterState::MOVING &&
                anyLimitSwitchPressed())
            {
                plotterSystem.reportFault(
                    plotter::FaultCode::
                        UNEXPECTED_LIMIT);
            }
        }

        plotterSystem.update();
    }

    // IMPORTANT:
    // Manual pulses can now be used before HOME or while the
    // started system is IDLE, so this must run regardless of
    // systemStarted.
    updateManualPulse();

    monitorSwitchChanges();
    monitorMoveTelemetry();
    monitorSystemChanges();
}