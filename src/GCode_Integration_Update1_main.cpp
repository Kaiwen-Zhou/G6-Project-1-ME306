/*
 * gcode_integration_main_update1.cpp
 *
 * Standalone integration entry point for the ME306 X-Y plotter.
 * Keep the team's src/main.cpp unchanged. For a local test only, copy this
 * file to src/main.cpp, build, upload, then restore the team's main.cpp.
 *
 * Supported motion commands:
 *   G28
 *   G01 X... Y... F...
 *   M999
 *
 * Safety console commands:
 *   !             immediate emergency stop; no newline required
 *   X / STOP / M112 + newline also request an emergency stop
 *   STATUS        print the current state
 *
 * This version uses the core system safety paths only: FSM fault handling,
 * homing timeout, unexpected-limit detection, emergency stop and G-code
 * soft-limit validation.
 *
 * All ordinary messages start with '#'. Lines without '#' are CSV telemetry.
 */

#include <Arduino.h>
#include <avr/interrupt.h>
#include <stdint.h>

#include "communication/GCodeController.h"
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
#include "system/PlotterSystem.h"
#include "system/TrajectoryPlanner.h"

namespace
{

// ---------------------------------------------------------------------------
// Bring-up settings: safe starting values, not final tuned values
// ---------------------------------------------------------------------------
// Begin with the P_MEDIUM preset from axis_pid_test_main.cpp. Replace the A
// and B values separately with the gains supported by your measured plots.
constexpr float INITIAL_A_KP = 0.12f;
constexpr float INITIAL_A_KI = 0.00f;
constexpr float INITIAL_A_KV = 0.00f;

constexpr float INITIAL_B_KP = 0.12f;
constexpr float INITIAL_B_KI = 0.00f;
constexpr float INITIAL_B_KV = 0.00f;

constexpr float CONTROLLER_MINIMUM_OUTPUT = -40.0f;
constexpr float CONTROLLER_MAXIMUM_OUTPUT = 40.0f;
constexpr float INTEGRAL_MINIMUM_OUTPUT = -20.0f;
constexpr float INTEGRAL_MAXIMUM_OUTPUT = 20.0f;

constexpr float POSITION_TOLERANCE_COUNTS = 20.0f;

// First hardware run: 40/255. Raise only after direction, encoder sign,
// active-high limits and emergency stop have all been verified.
constexpr uint8_t SAFE_MOTOR_OUTPUT_LIMIT = 40U;

// The parser reduces larger F commands to this value.
constexpr float MAXIMUM_FEEDRATE_MM_PER_MINUTE = 120.0f;
constexpr float MAXIMUM_ACCELERATION_MM_PER_SECOND_SQUARED = 10.0f;

constexpr unsigned long TELEMETRY_INTERVAL_MS = 20UL;

constexpr uint16_t SERIAL_LINE_CAPACITY =
    plotter::GCODE_MAX_LINE_LENGTH + 1U;

// ---------------------------------------------------------------------------
// Hardware and control objects
// ---------------------------------------------------------------------------
Encoder encoderA(true);
Encoder encoderB(false);

LimitSwitch leftLimit(
    PinConfig::LIMIT_SWITCH_LEFT_PIN,
    SystemConfig::LIMIT_SWITCH_INPUT_MODE,
    SystemConfig::LIMIT_SWITCH_DEBOUNCE_MS);

LimitSwitch rightLimit(
    PinConfig::LIMIT_SWITCH_RIGHT_PIN,
    SystemConfig::LIMIT_SWITCH_INPUT_MODE,
    SystemConfig::LIMIT_SWITCH_DEBOUNCE_MS);

LimitSwitch bottomLimit(
    PinConfig::LIMIT_SWITCH_BOTTOM_PIN,
    SystemConfig::LIMIT_SWITCH_INPUT_MODE,
    SystemConfig::LIMIT_SWITCH_DEBOUNCE_MS);

LimitSwitch topLimit(
    PinConfig::LIMIT_SWITCH_TOP_PIN,
    SystemConfig::LIMIT_SWITCH_INPUT_MODE,
    SystemConfig::LIMIT_SWITCH_DEBOUNCE_MS);

MotorDriver motorA(
    PinConfig::MOTOR_1_DIRECTION_PIN,
    PinConfig::MOTOR_1_PWM_PIN,
    SystemConfig::MOTOR_1_DIRECTION_INVERTED,
    SAFE_MOTOR_OUTPUT_LIMIT);

MotorDriver motorB(
    PinConfig::MOTOR_2_DIRECTION_PIN,
    PinConfig::MOTOR_2_PWM_PIN,
    SystemConfig::MOTOR_2_DIRECTION_INVERTED,
    SAFE_MOTOR_OUTPUT_LIMIT);

PIDController pidA(
    INITIAL_A_KP,
    INITIAL_A_KI,
    CONTROLLER_MINIMUM_OUTPUT,
    CONTROLLER_MAXIMUM_OUTPUT,
    INTEGRAL_MINIMUM_OUTPUT,
    INTEGRAL_MAXIMUM_OUTPUT,
    INITIAL_A_KV);

PIDController pidB(
    INITIAL_B_KP,
    INITIAL_B_KI,
    CONTROLLER_MINIMUM_OUTPUT,
    CONTROLLER_MAXIMUM_OUTPUT,
    INTEGRAL_MINIMUM_OUTPUT,
    INTEGRAL_MAXIMUM_OUTPUT,
    INITIAL_B_KV);

AxisController axisA(pidA, motorA, POSITION_TOLERANCE_COUNTS);
AxisController axisB(pidB, motorB, POSITION_TOLERANCE_COUNTS);

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
    leftLimit,
    rightLimit,
    bottomLimit,
    topLimit,
    homingConfig);

plotter::TrajectoryPlanner trajectoryPlanner;

plotter::PlotterSystem plotterSystem(
    axisA,
    axisB,
    xyCoordinator,
    trajectoryPlanner,
    homingController);

plotter::GCodeController gCodeController(
    plotterSystem,
    homingController,
    converter,
    encoderA,
    encoderB,
    MAXIMUM_FEEDRATE_MM_PER_MINUTE,
    MAXIMUM_ACCELERATION_MM_PER_SECOND_SQUARED);

// ---------------------------------------------------------------------------
// Application state
// ---------------------------------------------------------------------------
char serialLine[SERIAL_LINE_CAPACITY] = {};
uint16_t serialLineLength = 0U;
bool serialLineTooLong = false;
bool ignoreNextLineFeed = false;

unsigned long lastTelemetryMs = 0UL;
plotter::PlotterState lastState = plotter::PlotterState::IDLE;
HomingStage lastHomingStage = HomingStage::IDLE;
HomingPhase lastHomingPhase = HomingPhase::IDLE;

// ---------------------------------------------------------------------------
// Interrupt callbacks
// ---------------------------------------------------------------------------
void onLeftLimitInterrupt()   { leftLimit.notifyFromISR(); }
void onRightLimitInterrupt()  { rightLimit.notifyFromISR(); }
void onBottomLimitInterrupt() { bottomLimit.notifyFromISR(); }
void onTopLimitInterrupt()    { topLimit.notifyFromISR(); }

// ---------------------------------------------------------------------------
// Small helpers
// ---------------------------------------------------------------------------
char upperAscii(char character)
{
    if (character >= 'a' && character <= 'z')
    {
        return static_cast<char>(character - 'a' + 'A');
    }

    return character;
}

bool isSpace(char character)
{
    return character == ' ' || character == '\t';
}

bool lineEqualsIgnoringSpaces(const char* line, const char* expected)
{
    uint16_t lineIndex = 0U;
    uint16_t expectedIndex = 0U;

    while (true)
    {
        while (isSpace(line[lineIndex]))
        {
            ++lineIndex;
        }

        const char lineCharacter = upperAscii(line[lineIndex]);
        const char expectedCharacter = expected[expectedIndex];

        if (lineCharacter != expectedCharacter)
        {
            return false;
        }

        if (lineCharacter == '\0')
        {
            return true;
        }

        ++lineIndex;
        ++expectedIndex;
    }
}

void updateAllLimits()
{
    leftLimit.update();
    rightLimit.update();
    bottomLimit.update();
    topLimit.update();
}

bool anyLimitPressed()
{
    return leftLimit.isPressed() ||
           rightLimit.isPressed() ||
           bottomLimit.isPressed() ||
           topLimit.isPressed();
}

void printState(plotter::PlotterState state)
{
    switch (state)
    {
        case plotter::PlotterState::IDLE:   Serial.print(F("IDLE")); break;
        case plotter::PlotterState::HOMING: Serial.print(F("HOMING")); break;
        case plotter::PlotterState::MOVING: Serial.print(F("MOVING")); break;
        case plotter::PlotterState::FAULT:  Serial.print(F("FAULT")); break;
    }
}

void printFault(plotter::FaultCode fault)
{
    switch (fault)
    {
        case plotter::FaultCode::NONE:                  Serial.print(F("NONE")); break;
        case plotter::FaultCode::UNEXPECTED_LIMIT:      Serial.print(F("UNEXPECTED_LIMIT")); break;
        case plotter::FaultCode::WRONG_HOMING_LIMIT:    Serial.print(F("WRONG_HOMING_LIMIT")); break;
        case plotter::FaultCode::CONTRADICTORY_LIMITS:  Serial.print(F("CONTRADICTORY_LIMITS")); break;
        case plotter::FaultCode::HOMING_TIMEOUT:        Serial.print(F("HOMING_TIMEOUT")); break;
        case plotter::FaultCode::MOVE_TIMEOUT:          Serial.print(F("MOVE_TIMEOUT")); break;
        case plotter::FaultCode::ENCODER_NO_MOTION:     Serial.print(F("ENCODER_NO_MOTION")); break;
        case plotter::FaultCode::POSITION_OUT_OF_RANGE: Serial.print(F("POSITION_OUT_OF_RANGE")); break;
        case plotter::FaultCode::INTERNAL_ERROR:        Serial.print(F("INTERNAL_ERROR")); break;
    }
}

void printRejectReason(plotter::RejectReason reason)
{
    switch (reason)
    {
        case plotter::RejectReason::NONE:                      Serial.print(F("NONE")); break;
        case plotter::RejectReason::BUSY:                      Serial.print(F("BUSY")); break;
        case plotter::RejectReason::MACHINE_ZERO_UNKNOWN:      Serial.print(F("MACHINE_ZERO_UNKNOWN")); break;
        case plotter::RejectReason::FAULT_ACTIVE:              Serial.print(F("FAULT_ACTIVE")); break;
        case plotter::RejectReason::UNEXPECTED_EVENT:          Serial.print(F("UNEXPECTED_EVENT")); break;
        case plotter::RejectReason::INVALID_MOTION_PARAMETERS: Serial.print(F("INVALID_MOTION_PARAMETERS")); break;
    }
}

void printStatus()
{
    Serial.print(F("# STATUS state="));

    if (!gCodeController.systemStarted())
    {
        Serial.print(F("NOT_STARTED"));
    }
    else
    {
        printState(plotterSystem.state());
    }

    const Encoder::CountPair counts =
        Encoder::getCountPair(encoderA, encoderB);

    const Converter::CartesianDisplacement position =
        converter.motorToCartesianDisplacement(
            static_cast<float>(counts.countA),
            static_cast<float>(counts.countB));

    Serial.print(F(" limits="));
    Serial.print(leftLimit.isPressed() ? 1 : 0);
    Serial.print(',');
    Serial.print(rightLimit.isPressed() ? 1 : 0);
    Serial.print(',');
    Serial.print(bottomLimit.isPressed() ? 1 : 0);
    Serial.print(',');
    Serial.print(topLimit.isPressed() ? 1 : 0);
    Serial.print(F(" counts="));
    Serial.print(counts.countA);
    Serial.print(',');
    Serial.print(counts.countB);
    Serial.print(F(" xy_mm="));
    Serial.print(position.xMm, 3);
    Serial.print(',');
    Serial.println(position.yMm, 3);
}

void emergencyStop()
{
    const bool stopped = gCodeController.emergencyStop();
    serialLineLength = 0U;
    serialLineTooLong = false;

    Serial.println(
        stopped
            ? F("# EMERGENCY STOP: FAULT latched")
            : F("# EMERGENCY STOP: system was already stopped"));
}

void handleGCodeResult(const plotter::GCodeControllerResult& result)
{
    if (!result.lineComplete)
    {
        return;
    }

    if (!result.accepted)
    {
        Serial.print(F("# REJECTED: "));

        if (result.parseError != plotter::GCodeParseError::NONE)
        {
            Serial.println(
                plotter::GCodeParser::errorMessage(result.parseError));
        }
        else
        {
            Serial.print(
                plotter::GCodeController::controllerErrorMessage(
                    result.controllerError));

            if (result.rejectReason != plotter::RejectReason::NONE)
            {
                Serial.print(F(" FSM="));
                printRejectReason(result.rejectReason);
            }

            Serial.println();
        }

        return;
    }

    if (result.command.type == plotter::GCodeCommandType::NONE)
    {
        return;
    }

    if (result.command.type == plotter::GCodeCommandType::HOME)
    {
        Serial.println(F("# ACCEPTED G28: homing started"));
        return;
    }

    if (result.command.type == plotter::GCodeCommandType::RESET_FAULT)
    {
        Serial.println(F("# ACCEPTED M999: fault cleared"));
        return;
    }

    if (!result.movementStarted)
    {
        Serial.print(F("# ACCEPTED G01: feedrate stored F="));
        Serial.println(result.command.feedrateMmPerMinute, 2);
        return;
    }

    Serial.print(F("# ACCEPTED G01: dx="));
    // Serial.print(result.command.xDisplacementMm, 3);
    Serial.print(result.command.xOffsetMm, 3);
    Serial.print(F(" dy="));
    // Serial.print(result.command.yDisplacementMm, 3);
    Serial.print(result.command.yOffsetMm, 3);
    Serial.print(F(" F="));
    Serial.print(result.command.feedrateMmPerMinute, 2);

    if (result.command.feedrateWasLimited)
    {
        Serial.print(F(" (limited)"));
    }

    Serial.println();

    lastTelemetryMs = millis();
}

void processCompleteSerialLine()
{
    serialLine[serialLineLength] = '\0';

    if (lineEqualsIgnoringSpaces(serialLine, "X") ||
        lineEqualsIgnoringSpaces(serialLine, "STOP") ||
        lineEqualsIgnoringSpaces(serialLine, "M112"))
    {
        emergencyStop();
    }
    else if (lineEqualsIgnoringSpaces(serialLine, "STATUS"))
    {
        printStatus();
    }
    else if (serialLineTooLong)
    {
        Serial.println(F("# REJECTED: serial line is too long"));
    }
    else
    {
        handleGCodeResult(
            gCodeController.processLine(
                serialLine,
                !anyLimitPressed()));
    }

    serialLineLength = 0U;
    serialLineTooLong = false;
    serialLine[0] = '\0';
}

void pollSerial()
{
    while (Serial.available() > 0)
    {
        const char character = static_cast<char>(Serial.read());

        if (character == '!')
        {
            emergencyStop();
            continue;
        }

        if (character == '\n' && ignoreNextLineFeed)
        {
            ignoreNextLineFeed = false;
            continue;
        }

        if (character == '\r' || character == '\n')
        {
            ignoreNextLineFeed = (character == '\r');
            processCompleteSerialLine();
            continue;
        }

        ignoreNextLineFeed = false;

        if (serialLineLength < SERIAL_LINE_CAPACITY - 1U)
        {
            serialLine[serialLineLength] = character;
            ++serialLineLength;
        }
        else
        {
            serialLineTooLong = true;
        }
    }
}

void printTelemetry()
{
    if (!gCodeController.systemStarted() ||
        plotterSystem.state() != plotter::PlotterState::MOVING)
    {
        return;
    }

    const unsigned long currentTimeMs = millis();

    if ((currentTimeMs - lastTelemetryMs) < TELEMETRY_INTERVAL_MS)
    {
        return;
    }

    lastTelemetryMs = currentTimeMs;

    const XYCoordinatorTelemetry data =
        xyCoordinator.getTelemetry();

    const float errorXMm =
        data.referenceXDisplacementMm - data.actualXDisplacementMm;

    const float errorYMm =
        data.referenceYDisplacementMm - data.actualYDisplacementMm;

    Serial.print(currentTimeMs);
    Serial.print(',');
    Serial.print(data.referenceXDisplacementMm, 3);
    Serial.print(',');
    Serial.print(data.actualXDisplacementMm, 3);
    Serial.print(',');
    Serial.print(errorXMm, 3);
    Serial.print(',');
    Serial.print(data.referenceYDisplacementMm, 3);
    Serial.print(',');
    Serial.print(data.actualYDisplacementMm, 3);
    Serial.print(',');
    Serial.print(errorYMm, 3);
    Serial.print(',');
    Serial.print(data.motorA.trackingError, 2);
    Serial.print(',');
    Serial.print(data.motorA.motorOutput);
    Serial.print(',');
    Serial.print(data.motorA.integralOutput, 3);
    Serial.print(',');
    Serial.print(data.motorB.trackingError, 2);
    Serial.print(',');
    Serial.print(data.motorB.motorOutput);
    Serial.print(',');
    Serial.println(data.motorB.integralOutput, 3);
}

void reportStateChanges()
{
    if (!gCodeController.systemStarted())
    {
        return;
    }

    const plotter::PlotterState state = plotterSystem.state();

    if (state != lastState)
    {
        lastState = state;
        Serial.print(F("# FSM -> "));
        printState(state);

        if (state == plotter::PlotterState::FAULT)
        {
            Serial.print(F(" fault="));
            printFault(plotterSystem.activeFault());
        }

        Serial.println();
    }

    if (plotterSystem.state() == plotter::PlotterState::HOMING)
    {
        const HomingStage stage = homingController.stage();
        const HomingPhase phase = homingController.phase();

        if (stage != lastHomingStage || phase != lastHomingPhase)
        {
            lastHomingStage = stage;
            lastHomingPhase = phase;
            Serial.print(F("# HOMING stage="));
            Serial.print(static_cast<uint8_t>(stage));
            Serial.print(F(" phase="));
            Serial.println(static_cast<uint8_t>(phase));
        }
    }
}

}  // namespace

// Encoder A: D68/A14 = PCINT22 -> PCINT2_vect.
ISR(PCINT2_vect)
{
    encoderA.update();
}

// Encoder B: D52 = PCINT1 -> PCINT0_vect.
ISR(PCINT0_vect)
{
    encoderB.update();
}

void setup()
{
    Serial.begin(SystemConfig::SERIAL_BAUD_RATE);

    motorA.begin();
    motorB.begin();
    Encoder::zeroCountPair(encoderA, encoderB);

    leftLimit.begin(onLeftLimitInterrupt);
    rightLimit.begin(onRightLimitInterrupt);
    bottomLimit.begin(onBottomLimitInterrupt);
    topLimit.begin(onTopLimitInterrupt);

    gCodeController.begin();

    Serial.println();
    Serial.println(F("# ME306 G-code integration test ready"));
    Serial.println(F("# Active-high limits: HIGH=pressed"));
    Serial.println(F("# Motor output limit=40/255"));
    Serial.println(F("# Send G28 first. Emergency stop: !"));
    printStatus();
    Serial.println(
        F("time_ms,reference_x_mm,actual_x_mm,error_x_mm,reference_y_mm,actual_y_mm,error_y_mm,error_a_counts,pwm_a,integral_a,error_b_counts,pwm_b,integral_b"));
}

void loop()
{
    // Outside HOMING, the application owns switch debouncing.
    if (!gCodeController.systemStarted() ||
        plotterSystem.state() != plotter::PlotterState::HOMING)
    {
        updateAllLimits();
    }

    pollSerial();

    if (gCodeController.systemStarted())
    {
        if (plotterSystem.state() == plotter::PlotterState::MOVING &&
            anyLimitPressed())
        {
            const plotter::FSMResult result =
                plotterSystem.reportFault(
                    plotter::FaultCode::UNEXPECTED_LIMIT);

            if (result.accepted)
            {
                gCodeController.requireNewHoming();
            }
        }

        plotterSystem.update();

        if (gCodeController.updateAfterSystem())
        {
            const HomingResult result = homingController.result();
            Serial.print(F("# HOMING COMPLETE xTravelMm="));
            Serial.print(result.xTravelMm, 3);
            Serial.print(F(" yTravelMm="));
            Serial.println(result.yTravelMm, 3);
        }
    }

    printTelemetry();
    reportStateChanges();
}
