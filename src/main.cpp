/*
 * ME306 combined hardware diagnostic build
 *
 * Purpose
 * -------
 * Temporary, directly flashable diagnostic main for the Arduino Mega2560
 * plotter project. It exercises:
 *   - both encoder channels using the existing pin-change interrupt module;
 *   - all four active-low limit switches using both polling/debounce and
 *     dedicated FALLING external interrupts;
 *   - both MotorDriver instances using short, low-output serial-commanded
 *     pulses;
 *   - the four paired motor-command combinations from the supplied platform
 *     motion table.
 *
 * Safety behaviour
 * ----------------
 * Motors never start automatically. Every motor command times out after a
 * short pulse. Any raw or debounced limit-switch press stops both motors and
 * latches motion off. The latch can only be cleared after all four switches
 * are released.
 *
 * This file is intended to temporarily replace src/main.cpp. It depends on
 * the project's existing PinConfig, SystemConfig, Encoder, LimitSwitch, and
 * MotorDriver files.
 */

#include <Arduino.h>
#include <avr/interrupt.h>

#include "config/PinConfig.h"
#include "config/SystemConfig.h"
#include "hardware/Encoder.h"
#include "hardware/LimitSwitch.h"
#include "hardware/MotorDriver.h"

namespace DiagnosticConfig
{
    // Diagnostic-only limits. These do not belong in SystemConfig.
    constexpr int16_t MOTOR_TEST_PWM = 100;
    constexpr unsigned long MOTOR_TEST_DURATION_MS = 300UL;
    constexpr unsigned long TELEMETRY_INTERVAL_MS = 250UL;
}

enum LimitIndex : uint8_t
{
    LIMIT_LEFT = 0,
    LIMIT_RIGHT,
    LIMIT_BOTTOM,
    LIMIT_TOP,
    LIMIT_COUNT
};

enum LimitMask : uint8_t
{
    LIMIT_MASK_LEFT = 1U << LIMIT_LEFT,
    LIMIT_MASK_RIGHT = 1U << LIMIT_RIGHT,
    LIMIT_MASK_BOTTOM = 1U << LIMIT_BOTTOM,
    LIMIT_MASK_TOP = 1U << LIMIT_TOP
};

// Existing hardware modules
Encoder encoderA(true);
Encoder encoderB(false);

MotorDriver motor1(
    PinConfig::MOTOR_1_DIRECTION_PIN,
    PinConfig::MOTOR_1_PWM_PIN,
    SystemConfig::MOTOR_1_DIRECTION_INVERTED,
    SystemConfig::MOTOR_DEFAULT_OUTPUT_LIMIT);

MotorDriver motor2(
    PinConfig::MOTOR_2_DIRECTION_PIN,
    PinConfig::MOTOR_2_PWM_PIN,
    SystemConfig::MOTOR_2_DIRECTION_INVERTED,
    SystemConfig::MOTOR_DEFAULT_OUTPUT_LIMIT);

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

LimitSwitch *const limitSwitches[LIMIT_COUNT] =
{
    &leftLimit,
    &rightLimit,
    &bottomLimit,
    &topLimit
};

const uint8_t limitPins[LIMIT_COUNT] =
{
    PinConfig::LIMIT_SWITCH_LEFT_PIN,
    PinConfig::LIMIT_SWITCH_RIGHT_PIN,
    PinConfig::LIMIT_SWITCH_BOTTOM_PIN,
    PinConfig::LIMIT_SWITCH_TOP_PIN
};

const char *const limitNames[LIMIT_COUNT] =
{
    "LEFT",
    "RIGHT",
    "BOTTOM",
    "TOP"
};

// Written only by the four external-interrupt handlers.
volatile uint8_t rawLimitInterruptMask = 0U;
volatile bool limitStopRequested = false;

bool previousPressedState[LIMIT_COUNT] = {false, false, false, false};
bool limitStopLatched = false;
bool motorPulseActive = false;
bool telemetryEnabled = true;
unsigned long motorPulseStartMs = 0UL;
unsigned long lastTelemetryMs = 0UL;

struct EncoderSnapshot
{
    uint16_t countA;
    uint16_t countB;
    bool directionA;
    bool directionB;
};

// Encoder A uses D68 / PCINT22 / PCINT2_vect.
ISR(PCINT2_vect)
{
    encoderA.update();
}

// Encoder B uses D52 / PCINT1 / PCINT0_vect.
ISR(PCINT0_vect)
{
    encoderB.update();
}

// Limit-switch ISRs remain minimal. Debounce and reporting stay in loop().
void leftLimitISR()
{
    rawLimitInterruptMask |= LIMIT_MASK_LEFT;
    limitStopRequested = true;
}

void rightLimitISR()
{
    rawLimitInterruptMask |= LIMIT_MASK_RIGHT;
    limitStopRequested = true;
}

void bottomLimitISR()
{
    rawLimitInterruptMask |= LIMIT_MASK_BOTTOM;
    limitStopRequested = true;
}

void topLimitISR()
{
    rawLimitInterruptMask |= LIMIT_MASK_TOP;
    limitStopRequested = true;
}

EncoderSnapshot readEncoderSnapshot()
{
    EncoderSnapshot snapshot;

    // The current Encoder count is uint16_t on an 8-bit MCU, so protect the
    // multi-byte read from a simultaneous pin-change ISR.
    noInterrupts();
    snapshot.countA = encoderA.getCount();
    snapshot.countB = encoderB.getCount();
    snapshot.directionA = encoderA.getDirection();
    snapshot.directionB = encoderB.getDirection();
    interrupts();

    return snapshot;
}

bool anyRawLimitPressed()
{
    for (uint8_t index = 0; index < LIMIT_COUNT; ++index)
    {
        if (digitalRead(limitPins[index]) == HIGH)
        {
            return true;
        }
    }

    return false;
}

bool anyDebouncedLimitPressed()
{
    for (uint8_t index = 0; index < LIMIT_COUNT; ++index)
    {
        if (limitSwitches[index]->isPressed())
        {
            return true;
        }
    }

    return false;
}

bool allLimitsReleased()
{
    for (uint8_t index = 0; index < LIMIT_COUNT; ++index)
    {
        if ((digitalRead(limitPins[index]) == HIGH) ||
            limitSwitches[index]->isPressed())
        {
            return false;
        }
    }

    return true;
}

void stopAllMotors()
{
    motor1.stop();
    motor2.stop();
    motorPulseActive = false;
}

void printHelp()
{
    Serial.println();
    Serial.println(F("=== ME306 hardware diagnostic commands ==="));
    Serial.println(F("h or ? : show this command list"));
    Serial.println(F("p      : print one complete hardware snapshot"));
    Serial.println(F("t      : toggle continuous telemetry"));
    Serial.println(F("z      : atomically zero both encoder counts"));
    Serial.println(F("x      : immediately stop both motors"));
    Serial.println(F("c      : clear limit-stop latch (all switches must be released)"));
    Serial.println();
    Serial.println(F("Short single-motor pulses:"));
    Serial.println(F("1      : Motor 1, +100 command for 300 ms"));
    Serial.println(F("2      : Motor 1, -100 command for 300 ms"));
    Serial.println(F("3      : Motor 2, +100 command for 300 ms"));
    Serial.println(F("4      : Motor 2, -100 command for 300 ms"));
    Serial.println();
    Serial.println(F("Short paired pulses from the supplied motion table:"));
    Serial.println(F("R      : M1 +100, M2 +100 (candidate pen-right mapping)"));
    Serial.println(F("L      : M1 -100, M2 -100 (candidate pen-left mapping)"));
    Serial.println(F("U      : M1 -100, M2 +100 (candidate pen-up mapping)"));
    Serial.println(F("D      : M1 +100, M2 -100 (candidate pen-down mapping)"));
    Serial.println();
    Serial.println(F("Verify the single-motor direction and encoder sign before"));
    Serial.println(F("treating R/L/U/D labels as confirmed physical directions."));
    Serial.println(F("Current Encoder count is uint16_t: reverse movement from zero"));
    Serial.println(F("can appear as 65535, 65534, ... until the signed-count fix."));
    Serial.println();
}

void printLimitBits(bool raw)
{
    for (uint8_t index = 0; index < LIMIT_COUNT; ++index)
    {
        const bool pressed = raw
            ? (digitalRead(limitPins[index]) == HIGH)
            : limitSwitches[index]->isPressed();

        Serial.print(pressed ? '1' : '0');
    }
}

void printSnapshot()
{
    const EncoderSnapshot encoder = readEncoderSnapshot();

    Serial.print(F("t="));
    Serial.print(millis());

    Serial.print(F("  ENC_A="));
    Serial.print(encoder.countA);
    Serial.print(encoder.directionA ? F("(CW)") : F("(CCW)"));

    Serial.print(F("  ENC_B="));
    Serial.print(encoder.countB);
    Serial.print(encoder.directionB ? F("(CW)") : F("(CCW)"));

    // Bit order is always Left, Right, Bottom, Top.
    Serial.print(F("  SW_RAW[LRBT]="));
    printLimitBits(true);

    Serial.print(F("  SW_DEB[LRBT]="));
    printLimitBits(false);

    Serial.print(F("  M1="));
    Serial.print(motor1.getOutput());
    Serial.print(F("  M2="));
    Serial.print(motor2.getOutput());

    Serial.print(F("  LIMIT_LATCH="));
    Serial.println(limitStopLatched ? F("ON") : F("OFF"));
}

void reportRawLimitInterrupts(uint8_t mask)
{
    for (uint8_t index = 0; index < LIMIT_COUNT; ++index)
    {
        if ((mask & (1U << index)) != 0U)
        {
            Serial.print(F("[LIMIT INT] "));
            Serial.print(limitNames[index]);
            Serial.println(F(" raw FALLING edge"));
        }
    }
}

void serviceRawLimitInterrupts()
{
    uint8_t pendingMask;
    bool stopRequested;

    noInterrupts();
    pendingMask = rawLimitInterruptMask;
    rawLimitInterruptMask = 0U;
    stopRequested = limitStopRequested;
    limitStopRequested = false;
    interrupts();

    if (!stopRequested && (pendingMask == 0U))
    {
        return;
    }

    stopAllMotors();
    limitStopLatched = true;
    Serial.println(F("[SAFETY] Raw limit interrupt: both motors stopped; latch ON."));
    reportRawLimitInterrupts(pendingMask);
}

void updateAndReportLimitSwitches()
{
    for (uint8_t index = 0; index < LIMIT_COUNT; ++index)
    {
        limitSwitches[index]->update();
        const bool pressed = limitSwitches[index]->isPressed();

        if (pressed != previousPressedState[index])
        {
            previousPressedState[index] = pressed;

            Serial.print(F("[SWITCH] "));
            Serial.print(limitNames[index]);
            Serial.println(pressed ? F(" debounced PRESSED") : F(" debounced RELEASED"));

            if (pressed)
            {
                stopAllMotors();
                limitStopLatched = true;
                Serial.println(F("[SAFETY] Debounced limit press: both motors stopped; latch ON."));
            }
        }
    }
}

void startMotorPulse(int16_t motor1Command,
                     int16_t motor2Command,
                     const __FlashStringHelper *label)
{
    // Check both raw and debounced inputs so a just-pressed switch cannot be
    // bypassed during its debounce interval.
    if (limitStopLatched || anyRawLimitPressed() || anyDebouncedLimitPressed())
    {
        stopAllMotors();
        limitStopLatched = true;
        Serial.print(F("[BLOCKED] "));
        Serial.print(label);
        Serial.println(F(": release all switches, then send c."));
        return;
    }

    stopAllMotors();
    motor1.setOutput(motor1Command);
    motor2.setOutput(motor2Command);
    motorPulseStartMs = millis();
    motorPulseActive = true;

    Serial.print(F("[MOTOR] "));
    Serial.print(label);
    Serial.print(F(" started: M1="));
    Serial.print(motor1.getOutput());
    Serial.print(F(", M2="));
    Serial.print(motor2.getOutput());
    Serial.println(F(", automatic stop in 300 ms."));
}

void serviceMotorPulseTimeout()
{
    if (motorPulseActive &&
        ((millis() - motorPulseStartMs) >= DiagnosticConfig::MOTOR_TEST_DURATION_MS))
    {
        stopAllMotors();
        Serial.println(F("[MOTOR] Pulse complete; both motors stopped."));
    }
}

void zeroBothEncoders()
{
    noInterrupts();
    encoderA.zeroCount();
    encoderB.zeroCount();
    interrupts();

    Serial.println(F("[ENCODER] Both counts zeroed."));
}

void clearLimitLatch()
{
    if (!allLimitsReleased())
    {
        Serial.println(F("[BLOCKED] At least one limit input is still active."));
        return;
    }

    limitStopLatched = false;
    Serial.println(F("[SAFETY] All switches released; limit latch cleared."));
}

void handleSerialCommand(char command)
{
    switch (command)
    {
        case '\r':
        case '\n':
        case ' ':
        case '\t':
            break;

        case 'h':
        case 'H':
        case '?':
            printHelp();
            break;

        case 'p':
        case 'P':
            printSnapshot();
            break;

        case 't':
        case 'T':
            telemetryEnabled = !telemetryEnabled;
            Serial.print(F("[TELEMETRY] "));
            Serial.println(telemetryEnabled ? F("ON") : F("OFF"));
            break;

        case 'z':
        case 'Z':
            zeroBothEncoders();
            break;

        case 'x':
        case 'X':
            stopAllMotors();
            Serial.println(F("[MOTOR] Manual stop; both motors are at zero."));
            break;

        case 'c':
        case 'C':
            clearLimitLatch();
            break;

        case '1':
            startMotorPulse(
                DiagnosticConfig::MOTOR_TEST_PWM,
                0,
                F("Motor 1 positive"));
            break;

        case '2':
            startMotorPulse(
                -DiagnosticConfig::MOTOR_TEST_PWM,
                0,
                F("Motor 1 negative"));
            break;

        case '3':
            startMotorPulse(
                0,
                DiagnosticConfig::MOTOR_TEST_PWM,
                F("Motor 2 positive"));
            break;

        case '4':
            startMotorPulse(
                0,
                -DiagnosticConfig::MOTOR_TEST_PWM,
                F("Motor 2 negative"));
            break;

        // Upper-case paired commands avoid colliding with the lower-case
        // help/telemetry/control commands.
        case 'R':
            startMotorPulse(
                DiagnosticConfig::MOTOR_TEST_PWM,
                DiagnosticConfig::MOTOR_TEST_PWM,
                F("candidate RIGHT (+,+)"));
            break;

        case 'L':
            startMotorPulse(
                -DiagnosticConfig::MOTOR_TEST_PWM,
                -DiagnosticConfig::MOTOR_TEST_PWM,
                F("candidate LEFT (-,-)"));
            break;

        case 'U':
            startMotorPulse(
                -DiagnosticConfig::MOTOR_TEST_PWM,
                DiagnosticConfig::MOTOR_TEST_PWM,
                F("candidate UP (-,+)"));
            break;

        case 'D':
            startMotorPulse(
                DiagnosticConfig::MOTOR_TEST_PWM,
                -DiagnosticConfig::MOTOR_TEST_PWM,
                F("candidate DOWN (+,-)"));
            break;

        default:
            Serial.print(F("[UNKNOWN] Command '"));
            Serial.print(command);
            Serial.println(F("'. Send h for help."));
            break;
    }
}

void attachLimitInterrupt(uint8_t pin, void (*handler)(), const char *name)
{
    const int interruptNumber = digitalPinToInterrupt(pin);

    if (interruptNumber == NOT_AN_INTERRUPT)
    {
        Serial.print(F("[ERROR] "));
        Serial.print(name);
        Serial.print(F(" pin D"));
        Serial.print(pin);
        Serial.println(F(" is not external-interrupt capable."));
        limitStopLatched = true;
        return;
    }

    attachInterrupt(interruptNumber, handler, RISING);
}

void setup()
{
    Serial.begin(SystemConfig::SERIAL_BAUD_RATE);

    motor1.begin();
    motor2.begin();

    for (uint8_t index = 0; index < LIMIT_COUNT; ++index)
    {
        limitSwitches[index]->begin();
        previousPressedState[index] = limitSwitches[index]->isPressed();
    }

    attachLimitInterrupt(
        PinConfig::LIMIT_SWITCH_LEFT_PIN,
        leftLimitISR,
        limitNames[LIMIT_LEFT]);

    attachLimitInterrupt(
        PinConfig::LIMIT_SWITCH_RIGHT_PIN,
        rightLimitISR,
        limitNames[LIMIT_RIGHT]);

    attachLimitInterrupt(
        PinConfig::LIMIT_SWITCH_BOTTOM_PIN,
        bottomLimitISR,
        limitNames[LIMIT_BOTTOM]);

    attachLimitInterrupt(
        PinConfig::LIMIT_SWITCH_TOP_PIN,
        topLimitISR,
        limitNames[LIMIT_TOP]);

    if (anyRawLimitPressed() || anyDebouncedLimitPressed())
    {
        limitStopLatched = true;
    }

    Serial.println();
    Serial.println(F("ME306 combined hardware diagnostic ready."));
    Serial.println(F("Motor outputs are zero and never start automatically."));
    Serial.println(F("Serial: 115200 baud. Send h for commands."));
    printSnapshot();
}

void loop()
{
    // Safety-related work is serviced before serial motor commands.
    updateAndReportLimitSwitches();
    serviceRawLimitInterrupts();
    serviceMotorPulseTimeout();

    while (Serial.available() > 0)
    {
        handleSerialCommand(static_cast<char>(Serial.read()));
    }

    const unsigned long now = millis();
    if (telemetryEnabled &&
        ((now - lastTelemetryMs) >= DiagnosticConfig::TELEMETRY_INTERVAL_MS))
    {
        lastTelemetryMs = now;
        printSnapshot();
    }
}