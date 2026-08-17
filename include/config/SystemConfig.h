#pragma once

#include <Arduino.h>

#include "communication/GCodeCommand.h"

namespace SystemConfig {
constexpr unsigned long SERIAL_BAUD_RATE = 115200UL;

// Select how G01 X/Y values are interpreted before each command is converted
// to the relative displacement used by the trajectory and control layers.
// ABSOLUTE: G01 X10 Y20 moves to machine coordinate (10, 20).
// RELATIVE: G01 X10 Y20 moves +10 mm in X and +20 mm in Y.
constexpr plotter::GCodePositioningMode GCODE_POSITIONING_MODE = plotter::GCodePositioningMode::ABSOLUTE;

// The junction board supplies the external pull-down.
// Active-high: HIGH = pressed, LOW = released.
constexpr uint8_t LIMIT_SWITCH_INPUT_MODE = INPUT;
constexpr unsigned long LIMIT_SWITCH_DEBOUNCE_MS = 20UL;

constexpr uint8_t MOTOR_DEFAULT_OUTPUT_LIMIT = 255;

constexpr bool MOTOR_CLOCKWISE_DIRECTION = true;

constexpr float MOTOR_OUTPUT_DIAMETER_MM = 14.0f;

constexpr int32_t ENCODER_COUNTS_PER_OUTPUT_REVOLUTION = 4128L;

constexpr float MOTOR_A_MM_PER_COUNT =
    PI * MOTOR_OUTPUT_DIAMETER_MM / static_cast<float>(ENCODER_COUNTS_PER_OUTPUT_REVOLUTION);

constexpr float MOTOR_B_MM_PER_COUNT =
    PI * MOTOR_OUTPUT_DIAMETER_MM / static_cast<float>(ENCODER_COUNTS_PER_OUTPUT_REVOLUTION);

// +1: positive raw count is positive A/B displacement.
// -1: positive raw count is negative A/B displacement.
constexpr int8_t MOTOR_A_COORDINATE_SIGN = 1;
constexpr int8_t MOTOR_B_COORDINATE_SIGN = 1;

constexpr unsigned long MOTION_CONTROL_PERIOD_MICROS = 5000UL;
constexpr unsigned long MOVE_SETTLE_TIME_MICROS = 50000UL;

// Fixed Cartesian soft limits measured from the released origin positions.
// Set both to positive measured values before enabling G01 motion. Keeping a
// value at zero deliberately prevents the G-code controller from loading an
// unsafe guessed workspace.
constexpr float MACHINE_X_TRAVEL_MM = 250.0f; ////////////// TODO: measured X min-to-max travel
constexpr float MACHINE_Y_TRAVEL_MM = 250.0f; ////////////// TODO: measured Y min-to-max travel

// Position allowance used when deciding whether a pressed limit switch is
// physically consistent with the carriage being at that boundary. Tune this
// for switch overtravel, encoder error, and stopping distance.
constexpr float LIMIT_BOUNDARY_TOLERANCE_MM = 1.0f;

// Debounced limit states are checked against the current FSM state at this
// interval outside HOMING. LimitSwitch::update() itself still runs every loop.
constexpr unsigned long LIMIT_SAFETY_CHECK_INTERVAL_MS = 10UL;

// Initial origin-homing values. These PWM values and distances must be tuned
// on the real mechanism before full-speed testing.
constexpr uint8_t HOMING_COARSE_APPROACH_PWM = 125;
constexpr uint8_t HOMING_BACKOFF_PWM = 100;
constexpr uint8_t HOMING_FINE_APPROACH_PWM = 50;
constexpr uint8_t HOMING_FINAL_RELEASE_PWM = 30;

constexpr float HOMING_BACKOFF_DISTANCE_MM = 10.0f;

constexpr unsigned long HOMING_CONTACT_PAUSE_MS = 250UL;
constexpr unsigned long HOMING_FINE_CONTACT_PAUSE_MS = 250UL;
constexpr unsigned long HOMING_SEARCH_TIMEOUT_MS = 30000UL;
constexpr unsigned long HOMING_BACKOFF_TIMEOUT_MS = 5000UL;
constexpr unsigned long HOMING_FINAL_RELEASE_TIMEOUT_MS = 5000UL;
constexpr unsigned long HOMING_OVERALL_TIMEOUT_MS = 180000UL;

// true: stop on the ISR edge, then wait for software debounce confirmation.
// false: stop on the ISR edge and accept it immediately during homing.
constexpr bool HOMING_LIMIT_DEBOUNCE_ENABLED = true;
// Temporary values until hardware direction testing.
constexpr bool MOTOR_1_DIRECTION_INVERTED = false;
constexpr bool MOTOR_2_DIRECTION_INVERTED = false;
} // namespace SystemConfig
