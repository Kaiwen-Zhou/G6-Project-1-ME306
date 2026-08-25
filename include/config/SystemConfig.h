#pragma once

#include <Arduino.h>

#include "communication/GCodeCommand.h"

/**
 * Compile-time machine geometry, motion tuning, homing, and safety settings.
 *
 * Application modules consume these constants directly. Values describing the
 * physical mechanism or controller tuning should be changed only after the
 * corresponding hardware behaviour has been measured and verified.
 */
namespace SystemConfig {
constexpr unsigned long SERIAL_BAUD_RATE = 115200UL;

// Select how G01 X/Y values are interpreted before each command is converted
// to the relative displacement used by the trajectory and control layers.
// ABSOLUTE: G01 X10 Y20 moves to machine coordinate (10, 20).
// RELATIVE: G01 X10 Y20 moves +10 mm in X and +20 mm in Y.
constexpr plotter::GCodePositioningMode GCODE_POSITIONING_MODE = plotter::GCodePositioningMode::RELATIVE;

// The junction board supplies the external pull-down.
// Active-high: HIGH = pressed, LOW = released.
constexpr uint8_t LIMIT_SWITCH_INPUT_MODE = INPUT;
constexpr unsigned long LIMIT_SWITCH_DEBOUNCE_MS = 20UL;

constexpr uint8_t MOTOR_DEFAULT_OUTPUT_LIMIT = 255;

constexpr bool MOTOR_CLOCKWISE_DIRECTION = true;

constexpr float MOTOR_OUTPUT_DIAMETER_MM = 14.18f;

constexpr int32_t ENCODER_COUNTS_PER_OUTPUT_REVOLUTION = 4128L;

constexpr float MOTOR_A_MM_PER_COUNT =
    PI * MOTOR_OUTPUT_DIAMETER_MM / static_cast<float>(ENCODER_COUNTS_PER_OUTPUT_REVOLUTION);

constexpr float MOTOR_B_MM_PER_COUNT =
    PI * MOTOR_OUTPUT_DIAMETER_MM / static_cast<float>(ENCODER_COUNTS_PER_OUTPUT_REVOLUTION);

// +1: positive raw count is positive A/B displacement.
// -1: positive raw count is negative A/B displacement.
constexpr int8_t MOTOR_A_COORDINATE_SIGN = -1;
constexpr int8_t MOTOR_B_COORDINATE_SIGN = -1;

constexpr unsigned long MOTION_CONTROL_PERIOD_MICROS = 5000UL;
constexpr unsigned long MOVE_SETTLE_TIME_MICROS = 50000UL;

// Static-friction compensation added in the commanded movement direction.
constexpr float MOTION_BASE_PWM = 70.0f;

// Begin reducing MOTION_BASE_PWM when this percentage of the planned path
// remains. The base PWM then decreases linearly toward zero at the destination.
constexpr float MOTION_BASE_PWM_TAPER_START_REMAINING_PERCENT = 20.0f;

// Near the destination, retain at least this much static-friction
// compensation while outside position tolerance. It is used both for the
// final approach and for a requested reverse braking/correction direction.
constexpr float MOTION_ENDPOINT_MINIMUM_BASE_PWM = 40.0f;

// In the last part of a trajectory, stop rebuilding integral in the original
// movement direction and gently bleed any stored integral that still helps
// that direction. At remainingDistanceFraction == 0, normal integral action
// resumes for final settling and overshoot recovery.
constexpr float MOTION_INTEGRAL_BLEED_START_REMAINING_PERCENT = 20.0f;

// Integral-output units (PWM contribution) removed per second while the above
// endpoint policy is active. Start conservatively; increase if stale integral
// is still significant at the destination.
constexpr float MOTION_STALE_INTEGRAL_BLEED_RATE_PWM_PER_SECOND = 10.0f;

// Maximum PWM permitted opposite to the commanded movement direction.
constexpr float MOTION_MAXIMUM_REVERSE_CORRECTION_PWM = 120.0f;

// Fixed Cartesian soft limits measured from the released origin positions.
// Set both to positive measured values before enabling G01 motion. Keeping a
// value at zero deliberately prevents the G-code controller from loading an
// unsafe guessed workspace.
constexpr float MACHINE_X_TRAVEL_MM = 211.0f;
constexpr float MACHINE_Y_TRAVEL_MM = 135.0f;

// Position allowance used when deciding whether a pressed limit switch is
// physically consistent with the carriage being at that boundary. Tune this
// for switch overtravel, encoder error, and stopping distance.
constexpr float LIMIT_BOUNDARY_TOLERANCE_MM = 10.0f;

// Debounced limit states are checked against the current FSM state at this
// interval outside HOMING. LimitSwitch::update() itself still runs every loop.
constexpr unsigned long LIMIT_SAFETY_CHECK_INTERVAL_MS = 10UL;

// Initial origin-homing values. These PWM values and distances must be tuned
// on the real mechanism before full-speed testing.
constexpr uint8_t HOMING_COARSE_APPROACH_PWM = 160;
constexpr uint8_t HOMING_BACKOFF_PWM = 85;
constexpr uint8_t HOMING_FINE_APPROACH_PWM = 80;
constexpr uint8_t HOMING_FINAL_RELEASE_PWM = 65;

constexpr float HOMING_BACKOFF_DISTANCE_MM = 1.0f;

// Keep the axis perpendicular to the current homing direction at the position
// recorded when the current X or Y homing target starts. The same reference is
// retained through coarse approach, backoff, fine approach, and final release.
constexpr bool HOMING_STRAIGHTNESS_CORRECTION_ENABLED = true;
constexpr float HOMING_STRAIGHTNESS_KP_PWM_PER_MM = 5.0f;
constexpr uint8_t HOMING_STRAIGHTNESS_MAXIMUM_CORRECTION_PWM = 10;
constexpr float HOMING_STRAIGHTNESS_DEADBAND_MM = 0.5f;

// Ignore only the LEFT/X_MIN limit while Y is being homed. X homing and all
// other limit switches retain their normal homing safety behaviour.
constexpr bool HOMING_IGNORE_X_MIN_DURING_Y = true;

constexpr unsigned long HOMING_CONTACT_PAUSE_MS = 500UL;
constexpr unsigned long HOMING_FINE_CONTACT_PAUSE_MS = 500UL;
constexpr unsigned long HOMING_SEARCH_TIMEOUT_MS = 30000UL;
constexpr unsigned long HOMING_BACKOFF_TIMEOUT_MS = 30000UL;
constexpr unsigned long HOMING_FINAL_RELEASE_TIMEOUT_MS = 5000UL;
constexpr unsigned long HOMING_OVERALL_TIMEOUT_MS = 180000UL;

// true: stop on the ISR edge, then wait for software debounce confirmation.
// false: stop on the ISR edge and accept it immediately during homing.
constexpr bool HOMING_LIMIT_DEBOUNCE_ENABLED = false;
// Electrical direction inversion for the current motor wiring.
constexpr bool MOTOR_1_DIRECTION_INVERTED = false;
constexpr bool MOTOR_2_DIRECTION_INVERTED = true;
} // namespace SystemConfig
