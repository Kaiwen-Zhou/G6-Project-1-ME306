#pragma once

#include <Arduino.h>

namespace SystemConfig
{
constexpr unsigned long SERIAL_BAUD_RATE = 115200UL;

// The junction board supplies the external pull-down.
// Active-high: HIGH = pressed, LOW = released.
constexpr uint8_t LIMIT_SWITCH_INPUT_MODE = INPUT;
constexpr unsigned long LIMIT_SWITCH_DEBOUNCE_MS = 20UL;

constexpr uint8_t MOTOR_DEFAULT_OUTPUT_LIMIT = 255;

constexpr bool MOTOR_CLOCKWISE_DIRECTION = true;

constexpr float MOTOR_OUTPUT_DIAMETER_MM = 14.0f;

constexpr int32_t ENCODER_COUNTS_PER_OUTPUT_REVOLUTION = 4128L;

constexpr float MOTOR_A_MM_PER_COUNT =
    PI * MOTOR_OUTPUT_DIAMETER_MM /
    static_cast<float>(ENCODER_COUNTS_PER_OUTPUT_REVOLUTION);

constexpr float MOTOR_B_MM_PER_COUNT =
    PI * MOTOR_OUTPUT_DIAMETER_MM /
    static_cast<float>(ENCODER_COUNTS_PER_OUTPUT_REVOLUTION);

// +1: positive raw count is positive A/B displacement.
// -1: positive raw count is negative A/B displacement.
constexpr int8_t MOTOR_A_COORDINATE_SIGN = 1;
constexpr int8_t MOTOR_B_COORDINATE_SIGN = 1;

constexpr unsigned long MOTION_CONTROL_PERIOD_MICROS = 5000UL;
constexpr unsigned long MOVE_SETTLE_TIME_MICROS = 50000UL;

// Initial homing values. These PWM values and distances must be tuned on the
// real mechanism before full-speed testing.
constexpr uint8_t HOMING_COARSE_APPROACH_PWM = 60;
constexpr uint8_t HOMING_BACKOFF_PWM = 40;
constexpr uint8_t HOMING_FINE_APPROACH_PWM = 20;
constexpr uint8_t HOMING_FINAL_RELEASE_PWM = 10;

constexpr float HOMING_BACKOFF_DISTANCE_MM = 2.0f;

constexpr unsigned long HOMING_CONTACT_PAUSE_MS = 250UL;
constexpr unsigned long HOMING_FINE_CONTACT_PAUSE_MS = 250UL;
constexpr unsigned long HOMING_SEARCH_TIMEOUT_MS = 30000UL;
constexpr unsigned long HOMING_BACKOFF_TIMEOUT_MS = 5000UL;
constexpr unsigned long HOMING_FINAL_RELEASE_TIMEOUT_MS = 5000UL;
constexpr unsigned long HOMING_OVERALL_TIMEOUT_MS = 180000UL;

// Temporary values until hardware direction testing.
constexpr bool MOTOR_1_DIRECTION_INVERTED = false;
constexpr bool MOTOR_2_DIRECTION_INVERTED = false;
}
