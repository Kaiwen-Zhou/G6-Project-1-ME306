#pragma once

#include <Arduino.h>

namespace SystemConfig
{
    constexpr unsigned long SERIAL_BAUD_RATE = 115200UL;

    constexpr uint8_t LIMIT_SWITCH_INPUT_MODE =
        INPUT_PULLUP;

    constexpr unsigned long LIMIT_SWITCH_DEBOUNCE_MS =
        200UL;

    constexpr uint8_t MOTOR_DEFAULT_OUTPUT_LIMIT =
        255;

    constexpr bool MOTOR_CLOCKWISE_DIRECTION = true; // True = clockwise, False = counter-clockwise

    constexpr float MOTOR_OUTPUT_DIAMETER_MM = 14.0f;

    constexpr int32_t ENCODER_COUNTS_PER_OUTPUT_REVOLUTION = 4128L;

    constexpr float MOTOR_A_MM_PER_COUNT =
        PI * MOTOR_OUTPUT_DIAMETER_MM / static_cast<float>(ENCODER_COUNTS_PER_OUTPUT_REVOLUTION);

    constexpr float MOTOR_B_MM_PER_COUNT =
        PI * MOTOR_OUTPUT_DIAMETER_MM / static_cast<float>(ENCODER_COUNTS_PER_OUTPUT_REVOLUTION);
    // Calculations //
    // 48 counts per revolution of motor shaft
    // Gear ratio 172:1
    // Only counting change in A output so only counting half the actual counts
    // 48*172*0.5 = 4128 counts per output revolution
    // (2*pi*radius)/(4128 counts) = distance per count

    // +1: positive raw encoder count is positive A/B displacement
    // -1: positive raw encoder count is negative A/B displacement
    constexpr int8_t MOTOR_A_COORDINATE_SIGN = 1;
    constexpr int8_t MOTOR_B_COORDINATE_SIGN = 1;


    constexpr unsigned long MOTION_CONTROL_PERIOD_MICROS = 5000UL;  // 200 Hz

    constexpr unsigned long MOVE_SETTLE_TIME_MICROS = 50000UL;
    // Temporary values until hardware direction testing.
    constexpr bool MOTOR_1_DIRECTION_INVERTED = false;
    constexpr bool MOTOR_2_DIRECTION_INVERTED = false;
}
