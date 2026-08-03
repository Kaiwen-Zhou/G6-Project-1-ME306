#pragma once

#include <Arduino.h>

namespace SystemConfig
{
    constexpr unsigned long SERIAL_BAUD_RATE = 115200UL;

    constexpr uint8_t LIMIT_SWITCH_INPUT_MODE =
        INPUT_PULLUP;

    constexpr unsigned long LIMIT_SWITCH_DEBOUNCE_MS =
        20UL;

    constexpr uint8_t MOTOR_DEFAULT_OUTPUT_LIMIT =
        255;

    constexpr bool MOTOR_CLOCKWISE_DIRECTION = true; // True = clockwise, False = counter-clockwise

    constexpr uint8_t MOTOR_OUTPUT_DIAMETER = 14; // mm // value measured with ruler

    constexpr float MOTOR_OUTPUT_DISTANCE_PER_COUNT = (3.14159 * MOTOR_OUTPUT_DIAMETER) / 4128; // mm/count = 0.01065
    // Calculations //
    // 48 counts per revolution of motor shaft
    // Gear ratio 172:1
    // Only counting change in A output so only counting half the actual counts
    // 48*172*0.5 = 4128 counts per output revolution
    // (2*pi*radius)/(4128 counts) = distance per count

    // Temporary values until hardware direction testing.
    constexpr bool MOTOR_1_DIRECTION_INVERTED = false;
    constexpr bool MOTOR_2_DIRECTION_INVERTED = false;
}