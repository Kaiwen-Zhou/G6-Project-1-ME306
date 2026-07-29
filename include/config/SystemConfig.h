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

    // Temporary values until hardware direction testing.
    constexpr bool MOTOR_1_DIRECTION_INVERTED = false;
    constexpr bool MOTOR_2_DIRECTION_INVERTED = false;
}