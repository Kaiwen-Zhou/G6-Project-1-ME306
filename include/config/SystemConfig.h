#ifndef SYSTEM_CONFIG_H
#define SYSTEM_CONFIG_H

#include <Arduino.h>

namespace SystemConfig
{

    // Limit Switch
    constexpr uint8_t LIMIT_SWITCH_MODE = INPUT_PULLUP;

    constexpr unsigned long DEBOUNCE_TIME_MS = 20;
}

#endif // SYSTEM_CONFIG_H