#ifndef PIN_CONFIG_H
#define PIN_CONFIG_H

#include <Arduino.h>

namespace PinConfig
{
    //Limit switch pin
    constexpr uint8_t X_MIN_SWITCH = 22;
    constexpr uint8_t X_MAX_SWITCH = 23;

    constexpr uint8_t Y_MIN_SWITCH = 24;
    constexpr uint8_t Y_MAX_SWITCH = 25;
}

#endif // PIN_CONFIG_H