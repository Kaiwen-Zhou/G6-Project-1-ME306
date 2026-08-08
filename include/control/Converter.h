#pragma once

#include <Arduino.h>
#include "hardware/Encoder.h"

class Converter {
public:
    struct CartesianDelta {float deltaX; float deltaY;};

    Converter(Encoder& encoderA, Encoder& encoderB);

    // Call after encoders have been initialised or zeroed
    void begin();

    // Call repeatedly from loop/control update, not from an ISR
    void update();

    float getDeltaX() const;
    float getDeltaY() const;

    int32_t getDeltaA() const;
    int32_t getDeltaB() const;

private:
    CartesianDelta convertToXY(int32_t deltaA, int32_t deltaB) const;

    Encoder& encoderA_;
    Encoder& encoderB_;

    int32_t previousCountA_;
    int32_t previousCountB_;

    int32_t deltaA_;
    int32_t deltaB_;

    float deltaX_;
    float deltaY_;
};