#pragma once

#include <atomic>
#include <stdint.h>
#include <math.h>
#include <Arduino.h>

class Converter {
 public:
    Converter(float deltaX = 0, float deltaY = 0, int32_t deltaA = 0, int32_t deltaB = 0);

    void setDeltaX(float deltaX) {deltaX_ = deltaX;}
    void setDeltaY(float deltaY) {deltaY_ = deltaY;}
    void setDeltaA(int32_t deltaA) {deltaA_ = deltaA;}
    void setDeltaB(int32_t deltaB) {deltaB_ = deltaB;}

    float getDeltaX() const;
    float getDeltaY() const;
    int32_t getDeltaA() const;
    int32_t getDeltaB() const;

    int32_t convertToXY(int32_t deltaA, int32_t deltaB) const {
        // Conversion logic from motor-space (A/B) to Cartesian (X/Y)
    }



 private:
    std::atomic<float> deltaX_;
    std::atomic<float> deltaY_;
    std::atomic<int32_t> deltaA_;
    std::atomic<int32_t> deltaB_;
}; 