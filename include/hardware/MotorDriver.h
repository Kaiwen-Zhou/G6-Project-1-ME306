#pragma once

#include <Arduino.h>
#include <stdint.h>

/**
 * Low-level signed PWM control for one bidirectional DC motor.
 *
 * Configure the direction pin, PWM pin, optional direction inversion, and PWM
 * limit in the constructor. After begin(), setOutput() accepts a signed command:
 * its sign selects direction and its magnitude selects duty cycle. stop() sets
 * PWM to zero, which produces coast behaviour on the current L298N hardware.
 */
class MotorDriver {
    public:
        MotorDriver(uint8_t directionPin, uint8_t pwmPin, bool directionInverted = false, uint8_t outputLimit = 255);

        void begin();

        // Apply a signed motor command.
        // Positive and negative values select opposite directions.
        // Magnitude determines PWM duty cycle.
        void setOutput(int16_t command);

        // Immediately set PWM to zero.
        // On the current shield, stopping uses coast behaviour.
        void stop();

        // Change the maximum allowed PWM magnitude.
        void setOutputLimit(uint8_t outputLimit);

        // Return the signed output actually applied after limiting.
        int16_t getOutput() const;

        uint8_t getOutputLimit() const;

    private:
        uint8_t directionPin_;
        uint8_t pwmPin_;
        bool directionInverted_;
        uint8_t outputLimit_;
        int16_t currentOutput_;
};
