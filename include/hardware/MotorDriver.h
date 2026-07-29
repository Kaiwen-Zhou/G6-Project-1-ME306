#pragma once
/*
 * MotorDriver
 *
 * Handles the low-level control of one bidirectional DC motor using a
 * direction pin and a PWM pin. It does not calculate PID output, read encoder
 * feedback, or decide the required motion.
 *
 * Call begin() once during initialisation. setOutput() accepts a signed command:
 * the sign selects the direction and the magnitude sets the PWM duty cycle.
 * stop() immediately sets PWM to zero. The current applied output and output
 * limit can also be read or changed through the public interfaces.
 *
 * Direction pin, PWM pin, direction inversion, and output limit are configured
 * through the constructor. The PWM pin must support hardware PWM.
 * directionInverted allows the electrical direction to be reversed without
 * changing the sign convention used by higher-level control modules.
 *
 * On the current DFRobot L298N shield, stop() disables the PWM output and
 * therefore provides coast behaviour rather than active braking. The output
 * limit only limits the PWM command; it does not provide current or stall
 * protection.
 *
 * The module is intended to be used by AxisController or by standalone motor
 * test code. Additional stopping modes or diagnostic interfaces may be added
 * later if supported or required.
 */

#include <Arduino.h>
#include <stdint.h>

class MotorDriver {
 public:
    MotorDriver(uint8_t directionPin, 
                uint8_t pwmPin, 
                bool directionInverted = false, 
                uint8_t outputLimit = 255);


    void begin();

    // Apply a signed motor command.
    // Positive and negative values select opposite directions.
    // Magnitude determines PWM duty cycle.
    void setOutput(int16_t command);

    // Immediately set PWM to zero.
    // On the current shield, stopping uses coast behaviour.
    void stop();

    // Change the max allowed PWM magnitude
    void setOutputLimit(uint8_t outputLimit);

    // Return the signed output actually applied after limiting
    int16_t getOutput() const;

    uint8_t getOutputLimit() const;

 private:
    uint8_t directionPin_;
    uint8_t pwmPin_;
    bool directionInverted_;
    uint8_t outputLimit_;
    int16_t currentOutput_;

};
