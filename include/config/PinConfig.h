#pragma once

#include <Arduino.h>

namespace PinConfig
{
    // DFRobot L298N motor shield - PWM mode
    constexpr uint8_t MOTOR_1_DIRECTION_PIN = 4;
    constexpr uint8_t MOTOR_1_PWM_PIN       = 5;

    constexpr uint8_t MOTOR_2_DIRECTION_PIN = 7;
    constexpr uint8_t MOTOR_2_PWM_PIN       = 6;

    // NOTE:
    // Encoder pins can not be changed directly here, as related interrupts are set up in the Encoder class constructor. 
    // Changing these pins would require modifying the interrupt setup as well.
    // Encoder A
    // D68 = A14 / PCINT22
    // D69 = A15 / PCINT23
    constexpr uint8_t ENCODER_A_PIN_A = 68;
    constexpr uint8_t ENCODER_A_PIN_B = 69;

    // Encoder B
    // D52 = PCINT1
    // D53 = PCINT0
    constexpr uint8_t ENCODER_B_PIN_A = 52;
    constexpr uint8_t ENCODER_B_PIN_B = 53;

    // Active-low limit switches
    // Each switch has its own external-interrupt pin.
    constexpr uint8_t LIMIT_SWITCH_LEFT_PIN   = 2;
    constexpr uint8_t LIMIT_SWITCH_RIGHT_PIN  = 3;
    constexpr uint8_t LIMIT_SWITCH_BOTTOM_PIN = 18;
    constexpr uint8_t LIMIT_SWITCH_TOP_PIN    = 19;
}