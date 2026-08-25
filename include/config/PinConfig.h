#pragma once

#include <Arduino.h>

/**
 * Centralised Arduino Mega pin assignments for motors, encoders, and limits.
 *
 * Higher-level modules should use these constants rather than numeric pins.
 * Encoder pin changes also require matching AVR pin-change interrupt register
 * updates in Encoder.cpp.
 */
namespace PinConfig {
// DFRobot L298N motor shield - PWM mode
constexpr uint8_t MOTOR_1_DIRECTION_PIN = 4;
constexpr uint8_t MOTOR_1_PWM_PIN = 5;

constexpr uint8_t MOTOR_2_DIRECTION_PIN = 7;
constexpr uint8_t MOTOR_2_PWM_PIN = 6;

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

// Active-high limit switches.
// Each switch has its own external-interrupt pin.
constexpr uint8_t LIMIT_SWITCH_LEFT_PIN = 2;
constexpr uint8_t LIMIT_SWITCH_RIGHT_PIN = 3;
constexpr uint8_t LIMIT_SWITCH_BOTTOM_PIN = 18;
constexpr uint8_t LIMIT_SWITCH_TOP_PIN = 19;
} // namespace PinConfig
