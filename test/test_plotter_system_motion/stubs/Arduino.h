#pragma once

#include <stdint.h>

constexpr uint8_t LOW = 0;
constexpr uint8_t HIGH = 1;
constexpr uint8_t INPUT = 0;
constexpr uint8_t OUTPUT = 1;
constexpr uint8_t INPUT_PULLUP = 2;
constexpr uint8_t FALLING = 2;
constexpr uint8_t RISING = 3;
constexpr float PI = 3.14159265358979323846f;

unsigned long millis();
unsigned long micros();

void pinMode(uint8_t pin, uint8_t mode);
int digitalRead(uint8_t pin);
void digitalWrite(uint8_t pin, uint8_t value);
void analogWrite(uint8_t pin, int value);

int digitalPinToInterrupt(uint8_t pin);
void attachInterrupt(
    int interruptNumber,
    void (*isr)(),
    int mode);
