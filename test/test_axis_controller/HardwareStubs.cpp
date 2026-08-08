#include <stdint.h>

#include "hardware/Encoder.h"
#include "hardware/MotorDriver.h"

namespace
{
unsigned long fakeMicros = 0;
}

unsigned long micros()
{
    return fakeMicros;
}

void pinMode(uint8_t, uint8_t)
{
}

void digitalWrite(uint8_t, uint8_t)
{
}

void analogWrite(uint8_t, int)
{
}

// Only needed to link the temporary compatibility path.
int32_t Encoder::getCount()
{
    return 0;
}

// Native replacement for MotorDriver hardware operations.
MotorDriver::MotorDriver(
    uint8_t directionPin,
    uint8_t pwmPin,
    bool directionInverted,
    uint8_t outputLimit)
    : directionPin_(directionPin),
      pwmPin_(pwmPin),
      directionInverted_(directionInverted),
      outputLimit_(outputLimit),
      currentOutput_(0)
{
}

void MotorDriver::begin()
{
    currentOutput_ = 0;
}

void MotorDriver::setOutput(int16_t command)
{
    const int16_t limit =
        static_cast<int16_t>(outputLimit_);

    if (command > limit)
    {
        command = limit;
    }
    else if (command < -limit)
    {
        command = -limit;
    }

    currentOutput_ = command;
}

void MotorDriver::stop()
{
    currentOutput_ = 0;
}

void MotorDriver::setOutputLimit(
    uint8_t outputLimit)
{
    outputLimit_ = outputLimit;
}

int16_t MotorDriver::getOutput() const
{
    return currentOutput_;
}

uint8_t MotorDriver::getOutputLimit() const
{
    return outputLimit_;
}