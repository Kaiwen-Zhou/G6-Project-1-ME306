#include <stdint.h>

#include "hardware/Encoder.h"
#include "hardware/MotorDriver.h"

namespace
{
unsigned long fakeMicros = 0;

int32_t fakeCountA = 0;
int32_t fakeCountB = 0;

unsigned int countPairReadCount = 0;
}

namespace TestHardware
{
void reset()
{
    fakeMicros = 0;

    fakeCountA = 0;
    fakeCountB = 0;

    countPairReadCount = 0;
}

void setMicros(unsigned long value)
{
    fakeMicros = value;
}

void setEncoderCounts(
    int32_t countA,
    int32_t countB)
{
    fakeCountA = countA;
    fakeCountB = countB;
}

unsigned int getCountPairReadCount()
{
    return countPairReadCount;
}
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

// Native Encoder replacements.

Encoder::Encoder(bool isEncoderA)
    : pinA_(0),
      pinB_(0)
{
    (void)isEncoderA;
}

int32_t Encoder::getCount()
{
    return 0;
}

Encoder::CountPair Encoder::getCountPair(
    const Encoder& encoderA,
    const Encoder& encoderB)
{
    (void)encoderA;
    (void)encoderB;

    ++countPairReadCount;

    return CountPair{
        fakeCountA,
        fakeCountB
    };
}

// Native MotorDriver replacements.

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