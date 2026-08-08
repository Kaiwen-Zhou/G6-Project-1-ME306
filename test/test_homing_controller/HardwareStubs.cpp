#include <stdint.h>

#include "hardware/Encoder.h"
#include "hardware/MotorDriver.h"

namespace
{
constexpr uint8_t PIN_COUNT = 80;

unsigned long fakeMillis = 0;
unsigned long fakeMicros = 0;

uint8_t fakePinStates[PIN_COUNT] = {};

int32_t fakeCountA = 0;
int32_t fakeCountB = 0;

unsigned int countPairReadCount = 0;

Encoder* encoderAObject = nullptr;
Encoder* encoderBObject = nullptr;
}

namespace TestHardware
{
void reset()
{
    fakeMillis = 0;
    fakeMicros = 0;

    for (uint8_t index = 0; index < PIN_COUNT; ++index)
    {
        fakePinStates[index] = LOW;
    }

    fakeCountA = 0;
    fakeCountB = 0;
    countPairReadCount = 0;

    encoderAObject = nullptr;
    encoderBObject = nullptr;
}

void setMillis(unsigned long value)
{
    fakeMillis = value;
    fakeMicros = value * 1000UL;
}

void advanceMillis(unsigned long change)
{
    fakeMillis += change;
    fakeMicros += change * 1000UL;
}

void setEncoderCounts(
    int32_t countA,
    int32_t countB)
{
    fakeCountA = countA;
    fakeCountB = countB;
}

void setSwitchPressed(
    uint8_t pin,
    bool pressed)
{
    if (pin < PIN_COUNT)
    {
        fakePinStates[pin] = pressed ? HIGH : LOW;
    }
}

unsigned int getCountPairReadCount()
{
    return countPairReadCount;
}
}

unsigned long millis()
{
    return fakeMillis;
}

unsigned long micros()
{
    return fakeMicros;
}

void pinMode(uint8_t, uint8_t)
{
}

int digitalRead(uint8_t pin)
{
    if (pin >= PIN_COUNT)
    {
        return LOW;
    }

    return fakePinStates[pin];
}

void digitalWrite(uint8_t, uint8_t)
{
}

void analogWrite(uint8_t, int)
{
}

int digitalPinToInterrupt(uint8_t pin)
{
    return static_cast<int>(pin);
}

void attachInterrupt(int, void (*)(), int)
{
}

// Native Encoder replacements.

Encoder::Encoder(bool isEncoderA)
    : pinA_(0),
      pinB_(0)
{
    if (isEncoderA)
    {
        encoderAObject = this;
    }
    else
    {
        encoderBObject = this;
    }
}

void Encoder::update()
{
}

void Encoder::zeroCount()
{
    if (this == encoderAObject)
    {
        fakeCountA = 0;
    }
    else if (this == encoderBObject)
    {
        fakeCountB = 0;
    }
}

int32_t Encoder::getCount()
{
    if (this == encoderAObject)
    {
        return fakeCountA;
    }

    if (this == encoderBObject)
    {
        return fakeCountB;
    }

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
        fakeCountB};
}

void Encoder::zeroCountPair(
    Encoder& encoderA,
    Encoder& encoderB)
{
    (void)encoderA;
    (void)encoderB;

    fakeCountA = 0;
    fakeCountB = 0;
}

bool Encoder::getDirection() const
{
    return true;
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
