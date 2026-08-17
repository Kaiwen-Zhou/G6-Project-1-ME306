#include "hardware/MotorDriver.h"
#include "config/PinConfig.h"

// Constructor
MotorDriver::MotorDriver(uint8_t directionPin, uint8_t pwmPin, bool directionInverted, uint8_t outputLimit)
    : directionPin_(directionPin), pwmPin_(pwmPin), directionInverted_(directionInverted), outputLimit_(outputLimit),
      currentOutput_(0) {
}

// Initialise the motor driver hardware
void MotorDriver::begin() {
    pinMode(directionPin_, OUTPUT);
    pinMode(pwmPin_, OUTPUT);

    // set a known initial direction level
    digitalWrite(directionPin_, LOW);

    stop();
}

// Apply a singed motor command
void MotorDriver::setOutput(int16_t command) {
    int16_t limit = static_cast<int16_t>(outputLimit_);

    // limit the command to the configured output range
    if (command > limit) {
        command = limit;
    } else if (command < -limit) {
        command = -limit;
    }

    // a zero command uses the defined coast stop behaviour
    if (command == 0) {
        stop();
        return;
    }

    // disable PWM briefly before changeing direction
    if ((currentOutput_ > 0 && command < 0) || (currentOutput_ < 0 && command > 0)) {
        analogWrite(pwmPin_, 0);
    }

    // positive command normally produces HIGH on the direction pin
    // directionInverted_ reverses this mapping when required
    bool positiveCommand = (command > 0);
    bool directionLevel = positiveCommand != directionInverted_;

    digitalWrite(directionPin_, directionLevel ? HIGH : LOW);

    // PWM output uses only the command magnitude
    uint8_t pwmValue;

    if (positiveCommand) {
        pwmValue = static_cast<uint8_t>(command);
    } else {
        pwmValue = static_cast<uint8_t>(-command);
    }

    analogWrite(pwmPin_, pwmValue);

    // store the logical signed command actually applied
    currentOutput_ = command;
}

// Immediately stop motor output using coast behaviour
void MotorDriver::stop() {
    analogWrite(pwmPin_, 0);
    currentOutput_ = 0;
}

// change the max allowed PWM magnitude
void MotorDriver::setOutputLimit(uint8_t outputLimit) {
    outputLimit_ = outputLimit;

    // Immediately apply the new limit to the current output
    setOutput(currentOutput_);
}

// return the signed output actually applied after limiting
int16_t MotorDriver::getOutput() const {
    return currentOutput_;
}

uint8_t MotorDriver::getOutputLimit() const {
    return outputLimit_;
}