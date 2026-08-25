#include <Arduino.h>
#include <util/atomic.h>

#include "hardware/Encoder.h"
#include "config/PinConfig.h"
#include "config/SystemConfig.h"

const int encoderApinA = PinConfig::ENCODER_A_PIN_A;
const int encoderApinB = PinConfig::ENCODER_A_PIN_B;
const int encoderBpinA = PinConfig::ENCODER_B_PIN_A;
const int encoderBpinB = PinConfig::ENCODER_B_PIN_B;

Encoder::Encoder(bool isEncoderA) {
    if (isEncoderA) {
        pinA_ = encoderApinA;
        pinB_ = encoderApinB;

        // Configure the pin-change interrupt for encoder A pin D68 (PCINT22).
        PCICR |= (1 << PCIE2);    // Enable Pin Change Interrupt Control Register
        PCMSK2 |= (1 << PCINT22); // Enable interrupt for pin D68
    } else {
        pinA_ = encoderBpinA;
        pinB_ = encoderBpinB;

        // Configure the pin-change interrupt for encoder B pin D52 (PCINT1).
        PCICR |= (1 << PCIE0);   // Enable Pin Change Interrupt Control Register
        PCMSK0 |= (1 << PCINT1); // Enable interrupt for pin D52
    }

    pinMode(pinA_, INPUT);
    pinMode(pinB_, INPUT);
}

void Encoder::update() {
    const bool currentA = digitalRead(pinA_);
    const bool currentB = digitalRead(pinB_);

    direction_ = (currentA == currentB);

    if (direction_) {
        ++count_;
    } else {
        --count_;
    }
}

void Encoder::zeroCount() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        count_ = 0;
    }
}

int32_t Encoder::getCount() {
    int32_t snapshot;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        snapshot = count_;
    }

    return snapshot;
}

void Encoder::zeroCountPair(Encoder& encoderA, Encoder& encoderB) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        encoderA.count_ = 0;
        encoderB.count_ = 0;
    }
}

Encoder::CountPair Encoder::getCountPair(const Encoder& encoderA, const Encoder& encoderB) {
    CountPair snapshot;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        snapshot.countA = encoderA.count_;
        snapshot.countB = encoderB.count_;
    }

    return snapshot;
}

bool Encoder::getDirection() const {
    return direction_;
}
