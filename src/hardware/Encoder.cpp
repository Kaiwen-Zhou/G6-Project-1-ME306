// === Encoder.cpp === //
// This file contains the Encoder class for interfacing with the encoders. It keeps track of the encoder count and direction of rotation. The update function is intented to be called within a pin change interrupt. The interrupt will be triggered when the encoders A output changes.

// === Usage === //
// Encoder EncoderA(true);
// Encoder EncoderB(false);
/*
ISR(PCINT2_vect) {
  // Encoder A
  EncoderA.update();
}
ISR(PCINT0_vect) {
  // Encoder B
  EncoderB.update();
}
ISR(TIMER1_COMPA_vect) {
  // Update encoder velocities
  EncoderA.velocityUpdate();
  EncoderB.velocityUpdate();
}
*/

// === Configurations === //
// Currently uses Timer1 for velocity calculations
// Since using pin change interrupts each encoder has to use a different register as only one interrupt service routine can be used per register (e.g. PB0-7).
// Pins are defined in this file to ensure they are consistent.
// Using D68/A14 (PK6)(PCINT22) for encoder A interrupt
// Using D52 (PB1)(PCINT1) for encoder B interrupt


// === Future Improvements === //
// Currently store position data in the form of counts but position in mm and velocity and acceleration potentially needed for control loop. Need to decide if to implement that as part of encoder class or not.
// More fault checking and need to implement how system is notified when faults occured. Currently just prints error message to serial

#include <Arduino.h>
#include <util/atomic.h>
#include "hardware/Encoder.h"
#include "config/PinConfig.h"
#include "config/SystemConfig.h"

// Pin definitions
const int encoderApinA = PinConfig::ENCODER_A_PIN_A;    // A14 // Yellow wire 
const int encoderApinB = PinConfig::ENCODER_A_PIN_B;    // A15 // White wire
const int encoderBpinA = PinConfig::ENCODER_B_PIN_A;    // Yellow wire
const int encoderBpinB = PinConfig::ENCODER_B_PIN_B;    // White wire

Encoder::Encoder(bool isEncoderA) {
    // Initialize the encoder pins and set them as inputs
    if (isEncoderA)  {
        pinA_ = encoderApinA;
        pinB_ = encoderApinB;
        // Setup pin change interrupt for encoder A pin D68 (PCINT22)
        PCICR |= (1 << PCIE2); // Enable Pin Change Interrupt Control Register
        PCMSK2 |= (1 << PCINT22); // Enable interrupt for pin D68
    } else {
        pinA_ = encoderBpinA;
        pinB_ = encoderBpinB;
        // Setup pin change interrupt for encoder B pin D52 (PCINT1)
        PCICR |= (1 << PCIE0); // Enable Pin Change Interrupt Control Register
        PCMSK0 |= (1 << PCINT1); // Enable interrupt for pin D52
    }
    pinMode(pinA_, INPUT);
    pinMode(pinB_, INPUT);
}

void Encoder::update()
{
    const bool currentA = digitalRead(pinA_);
    const bool currentB = digitalRead(pinB_);

    direction_ = (currentA == currentB);

    if (direction_)
    {
        ++count_;
    }
    else
    {
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

