// === Encoder.cpp === //
// This file contains the Encoder class for interfacing with the encoders. It keeps track of the encoder count and direction of rotation. The update function is intented to be called within a pin change interrupt. The interrupt will be triggered when the encoders A output changes.

// === Configurations === //
// Since using pin change interrupts each encoder has to use a different register as only one interrupt service routine can be used per register (e.g. PB0-7).
// Pins are defined in this file to ensure they are consistent.
// Using D68 (PK6)(PCINT22) for encoder A interrupt
// Using D52 (PB1)(PCINT1) for encoder B interrupt

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
*/

// === Future Improvements === //
// Currently store position data in the form of counts but position in mm and velocity and acceleration potentially needed for control loop. Need to decide if to implement that as part of encoder class or not.
// More fault checking and need to implement how system is notified when faults occured. Currently just prints error message to serial

#include <Arduino.h>
#include "hardware/Encoder.h"
#include "config/PinConfig.h"

// Pin definitions

const int encoderApinA = PinConfig::ENCODER_A_PIN_A;    // A14 // Yellow wire 
const int encoderApinB = PinConfig::ENCODER_A_PIN_B;    // A15 // White wire
const int encoderBpinA = PinConfig::ENCODER_B_PIN_A;    // Yellow wire
const int encoderBpinB = PinConfig::ENCODER_B_PIN_B;    // White wire

Encoder::Encoder(bool isEncoderA) {
    // Initialize the encoder pins and set them as inputs
    if (isEncoderA)  {
        pinA = encoderApinA;
        pinB = encoderApinB;
        // Setup pin change interrupt for encoder A pin D68 (PCINT22)
        PCICR |= (1 << PCIE2); // Enable Pin Change Interrupt Control Register
        PCMSK2 |= (1 << PCINT22); // Enable interrupt for pin D68
    } else {
        pinA = encoderBpinA;
        pinB = encoderBpinB;
        // Setup pin change interrupt for encoder B pin D52 (PCINT1)
        PCICR |= (1 << PCIE0); // Enable Pin Change Interrupt Control Register
        PCMSK0 |= (1 << PCINT1); // Enable interrupt for pin D52
    }
    pinMode(pinA, INPUT);
    pinMode(pinB, INPUT);
    Aprev = digitalRead(pinA);
    Bprev = digitalRead(pinB);
}

void Encoder::update() {
    /*
    Update should only be called during pin change interrupts for pinA. This ensures that the encoder is updated only when there is a change in the state of pinA (the motor is turning)
    */
    Acur = digitalRead(pinA);
    Bcur = digitalRead(pinB);

    // Direction is true for clockwise and false for counter clockwise
    // COULD RENAME GET DIRECTION FUNCTION TO INDICATE IF TURNING CLOCKWISE OR COUNTER CLOCKWISE
    direction = (Acur == Bprev);

    // Increment counter in direction of movement
    if (direction) {
        count++;
    } else {
        count--;
    }

    Aprev = Acur;
    Bprev = Bcur;

    // TODO
    // implement check to fault if enocder is moving to fast for function to keep up with. If both A and B have changed values or Aprev = Acur or could implement a timer to check if the time between updates is too short 
}

void Encoder::zeroCount() {
    count = 0;
}

uint16_t Encoder::getCount() {
    return count;
}

bool Encoder::getDirection() {
    return direction;
}
