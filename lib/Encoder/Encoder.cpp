// === Encoder.cpp === //
// This file contains the Encoder class for interfacing with the encoders. It keeps track of the encoder count and direction of rotation. The update function is intented to be called within a pin change interrupt. The interrupt will be triggered when the encoders A output changes.

// === Configurations === //
// Since using pin change interrupts each encoder has to use a different register as only one interrupt service routine can be used per register (e.g. PB0-7).
// Pins are defined in this file to ensure they are consistent.
// Using D68 (PK6)(PCINT22) for encoder A interrupt
// Using D52 (PB1)(PCINT1) for encoder B interrupt

// === Future Improvements === //
// Currently store position data in the form of counts but position in mm and velocity and acceleration potentially needed for control loop. Need to decide if to implement that as part of encoder class or not.
// More fault checking and need to implement how system is notified when faults occured. Currently just prints error message to serial

#include <Arduino.h>
#include "Encoder.h"

// Pin definitions
const int encoderApinA = 68;    // Yellow wire
const int encoderApinB = 69;    // White wire
const int encoderBpinA = 52;    // Yellow wire
const int encoderBpinB = 53;    // White wire


Encoder::Encoder(bool isEncoderA) {
    // Initialize the encoder pins and set them as inputs
    if (isEncoderA)  {
        pinA = encoderApinA;
        pinB = encoderApinB;
    } else {
        pinA = encoderBpinA;
        pinB = encoderBpinB;
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
    
    direction = Aprev != Bprev;
    
    // Increment counter in direction of movement
    if (direction) {
        count++;
    } else {
        count--;
    }
    
    // TODO
    // implement check to fault if enocder is moving to fast for function to keep up with. If both A and B have changed values or Aprev = Acur or could implement a timer to check if the time between updates is too short 
}

void Encoder::zeroCount() {
    count = 0;
}

int Encoder::getCount() {
    return count;
}

bool Encoder::getDirection() {
    return direction;
}
