#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

/**
 * Encoder count and direction tracking for the two plotter motors.
 *
 * Construct one instance for encoder A and one for encoder B, then call update()
 * from their pin-change ISRs. Count reads, pair snapshots, and zeroing use AVR
 * atomic sections in Encoder.cpp so the main loop can safely access 32-bit data.
 */
class Encoder {
    public:
        struct CountPair {
                int32_t countA;
                int32_t countB;
        };

        Encoder(bool isEncoderA);

        // Called from the relevant pin-change ISR.
        void update();

        void zeroCount();

        int32_t getCount();

        static void zeroCountPair(Encoder& encoderA, Encoder& encoderB);

        // Read both 32-bit counts within one atomic block.
        static CountPair getCountPair(const Encoder& encoderA, const Encoder& encoderB);

        bool getDirection() const;

    private:
        int pinA_;
        int pinB_;

        volatile int32_t count_ = 0;
        volatile bool direction_ = true; // True = clockwise
};

#endif // ENCODER_H
