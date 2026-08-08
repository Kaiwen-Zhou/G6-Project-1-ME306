#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
  public:
    struct CountPair {
      int32_t countA;
      int32_t countB;
    };

    Encoder(bool isEncoderA);

    // Called from the relevant pin-change ISR
    void update();

    void zeroCount();
    
    int32_t getCount();

    // Read both 32-bit counts within one atomic block.
    static CountPair getCountPair(const Encoder& encoderA, const Encoder& encoderB);

    bool getDirection() const;

  private:
    int pinA_;
    int pinB_;

    volatile int32_t count_ = 0;
    volatile bool direction_ = true;   // True = clockwise
};

#endif // ENCODER_H
