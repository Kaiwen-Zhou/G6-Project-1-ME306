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
    void initializeEncoderTimer();
    void update();
    void updateVelocity();
    void zeroCount();
    int32_t getCount();

    // snapshot two encoders at one instant
    static CountPair getCountPair(const Encoder& encoderA, const Encoder& encoderB);

    bool getDirection();
    float getDistance();
    float getVelocity();

  private:
    int pinA;
    int pinB;
    volatile int32_t count = 0;
    volatile bool direction = true;   // True = clockwise
    volatile bool Aprev = false;
    volatile bool Bprev = false;
    volatile bool Acur = false;
    volatile bool Bcur = false;
    volatile int32_t lastCount = 0;
    volatile float velocity = 0;      // mm/s // FLOAT INACCURATE SHOULD CHANGE TO count/s
};

#endif // ENCODER_H
