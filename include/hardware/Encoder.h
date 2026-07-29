#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
  public:
    Encoder(bool isEncoderA);
    void initializeEncoderTimer();
    void update();
    void updateVelocity();
    void zeroCount();
    int32_t getCount();
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
    volatile float velocity = 0;      // mm/s
};

#endif // ENCODER_H
