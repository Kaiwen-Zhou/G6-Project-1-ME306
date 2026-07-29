#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
  public:
    Encoder(bool isEncoderA);
    void update();
    void zeroCount();
    int32_t getCount();
    bool getDirection();
    float getDistance();

  private:
    int pinA;
    int pinB;
    volatile int32_t count = 0;
    volatile bool direction = true;
    volatile bool Aprev = false;
    volatile bool Bprev = false;
    volatile bool Acur = false;
    volatile bool Bcur = false;
};

#endif // ENCODER_H
