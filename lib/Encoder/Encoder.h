#ifndef ENCODER_H
#define ENCODER_H

#include <Arduino.h>

class Encoder {
  public:
    Encoder(bool isEncoderA);
    void update();
    void zeroCount();
    uint16_t getCount();
    bool getDirection();

  private:
    int pinA;
    int pinB;
    uint16_t count = 0;
    bool direction = true;
    bool Aprev = false;
    bool Bprev = false;
    bool Acur = false;
    bool Bcur = false;
};

#endif // ENCODER_H
