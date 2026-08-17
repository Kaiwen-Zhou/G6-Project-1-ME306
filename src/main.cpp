#include <Arduino.h>
#include <avr/interrupt.h>

#include "app/PlotterApplication.h"

namespace {
plotter::PlotterApplication application;
}

ISR(PCINT2_vect) {
    application.onEncoderAInterrupt();
}

ISR(PCINT0_vect) {
    application.onEncoderBInterrupt();
}

void setup() {
    application.begin();
}

void loop() {
    application.update();
}
