#include "hardware/LimitSwitch.h"

LimitSwitch::LimitSwitch(uint8_t pin, uint8_t mode, unsigned long debounceTime) {
    pinNumber = pin;
    inputMode = mode;
    debounceDelay = debounceTime;

    // Assume released until the hardware is initialised.
    stableState = LOW;
    lastReading = LOW;

    // No debounce timing has started yet.
    lastDebounceTime = 0;

    // Interrupt state flags.
    interruptPending = false;
    verificationActive = false;

    // One-shot event flags.
    pressedEvent = false;
    releasedEvent = false;
    rejectedInterruptEvent = false;
}

void LimitSwitch::begin(void (*isr)()) {
    pinMode(pinNumber, inputMode);

    stableState = digitalRead(pinNumber);
    lastReading = stableState;

    // Attach an optional rising-edge interrupt to this switch input.
    if (isr != nullptr) {
        attachInterrupt(digitalPinToInterrupt(pinNumber), isr, RISING);
    }
}

void LimitSwitch::notifyFromISR() {
    interruptPending = true;
}

bool LimitSwitch::isInterruptVerificationPending() const {
    return interruptPending || verificationActive;
}

void LimitSwitch::update() {
    bool currentReading = digitalRead(pinNumber);

    // Begin debounce verification for a rising edge latched by the ISR.
    if (interruptPending) {
        interruptPending = false;
        verificationActive = true;
        lastDebounceTime = millis();
    }

    // Accept the ISR edge only if the input remains pressed for the full
    // debounce interval.
    if (verificationActive) {
        if ((millis() - lastDebounceTime) >= debounceDelay) {
            currentReading = digitalRead(pinNumber);
            if (currentReading == HIGH) {
                if (stableState != HIGH) {
                    stableState = HIGH;
                    pressedEvent = true;

                    // Keep polling debounce synchronised with the accepted edge.
                    lastReading = HIGH;
                }
            } else {
                // The input returned LOW during debounce, so treat the edge as
                // contact bounce or electrical noise.
                rejectedInterruptEvent = true;
            }

            verificationActive = false;
        }
    }

    // Restart polling debounce whenever the raw input changes.
    if (currentReading != lastReading) {
        lastDebounceTime = millis();
    }

    // Accept a new stable state after the raw input remains unchanged for the
    // full debounce interval.
    if ((millis() - lastDebounceTime) >= debounceDelay) {
        if (currentReading != stableState) {
            stableState = currentReading;

            if (stableState == HIGH) {
                pressedEvent = true;
            } else {
                releasedEvent = true;
            }
        }
    }
    // Save the raw reading for the next update.
    lastReading = currentReading;
}

bool LimitSwitch::isPressed() const {
    return (stableState == HIGH);
}

bool LimitSwitch::isReleased() const {
    return !isPressed();
}

bool LimitSwitch::getState() const {
    return stableState;
}

bool LimitSwitch::consumePressedEvent() {
    bool event = pressedEvent;
    pressedEvent = false;
    return event;
}

bool LimitSwitch::consumeReleasedEvent() {
    bool event = releasedEvent;
    releasedEvent = false;
    return event;
}

bool LimitSwitch::consumeRejectedInterruptEvent() {
    bool event = rejectedInterruptEvent;
    rejectedInterruptEvent = false;
    return event;
}
