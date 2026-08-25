#ifndef LIMIT_SWITCH_H
#define LIMIT_SWITCH_H

#include <Arduino.h>

/**
 * Active-high mechanical limit input with non-blocking software debounce.
 *
 * Call begin() once and update() every main-loop iteration. An optional rising-
 * edge ISR can call notifyFromISR(); the class then verifies the edge through
 * the same debounce interval and exposes one-shot press, release, and rejected-
 * interrupt events.
 */
class LimitSwitch {
    public:
        LimitSwitch(uint8_t pin, uint8_t mode = INPUT, unsigned long debounceTime = 20);
        // pin: Arduino digital input pin.
        // mode: INPUT for the current externally pulled-down inputs.
        // debounceTime: debounce interval in milliseconds.

        // Initialise the switch hardware.
        void begin(void (*isr)() = nullptr);

        // Updates the switch state.
        // Reads the raw input signal, performs software debouncing, and updates the stable state.
        void update();

        // Called only by the ISR if an interrupt is attached to the switch pin.
        void notifyFromISR();

        // Returns true while an ISR-triggered rising edge is awaiting debounce verification.
        bool isInterruptVerificationPending() const;

        // Returns true if the switch is pressed.
        // The current junction-board inputs are active-high: HIGH means pressed.
        bool isPressed() const;

        // Returns true if the switch is released.
        bool isReleased() const;

        // Returns the stable raw digital state (HIGH = pressed; LOW = released).
        bool getState() const;

        // One-shot events. Returns true only once when the switch is pressed or released.
        bool consumePressedEvent();
        bool consumeReleasedEvent();
        bool consumeRejectedInterruptEvent();

    private:
        uint8_t pinNumber; // Arduino pin connected to the switch

        uint8_t inputMode; // INPUT or INPUT_PULLUP

        unsigned long debounceDelay; // Debounce time delay in ms

        volatile bool stableState; // Debounced stable state

        volatile bool lastReading; // Previous raw reading

        volatile unsigned long lastDebounceTime; // Time when the last state change occurred
        volatile bool interruptPending;          // Flag to indicate if an interrupt has occurred

        bool verificationActive; // Flag to indicate if verification is active

        bool pressedEvent; // Flag to indicate if a pressed event has occurred

        bool releasedEvent; // Flag to indicate if a released event has occurred

        bool rejectedInterruptEvent; // Flag to indicate if a rejected interrupt event has occurred
};

#endif // LIMIT_SWITCH_H
