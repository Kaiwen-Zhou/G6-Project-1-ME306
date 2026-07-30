#include "hardware/LimitSwitch.h"

//Constructor
LimitSwitch::LimitSwitch(uint8_t pin,
                         uint8_t mode,
                         unsigned long debounceTime)
{
    pinNumber = pin;
    inputMode = mode;
    debounceDelay = debounceTime;

    // Assume released until hardware is initialised
    stableState = HIGH;
    lastReading = HIGH;

    // No debounce timing has started yet
    lastDebounceTime = 0;

    // Interrupt state flags
    interruptPending = false;
    verificationActive = false;

    // One-shot event flags
    pressedEvent = false;
    releasedEvent = false;
    rejectedInterruptEvent = false;
}

//Initialise hardware
void LimitSwitch::begin(void (*isr)())
{
    // Configure input mode
    pinMode(pinNumber, inputMode);

    // Read current switch state
    stableState = digitalRead(pinNumber);

    // Store as previous reading
    lastReading = stableState;

    // If an ISR is provided, attach it to the pin interrupt
    if (isr != nullptr)
    {
        attachInterrupt(
            digitalPinToInterrupt(pinNumber),
            isr,
            FALLING);
    }
}

// Called by the ISR to notify that an interrupt has occurred on the switch pin
void LimitSwitch::notifyFromISR()
{
    interruptPending = true;
}

//Update switch state with software debouncing.
void LimitSwitch::update()
{
    // Read the current raw input
    bool currentReading = digitalRead(pinNumber);

    // If an interrupt has occurred, set the verification flag
    if (interruptPending)
    {
        interruptPending = false;
        verificationActive = true;
        lastDebounceTime = millis(); // Start debounce timing
    }

    // If verification is active and the switch is reached, clear the verification flag
    if (verificationActive)
    {
        if ((millis() - lastDebounceTime) >= debounceDelay)
        {
            currentReading = digitalRead(pinNumber);
            // If the switch is still pressed, accept it as a valid press event
            if (currentReading == LOW)
            {
                if (stableState != LOW)
                {
                    stableState = LOW;
                    pressedEvent = true;

                    // Modified: Keep the lastReading consistent so the pooling debounce does not immediately restart.
                    lastReading = LOW;
                }
            }
            // If the switch is released, clear the verification flag
            else
            {
                rejectedInterruptEvent = true; // Interrupt occurred but the switch returned HIGH after the debounce interval.
                // Treat as contact bounce or electrical noise.
            }

            verificationActive = false; // Clear verification flag after processing
        }
    }

    // If the raw input has changed,restart the debounce timer.
    if (currentReading != lastReading)
    {
        lastDebounceTime = millis();
    }

    // If the input has remained unchanged for the debounce interval, accept it as the new stable state.
    if ((millis() - lastDebounceTime) >= debounceDelay)
    {
        if (currentReading != stableState)
        {
            stableState = currentReading;

            if (stableState == LOW)
            {
                pressedEvent = true;
            }
            else
            {
                releasedEvent = true;
            }
        }
    }
    // Save reading for next update
    lastReading = currentReading;
}

//Active-low -- (LOW means switch is pressed.)
bool LimitSwitch::isPressed() const
{
    return (stableState == LOW);
}

//Returns true if switch is released.
bool LimitSwitch::isReleased() const 
{
    return !isPressed();
}

//Returns the stable raw state.
bool LimitSwitch::getState() const
{
    return stableState;
}

// One-shot events, returns true only once when the switch is pressed.
bool LimitSwitch::consumePressedEvent()
{
    bool event = pressedEvent;
    pressedEvent = false; // Reset the event flag after consumption
    return event;
}

// One-shot events, returns true only once when the switch is released.
bool LimitSwitch::consumeReleasedEvent()
{
    bool event = releasedEvent;
    releasedEvent = false; // Reset the event flag after consumption
    return event;
}

// One-shot events, returns true only once when a rejected interrupt event occurs.
bool LimitSwitch::consumeRejectedInterruptEvent()
{
    bool event = rejectedInterruptEvent;
    rejectedInterruptEvent = false; // Reset the event flag after consumption
    return event;
}
