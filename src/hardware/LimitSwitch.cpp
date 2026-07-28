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
}

//Initialise hardware
void LimitSwitch::begin()
{
    // Configure input mode
    pinMode(pinNumber, inputMode);

    // Read current switch state
    stableState = digitalRead(pinNumber);

    // Store as previous reading
    lastReading = stableState;
}

//Update switch state with software debouncing.
void LimitSwitch::update()
{
    // Read the current raw input
    bool currentReading = digitalRead(pinNumber);

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