#ifndef HOMING_CONTROLLER_H
#define HOMING_CONTROLLER_H

#include <Arduino.h>
// Non-blocking HomingController skeleton
// Manage the homing process of a stepper motor using limit switches and a state machine.
class HomingController
{
public:
    // Homing state machine states
    enum class HomingStage
    {
        IDLE, // No homing in process
        START, // Start homing process
        HOME_X, // Home X axis
        HOME_Y, // Home Y axis
        ZERO_POSITION, // Set the current position as the zero reference
        COMPLETE, // Homing process complete
        ABORT // Homing process aborted due to error
    };

    // Expected limit switch states for homing
    enum class ExpectedSwitch
    {
        NONE, // No limit switch expected
        // Expected X limit switch state
        X_MIN,
        X_MAX,
        // Expected Y limit switch state
        Y_MIN,
        Y_MAX
    };

    // Constructor
    explicit HomingController(unsigned long timeout = 5000); // Default timeout of 5 seconds
    // timeout = Maximum time allowed for the homing process before aborting 

    // Initialise the controller
    void begin();

    // Start the homing process
    void start();

    // Update the homing process (non-blocking)
    void update();

    // Abort the homing process
    void abort();

    // Assign the current position as the zero reference
    void assignZero();

    // Mock switch trigger for testing stage transitions before hardware is connected
    void mockSwitchTriggered(); 

    // Get the current homing stage
    HomingStage getStage() const;

    // Get the expected limit switch state for the current stage
    ExpectedSwitch getExpectedSwitch() const;

private:
    // Change the homing stage 
    void changeStage(HomingStage newStage);

    // Start the timeout timer
    void resetStageTimer();
    
private:
    HomingStage stage; // Current homing stage

    ExpectedSwitch expectedSwitch; // Expected limit switch state for the current stage

    unsigned long stageStartTime; // Time when the current stage started

    unsigned long timeoutMs; // Maximum time allowed for the homing process before aborting

    bool mockSwitchPressed; // Flag to simulate a limit switch press for testing stage transitions
};

#endif // HOMING_CONTROLLER_H