#include "control/HomingController.h"

// Constructor
HomingController::HomingController(unsigned long timeout)
{
    stage = HomingStage::IDLE; // Initial stage is IDLE
    expectedSwitch = ExpectedSwitch::NONE; // No limit switch expected initially
    stageStartTime = 0; // Initialize stage start time to 0
    timeoutMs = timeout; // Set the maximum time allowed for the homing process before aborting
    mockSwitchPressed = false; // Initialize mock switch pressed flag to false  
}

// Initialise the controller
void HomingController::begin()
{
    stage = HomingStage::IDLE; // Set initial stage to IDLE
    expectedSwitch = ExpectedSwitch::NONE; // No limit switch expected initially
    resetStageTimer(); // Reset the stage timer
}

// Start the homing process
void HomingController::start()
{
    changeStage(HomingStage::START);
}

// Main update function for the homing process (non-blocking)
void HomingController::update()
{
    // Timeout check: If the current stage has exceeded the allowed time, abort the homing process
    if (stage != HomingStage::IDLE && 
        stage != HomingStage::COMPLETE &&
        stage != HomingStage::ABORT)
    {
        if (millis() - stageStartTime > timeoutMs)
        {
            abort();
            return;
        }
    }
    // Handle the current stage of the homing process
    switch (stage)
    {
        case HomingStage::IDLE:
            // Implementation for IDLE stage
            break;

        case HomingStage::START:
            // Implementation for START stage
            expectedSwitch = ExpectedSwitch::X_MIN;
            changeStage(HomingStage::HOME_X);
            break;

        case HomingStage::HOME_X:
            // Implementation for HOME_X stage
            if (mockSwitchPressed)
            {
                mockSwitchPressed = false;
                expectedSwitch = ExpectedSwitch::Y_MIN;
                changeStage(HomingStage::HOME_Y);
            }
            break;

        case HomingStage::HOME_Y:
            // Implementation for HOME_Y stage
            if (mockSwitchPressed)
            {
                mockSwitchPressed = false;
                expectedSwitch = ExpectedSwitch::NONE;
                changeStage(HomingStage::ZERO_POSITION);
            }
            break;

        case HomingStage::ZERO_POSITION:
            // Implementation for ZERO_POSITION stage
            assignZero();
            changeStage(HomingStage::COMPLETE);
            break;

        case HomingStage::COMPLETE:
            // Implementation for COMPLETE stage
            break;

        case HomingStage::ABORT:
            // Implementation for ABORT stage
            break;
    }
}

// Abort the homing process
void HomingController::abort()
{
    changeStage(HomingStage::ABORT);
}

// Assign the current position as the zero reference
void HomingController::assignZero()
{
    // For future implementation: Set the current position as the zero reference for the system.
}

// Mock switch trigger for testing stage transitions before hardware is connected
void HomingController::mockSwitchTriggered()
{
    mockSwitchPressed = true;
}

// Change the homing stage and reset the timer
void HomingController::changeStage(HomingStage newStage)
{
    stage = newStage;
    resetStageTimer();  
}

// Reset the stage timer to the current time
void HomingController::resetStageTimer()
{
    stageStartTime = millis();
}

// Get the current homing stage
HomingController::HomingStage 
HomingController::getStage() const
{
    return stage;
}

// Get the expected limit switch state for the current stage
HomingController::ExpectedSwitch
HomingController::getExpectedSwitch() const
{
    return expectedSwitch;
}
