#include "control/HomingController.h"

// Constructor
HomingController::HomingController(unsigned long timeout)
{
    stage = HomingStage::IDLE; // Initial stage is IDLE
    expectedSwitch = ExpectedSwitch::NONE; // No limit switch expected initially
    stageStartTime = 0; // Initialize stage start time to 0
    timeoutMs = timeout; // Set the maximum time allowed for the homing process before aborting
    xAxis_ = nullptr; // Initialize X axis controller pointer to nullptr
    yAxis_ = nullptr; // Initialize Y axis controller pointer to nullptr
    xMinSwitch_ = nullptr; // Initialize X min limit switch pointer to nullptr
    xMaxSwitch_ = nullptr; // Initialize X max limit switch pointer to nullptr
    yMinSwitch_ = nullptr; // Initialize Y min limit switch pointer to nullptr
    yMaxSwitch_ = nullptr; // Initialize Y max limit switch pointer to nullptr
}

// Initialise the controller
void HomingController::begin()
{
    stage = HomingStage::IDLE; // Set initial stage to IDLE
    expectedSwitch = ExpectedSwitch::NONE; // No limit switch expected initially
    resetStageTimer(); // Reset the stage timer
}

// Connect AxisControllers
void HomingController::attachAxes(AxisController* xAxis, AxisController* yAxis)
{
    xAxis_ = xAxis;
    yAxis_ = yAxis;
}

// Connect LimitSwitches
void HomingController::attachLimitSwitches(LimitSwitch* xMin, LimitSwitch* xMax, LimitSwitch* yMin, LimitSwitch* yMax)
{
    xMinSwitch_ = xMin;
    xMaxSwitch_ = xMax;
    yMinSwitch_ = yMin;
    yMaxSwitch_ = yMax;
}

// Start the homing process
void HomingController::start()
{
    changeStage(HomingStage::START);
}

// Main update function for the homing process (non-blocking)
void HomingController::update()
{
    // Update all connected limit switches
    if (xMinSwitch_) xMinSwitch_->update();
    if (xMaxSwitch_) xMaxSwitch_->update();
    if (yMinSwitch_) yMinSwitch_->update();
    if (yMaxSwitch_) yMaxSwitch_->update();

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
            // Start tracking on the X axis if it's connected
            if (xAxis_)
            {
                xAxis_->startTracking();
            }

            changeStage(HomingStage::HOME_X);

            break;

        case HomingStage::HOME_X:
            // Implementation for HOME_X stage
            if (xMinSwitch_ && xMinSwitch_->consumePressedEvent()) // Check if the X min limit switch has been pressed
            {
                if (xAxis_) // Stop the X axis if it's connected
                {
                    xAxis_->stop();
                }
                expectedSwitch = ExpectedSwitch::Y_MIN; // Set the expected switch to Y_MIN for the next stage
                if (yAxis_) // Start tracking on the Y axis if it's connected
                {
                    yAxis_->startTracking();
                }
                changeStage(HomingStage::HOME_Y); // Move to the HOME_Y stage to home the Y axis
            }

            break;

        case HomingStage::HOME_Y:
            // Implementation for HOME_Y stage
            if (yMinSwitch_ && yMinSwitch_->consumePressedEvent()) // Check if the Y min limit switch has been pressed
            {
                if (yAxis_) // Stop the Y axis if it's connected
                {
                    yAxis_->stop();
                }
                expectedSwitch = ExpectedSwitch::NONE; // No limit switch expected after homing is complete
                changeStage(HomingStage::ZERO_POSITION); // Move to the ZERO_POSITION stage to assign the current position as the zero reference
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
    if (xAxis_) xAxis_->stop(); // Stop the X axis if it's connected
    if (yAxis_) yAxis_->stop(); // Stop the Y axis if it's connected

    changeStage(HomingStage::ABORT);
}

// Assign encoder zero
void HomingController::assignZero()
{
    if (xAxis_) xAxis_->reset();
    if (yAxis_) yAxis_->reset();
}

// Mock interface for testing
void HomingController::switchTriggered(ExpectedSwitch sw)
{
    switch (sw) // Simulate a limit switch press event for testing stage transitions
    {
        case ExpectedSwitch::X_MIN: // Simulate X min limit switch press
            if (xMinSwitch_)
            {
                while (xMinSwitch_->consumePressedEvent()) {} // Consume all pending pressed events for the X min limit switch
            }
            break;

        case ExpectedSwitch::Y_MIN: // Simulate Y min limit switch press
            if (yMinSwitch_)
            {
                while (yMinSwitch_->consumePressedEvent()) {} // Consume all pending pressed events for the Y min limit switch
            }
            break;

        default:
            break;
    }
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
HomingController::HomingStage HomingController::getStage() const
{
    return stage;
}

// Get the expected limit switch state for the current stage
HomingController::ExpectedSwitch HomingController::getExpectedSwitch() const
{
    return expectedSwitch;
}
