#include "control/AxisController.h"

#include <Arduino.h>

namespace
{
constexpr float MICROSECONDS_TO_SECONDS = 0.000001f;
}

AxisController::AxisController(Encoder& encoder,
                               PIDController& pidController,
                               MotorDriver& motor,
                               int32_t positionTolerance,
                               unsigned long updateIntervalMicros)
    : encoder_(encoder),
      pidController_(pidController),
      motor_(motor),
      positionTolerance_(positionTolerance),
      updateIntervalMicros_(updateIntervalMicros),
      referencePosition_(0),
      lastUpdateMicros_(0),
      active_(false)
{
    if (positionTolerance_ < 0)
    {
        positionTolerance_ = 0;
    }

    // Avoid a zero-length control interval
    if (updateIntervalMicros_ == 0) {
        updateIntervalMicros_ = 1;
    }
}

void AxisController::begin()
{
    motor_.begin();
    reset();
}

void AxisController::startTracking()
{
    motor_.stop();
    pidController_.reset();

    // Begin from the current encoder position so that starting the controller
    // does not immediately create an unintended position error.
    referencePosition_ = static_cast<int32_t>(encoder_.getCount());

    lastUpdateMicros_ = micros();
    active_ = true;
}

void AxisController::setReferencePosition(int32_t referencePosition)
{
    if (!active_) {
        return;
    }

    // Do not stop motors, reset the PI or restart the update timer here
    // A trajectroy planner may call this function every control cycle
    referencePosition_ = referencePosition;
}

void AxisController::update()
{
    if (!active_)
    {
        return;
    }

    const unsigned long currentTimeMicros = micros();
    const unsigned long elapsedMicros = currentTimeMicros - lastUpdateMicros_;

    if (elapsedMicros < updateIntervalMicros_)
    {
        return;
    }

    lastUpdateMicros_ = currentTimeMicros;

    // The current Encoder interface returns uint16_t. Convert it to a signed
    // value before subtracting so reverse errors are calculated correctly.
    const int32_t currentPosition =
        static_cast<int32_t>(encoder_.getCount());

    const int32_t trackingError = referencePosition_ - currentPosition;

    if (isErrorWithinTolerance(trackingError))
    {
        // The controller remains active as this may only be an intermediate trajectory reference
        motor_.stop();
        return;
    }

    const float timeStepSeconds =
        static_cast<float>(elapsedMicros) * MICROSECONDS_TO_SECONDS;

    const float controllerOutput =
        pidController_.update(static_cast<float>(trackingError),
                              timeStepSeconds);

    motor_.setOutput(static_cast<int16_t>(controllerOutput));
}

void AxisController::stop()
{
    motor_.stop();
    pidController_.reset();
    active_ = false;
}

void AxisController::reset()
{
    stop();
    referencePosition_ = static_cast<int32_t>(encoder_.getCount());
    lastUpdateMicros_ = micros();
}

bool AxisController::isActive() const
{
    return active_;
}

bool AxisController::isWithinTolerance() const 
{
    const int32_t currentPosition = static_cast<int32_t>(encoder_.getCount());
    const int32_t trackingError = referencePosition_ - currentPosition;

    return isErrorWithinTolerance(trackingError);
}

AxisTelemetry AxisController::getTelemetry() const
{
    const int32_t currentPosition =
        static_cast<int32_t>(encoder_.getCount());

    const int32_t trackingError = referencePosition_ - currentPosition;

    return {
        referencePosition_,
        currentPosition,
        trackingError,
        motor_.getOutput(),
        pidController_.getIntegralOutput(),
        isErrorWithinTolerance(trackingError),
        active_,
    };
}

bool AxisController::isErrorWithinTolerance(int32_t error) const
{
    return  error >= -positionTolerance_ &&
            error <= positionTolerance_;
}