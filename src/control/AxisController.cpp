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
                               unsigned long updateIntervalMicros,
                               uint8_t requiredSettledUpdates)
    : encoder_(encoder),
      pidController_(pidController),
      motor_(motor),
      positionTolerance_(positionTolerance),
      updateIntervalMicros_(updateIntervalMicros),
      requiredSettledUpdates_(requiredSettledUpdates),
      targetPosition_(0),
      lastUpdateMicros_(0),
      settledUpdateCount_(0),
      active_(false),
      complete_(false)
{
    if (positionTolerance_ < 0)
    {
        positionTolerance_ = 0;
    }

    if (requiredSettledUpdates_ == 0)
    {
        requiredSettledUpdates_ = 1;
    }
}

void AxisController::begin()
{
    motor_.begin();
    reset();
}

void AxisController::setTargetPosition(int32_t targetPosition)
{
    motor_.stop();
    pidController_.reset();

    targetPosition_ = targetPosition;
    settledUpdateCount_ = 0;
    active_ = true;
    complete_ = false;
    lastUpdateMicros_ = micros();
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
    const int32_t positionError = targetPosition_ - currentPosition;

    if (isWithinTolerance(positionError))
    {
        motor_.stop();
        pidController_.reset();

        ++settledUpdateCount_;

        if (settledUpdateCount_ >= requiredSettledUpdates_)
        {
            finishMove();
        }

        return;
    }

    settledUpdateCount_ = 0;

    const float timeStepSeconds =
        static_cast<float>(elapsedMicros) * MICROSECONDS_TO_SECONDS;

    const float controllerOutput =
        pidController_.update(static_cast<float>(positionError),
                              timeStepSeconds);

    motor_.setOutput(static_cast<int16_t>(controllerOutput));
}

void AxisController::stop()
{
    motor_.stop();
    pidController_.reset();
    settledUpdateCount_ = 0;
    active_ = false;
    complete_ = false;
}

void AxisController::reset()
{
    stop();
    targetPosition_ = static_cast<int32_t>(encoder_.getCount());
    lastUpdateMicros_ = micros();
}

bool AxisController::isActive() const
{
    return active_;
}

bool AxisController::isComplete() const
{
    return complete_;
}

AxisTelemetry AxisController::getTelemetry() const
{
    const int32_t currentPosition =
        static_cast<int32_t>(encoder_.getCount());
    const int32_t positionError = targetPosition_ - currentPosition;

    return {
        targetPosition_,
        currentPosition,
        positionError,
        motor_.getOutput(),
        pidController_.getIntegralOutput(),
        isWithinTolerance(positionError),
        active_,
        complete_
    };
}

bool AxisController::isWithinTolerance(int32_t error) const
{
    return error >= -positionTolerance_ &&
           error <= positionTolerance_;
}

void AxisController::finishMove()
{
    motor_.stop();
    pidController_.reset();
    active_ = false;
    complete_ = true;
}