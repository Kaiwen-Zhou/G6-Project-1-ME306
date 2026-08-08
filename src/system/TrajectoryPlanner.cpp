#include "system/TrajectoryPlanner.h"

#include <math.h>

namespace plotter
{

namespace
{
constexpr float SECONDS_PER_MINUTE = 60.0f;
}

TrajectoryPlanner::TrajectoryPlanner()
    : targetXDisplacementMm_(0.0f),
      targetYDisplacementMm_(0.0f),
      directionX_(0.0f),
      directionY_(0.0f),
      pathLengthMm_(0.0f),
      accelerationMmPerSecondSquared_(0.0f),
      peakVelocityMmPerSecond_(0.0f),
      accelerationTimeSeconds_(0.0f),
      cruiseTimeSeconds_(0.0f),
      totalTimeSeconds_(0.0f),
      accelerationDistanceMm_(0.0f),
      cruiseDistanceMm_(0.0f),
      elapsedTimeSeconds_(0.0f),
      reference_{0.0f, 0.0f, 0.0f, 0.0f, false},
      active_(false),
      complete_(false)
{
}

bool TrajectoryPlanner::startMove(
    float startXMm,
    float startYMm,
    float targetXMm,
    float targetYMm,
    float feedrateMmPerMinute,
    float maxAccelerationMmPerSecondSquared) {
    if (feedrateMmPerMinute <= 0.0f || maxAccelerationMmPerSecondSquared <= 0.0f) {
        active_ = false;
        complete_ = false;

        reference_.xVelocityMmPerSecond = 0.0f;
        reference_.yVelocityMmPerSecond = 0.0f;
        reference_.complete = false;

        return false;
    }

    // Store the requested movement relative to this move's starting point.
    targetXDisplacementMm_ = targetXMm - startXMm;
    targetYDisplacementMm_ = targetYMm - startYMm;

    pathLengthMm_ =
        sqrtf(
            targetXDisplacementMm_ * targetXDisplacementMm_ +
            targetYDisplacementMm_ * targetYDisplacementMm_);

    elapsedTimeSeconds_ = 0.0f;

    // Every new move begins at zero relative displacement.
    reference_ = {0.0f, 0.0f, 0.0f, 0.0f, false};

    complete_ = false;

    // A zero-distance command is already complete.
    if (pathLengthMm_ == 0.0f) {
        reference_ = {0.0f, 0.0f, 0.0f, 0.0f, true};

        active_ = false;
        complete_ = true;

        return true;
    }

    directionX_ = targetXDisplacementMm_ / pathLengthMm_;

    directionY_ = targetYDisplacementMm_ / pathLengthMm_;

    accelerationMmPerSecondSquared_ = maxAccelerationMmPerSecondSquared;

    const float requestedVelocityMmPerSecond = feedrateMmPerMinute / SECONDS_PER_MINUTE;

    // Distance required to accelerate from zero to the requested velocity.
    const float requestedAccelerationDistanceMm =
        (requestedVelocityMmPerSecond * requestedVelocityMmPerSecond) /
        (2.0f * accelerationMmPerSecondSquared_);

    if (2.0f * requestedAccelerationDistanceMm >= pathLengthMm_) {
        // Short move: triangular velocity profile.
        accelerationDistanceMm_ = 0.5f * pathLengthMm_;

        cruiseDistanceMm_ = 0.0f;

        peakVelocityMmPerSecond_ =
            sqrtf(pathLengthMm_ * accelerationMmPerSecondSquared_);
    }
    else {
        // Long move: trapezoidal velocity profile.
        accelerationDistanceMm_ = requestedAccelerationDistanceMm;

        cruiseDistanceMm_ =
            pathLengthMm_ - 2.0f * accelerationDistanceMm_;

        peakVelocityMmPerSecond_ = requestedVelocityMmPerSecond;
    }

    accelerationTimeSeconds_ =
        peakVelocityMmPerSecond_ / accelerationMmPerSecondSquared_;

    if (cruiseDistanceMm_ > 0.0f) {
        cruiseTimeSeconds_ = cruiseDistanceMm_ / peakVelocityMmPerSecond_;
    }
    else {
        cruiseTimeSeconds_ = 0.0f;
    }

    totalTimeSeconds_ =
        2.0f * accelerationTimeSeconds_ + cruiseTimeSeconds_;

    active_ = true;

    return true;
}

TrajectoryReference TrajectoryPlanner::update(float timeStepSeconds) {
    if (!active_ || timeStepSeconds <= 0.0f) {
        return reference_;
    }

    elapsedTimeSeconds_ += timeStepSeconds;

    float distanceAlongPathMm = 0.0f;
    float pathVelocityMmPerSecond = 0.0f;

    if (elapsedTimeSeconds_ < accelerationTimeSeconds_) {
        // Acceleration:
        // s = 0.5*a*t^2
        // v = a*t
        distanceAlongPathMm =
            0.5f *
            accelerationMmPerSecondSquared_ *
            elapsedTimeSeconds_ *
            elapsedTimeSeconds_;

        pathVelocityMmPerSecond =
            accelerationMmPerSecondSquared_ * elapsedTimeSeconds_;
    }
    else if (elapsedTimeSeconds_ < accelerationTimeSeconds_ + cruiseTimeSeconds_) {
        // Constant-velocity cruise.
        const float timeAtCruiseSeconds =
            elapsedTimeSeconds_ - accelerationTimeSeconds_;

        distanceAlongPathMm =
            accelerationDistanceMm_ + peakVelocityMmPerSecond_ * timeAtCruiseSeconds;

        pathVelocityMmPerSecond = peakVelocityMmPerSecond_;
    }
    else if (elapsedTimeSeconds_ < totalTimeSeconds_) {
        // Deceleration.
        const float decelerationTimeSeconds =
            elapsedTimeSeconds_ - accelerationTimeSeconds_ - cruiseTimeSeconds_;

        distanceAlongPathMm =
            accelerationDistanceMm_ +
            cruiseDistanceMm_ +
            peakVelocityMmPerSecond_ * decelerationTimeSeconds -
            0.5f * accelerationMmPerSecondSquared_ * decelerationTimeSeconds * decelerationTimeSeconds;

        pathVelocityMmPerSecond =
            peakVelocityMmPerSecond_ - accelerationMmPerSecondSquared_ * decelerationTimeSeconds;
    }
    else {
        // Use exact final displacement to avoid rounding drift.
        reference_ = {
            targetXDisplacementMm_,
            targetYDisplacementMm_,
            0.0f,
            0.0f,
            true
        };

        active_ = false;
        complete_ = true;

        return reference_;
    }

    // Guard against small floating-point errors.
    if (distanceAlongPathMm > pathLengthMm_) {
        distanceAlongPathMm = pathLengthMm_;
    }

    if (pathVelocityMmPerSecond < 0.0f) {
        pathVelocityMmPerSecond = 0.0f;
    }

    reference_.xDisplacementMm = directionX_ * distanceAlongPathMm;

    reference_.yDisplacementMm = directionY_ * distanceAlongPathMm;

    reference_.xVelocityMmPerSecond = directionX_ * pathVelocityMmPerSecond;

    reference_.yVelocityMmPerSecond = directionY_ * pathVelocityMmPerSecond;

    reference_.complete = false;

    return reference_;
}

void TrajectoryPlanner::stop() {
    active_ = false;
    complete_ = false;

    // Hold the latest generated displacement.
    reference_.xVelocityMmPerSecond = 0.0f;
    reference_.yVelocityMmPerSecond = 0.0f;
    reference_.complete = false;
}

bool TrajectoryPlanner::isActive() const {
    return active_;
}

bool TrajectoryPlanner::isComplete() const {
    return complete_;
}

}  // namespace plotter