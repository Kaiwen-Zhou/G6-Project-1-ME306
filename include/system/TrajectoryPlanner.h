#pragma once

/**
 * TrajectoryPlanner.h
 * Linear Cartesian trajectory planner for the MECHENG 306 X-Y plotter.
 *
 * One move is defined by a fixed start point and a fixed target point.
 * The planner uses one shared path distance for X and Y, so every generated
 * reference lies on the straight line between those two points.
 *
 * The feedrate is supplied in mm/min to match the G01 F value. The planner
 * converts it to mm/s and generates a trapezoidal velocity profile. For a
 * short move, it automatically generates a triangular velocity profile.
 *
 * This class does not parse G-code, convert X/Y to encoder counts, read
 * encoders, or control motors. Those jobs belong to other modules.
 */

namespace plotter
{

struct TrajectoryReference
{
    float xPositionMm;
    float yPositionMm;
    float xVelocityMmPerSecond;
    float yVelocityMmPerSecond;
    // This becomes true when reference generation reaches the target.
    // The system must still wait for both AxisControllers to settle.
    bool complete;
};

class TrajectoryPlanner
{
public:
    TrajectoryPlanner();

    // Start one straight-line move.
    //
    // startX/startY and targetX/targetY are Cartesian positions in mm.
    // For this project's relative G-code, the caller calculates:
    // target = start + G01 offset before calling this function.
    // feedrateMmPerMinute is the G01 F value.
    // maxAccelerationMmPerSecondSquared is the path acceleration limit.
    //
    // Returns false when feedrate or acceleration is not positive.
    bool startMove(float startXMm,
                   float startYMm,
                   float targetXMm,
                   float targetYMm,
                   float feedrateMmPerMinute,
                   float maxAccelerationMmPerSecondSquared);

    // Advance the trajectory by dt seconds and return the new reference.
    // A non-positive dt leaves the reference unchanged.
    TrajectoryReference update(float timeStepSeconds);

    // Cancel the current move and hold the latest generated position.
    void stop();

    bool isActive() const;
    bool isComplete() const;

private:
    float startXMm_;
    float startYMm_;
    float targetXMm_;
    float targetYMm_;

    float directionX_;
    float directionY_;
    float pathLengthMm_;

    float accelerationMmPerSecondSquared_;
    float peakVelocityMmPerSecond_;

    float accelerationTimeSeconds_;
    float cruiseTimeSeconds_;
    float totalTimeSeconds_;

    float accelerationDistanceMm_;
    float cruiseDistanceMm_;
    float elapsedTimeSeconds_;

    TrajectoryReference reference_;
    bool active_;
    bool complete_;
};

}  // namespace plotter
