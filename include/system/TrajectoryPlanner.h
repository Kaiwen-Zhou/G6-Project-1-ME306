#pragma once

/**
 * Linear Cartesian trajectory planner for the plotter.
 *
 * One move is defined by a fixed start point and target point. The generated
 * displacement reference is relative to the start of the current move:
 *
 *     start of move -> displacement (0, 0)
 *     end of move   -> displacement (target - start)
 *
 * This allows the output to be passed directly to XYCoordinator.
 *
 * Feedrate is supplied in mm/min to match the G1 F value. Internally the
 * planner generates either a trapezoidal or triangular velocity profile.
 */

namespace plotter
{

struct TrajectoryReference
{
    // Relative to the start of the current move.
    float xDisplacementMm;
    float yDisplacementMm;

    float xVelocityMmPerSecond;
    float yVelocityMmPerSecond;

    // This becomes true when reference generation reaches the target.
    // The system must still wait for both motor controllers to settle.
    bool complete;
};

class TrajectoryPlanner
{
public:
    TrajectoryPlanner();

    // Start one straight-line move.
    //
    // startX/startY and targetX/targetY are Cartesian positions in mm.
    // Generated references are still relative to startX/startY.
    //
    // feedrateMmPerMinute is the G1 F value.
    // maxAccelerationMmPerSecondSquared is the path acceleration limit.
    //
    // Returns false when feedrate or acceleration is not positive.
    bool startMove(
        float startXMm,
        float startYMm,
        float targetXMm,
        float targetYMm,
        float feedrateMmPerMinute,
        float maxAccelerationMmPerSecondSquared);

    // Advance the trajectory by dt seconds.
    // A non-positive dt leaves the reference unchanged.
    TrajectoryReference update(float timeStepSeconds);

    // Cancel the current move and hold the latest displacement.
    void stop();

    bool isActive() const;
    bool isComplete() const;

private:
    // Final displacement relative to the beginning of the move.
    float targetXDisplacementMm_;
    float targetYDisplacementMm_;

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
