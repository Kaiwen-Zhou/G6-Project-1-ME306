#include "system/PlotterSystem.h"

#include <Arduino.h>

#include "config/SystemConfig.h"

namespace plotter {

namespace {
constexpr float MICROSECONDS_TO_SECONDS = 0.000001f;
}

PlotterSystem::PlotterSystem(AxisController& axisA, AxisController& axisB, 
                             XYCoordinator& xyCoordinator,
                             TrajectoryPlanner& trajectoryPlanner, 
                             HomingController& homingController)
    : axisA_(axisA), axisB_(axisB), 
      xyCoordinator_(xyCoordinator), 
      trajectoryPlanner_(trajectoryPlanner),
      homingController_(homingController), 
      pendingXDisplacementMm_(0.0f), pendingYDisplacementMm_(0.0f),
      pendingFeedrateMmPerMinute_(0.0f), 
      pendingAccelerationMmPerSecondSquared_(0.0f), 
      lastTrajectoryUpdateMicros_(0),
      moveSettling_(false), moveSettlingStartMicros_(0) {
}

void PlotterSystem::begin() {
    // XYCoordinator owns the coordinated A/B control lifecycle and
    // initialises both AxisController instances.
    xyCoordinator_.begin();
    homingController_.begin();

    fsm_.begin();

    // Startup homing remains a system policy.
    dispatchAndExecute(FSMEventType::HOMING_REQUESTED);
}

void PlotterSystem::update() {
    switch (fsm_.state()) {
    case PlotterState::IDLE:
        break;

    case PlotterState::HOMING:
        updateHoming();
        break;

    case PlotterState::MOVING:
        updateMoving();
        break;

    case PlotterState::FAULT:
        // ENTER_FAULT already stopped all motion.
        break;
    }
}

FSMResult PlotterSystem::requestHoming() {
    return dispatchAndExecute(FSMEventType::HOMING_REQUESTED);
}

FSMResult PlotterSystem::requestMove(float xDisplacementMm, float yDisplacementMm, float feedrateMmPerMinute,
                                     float maxAccelerationMmPerSecondSquared) {
    // Using !(value > 0) also rejects NaN.
    if (!(feedrateMmPerMinute > 0.0f) || !(maxAccelerationMmPerSecondSquared > 0.0f)) {
        const PlotterState currentState = fsm_.state();

        return {false, currentState, currentState, PlotterAction::NONE, RejectReason::INVALID_MOTION_PARAMETERS};
    }

    const FSMResult result = fsm_.dispatch(FSMEventType::MOVE_REQUESTED);

    if (!result.accepted) {
        return result;
    }

    // Store the command payload only after the FSM accepts the move.
    pendingXDisplacementMm_ = xDisplacementMm;

    pendingYDisplacementMm_ = yDisplacementMm;

    pendingFeedrateMmPerMinute_ = feedrateMmPerMinute;

    pendingAccelerationMmPerSecondSquared_ = maxAccelerationMmPerSecondSquared;

    executeAction(result.action);

    return result;
}

FSMResult PlotterSystem::reportFault(FaultCode faultCode) {
    return dispatchAndExecute(FSMEventType::FAULT_DETECTED, faultCode);
}

FSMResult PlotterSystem::resetFault() {
    return dispatchAndExecute(FSMEventType::FAULT_RESET_REQUESTED);
}

PlotterState PlotterSystem::state() const {
    return fsm_.state();
}

bool PlotterSystem::machineZeroKnown() const {
    return fsm_.machineZeroKnown();
}

FaultCode PlotterSystem::activeFault() const {
    return fsm_.activeFault();
}

FSMResult PlotterSystem::dispatchAndExecute(FSMEventType event, FaultCode faultCode) {
    const FSMResult result = fsm_.dispatch(event, faultCode);

    if (result.accepted) {
        executeAction(result.action);
    }

    return result;
}

void PlotterSystem::executeAction(PlotterAction action) {
    switch (action) {
    case PlotterAction::NONE:
        break;

    case PlotterAction::START_HOMING:
        stopAllMotion();

        if (!homingController_.start()) {
            reportFault(mapHomingFault(homingController_.fault()));
        }
        break;

    case PlotterAction::START_MOVING: {
        moveSettling_ = false;

        // Because the planner now outputs displacement relative to
        // the current move, it can use (0, 0) as its local start.
        const bool trajectoryStarted =
            trajectoryPlanner_.startMove(0.0f, 0.0f, pendingXDisplacementMm_, pendingYDisplacementMm_,
                                         pendingFeedrateMmPerMinute_, pendingAccelerationMmPerSecondSquared_);

        if (!trajectoryStarted) {
            // Parameters were already validated, so failure here
            // indicates an internal inconsistency.
            reportFault(FaultCode::INTERNAL_ERROR);

            break;
        }

        // Capture one synchronized A/B encoder snapshot and use it
        // as the origin of this move.
        xyCoordinator_.startMove();

        lastTrajectoryUpdateMicros_ = micros();

        // Apply the initial zero-displacement reference.
        const TrajectoryReference initialReference = trajectoryPlanner_.update(0.0f);

        xyCoordinator_.setCartesianReference(initialReference.xDisplacementMm, initialReference.yDisplacementMm,
                                             initialReference.xVelocityMmPerSecond,
                                             initialReference.yVelocityMmPerSecond);

        break;
    }

    case PlotterAction::FINISH_HOMING:
        trajectoryPlanner_.stop();
        homingController_.stop();

        // HomingController has set both encoder counts to zero
        // Re-synchronise both motor-space controllers to that zero
        xyCoordinator_.reset();

        moveSettling_ = false;
        break;

    case PlotterAction::FINISH_MOVING:
    case PlotterAction::ENTER_FAULT:
    case PlotterAction::CLEAR_FAULT:
        stopAllMotion();
        break;
    }
}

void PlotterSystem::updateHoming() {
    homingController_.update();

    if (homingController_.hasFault()) {
        reportFault(mapHomingFault(homingController_.fault()));
        return;
    }

    if (homingController_.isComplete()) {
        dispatchAndExecute(FSMEventType::HOMING_COMPLETED);
    }
}

void PlotterSystem::updateMoving() {
    const unsigned long currentTimeMicros = micros();

    const unsigned long elapsedMicros = currentTimeMicros - lastTrajectoryUpdateMicros_;

    if (elapsedMicros > 0) {
        lastTrajectoryUpdateMicros_ = currentTimeMicros;

        const float timeStepSeconds = static_cast<float>(elapsedMicros) * MICROSECONDS_TO_SECONDS;

        const TrajectoryReference reference = trajectoryPlanner_.update(timeStepSeconds);

        xyCoordinator_.setCartesianReference(reference.xDisplacementMm, reference.yDisplacementMm,
                                             reference.xVelocityMmPerSecond, reference.yVelocityMmPerSecond);
    }

    // XYCoordinator performs one synchronized encoder snapshot and
    // gives both AxisControllers the same dt.
    xyCoordinator_.update();

    // Intermediate trajectory references are not move completion.
    if (!trajectoryPlanner_.isComplete()) {
        moveSettling_ = false;
        return;
    }

    // The trajectory has reached the final reference, but the physical
    // A/B axes may still be following it.
    if (!axisA_.isWithinTolerance() || !axisB_.isWithinTolerance()) {
        moveSettling_ = false;
        return;
    }

    if (!moveSettling_) {
        moveSettling_ = true;
        moveSettlingStartMicros_ = currentTimeMicros;

        return;
    }

    const unsigned long settledTimeMicros = currentTimeMicros - moveSettlingStartMicros_;

    if (settledTimeMicros >= SystemConfig::MOVE_SETTLE_TIME_MICROS) {
        dispatchAndExecute(FSMEventType::MOVE_COMPLETED);
    }
}

void PlotterSystem::stopAllMotion() {
    trajectoryPlanner_.stop();
    xyCoordinator_.stop();

    moveSettling_ = false;
}

FaultCode PlotterSystem::mapHomingFault(HomingFault fault) {
    switch (fault) {
    case HomingFault::TIMEOUT:
        return FaultCode::HOMING_TIMEOUT;

    case HomingFault::WRONG_LIMIT:
        return FaultCode::WRONG_HOMING_LIMIT;

    case HomingFault::CONTRADICTORY_LIMITS:
        return FaultCode::CONTRADICTORY_LIMITS;

    case HomingFault::INVALID_CONFIGURATION:
    case HomingFault::NONE:
        return FaultCode::INTERNAL_ERROR;
    }

    return FaultCode::INTERNAL_ERROR;
}

} // namespace plotter
