#include "system/PlotterSystem.h"

#include <Arduino.h>

namespace plotter {

namespace
{
// Temporary settling requirement for fixed A/B target tests.
// Move this value to SystemConfig later.
constexpr unsigned long MOVE_SETTLE_TIME_MICROS = 50000UL;
}

PlotterSystem::PlotterSystem(AxisController& axisA,
                             AxisController& axisB)
    : axisA_(axisA),
      axisB_(axisB),
      pendingAxisATargetCount_(0),
      pendingAxisBTargetCount_(0),
      moveSettling_(false),
      moveSettlingStartMicros_(0)
{
}

void PlotterSystem::begin()
{
    axisA_.begin();
    axisB_.begin();

    fsm_.begin();

    // Startup homing is a system policy rather than an FSM reset state.
    // The system therefore follows IDLE -> HOMING after initialisation.
    dispatchAndExecute(FSMEventType::HOMING_REQUESTED);
}

void PlotterSystem::update()
{
    switch (fsm_.state())
    {
        case PlotterState::IDLE:
            break;

        case PlotterState::HOMING:
            updateHoming();
            break;

        case PlotterState::MOVING:
            updateMoving();
            break;

        case PlotterState::FAULT:
            // Both motor-space controllers were stopped by ENTER_FAULT.
            break;
    }
}

FSMResult PlotterSystem::requestHoming()
{
    return dispatchAndExecute(FSMEventType::HOMING_REQUESTED);
}

FSMResult PlotterSystem::requestMove(
    int32_t axisATargetCount,
    int32_t axisBTargetCount)
{
    const FSMResult result =
        fsm_.dispatch(FSMEventType::MOVE_REQUESTED);

    if (!result.accepted)
    {
        return result;
    }

    // These are absolute motor-space encoder-count targets.
    // Store the payload only after the FSM accepts the request.
    pendingAxisATargetCount_ = axisATargetCount;
    pendingAxisBTargetCount_ = axisBTargetCount;

    executeAction(result.action);

    return result;
}

FSMResult PlotterSystem::reportHomingComplete()
{
    return dispatchAndExecute(FSMEventType::HOMING_COMPLETED);
}

FSMResult PlotterSystem::reportFault(FaultCode faultCode)
{
    return dispatchAndExecute(
        FSMEventType::FAULT_DETECTED,
        faultCode);
}

FSMResult PlotterSystem::resetFault()
{
    return dispatchAndExecute(
        FSMEventType::FAULT_RESET_REQUESTED);
}

PlotterState PlotterSystem::state() const
{
    return fsm_.state();
}

bool PlotterSystem::machineZeroKnown() const
{
    return fsm_.machineZeroKnown();
}

FaultCode PlotterSystem::activeFault() const
{
    return fsm_.activeFault();
}

FSMResult PlotterSystem::dispatchAndExecute(
    FSMEventType event,
    FaultCode faultCode)
{
    const FSMResult result =
        fsm_.dispatch(event, faultCode);

    if (result.accepted)
    {
        executeAction(result.action);
    }

    return result;
}

void PlotterSystem::executeAction(PlotterAction action)
{
    switch (action)
    {
        case PlotterAction::NONE:
            break;

        case PlotterAction::START_HOMING:
            stopAllAxes();

            // HomingController will be started here once its
            // non-blocking interface is available.
            break;

        case PlotterAction::START_MOVING:
            moveSettling_ = false;

            // startTracking() resets each PID only once.
            axisA_.startTracking();
            axisB_.startTracking();

            // Apply the fixed A/B references without resetting the PIDs.
            axisA_.setReferencePosition(
                pendingAxisATargetCount_);

            axisB_.setReferencePosition(
                pendingAxisBTargetCount_);
            break;

        case PlotterAction::FINISH_HOMING:
            stopAllAxes();
            break;

        case PlotterAction::FINISH_MOVING:
            stopAllAxes();
            break;

        case PlotterAction::ENTER_FAULT:
            stopAllAxes();
            break;

        case PlotterAction::CLEAR_FAULT:
            stopAllAxes();
            axisA_.reset();
            axisB_.reset();
            break;
    }
}

void PlotterSystem::updateHoming()
{
    // Future implementation:
    //
    // homingController_.update();
    //
    // if (homingController_.isComplete())
    // {
    //     dispatchAndExecute(FSMEventType::HOMING_COMPLETED);
    // }
    //
    // if (homingController_.hasFault())
    // {
    //     reportFault(homingController_.faultCode());
    // }
}

void PlotterSystem::updateMoving()
{
    axisA_.update();
    axisB_.update();

    // For the current fixed-target implementation, the movement is
    // settled when both motor-space controllers remain within tolerance
    // continuously for the configured settling time.
    if (!axisA_.isWithinTolerance() ||
        !axisB_.isWithinTolerance())
    {
        moveSettling_ = false;
        return;
    }

    const unsigned long currentTimeMicros = micros();

    if (!moveSettling_)
    {
        moveSettling_ = true;
        moveSettlingStartMicros_ = currentTimeMicros;
        return;
    }

    const unsigned long settledTimeMicros =
        currentTimeMicros - moveSettlingStartMicros_;

    if (settledTimeMicros >= MOVE_SETTLE_TIME_MICROS)
    {
        dispatchAndExecute(FSMEventType::MOVE_COMPLETED);
    }
}

void PlotterSystem::stopAllAxes()
{
    axisA_.stop();
    axisB_.stop();

    moveSettling_ = false;
}

}  // namespace plotter