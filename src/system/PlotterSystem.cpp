#include "system/PlotterSystem.h"

namespace plotter {

PlotterSystem::PlotterSystem(AxisController& axisX, AxisController& axisY)
    : axisX_(axisX),
      axisY_(axisY),
      pendingAxisXTargetCount_(0),
      pendingAxisYTargetCount_(0) {}

void PlotterSystem::begin() {
    axisX_.begin();
    axisY_.begin();

    fsm_.begin();

    // Startup homing - a system policy rather than an FSM reset state
    // Keeps the FSM reusable while ensuring startup follows IDLE -> HOMING.
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
            // both axes are stopped by the Enter Fault action
            break;
    }
}

FSMResult PlotterSystem::requestHoming() {
    return dispatchAndExecute(FSMEventType::HOMING_REQUESTED);
}

FSMResult PlotterSystem::requestMove(int32_t axisXTargetCount, int32_t axisYTargetCount) {
    const FSMResult result = fsm_.dispatch(FSMEventType::MOVE_REQUESTED);

    if (!result.accepted) {
        return result;
    }

    // store the command payload only after the FSM accepts the request
    pendingAxisXTargetCount_ = axisXTargetCount;
    pendingAxisYTargetCount_ = axisYTargetCount;
    executeAction(result.action);

    return result;
}

FSMResult PlotterSystem::reportHomingComplete() {
    return dispatchAndExecute(FSMEventType::HOMING_COMPLETED);
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
            stopAllAxes();
            // The HomingController will be started here onece its interface is availabel
            // The system safely remains in HOMING currently
            break;
        
        case PlotterAction::START_MOVING:
            axisX_.setTargetPosition(pendingAxisXTargetCount_);
            axisY_.setTargetPosition(pendingAxisYTargetCount_);
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
            axisX_.reset();
            axisY_.reset();
            break;
    }
}

void PlotterSystem::updateHoming() {
    // Compilable placeholder for future homing update logic
    // waiting for HomingController 
    // The HomingController will perform one non-blocking homing step here and report HOMING_COMPLETED or a fault.

}

void PlotterSystem::updateMoving() {
    axisX_.update();
    axisY_.update();

    if (axisX_.isComplete() && axisY_.isComplete()) {
        dispatchAndExecute(FSMEventType::MOVE_COMPLETED);
    }
}

void PlotterSystem::stopAllAxes() {
    axisX_.stop();
    axisY_.stop();
}

}   // namespace plotter