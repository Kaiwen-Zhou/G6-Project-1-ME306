#include "system/PlotterFSM.h"

namespace plotter {

void PlotterFSM::begin() {
    machineZeroKnown_ = false;
    state_ = PlotterState::IDLE;
    activeFault_ = FaultCode::NONE;
}

FSMResult PlotterFSM::dispatch(FSMEventType event, FaultCode faultCode) {
    // Fault handling is checked before state-specific events so that a fault
    // can interrupt normal operation from any state.
    if (event == FSMEventType::FAULT_DETECTED) {
        if (faultCode == FaultCode::NONE) {
            return reject(RejectReason::UNEXPECTED_EVENT);
        }

        // Preserve the first latched fault and avoid repeating the FAULT entry
        // action while the FSM is already in the FAULT state.
        if (state_ == PlotterState::FAULT) {
            return {true, state_, state_, PlotterAction::NONE, RejectReason::NONE};
        }

        activeFault_ = faultCode;

        return transitionTo(PlotterState::FAULT, PlotterAction::ENTER_FAULT);
    }

    // A fault code is valid only when dispatching FAULT_DETECTED.
    if (faultCode != FaultCode::NONE) {
        return reject(RejectReason::UNEXPECTED_EVENT);
    }

    switch (state_) {
    case PlotterState::IDLE:
        switch (event) {
        case FSMEventType::HOMING_REQUESTED:
            // Starting a new homing operation invalidates the previous machine zero.
            machineZeroKnown_ = false;

            return transitionTo(PlotterState::HOMING, PlotterAction::START_HOMING);

        case FSMEventType::MOVE_REQUESTED:
            if (!machineZeroKnown_) {
                return reject(RejectReason::MACHINE_ZERO_UNKNOWN);
            }

            return transitionTo(PlotterState::MOVING, PlotterAction::START_MOVING);

        default:
            return reject(RejectReason::UNEXPECTED_EVENT);
        }

    case PlotterState::HOMING:
        switch (event) {
        case FSMEventType::HOMING_COMPLETED:
            machineZeroKnown_ = true;

            return transitionTo(PlotterState::IDLE, PlotterAction::FINISH_HOMING);

        case FSMEventType::HOMING_REQUESTED:
            return reject(RejectReason::BUSY);

        case FSMEventType::MOVE_REQUESTED:
            return reject(RejectReason::BUSY);

        default:
            return reject(RejectReason::UNEXPECTED_EVENT);
        }

    case PlotterState::MOVING:
        switch (event) {
        case FSMEventType::MOVE_COMPLETED:
            return transitionTo(PlotterState::IDLE, PlotterAction::FINISH_MOVING);

        case FSMEventType::HOMING_REQUESTED:
            return reject(RejectReason::BUSY);

        case FSMEventType::MOVE_REQUESTED:
            return reject(RejectReason::BUSY);

        default:
            return reject(RejectReason::UNEXPECTED_EVENT);
        }

    case PlotterState::FAULT:
        switch (event) {
        // The application verifies physical reset conditions before dispatch.
        case FSMEventType::FAULT_RESET_REQUESTED:
            activeFault_ = FaultCode::NONE;

            return transitionTo(PlotterState::IDLE, PlotterAction::CLEAR_FAULT);

        case FSMEventType::HOMING_REQUESTED:
            return reject(RejectReason::FAULT_ACTIVE);

        case FSMEventType::MOVE_REQUESTED:
            return reject(RejectReason::FAULT_ACTIVE);

        default:
            return reject(RejectReason::UNEXPECTED_EVENT);
        }
    }

    // Defensive fallback in case state_ is corrupted.
    activeFault_ = FaultCode::INTERNAL_ERROR;
    machineZeroKnown_ = false;

    return transitionTo(PlotterState::FAULT, PlotterAction::ENTER_FAULT);
}

PlotterState PlotterFSM::state() const {
    return state_;
}

bool PlotterFSM::machineZeroKnown() const {
    return machineZeroKnown_;
}

FaultCode PlotterFSM::activeFault() const {
    return activeFault_;
}

FSMResult PlotterFSM::transitionTo(PlotterState nextState, PlotterAction action) {
    const PlotterState previousState = state_;
    state_ = nextState;

    return {true, previousState, state_, action, RejectReason::NONE};
}

FSMResult PlotterFSM::reject(RejectReason reason) const {
    return {false, state_, state_, PlotterAction::NONE, reason};
}

} // namespace plotter
