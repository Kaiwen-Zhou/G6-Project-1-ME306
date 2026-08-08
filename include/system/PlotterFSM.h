#pragma once

#include <stdint.h>

/**
 * PlotterFSM.h
 * Hardware-independent top-level state machine for the X-Y plotter.
 *
 * PlotterFSM accepts high-level events, determines whether they are valid in
 * the current state, and returns an FSMResult containing the resulting state,
 * action, and any rejection reason. PlotterSystem should call begin() during
 * initialisation, pass events through dispatch(), and use state(),
 * machineZeroKnown(), and activeFault() for status queries.
 *
 * The FSM does not directly control hardware or queue pending commands. Move
 * requests currently require machine zero to be known, and requests received
 * while HOMING or MOVING are rejected as BUSY. The fault-reset transition is
 * included, but its external safety checks and reset policy are still to be
 * decided. Further states, events, actions, and fault codes can be added later.
 */

namespace plotter {

enum class PlotterState : uint8_t {
    IDLE, 
    HOMING,
    MOVING,
    FAULT
};

enum class FSMEventType : uint8_t {
    HOMING_REQUESTED,
    MOVE_REQUESTED,
    HOMING_COMPLETED,
    MOVE_COMPLETED,
    FAULT_DETECTED,
    FAULT_RESET_REQUESTED     
};

enum class FaultCode : uint8_t {
    NONE,
    UNEXPECTED_LIMIT,
    WRONG_HOMING_LIMIT,
    CONTRADICTORY_LIMITS,
    HOMING_TIMEOUT,
    MOVE_TIMEOUT,
    ENCODER_NO_MOTION,
    POSITION_OUT_OF_RANGE,
    INTERNAL_ERROR
};

enum class RejectReason : uint8_t {
    NONE,
    BUSY,
    MACHINE_ZERO_UNKNOWN,
    FAULT_ACTIVE,
    UNEXPECTED_EVENT,
    INVALID_MOTION_PARAMETERS
};

enum class PlotterAction : uint8_t {
    NONE,
    START_HOMING,
    START_MOVING,
    FINISH_HOMING,
    FINISH_MOVING,
    ENTER_FAULT,
    CLEAR_FAULT
};

struct FSMResult {
    bool accepted;
    PlotterState previousState;
    PlotterState currentState;
    PlotterAction action;
    RejectReason rejectReason;

    bool stateChanged() const {
        return previousState != currentState;
    }
};

class PlotterFSM {
 public:
    void begin();
    FSMResult dispatch(FSMEventType event, FaultCode faultCode = FaultCode::NONE);
    PlotterState state() const;
    bool machineZeroKnown() const;
    FaultCode activeFault() const;

 private:
    FSMResult transitionTo(PlotterState nextState, PlotterAction action);
    FSMResult reject(RejectReason reason) const;
    bool machineZeroKnown_ = false;
    PlotterState state_ = PlotterState::IDLE;   // default initial state
    FaultCode activeFault_ = FaultCode::NONE;
};

}  // namespace plotter
