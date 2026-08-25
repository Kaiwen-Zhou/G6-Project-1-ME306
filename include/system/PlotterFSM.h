#pragma once

#include <stdint.h>

/**
 * Hardware-independent top-level state machine for the X-Y plotter.
 *
 * PlotterFSM accepts high-level events, determines whether they are valid in
 * the current state, and returns an FSMResult containing the resulting state,
 * action, and any rejection reason. PlotterSystem should call begin() during
 * initialisation and execute the action returned by each accepted dispatch().
 *
 * The FSM does not control hardware or queue commands. Move requests require a
 * known machine zero, busy states reject new work, and FAULT preserves the
 * existing machine-zero flag. The application layer decides whether physical
 * switch conditions permit M999 before requesting a fault reset.
 */

namespace plotter {

enum class PlotterState : uint8_t { IDLE, HOMING, MOVING, FAULT };

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
        PlotterState state_ = PlotterState::IDLE; // default initial state
        FaultCode activeFault_ = FaultCode::NONE;
};

} // namespace plotter
