#pragma once

#include <stdint.h>

#include "PlotterFSM.h"
#include "control/AxisController.h"

/**
 * PlotterSystem.h
 * System-level coordinator for the X-Y plotter.
 *
 * PlotterSystem connects the hardware-independent PlotterFSM to the two motor
 * AxisController objects. It owns the high-level operation lifecycle, provides
 * one shared path for stopping both axes, and exposes event-style entry points
 * for later G-code, homing, and safety modules.
 *
 * The first version deliberately leaves the physical homing sequence and
 * Cartesian trajectory generation outside this class. Move targets are motor
 * encoder counts (A and B), not Cartesian X-Y coordinates.
 */

namespace plotter {

class PlotterSystem {
 public:
    PlotterSystem(AxisController& axisA, AxisController& axisB);

    // Initialise both axes and automatically enter the startup HOMING state.
    void begin();

    // Call repeatedly from loop(). All operation updates remain non-blocking.
    void update();

    // High-level command/event entry points.
    FSMResult requestHoming();
    FSMResult requestMove(int32_t axisATargetCount, int32_t axisBTargetCount);

    // Call only after the homing module has stopped the mechanism and zeroed
    // both encoder counts successfully.
    FSMResult reportHomingComplete();

    // Safety/fault entry point. FaultCode::NONE is rejected by the FSM.
    FSMResult reportFault(FaultCode faultCode);

    // Clear a latched fault and return to IDLE. The FSM decides whether the
    // previously established machine zero remains known.
    FSMResult resetFault();

    PlotterState state() const;
    bool machineZeroKnown() const;
    FaultCode activeFault() const;

 private:
    FSMResult dispatchAndExecute(
        FSMEventType event,
        FaultCode faultCode = FaultCode::NONE);

    void executeAction(PlotterAction action);
    void updateHoming();
    void updateMoving();
    void stopAllAxes();

    PlotterFSM fsm_;
    AxisController& axisA_;
    AxisController& axisB_;

    int32_t pendingAxisATargetCount_;
    int32_t pendingAxisBTargetCount_;

    bool moveSettling_;
    unsigned long moveSettlingStartMicros_;
};

}  // namespace plotter
