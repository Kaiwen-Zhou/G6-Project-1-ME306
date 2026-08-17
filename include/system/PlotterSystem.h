#pragma once

#include <stdint.h>

#include "system/PlotterFSM.h"
#include "system/TrajectoryPlanner.h"
#include "control/AxisController.h"
#include "control/HomingController.h"
#include "control/XYCoordinator.h"

/**
 * PlotterSystem.h
 * System-level coordinator for the X-Y plotter.
 *
 * PlotterSystem connects the hardware-independent PlotterFSM to homing,
 * Cartesian trajectory
 * generation, coordinated A/B control, and the shared
 * fault-stop path. G-code remains an
 * application-layer client of this class.
 *
 */

namespace plotter {

/**
 * Top-level operation coordinator.
 *
 * For MOVING:
 *
 * TrajectoryPlanner
 *     -> Cartesian displacement and velocity reference
 * XYCoordinator
 *     -> synchronized A/B motor-space references
 * AxisController
 *     -> PID and motor output
 *
 * G1 X/Y values are treated as displacement relative to the beginning
 * of the current move.
 */
class PlotterSystem {
    public:
        PlotterSystem(AxisController& axisA, AxisController& axisB, XYCoordinator& xyCoordinator,
                      TrajectoryPlanner& trajectoryPlanner, HomingController& homingController);

        void begin();
        void update();

        FSMResult requestHoming();

        // Request one relative Cartesian move.
        //
        // xDisplacementMm and yDisplacementMm are relative to the beginning
        // of this move.
        //
        // feedrateMmPerMinute corresponds to the G1 F value.
        FSMResult requestMove(float xDisplacementMm, float yDisplacementMm, float feedrateMmPerMinute,
                              float maxAccelerationMmPerSecondSquared);

        FSMResult reportFault(FaultCode faultCode);
        FSMResult resetFault();

        PlotterState state() const;
        bool machineZeroKnown() const;
        FaultCode activeFault() const;

    private:
        FSMResult dispatchAndExecute(FSMEventType event, FaultCode faultCode = FaultCode::NONE);

        void executeAction(PlotterAction action);

        void updateHoming();
        void updateMoving();
        void stopAllMotion();
        static FaultCode mapHomingFault(HomingFault fault);

        PlotterFSM fsm_;

        // Axis references are retained so PlotterSystem can check whether
        // both final motor-space references have settled.
        AxisController& axisA_;
        AxisController& axisB_;

        XYCoordinator& xyCoordinator_;
        TrajectoryPlanner& trajectoryPlanner_;
        HomingController& homingController_;

        float pendingXDisplacementMm_;
        float pendingYDisplacementMm_;
        float pendingFeedrateMmPerMinute_;
        float pendingAccelerationMmPerSecondSquared_;

        unsigned long lastTrajectoryUpdateMicros_;

        bool moveSettling_;
        unsigned long moveSettlingStartMicros_;
};

} // namespace plotter
