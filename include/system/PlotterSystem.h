#pragma once

#include <stdint.h>

#include "system/PlotterFSM.h"
#include "system/TrajectoryPlanner.h"
#include "control/AxisController.h"
#include "control/HomingController.h"
#include "control/XYCoordinator.h"

/**
 * System-level operation coordinator for the X-Y plotter.
 *
 * Call begin() once, submit homing/move/fault requests through the public
 * methods, and call update() from the main loop. PlotterSystem executes FSM
 * actions by coordinating HomingController, TrajectoryPlanner, XYCoordinator,
 * and the shared motion-stop path. Move X/Y inputs are relative Cartesian
 * displacements from the beginning of the requested move.
 */
namespace plotter {

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
