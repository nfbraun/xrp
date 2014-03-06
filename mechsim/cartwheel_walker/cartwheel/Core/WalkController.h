#pragma once

#include "SwingController.h"
#include "InvPendulum.h"

/**
    A simplified version of TurnController for analysis/testing.
    This controller does straight line walking on an even surface with constant desired velocities.
*/

class WalkController {
  public:
    WalkController(double velDSagittal, double velDCoronal = 0.);
    
    // this method gets called at every simulation time step
    HighLevelTarget simStepPlan(const RobotInfo& rinfo, double dt);
    
    double getStepTime() const { return 0.8; }

  private:
    double adjustStepHeight(const RobotInfo& rinfo);

  private:
    /*** "output vars" ***/
    //desired velocity in the sagittal plane
    double ll_velDSagittal;
    //desired velocity in the coronal plane...
    double ll_velDCoronal;
    
    double ll_desiredHeading;
    
    //this is a desired foot height trajectory that we may wish to follow, relative to the current location of the CM
    Trajectory1d ll_swingFootHeightTrajectory;
};
