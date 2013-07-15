#pragma once

#include "SwingController.h"
#include <MathLib/Segment.h>
#include <Core/WorldOracle.h>

#include "InvPendulum.h"

/**
	A two-step, arbitrary velocity to arbitrary velocity, parameterizable rotation controller.
*/

class TurnController {
  public:
	TurnController(WorldOracle* w = NULL);
	
	// this method gets called at every simulation time step
	HighLevelTarget simStepPlan(const RobotInfo& rinfo, double dt);
	
	// this method gets called every time the controller transitions to a new state
	void conTransitionPlan(const RobotInfo& rinfo);
	
	// ask for a heading...
	void requestHeading(const RobotInfo& rinfo, double v);
	void requestVelocities(double velDS, double velDC);
	
	double getStepTime() const { return ll_stepTime; }

  private:
	void calcStepPlan(const RobotInfo& rinfo, double dt);
	
	// commence the turn...
	void initiateTurn(const RobotInfo& rinfo, double finalHeading);
	
	void requestStepTime(double t) { stepTime = t; }
    void requestStepHeight(double h) { stepHeight = h; }
    
	void setDesiredHeading(double v);
	void setVelocities(double velDS, double velDC);

	void adjustStepHeight(const RobotInfo& rinfo);

	double getDesiredVelocitySagittal() const { return velDSagittal; }

  private:
	WorldOracle* wo;

	//these are attributes/properties of the motion
	double desiredHeading;
	double velDSagittal;
	double velDCoronal;

    double stepTime;
    double stepHeight;

	double initialHeading;
	double finalHeading;
	double turnAngle;
	bool stillTurning;
	Vector3d desiredVelocity;
	Vector3d initialVelocity;

	bool headingRequested;
	double requestedHeadingValue;

	//keep a copy of the body twist and desired heading here, so that they can't be messed with somewhere else...
	double turningDesiredHeading;
	double initialTiming;
	    
    /*** "output vars" ***/
    //desired velocity in the sagittal plane
    double ll_velDSagittal;
    //desired velocity in the coronal plane...
    double ll_velDCoronal;
	
	double ll_desiredHeading;
	double ll_stepTime;
	
	//this is a desired foot trajectory that we may wish to follow, expressed separately, for the 3 components,
	//and relative to the current location of the CM
	Trajectory1d ll_swingFootHeightTrajectory;
	
	//this variable can be used to quickly alter the desired height, if panic ensues...
	double ll_panicHeight;
	//and this should be used to add height for the leg (i.e. if it needs to step over an obstacle that wasn't planned for).
	double ll_unplannedForHeight;
};

