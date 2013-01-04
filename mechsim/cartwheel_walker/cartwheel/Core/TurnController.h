#pragma once

#include <Core/IKVMCController.h>
#include <MathLib/Segment.h>
#include <Core/WorldOracle.h>

#include "InvPendulum.h"

/**
	A two-step, arbitrary velocity to arbitrary velocity, parameterizable rotation controller.
*/

class TurnController {
/* BEGIN BehaviourController */
protected:
	Character* bip;
	IKVMCController* lowLCon;
	WorldOracle* wo;

	//these are attributes/properties of the motion
	double desiredHeading;
	//double ubSagittalLean;
	//double ubCoronalLean;
	//double ubTwist;
	//double duckWalk;
	double velDSagittal;
	double velDCoronal;
	//double kneeBend;

public:
    void requestStepTime(double t) { stepTime = t; }
    void requestStepHeight(double h) { stepHeight = h; }
    
	void setDesiredHeading(double v);
	void setVelocities(double velDS, double velDC);

public:
	void adjustStepHeight();

	void requestVelocities(double velDS, double velDC);
	void requestCoronalStepWidth(double corSW);

	double getDesiredVelocitySagittal() const { return velDSagittal; }
	double getCoronalStepWidth() const { return lowLCon->ip.coronalStepWidth; }

/* END BehaviourController */

protected:
    double stepTime;
    double stepHeight;

	double initialHeading;
	double finalHeading;
	double turnAngle;
	bool stillTurning;
	Vector3d desiredVelocity;
	Vector3d initialVelocity;
	//some quaternions that will be useful..
	Quaternion currentHeadingQ, currentDesiredHeadingQ, finalHeadingQ, upperBodyTwistQ, tmpQ, tmpQ2;

	bool headingRequested;
	double requestedHeadingValue;

	//keep a copy of the body twist and desired heading here, so that they can't be messed with somewhere else...
	double turningBodyTwist;
	double turningDesiredHeading;
	double initialTiming;

	/**
		commence the turn...
	*/
	void initiateTurn(double finalHeading);

public:
	TurnController(Character* b, IKVMCController* llc, WorldOracle* w = NULL);

	/**
		this method gets called at every simulation time step
	*/
	void simStepPlan(double dt);

	/**
		this method gets called every time the controller transitions to a new state
	*/
	void conTransitionPlan();

	/**
		ask for a heading...
	*/
	void requestHeading(double v);
};


