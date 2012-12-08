#pragma once

#include <Core/IKVMCController.h>
#include <MathLib/Segment.h>
#include <Core/WorldOracle.h>

#include "InvPendulum.h"

/**
	This class implements an intermediate-level controller. Given a low level controller (of the type IKVMCController, for now),
	this class knows how to set a series of parameters in order to accomplish higher level behaviours.

	NOTE: We will assume a fixed character morphology (i.e. joints & links), and a fixed controller structure 
	(i.e. trajectories,	components).
*/

class BehaviourController{
protected:
	Character* bip;
	IKVMCController* lowLCon;
	WorldOracle* wo;

	double ankleBaseHeight;

	//these are attributes/properties of the motion
	double desiredHeading;
	double ubSagittalLean;
	double ubCoronalLean;
	double ubTwist;
	double duckWalk;
	double velDSagittal;
	double velDCoronal;
	double kneeBend;

	double stepTime;
	double stepHeight;

public:
/*
	//DEBUG ONLY
	Point3d predSwingFootPosDebug;
	Point3d viaPointSuggestedDebug;
	Point3d suggestedFootPosDebug;
	Segment swingSegmentDebug;
	Segment crossSegmentDebug;
*/

	/**
		a set of useful virtual functions, whose behavior can be overridden
	*/
	//virtual void setUpperBodyPose(double leanSagittal, double leanCoronal, double twist);
	virtual void setDesiredHeading(double v);
	virtual void setVelocities(double velDS, double velDC);

public:
	BehaviourController(Character* b, IKVMCController* llc, WorldOracle* w = NULL);

	virtual void adjustStepHeight();

	//virtual void setElbowAngles(double leftElbowAngle, double rightElbowAngle);
	//virtual void setShoulderAngles(double leftTwist, double rightTwist, double leftAdduction, double rightAdduction, double leftSwing, double rightSwing);


	virtual void requestStepTime(double stepTime);
	virtual void requestStepHeight(double stepHeight);
	virtual void requestVelocities(double velDS, double velDC);
	virtual void requestUpperBodyPose(double leanS, double leanC, double twist);
	virtual void requestKneeBend(double kb);
	virtual void requestDuckFootedness(double df);
	virtual void requestCoronalStepWidth(double corSW);

	double getDesiredStepTime() const { return stepTime; }
	double getDesiredVelocitySagittal() const { return velDSagittal; }
	double getCoronalStepWidth() const { return ip.coronalStepWidth; }

	/**
		ask for a heading...
	*/
	//virtual void requestHeading(double v);

	/**
		sets a bunch of parameters to some default initial value
	*/
	virtual void initializeDefaultParameters();

	/**
		this method gets called at every simulation time step
	*/
	virtual void simStepPlan(double dt);

	/**
		this method gets called every time the controller transitions to a new state
	*/
	virtual void conTransitionPlan();

	/**
		this method determines if the character should aim to abort the given plan, and do something else instead (like maybe transition to the
		next state of the FSM early).
	*/
	virtual bool shouldAbort();

	/**
		this method is used to indicate what the behaviour of the character should be, once it decides to abort its plan.
	*/
	virtual void onAbort();
	
	InvPendulum ip;
};


