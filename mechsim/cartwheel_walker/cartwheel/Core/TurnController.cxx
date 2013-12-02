#include "TurnController.h"
#include "CWConfig.h"

TurnController::TurnController(WorldOracle* w)
{
	this->wo = w;
	
	ll_velDSagittal = 0;
    ll_velDCoronal = 0;
	ll_desiredHeading = 0.;
	
	ll_panicHeight = 0;
	ll_unplannedForHeight = 0;
	
	requestStepTime(0.8);
	requestStepHeight(0.);

	desiredHeading = 0;
	velDSagittal = 0;
	velDCoronal = 0;

	headingRequested = false;
	stillTurning = false;
	requestedHeadingValue = 0;
	initialTiming = 0;
}

/* BEGIN BehaviourController */
void TurnController::requestVelocities(double velDS, double velDC){
	velDSagittal = velDS;
	velDCoronal = velDC;
}

void TurnController::setVelocities(double velDS, double velDC){
	ll_velDSagittal = velDS;
	ll_velDCoronal = velDC;
}

void TurnController::setDesiredHeading(double v){
	ll_desiredHeading = v;
}

void TurnController::adjustStepHeight(const RobotInfo& rinfo)
{
	ll_unplannedForHeight = 0;
	if (wo != NULL)
		//the trajectory of the foot was generated without taking the environment into account, so check to see if there are any un-planned bumps (at somepoint in the near future)
		ll_unplannedForHeight = wo->getWorldHeightAt(Vector3d(rinfo.swingFootPos() + rinfo.swingFootVel() * 0.1)) * 1.5;

	//if the foot is high enough, we shouldn't do much about it... also, if we're close to the start or end of the
	//walk cycle, we don't need to do anything... the thing below is a quadratic that is 1 at 0.5, 0 at 0 and 1...
	double panicIntensity = -4 * rinfo.phi() * rinfo.phi() + 4 * rinfo.phi();
	panicIntensity *= InvPendulum::getPanicLevel(rinfo, ll_velDSagittal, ll_velDCoronal);
	ll_panicHeight = panicIntensity * 0.05;
}

/* END BehaviourController */

/**
	ask for a heading...
*/
void TurnController::requestHeading(const RobotInfo& rinfo, double v)
{
	stillTurning = false;
	headingRequested = true;
	requestedHeadingValue = v;

	//if the turn angle is pretty small, then just turn right away (or the old way... who knows).
	const Quaternion currentHeadingQ = rinfo.characterFrame();
	const Quaternion finalHeadingQ = Quaternion::QFromAngleAndAxis(v, CWConfig::UP);
	const Quaternion tmpQ = finalHeadingQ * currentHeadingQ.getComplexConjugate();
	turnAngle = tmpQ.getRotationAngle(CWConfig::UP);
	//printf("turnAngle: %lf\n", turnAngle);

	initialTiming = stepTime;

	if (fabs(turnAngle) < 1.0)
		initiateTurn(rinfo, v);
}

/**
	this method gets called at every simulation time step
*/
void TurnController::calcStepPlan(const RobotInfo& rinfo, double dt)
{
	/* BEGIN BehaviourController::simStepPlan(dt); */
	// lowLCon->updateSwingAndStanceReferences();
	//if (lowLCon->getPhase() <= 0.01)
	//	lowLCon->ip.swingFootStartPos = lowLCon->getSwingFoot()->getWorldCoordinatesForPoint(bip->getJoints()[lowLCon->getSwingAnkleIndex()]->getChildJointPosition());

	setDesiredHeading(desiredHeading);
	setVelocities(velDSagittal, velDCoronal);

	//adjust for panic mode or unplanned terrain...
	adjustStepHeight(rinfo);

	//and see if we're really in trouble...
    //	if (shouldAbort()) onAbort();
	/* END */
	if (stillTurning == false)
		return;

	//this is this guy's equivalent of panic management...
	double vLength = rinfo.getV().norm();
	if (vLength > 1.5*initialVelocity.norm() && vLength > 1.5*desiredVelocity.norm() && vLength > 0.5){
		std::cerr << "Panic in TurnController::simStepPlan" << std::endl;
		ll_stepTime *= 0.99;
//		tprintf("velocity is too large... changing transition time to: %lf\n", lowLCon->states[lowLCon->FSMStateIndex]->stateTime);
	} else {
		ll_stepTime = initialTiming;
	}

	//2 rads per second...
	double turnRate = dt * 2;

	//TODO: get all those limits on the twisting deltas to be related to dt, so that we get uniform turning despite the
	//having different dts

	const Quaternion currentHeadingQ = rinfo.characterFrame();
	const Quaternion currentDesiredHeadingQ =
	    Quaternion::QFromAngleAndAxis(turningDesiredHeading, CWConfig::UP);
	const Quaternion finalHeadingQ =
	    Quaternion::QFromAngleAndAxis(finalHeading, CWConfig::UP);

	//this is the angle between the current heading and the final desired heading...
	const Quaternion tmpQ = currentHeadingQ * finalHeadingQ.getComplexConjugate();
	double curToFinal = tmpQ.getRotationAngle(CWConfig::UP);
	//this is the angle between the set desired heading and the final heading - adding this to the current desired heading would reach finalHeading in one go...
	const Quaternion tmpQ2 = finalHeadingQ * currentDesiredHeadingQ.getComplexConjugate();
	double desToFinal = tmpQ2.getRotationAngle(CWConfig::UP);

	//do we still need to increase the desired heading?
	if (fabs(desToFinal) > 0.01){
		boundToRange(&desToFinal, -turnRate, turnRate);
		turningDesiredHeading += desToFinal;
	}

	double t = rinfo.phi() - 0.2;
	boundToRange(&t, 0, 1);

	//are we there yet (or close enough)?
	if (fabs(curToFinal) < 0.2){
		//printf("done turning!\n");
		desiredHeading = turningDesiredHeading = finalHeading;
		//reset everything...
		setDesiredHeading(desiredHeading);
		setVelocities(velDSagittal, velDCoronal);
		ll_stepTime = initialTiming;
		stillTurning = false;
	}else{
		//still turning... so we need to still specify the desired velocity, in character frame...
		t = fabs(curToFinal/turnAngle) - 0.3;
		boundToRange(&t, 0, 1);
		Vector3d vD = initialVelocity*t + desiredVelocity*(1-t);
		vD = Quaternion(rinfo.characterFrame()).inverseRotate(vD);
		setVelocities(vD.x(), vD.y());
	}

	setDesiredHeading(turningDesiredHeading);
}

HighLevelTarget TurnController::simStepPlan(const RobotInfo& rinfo, double dt)
{
    HighLevelTarget target;
    
    calcStepPlan(rinfo, dt);
    
    const double diff_dt = 0.001;
    const double hNow = ll_swingFootHeightTrajectory.evaluate_catmull_rom(rinfo.phi()) + ll_panicHeight + ll_unplannedForHeight;
    const double hFuture = ll_swingFootHeightTrajectory.evaluate_catmull_rom(rinfo.phi()+diff_dt) + ll_panicHeight + ll_unplannedForHeight;
    
    target.velDSagittal = ll_velDSagittal;
    target.velDCoronal = ll_velDCoronal;
    target.desiredHeading = ll_desiredHeading;
    target.swingFootHeight = hNow;
    target.swingFootHeightVel = (hFuture-hNow)/diff_dt;
    
    return target;
}

/**
	this method gets called every time the controller transitions to a new state
*/
void TurnController::conTransitionPlan(const RobotInfo& rinfo)
{
    //we should estimate these from the character info...
    const double ankleBaseHeight = 0.04;
    
    ll_stepTime = stepTime;
    
    //now prepare the step information for the following step:
    ll_swingFootHeightTrajectory.clear();
    
    ll_swingFootHeightTrajectory.addKnot(0, ankleBaseHeight);
    ll_swingFootHeightTrajectory.addKnot(0.5, ankleBaseHeight + 0.01 + 0.1 + 0 + stepHeight);
    ll_swingFootHeightTrajectory.addKnot(1, ankleBaseHeight + 0.01);
    
	if (headingRequested)
		initiateTurn(rinfo, requestedHeadingValue);
}


/**
	commence turning...
*/
void TurnController::initiateTurn(const RobotInfo& rinfo, double finalDHeading)
{
	if (stillTurning == false){
		turningDesiredHeading = rinfo.headingAngle();
	}

	headingRequested = false;
	stillTurning = true;

	const Quaternion currentHeadingQ = rinfo.characterFrame();
	const Quaternion finalHeadingQ = Quaternion::QFromAngleAndAxis(finalDHeading, CWConfig::UP);
	const Quaternion tmpQ = finalHeadingQ * currentHeadingQ.getComplexConjugate();
	turnAngle = tmpQ.getRotationAngle(CWConfig::UP);
	finalHeading = finalHeadingQ.getRotationAngle(CWConfig::UP);
	initialHeading = currentHeadingQ.getRotationAngle(CWConfig::UP);

	//printf("turnAngle: %lf. InitialHeading: %lf. Final Heading: %lf\n", turnAngle, initialHeading, finalHeading);

	initialVelocity = rinfo.comVel();
	double finalVDSagittal = velDSagittal;
	boundToRange(&finalVDSagittal, -0.5, 0.6);
	if (fabs(turnAngle) > 2.5)
		boundToRange(&finalVDSagittal, -0.2, 0.3);
	desiredVelocity = Vector3d(finalVDSagittal, 0., 0.).rotate(finalHeading, Vector3d(0,0,1));

	if (((rinfo.stance() == LEFT_STANCE && turnAngle < -1.5) || (rinfo.stance() == RIGHT_STANCE && turnAngle > 1.5)) && finalVDSagittal >=0){
		std::cout << "this is the bad side... try a smaller heading first..." << std::endl;
		if (rinfo.stance() == LEFT_STANCE) {
		    initiateTurn(rinfo, initialHeading - 1.4);
		} else {
		    initiateTurn(rinfo, initialHeading + 1.4);
		}
		desiredVelocity /= 0.5;
		headingRequested = true;
		requestedHeadingValue = finalDHeading;
	}
}

