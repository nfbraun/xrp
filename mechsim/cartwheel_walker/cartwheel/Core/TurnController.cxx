#include "TurnController.h"

TurnController::TurnController(Character* b, IKVMCController* llc, WorldOracle* w)
{
	this->bip = b;
	this->lowLCon = llc;
	this->wo = w;
	
	requestStepTime(0.6);
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
	lowLCon->velDSagittal = velDS;
	lowLCon->velDCoronal = velDC;
}

void TurnController::setDesiredHeading(double v){
	lowLCon->setDesiredHeading(v);
}

void TurnController::requestCoronalStepWidth(double corSW) {
	lowLCon->ip.coronalStepWidth = corSW;
}

void TurnController::adjustStepHeight(){
	lowLCon->unplannedForHeight = 0;
	if (wo != NULL)
		//the trajectory of the foot was generated without taking the environment into account, so check to see if there are any un-planned bumps (at somepoint in the near future)
		lowLCon->unplannedForHeight = wo->getWorldHeightAt(lowLCon->getSwingFootPos() + lowLCon->getSwingFootVel() * 0.1) * 1.5;

	//if the foot is high enough, we shouldn't do much about it... also, if we're close to the start or end of the
	//walk cycle, we don't need to do anything... the thing below is a quadratic that is 1 at 0.5, 0 at 0 and 1...
	double panicIntensity = -4 * lowLCon->getPhase() * lowLCon->getPhase() + 4 * lowLCon->getPhase();
	panicIntensity *= lowLCon->ip.getPanicLevel();
	lowLCon->panicHeight = panicIntensity * 0.05;
}

/* END BehaviourController */

/**
	ask for a heading...
*/
void TurnController::requestHeading(double v){
	stillTurning = false;
	headingRequested = true;
	requestedHeadingValue = v;

	//if the turn angle is pretty small, then just turn right away (or the old way... who knows).
	currentHeadingQ = bip->getHeading();
	finalHeadingQ.setToRotationQuaternion(v, PhysicsGlobals::up);
	tmpQ.setToProductOf(finalHeadingQ, currentHeadingQ, false, true);
	turnAngle = tmpQ.getRotationAngle(PhysicsGlobals::up);
	//printf("turnAngle: %lf\n", turnAngle);

	initialTiming = stepTime;

	if (fabs(turnAngle) < 1.0)
		initiateTurn(v);
}

/**
	this method gets called at every simulation time step
*/
void TurnController::simStepPlan(double dt)
{
	/* BEGIN BehaviourController::simStepPlan(dt); */
	// lowLCon->updateSwingAndStanceReferences();
	//if (lowLCon->getPhase() <= 0.01)
	//	lowLCon->ip.swingFootStartPos = lowLCon->getSwingFoot()->getWorldCoordinatesForPoint(bip->getJoints()[lowLCon->getSwingAnkleIndex()]->getChildJointPosition());

	setDesiredHeading(desiredHeading);
	setVelocities(velDSagittal, velDCoronal);

	//adjust for panic mode or unplanned terrain...
	adjustStepHeight();

	//and see if we're really in trouble...
    //	if (shouldAbort()) onAbort();
	/* END */
	if (stillTurning == false)
		return;

	//this is this guy's equivalent of panic management...
	double vLength = lowLCon->getV().length();
	if (vLength > 1.5*initialVelocity.length() && vLength > 1.5*desiredVelocity.length() && vLength > 0.5){
		std::cerr << "Panic in TurnController::simStepPlan" << std::endl;
		lowLCon->setStepTime(lowLCon->getStepTime() * 0.99);
//		tprintf("velocity is too large... changing transition time to: %lf\n", lowLCon->states[lowLCon->FSMStateIndex]->stateTime);
	} else {
		lowLCon->setStepTime(initialTiming);
	}

	//2 rads per second...
	double turnRate = dt * 2;

	//TODO: get all those limits on the twisting deltas to be related to dt, so that we get uniform turning despite the
	//having different dts

	currentHeadingQ = bip->getHeading();
	currentDesiredHeadingQ.setToRotationQuaternion(turningDesiredHeading, PhysicsGlobals::up);
	finalHeadingQ.setToRotationQuaternion(finalHeading, PhysicsGlobals::up);

	//this is the angle between the current heading and the final desired heading...
	tmpQ.setToProductOf(currentHeadingQ, finalHeadingQ, false, true);
	double curToFinal = tmpQ.getRotationAngle(PhysicsGlobals::up);
	//this is the angle between the set desired heading and the final heading - adding this to the current desired heading would reach finalHeading in one go...
	tmpQ.setToProductOf(finalHeadingQ, currentDesiredHeadingQ, false, true);
	double desToFinal = tmpQ.getRotationAngle(PhysicsGlobals::up);

	//do we still need to increase the desired heading?
	if (fabs(desToFinal) > 0.01){
		boundToRange(&desToFinal, -turnRate, turnRate);
		turningDesiredHeading += desToFinal;
	}

	double t = (lowLCon->getPhase()) - 0.2;
	boundToRange(&t, 0, 1);

	//are we there yet (or close enough)?
	if (fabs(curToFinal) < 0.2){
		//printf("done turning!\n");
		desiredHeading = turningDesiredHeading = finalHeading;
		//reset everything...
		setDesiredHeading(desiredHeading);
		setVelocities(velDSagittal, velDCoronal);
		lowLCon->setStepTime(initialTiming);
		stillTurning = false;
	}else{
		//still turning... so we need to still specify the desired velocity, in character frame...
		t = fabs(curToFinal/turnAngle) - 0.3;
		boundToRange(&t, 0, 1);
		Vector3d vD = initialVelocity*t + desiredVelocity*(1-t);
		vD = lowLCon->getCharacterFrame().inverseRotate(vD);
		setVelocities(vD.z, vD.x);
	}

	setDesiredHeading(turningDesiredHeading);
}

/**
	this method gets called every time the controller transitions to a new state
*/
void TurnController::conTransitionPlan()
{
    //we should estimate these from the character info...
    const double ankleBaseHeight = 0.04;
    
    lowLCon->setStepTime(stepTime);
    lowLCon->ip.swingFootStartPos = lowLCon->getSwingFootPos();
    
    //now prepare the step information for the following step:
    lowLCon->swingFootHeightTrajectory.clear();
    
    lowLCon->swingFootHeightTrajectory.addKnot(0, ankleBaseHeight);
    lowLCon->swingFootHeightTrajectory.addKnot(0.5, ankleBaseHeight + 0.01 + 0.1 + 0 + stepHeight);
    lowLCon->swingFootHeightTrajectory.addKnot(1, ankleBaseHeight + 0.01);
    
	if (headingRequested)
		initiateTurn(requestedHeadingValue);
}


/**
	commence turning...
*/
void TurnController::initiateTurn(double finalDHeading){
	if (stillTurning == false){
		turningDesiredHeading = bip->getHeadingAngle();;
	}

	headingRequested = false;
	stillTurning = true;

	currentHeadingQ = bip->getHeading();
	finalHeadingQ.setToRotationQuaternion(finalDHeading, PhysicsGlobals::up);
	tmpQ.setToProductOf(finalHeadingQ, currentHeadingQ, false, true);

	turnAngle = tmpQ.getRotationAngle(PhysicsGlobals::up);
	finalHeading = finalHeadingQ.getRotationAngle(PhysicsGlobals::up);
	initialHeading = currentHeadingQ.getRotationAngle(PhysicsGlobals::up);

	//printf("turnAngle: %lf. InitialHeading: %lf. Final Heading: %lf\n", turnAngle, initialHeading, finalHeading);

	initialVelocity = bip->getCOMVelocity();
	double finalVDSagittal = velDSagittal;
	boundToRange(&finalVDSagittal, -0.5, 0.6);
	if (fabs(turnAngle) > 2.5)
		boundToRange(&finalVDSagittal, -0.2, 0.3);
	desiredVelocity = Vector3d(0,0,finalVDSagittal).rotate(finalHeading, Vector3d(0,1,0));

	if (((lowLCon->getStance() == LEFT_STANCE && turnAngle < -1.5) || (lowLCon->getStance() == RIGHT_STANCE && turnAngle > 1.5)) && finalVDSagittal >=0){
		std::cout << "this is the bad side... try a smaller heading first..." << std::endl;
		if (lowLCon->getStance() == LEFT_STANCE) {
		    initiateTurn(initialHeading - 1.4);
		} else {
		    initiateTurn(initialHeading + 1.4);
		}
		desiredVelocity /= 0.5;
		headingRequested = true;
		requestedHeadingValue = finalDHeading;
	}
}

