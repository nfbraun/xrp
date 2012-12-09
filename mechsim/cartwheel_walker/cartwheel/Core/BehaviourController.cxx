#include "BehaviourController.h"
#include <MathLib/Trajectory.h>

BehaviourController::BehaviourController(Character* b, IKVMCController* llc, WorldOracle* w)
{
	this->bip = b;
	this->lowLCon = llc;
	this->wo = w;

	//we should estimate these from the character info...
	ankleBaseHeight = 0.04;

	desiredHeading = 0;
	ubSagittalLean = 0;
	ubCoronalLean = 0;
	ubTwist = 0;
	duckWalk = 0;
	duckWalk = 0;
	velDSagittal = 0;
	velDCoronal = 0;
	kneeBend = 0;

	stepTime = 0.6;
	stepHeight = 0;
}

/**
	ask for a heading...
*/
/* void BehaviourController::requestHeading(double v){
	desiredHeading = v;
} */


void BehaviourController::requestVelocities(double velDS, double velDC){
	velDSagittal = velDS;
	velDCoronal = velDC;
}

/**
	sets a bunch of parameters to some default initial value
*/
void BehaviourController::initializeDefaultParameters(){
	lowLCon->updateDAndV();
	desiredHeading = 0;
}

void BehaviourController::setVelocities(double velDS, double velDC){
	lowLCon->velDSagittal = velDS;
	lowLCon->velDCoronal = velDC;
}

void BehaviourController::setDesiredHeading(double v){
	lowLCon->setDesiredHeading(v);
}

void BehaviourController::requestCoronalStepWidth(double corSW) {
	lowLCon->ip.coronalStepWidth = corSW;
}

void BehaviourController::adjustStepHeight(){
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

/**
	this method gets called at every simulation time step
*/
void BehaviourController::simStepPlan(double dt){
	lowLCon->updateSwingAndStanceReferences();
	if (lowLCon->getPhase() <= 0.01)
		lowLCon->ip.swingFootStartPos = lowLCon->getSwingFoot()->getWorldCoordinatesForPoint(bip->getJoints()[lowLCon->getSwingAnkleIndex()]->getChildJointPosition());

	//set some of these settings
	//setUpperBodyPose(ubSagittalLean, ubCoronalLean, ubTwist);
	//setKneeBend(kneeBend);
	//setDuckWalkDegree((lowLCon->stance == LEFT_STANCE)?(-duckWalk):(duckWalk));
	setDesiredHeading(desiredHeading);
	setVelocities(velDSagittal, velDCoronal);

	//adjust for panic mode or unplanned terrain...
	adjustStepHeight();

	//and see if we're really in trouble...
//	if (shouldAbort()) onAbort();
}

void BehaviourController::requestStepTime(double stepTime){
	this->stepTime = stepTime;
}

void BehaviourController::requestStepHeight(double stepHeight){
	this->stepHeight = stepHeight;
}

/**
	this method gets called every time the controller transitions to a new state
*/
void BehaviourController::conTransitionPlan(){
	lowLCon->updateSwingAndStanceReferences();
	lowLCon->updateDAndV();
	lowLCon->getState()->setStateTime(stepTime);

	lowLCon->updateSwingAndStanceReferences();
	lowLCon->ip.swingFootStartPos = lowLCon->getSwingFootPos();

	//now prepare the step information for the following step:
	lowLCon->swingFootHeightTrajectory.clear();

	lowLCon->swingFootHeightTrajectory.addKnot(0, ankleBaseHeight);
	lowLCon->swingFootHeightTrajectory.addKnot(0.5, ankleBaseHeight + 0.01 + 0.1 + 0 + stepHeight);
	lowLCon->swingFootHeightTrajectory.addKnot(1, ankleBaseHeight + 0.01);
}

/**
	this method determines if the character should aim to abort the given plan, and do something else instead (like maybe transition to the
	next state of the FSM early).
*/
bool BehaviourController::shouldAbort(){
	Vector3d step(lowLCon->getStanceFootPos(), lowLCon->getSwingFootPos());
	step = lowLCon->getCharacterFrame().inverseRotate(step);

	//TODO: MIGHT NEED TO MAKE THESE CONSTANTS A FUNCTION OF THE LEG LENGTH AT SOME POINT!!!!!
	if ((step.z > 0.6 && lowLCon->getV().z > 0.2) || (step.z < -0.55 && lowLCon->getV().z < -0.2))
		return true;
	
	if (lowLCon->getStance() == LEFT_STANCE){
		if ((step.x < -0.45 && lowLCon->getV().x < -0.2) || (step.x > 0.35 && lowLCon->getV().x > 0.2))
			return true;
	}else{
		if ((step.x > 0.45 && lowLCon->getV().x > 0.2) || (step.x < -0.35 && lowLCon->getV().x < -0.2))
			return true;
	}

	return false;
}

/**
	this method is used to indicate what the behaviour of the character should be, once it decides to abort its plan.
*/
void BehaviourController::onAbort(){
	//force a premature switch to the next controller state
	if (lowLCon->getPhase() > 0.2){
		lowLCon->setPhase(1.);
		std::cout << "PANIC!!!" << std::endl;
	}
//	Globals::animationRunning = false;
//	return;
}

void BehaviourController::requestUpperBodyPose(double leanS, double leanC, double twist){
	this->ubSagittalLean = leanS;
	this->ubCoronalLean = leanC;
	this->ubTwist = twist;
}

void BehaviourController::requestKneeBend(double kb){
	this->kneeBend = kb;
}

void BehaviourController::requestDuckFootedness(double df){
	this->duckWalk = df;
}

