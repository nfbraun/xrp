/*
	Simbicon 1.5 Controller Editor Framework, 
	Copyright 2009 Stelian Coros, Philippe Beaudoin and Michiel van de Panne.
	All rights reserved. Web: www.cs.ubc.ca/~van/simbicon_cef

	This file is part of the Simbicon 1.5 Controller Editor Framework.

	Simbicon 1.5 Controller Editor Framework is free software: you can 
	redistribute it and/or modify
	it under the terms of the GNU General Public License as published by
	the Free Software Foundation, either version 3 of the License, or
	(at your option) any later version.

	Simbicon 1.5 Controller Editor Framework is distributed in the hope 
	that it will be useful, but WITHOUT ANY WARRANTY; without even the 
	implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  
	See the GNU General Public License for more details.

	You should have received a copy of the GNU General Public License
	along with Simbicon 1.5 Controller Editor Framework. 
	If not, see <http://www.gnu.org/licenses/>.
*/

#include "SimBiController.h"
#include <iostream>
#include <stdexcept>

SimBiController::SimBiController(Character* b)
{
	if (b == NULL)
		throw std::runtime_error("Cannot create a SIMBICON controller if there is no associated biped!!");

	this->character = b;
	jointCount = b->getJointCount();

	for (int i=0;i<jointCount;i++){
		character->getJoints()[i]->id = i;
	}

	//characters controlled by a simbicon controller are assumed to have: 2 feet
	lFoot = b->getARB(R_L_FOOT);
	rFoot = b->getARB(R_R_FOOT);

	if (rFoot == NULL || lFoot == NULL)
		throw std::runtime_error("The biped must have the rigid bodies lFoot and rFoot!");
	
	//and two hips connected to the root
	KTJoint* lHip = b->getJoint(J_L_HIP);
	KTJoint* rHip = b->getJoint(J_R_HIP);

	if (rFoot == NULL || lFoot == NULL)
		throw std::runtime_error("The biped must have the joints lHip and rHip!");

	root = b->getRoot();
	
	if (lHip->getParent() != rHip->getParent() || lHip->getParent() != root)
		throw std::runtime_error("The biped's hips must have a common parent, which should be the root of the figure!");

	setStance(LEFT_STANCE);
	phi = 0;
	desiredHeading = 0;

	//stanceHipDamping = -1;
	//stanceHipMaxVelocity = 4;
	
	// bodyTouchedTheGround = false;
}

/**
	This method is used to set the stance 
*/
void SimBiController::setStance(int newStance)
{
	stance = newStance;
	if (stance == LEFT_STANCE){
		stanceFoot = lFoot;
		swingFoot = rFoot;
		swingHipIndex = J_R_HIP;
		stanceHipIndex = J_L_HIP;
	}else{
		stanceFoot = rFoot;
		swingFoot = lFoot;
		swingHipIndex = J_L_HIP;
		stanceHipIndex = J_R_HIP;
	}
}

void SimBiController::toggleStance()
{
    if(stance == LEFT_STANCE) setStance(RIGHT_STANCE);
    else setStance(LEFT_STANCE);
}

/**
	This method should be called when the controller transitions to this state.
*/
void SimBiController::transitionToState(int stateIndex){
	toggleStance();
//	tprintf("Transition to state: %d (stance = %s) (phi = %lf)\n", stateIndex, (stance == LEFT_STANCE)?("left"):("right"), phi);
	//reset the phase...
	this->phi = 0;
}

/**
	This method returns the net force on the body rb, acting from the ground
*/
Vector3d SimBiController::getForceOn(RigidBody* rb, const std::vector<ContactPoint>& cfs){
	Vector3d fNet = Vector3d();
	for (unsigned int i=0;i<cfs.size();i++){
		if (cfs[i].rb1 == rb)
			fNet += cfs[i].f;
		if (cfs[i].rb2 == rb)
			fNet -= cfs[i].f;
	}
	return fNet;
}





/**
	check to see if rb is the same as whichBody or any of its children
*/
bool SimBiController::haveRelationBetween(RigidBody* rb, RigidBody* whichBody){
	//check against the feet
	if (rb == whichBody)
		return true;
	for (unsigned int j=0;j<((ArticulatedRigidBody*)whichBody)->cJoints.size();j++)
		if (((ArticulatedRigidBody*)whichBody)->cJoints[j]->child == rb)
			return true;
	return false;
}

/**
	This method is used to determine if the rigid body that is passed in as a parameter is a
	part of a foot
*/
bool SimBiController::isFoot(RigidBody* rb){
	//check against the feet
	return haveRelationBetween(rb, lFoot) || haveRelationBetween(rb, rFoot);

	if (rb == lFoot || rb == rFoot)
		return true;
	//and against the toes
	for (unsigned int j=0;j<((ArticulatedRigidBody*)lFoot)->cJoints.size();j++)
		if (((ArticulatedRigidBody*)lFoot)->cJoints[j]->child == rb)
			return true;
	for (unsigned int j=0;j<((ArticulatedRigidBody*)rFoot)->cJoints.size();j++)
		if (((ArticulatedRigidBody*)rFoot)->cJoints[j]->child == rb)
			return true;

	return false;

}

/**
	This method returns true if the rigid body that is passed in as a parameter is a swing foot, false otherwise
*/
bool SimBiController::isSwingFoot(RigidBody* rb){
	//check against the feet
	if (rb == swingFoot)
		return true;
	//and against the toes
	for (unsigned int j=0;j<((ArticulatedRigidBody*)swingFoot)->cJoints.size();j++)
		if (((ArticulatedRigidBody*)swingFoot)->cJoints[j]->child == rb)
			return true;
	return false;
}

/**
	This method returns true if the rigid body that is passed in as a parameter is a swing foot, false otherwise
*/
bool SimBiController::isStanceFoot(RigidBody* rb){
	//check against the feet
	if (rb == stanceFoot)
		return true;
	//and against the toes
	for (unsigned int j=0;j<((ArticulatedRigidBody*)stanceFoot)->cJoints.size();j++)
		if (((ArticulatedRigidBody*)stanceFoot)->cJoints[j]->child == rb)
			return true;
	return false;
}

/**
	This method returns the net force on the body rb, acting from the ground
*/
Vector3d SimBiController::getForceOnFoot(RigidBody* foot, const std::vector<ContactPoint>& cfs)
{
	Vector3d fNet = getForceOn(foot, cfs);

	//we will also look at all children of the foot that is passed in (to take care of toes).
	for (unsigned int i=0;i<((ArticulatedRigidBody*)foot)->cJoints.size();i++){
		fNet += getForceOn(((ArticulatedRigidBody*)foot)->cJoints[i]->child, cfs);
	}
	return fNet;
}

/**
	This method is used to advance the controller in time. It takes in a list of the contact points, since they might be
	used to determine when to transition to a new state. This method returns -1 if the controller does not advance to a new state,
	or the index of the state that it transitions to otherwise.
*/
int SimBiController::advanceInTime(double dt, const std::vector<ContactPoint>& cfs){

	if( dt <= 0 )
		return -1;

	//advance the phase of the controller
	this->phi += dt/state.getStateTime();

	//see if we have to transition to the next state in the FSM, and do it if so...
	if (needTransition(phi, fabs(getForceOnFoot(swingFoot, cfs).dotProductWith(PhysicsGlobals::up)), fabs(getForceOnFoot(stanceFoot, cfs).dotProductWith(PhysicsGlobals::up)))){
		int newStateIndex = 0; //_states[FSMStateIndex]->getNextStateIndex();
		transitionToState(0);
		return newStateIndex;
	}

	//if we didn't transition to a new state...
	return -1;
}

/**
	This method is used to return the ratio of the weight that is supported by the stance foot.
*/
double SimBiController::getStanceFootWeightRatio(const std::vector<ContactPoint>& cfs){
	Vector3d stanceFootForce = getForceOnFoot(stanceFoot, cfs);
	Vector3d swingFootForce = getForceOnFoot(swingFoot, cfs);
	double totalYForce = (stanceFootForce + swingFootForce).dotProductWith(PhysicsGlobals::up);

	if (IS_ZERO(totalYForce))
		return -1;
	else
		return stanceFootForce.dotProductWith(PhysicsGlobals::up) / totalYForce;
}


/**
	This method is used to compute the distribution of forces between the two feet
*/
/*** UNUSED ***/
#if 0
void SimBiController::computeToeAndHeelForces(const std::vector<ContactPoint>& cfs)
{
	forceStanceToe.setValues(0,0,0);
	forceSwingToe.setValues(0,0,0);
	forceStanceHeel.setValues(0,0,0);
	forceSwingHeel.setValues(0,0,0);

	toeSwingPos.setValues(0,0,0);
	toeStancePos.setValues(0,0,0);
	heelSwingPos.setValues(0,0,0);
	heelStancePos.setValues(0,0,0);

	swingToeInContact = false;
	stanceToeInContact = false;
	swingHeelInContact = false;
	stanceHeelInContact = false;

	if (cfs.size() == 0){
		haveToeAndHeelInformation = false;
		return;
	}
	haveToeAndHeelInformation = true;

	int swingToeCount = 0, stanceToeCount = 0, swingHeelCount = 0, stanceHeelCount = 0;

	for (unsigned int i=0;i<cfs.size();i++){
		if (isStanceFoot(cfs[i].rb1) || isStanceFoot(cfs[i].rb2)){
			//need to determine if this is a heel or the toe...
			Point3d localPoint = stanceFoot->getLocalCoordinatesForPoint(cfs[i].cp);
			if (localPoint.z > 0){
				forceStanceToe += cfs[i].f;
				toeStancePos += cfs[i].cp;
				stanceToeCount++;
				stanceToeInContact = true;
			}else{
				forceStanceHeel += cfs[i].f;
				heelStancePos += cfs[i].cp;
				stanceHeelCount++;
				stanceHeelInContact = true;
			}
		}

		if (isSwingFoot(cfs[i].rb1) || isSwingFoot(cfs[i].rb2)){
			//need to determine if this is a heel or the toe...
			Point3d localPoint = swingFoot->getLocalCoordinatesForPoint(cfs[i].cp);
			if (localPoint.z > 0){
				forceSwingToe += cfs[i].f;
				toeSwingPos += cfs[i].cp;
				swingToeCount++;
				swingToeInContact = true;
			}else{
				forceSwingHeel += cfs[i].f;
				heelSwingPos += cfs[i].cp;
				swingHeelCount++;
				swingHeelInContact = true;
			}
		}
	}

	if (stanceToeCount > 0)
		toeStancePos /= stanceToeCount;

	if (stanceHeelCount > 0)
		heelStancePos /= stanceHeelCount;

	if (swingToeCount > 0)
		toeSwingPos /= swingToeCount;

	if (swingHeelCount > 0)
		heelSwingPos /= swingHeelCount;
}
#endif

void SimBiController::initControlParams()
{
	//there are two stages here. First we will compute the pose (i.e. relative orientations), using the joint trajectories for the current state
	//and then we will compute the PD torques that are needed to drive the links towards their orientations - here the special case of the
	//swing and stance hips will need to be considered

	//always start from a neutral desired pose, and build from there...
	for(int jIndex=0; jIndex < J_MAX; jIndex++) {
		poseControl.controlParams[jIndex].controlled = true;
		poseControl.controlParams[jIndex].relToFrame = false;
		poseControl.controlParams[jIndex].strength = 1.0;
	}
	
	poseControl.controlParams[J_L_ANKLE].relToFrame = true;
	poseControl.controlParams[J_L_ANKLE].frame = characterFrame();
	poseControl.controlParams[J_L_ANKLE].frameAngularVelocityInWorld = Vector3d(0,0,0);
	
	poseControl.controlParams[J_R_ANKLE].relToFrame = true;
	poseControl.controlParams[J_R_ANKLE].frame = characterFrame();
	poseControl.controlParams[J_R_ANKLE].frameAngularVelocityInWorld = Vector3d(0,0,0);
	
	poseControl.controlParams[swingHipIndex].relToFrame = true;
	poseControl.controlParams[swingHipIndex].frame = characterFrame();
	poseControl.controlParams[swingHipIndex].frameAngularVelocityInWorld = Vector3d(0,0,0);

	//and this is the desired orientation for the root
	// qRootD = Quaternion(1, 0, 0, 0);
	rootControlParams.strength = 1.0;
}

