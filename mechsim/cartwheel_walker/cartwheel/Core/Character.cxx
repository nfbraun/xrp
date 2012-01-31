#include "Character.h"
#include <iostream>

/**
	the constructor
*/
Character::Character() : ArticulatedFigure() {
}

/**
	the destructor
*/
Character::~Character(void){
	//nothing to do. We'll let whoever created the world deal with freeing it up
}

/**
	This method is used to populate the relative orientation of the parent and child bodies of joint i.
*/
void Character::getRelativeOrientation(int i, Quaternion* qRel){
	//rotation from child frame to world, and then from world to parent == rotation from child to parent
	joints[i]->computeRelativeOrientation(*qRel);
}

/**
	This method is used to get the relative angular velocities of the parent and child bodies of joint i,
	expressed in parent's local coordinates. 
	We'll assume that i is in the range 0 - joints.size()-1!!!
*/
void Character::getRelativeAngularVelocity(int i, Vector3d* wRel){
	*wRel = joints[i]->child->state.angularVelocity - joints[i]->parent->state.angularVelocity;
	//we will store wRel in the parent's coordinates, to get an orientation invariant expression for it
	*wRel = joints[i]->parent->getLocalCoordinatesForVector(*wRel);
}


/**
	This method populates the dynamic array passed in with the state of the character.
	For the state, we consider the 13-dimensional state of the root, and then only
	the relative orientation and angular velocity (as measured from the parent) for
	every other link. The velocities of the CM are derived from this information,
	using the velocity propagation technique (if that's what it is called).		
	The only thing we will not be storing explicitly is the positions of the CMs of the rigid bodies. 
	The order in which the bodies appear is given by the array of joints. 
	This works under the assumption that in the joint 
	sequence, the parent of any rigid body appears before its children (assuming that for each joint
	we read the parent first and then the child). 
*/
// disabled (NB)
/* void Character::getState(ReducedCharacterState* state){
	state->setRootState(getRoot()->state);

	//now each joint introduces one more rigid body, so we'll only record its state relative to its parent.
	//we are assuming here that each joint is revolute!!!
	Quaternion qRel;
	Vector3d wRel;
	
	for (uint i=0;i<J_MAX;i++){
		getRelativeOrientation(i, &qRel);
	    state->setJointRelativeOrientation(qRel, i);

		getRelativeAngularVelocity(i, &wRel);
		state->setJointRelativeAngVelocity(wRel, i);
	}
} */

/**
	this method is used to return the current heading of the character, specified as an angle measured in radians
*/
double Character::getHeadingAngle(){
	//first we need to get the current heading of the character. Also, note that q and -q represent the same rotation
	Quaternion q = getHeading();
	if (q.s<0){
		q.s = -q.s;
		q.v = -q.v;
	}
	double currentHeading = 2 * safeACOS(q.s);
	if (q.v.dotProductWith(PhysicsGlobals::up) < 0)
		currentHeading = -currentHeading;
	return currentHeading;
}


/**
	This method returns the dimension of the state. Note that we will consider
	each joint as having 3-DOFs (represented by the 4 values of the quaternion)
	without taking into account what type of joint it is (i.e. a hinge joint
	has only one degree of freedom, but we will still consider the relative orientation
	of the child relative to the parent as a quaternion.
*/
int Character::getStateDimension(){
	//13 for the root, and 7 for every other body (and each body is introduced by a joint).
	return 13 + 7 * J_MAX;
}


/**
	This method is used to mirror the given orientation. It is assumed that rotations in the sagittal plane
	(about parent frame x-axis) are to stay the same, while the rotations about the other axes need to be reversed.
*/
Quaternion mirrorOrientation(const Quaternion& q){
	//get the rotation about the parent's x-axis
	Quaternion qSagittal = q.getComplexConjugate().decomposeRotation(Vector3d(1, 0, 0)).getComplexConjugate();
	//this is what is left, if we removed that component of the rotation
	Quaternion qOther = q * qSagittal.getComplexConjugate();
	//and now negate the non-sagittal part of the rotation, but keep the rotation in the sagittal plane
	return qOther.getComplexConjugate() * qSagittal;
}

/**
	This method is used to multiply, element-wise, the two vectors that are passed in  
*/
Vector3d elemWiseMultiply(const Vector3d& a, const Vector3d& b){
	return Vector3d(a.x * b.x, a.y * b.y, a.z * b.z);
}

/**
	This method is used to compute the center of mass of the articulated figure.
*/
Vector3d Character::getCOM()
{
	Vector3d COM = Vector3d(getRoot()->getCMPosition()) * getRoot()->getMass();
	double curMass = getRoot()->getMass();
	double totalMass = curMass;
	for (unsigned int i=0; i < J_MAX; i++){
		curMass = joints[i]->child->getMass();
		totalMass += curMass;
		COM.addScaledVector(joints[i]->child->getCMPosition() , curMass);
	}

	COM /= totalMass;

	return COM;
}

/**
	This method is used to compute the velocity of the center of mass of the articulated figure.
*/
Vector3d Character::getCOMVelocity()
{
	Vector3d COMVel = Vector3d(getRoot()->getCMVelocity()) * getRoot()->getMass();
	double curMass = getRoot()->getMass();
	double totalMass = curMass;
	for (unsigned int i=0; i < J_MAX; i++){
		curMass = joints[i]->child->getMass();
		totalMass += curMass;
		COMVel.addScaledVector(joints[i]->child->getCMVelocity() , curMass);
	}

	COMVel /= totalMass;

	return COMVel;
}

/**
	this method is used to return the current heading of the character
*/
Quaternion Character::getHeading()
{
	//get the current root orientation, that contains information regarding the current heading and retrieve the twist about the vertical axis
	return computeHeading(getRoot()->getOrientation());
}

