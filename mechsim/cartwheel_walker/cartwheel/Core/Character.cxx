#include "Character.h"
#include <iostream>

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
	if (q.v.dot(PhysicsGlobals::up) < 0)
		currentHeading = -currentHeading;
	return currentHeading;
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
		COM += curMass * joints[i]->child->getCMPosition();
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
		COMVel += curMass * joints[i]->child->getCMVelocity();
	}

	COMVel /= totalMass;

	return COMVel;
}

/**
	This method decomposes the rotation that is passed in into a rotation by the vertical axis - called vertical twist or heading, and everything else:
	rot = qHeading * qOther;
	The returned orientation is qHeading.
*/
Quaternion computeHeading(const Quaternion& rot) {
	return rot.getComplexConjugate().decomposeRotation(PhysicsGlobals::up).getComplexConjugate();
}

/**
	this method is used to return the current heading of the character
*/
Quaternion Character::getHeading()
{
	//get the current root orientation, that contains information regarding the current heading and retrieve the twist about the vertical axis
	return computeHeading(getRoot()->getOrientation());
}

