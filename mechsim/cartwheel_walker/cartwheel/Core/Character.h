#pragma once

#include <Physics/PhysicsGlobals.h>
#include <Physics/ArticulatedFigure.h>
#include <vector>
#include <stdexcept>
#include "SimGlobals.h"

/**
	A character is an articulated figure - This class implements methods that allow it to easily save and restore its state, etc.
*/

class Character : public ArticulatedFigure {

public:
	/**
		the constructor
	*/
	Character();

	/**
		the destructor
	*/
	~Character(void);

	/**
		This method is used to populate the relative orientation of the parent and child bodies of joint i.
	*/
	void getRelativeOrientation(int i, Quaternion* qRel);

	/**
		This method is used to get the relative angular velocities of the parent and child bodies of joint i,
		expressed in parent's local coordinates. 
	*/
	void getRelativeAngularVelocity(int i, Vector3d* wRel);


	/**
		This method populates the dynamic array passed in with the state of the character.
		For the state, we consider the 13-dimensional state of the root, and then only
		the relative orientation and angular velocity (as measured in parent coordinates) for
		every other link. The velocities of the CM are derived from this information,
		using the velocity propagation technique (if that's what it is called).		
		The order in which the bodies appear is given by the array of joints. 
		This works under the assumption that in the joint 
		sequence, the parent of any rigid body appears before its children (assuming that for each joint
		we read the parent first and then the child). 
	*/
	// disabled (NB)
	// void getState(ReducedCharacterState* state);

	/**
		This method populates the state of the current character with the values that are passed
		in the dynamic array. The same conventions as for the getState() method are assumed.
		We'll assume that the index of the first state variable in the state array is given by
		'start'.
	*/
	//void setState(ReducedCharacterStateArray* state, int start = 0, bool hackFlag = true)
	//    { throw std::runtime_error("setState() must not be called"); }

	/**
		This method returns the dimension of the state. Note that we will consider
		each joint as having 3-DOFs (represented by the 4 values of the quaternion)
		without taking into account what type of joint it is (i.e. a hinge joint
		has only one degree of freedom, but we will still consider the relative orientation
		of the child relative to the parent as a quaternion.
	*/
	int getStateDimension();

	/**
		this method is used to return the current heading of the character
	*/
	Quaternion getHeading();

	/**
		this method is used to return the current heading of the character, specified as an angle measured in radians
	*/
	double getHeadingAngle();

	/**
		This method is used to compute the center of mass of the articulated figure.
	*/
	Vector3d getCOM();

	/**
		This method is used to compute the velocity of the center of mass of the articulated figure.
	*/
	Vector3d getCOMVelocity();

};

/**
	This method decomposes the rotation that is passed in into a rotation by the vertical axis - called vertical twist or heading, and everything else:
	rot = qHeading * qOther;
	The returned orientation is qHeading.
*/
inline Quaternion computeHeading(const Quaternion& rot){
	return rot.getComplexConjugate().decomposeRotation(PhysicsGlobals::up).getComplexConjugate();
}
