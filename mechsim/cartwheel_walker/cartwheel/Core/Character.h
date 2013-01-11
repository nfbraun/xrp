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
	Character() : ArticulatedFigure() {}

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
