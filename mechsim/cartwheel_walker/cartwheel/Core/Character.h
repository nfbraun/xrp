#pragma once

#include <Physics/PhysicsGlobals.h>
#include <Physics/ArticulatedFigure.h>
#include <vector>
#include <stdexcept>
#include "SimGlobals.h"

/**
	A character is an articulated figure - This class implements methods that allow it to easily save and restore its state, etc.
*/

class ReducedCharacterState;

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

class ReducedCharacterState{
private:
	RBState fRootState;
	Quaternion fJointOrient[J_MAX];
	Vector3d fJointAngVel[J_MAX];
public:
    ReducedCharacterState() {
        setPosition(Vector3d(0., 0., 0.));
        setVelocity(Vector3d(0., 0., 0.));
        setOrientation(Quaternion(1, 0, 0, 0));
        setAngularVelocity(Vector3d(0., 0., 0.));
        
        for (int i=0;i<J_MAX;i++){
            setJointRelativeAngVelocity(Vector3d(0., 0., 0.), i);
            setJointRelativeOrientation(Quaternion(1., 0., 0., 0.), i);
        }
    }
    
	/**
		gets the root position.
	*/
	inline Vector3d getPosition() { return fRootState.position; }

	/**
		sets the root position.
	*/
	inline void setPosition(const Vector3d& p) { fRootState.position = p; }

	/**
		gets the root orientation.
	*/
	inline Quaternion getOrientation() { return fRootState.orientation; }

	/**
		sets the root orientation.
	*/
	inline void setOrientation(const Quaternion& q)
	    { fRootState.orientation = q; }

	/**
		gets the root velocity.
	*/
	inline Vector3d getVelocity() { return fRootState.velocity; }

	/**
		sets the root velocity.
	*/
	inline void setVelocity(const Vector3d& v){ fRootState.velocity = v; }

	/**
		gets the root angular velocity.
	*/
	inline Vector3d getAngularVelocity() { return fRootState.angularVelocity; }

	/**
		sets the root angular velocity.
	*/
	inline void setAngularVelocity(const Vector3d& v)
	    { fRootState.angularVelocity = v; }
	
	/**
	    sets the entire root state at once.
	*/
	inline void setRootState(const RBState& state)  { fRootState = state; }
	
	/**
		gets the relative orientation for joint jIndex
	*/
	inline Quaternion getJointRelativeOrientation(int jIndex)
	{ return fJointOrient[jIndex]; }

	/**
		sets the orientation for joint jIndex
	*/
	inline void setJointRelativeOrientation(const Quaternion& q, int jIndex)
		{ fJointOrient[jIndex] = q; }

	/**
		gets the relative angular velocity for joint jIndex
	*/
	inline Vector3d getJointRelativeAngVelocity(int jIndex)
		{ return fJointAngVel[jIndex]; }

	/**
		sets the orientation for joint jIndex
	*/
	inline void setJointRelativeAngVelocity(const Vector3d& w, int jIndex)
		{ fJointAngVel[jIndex] = w; }

};
