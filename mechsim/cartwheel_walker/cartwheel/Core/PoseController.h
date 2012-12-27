#pragma once

#include <vector>
#include <string>
#include <stdexcept>
#include <Core/Controller.h>
#include "Character.h"

/**
	This class is used as a container for the properties needed by a PD controller
*/
class ControlParams {
public:
	//this variable is set to true if the current joint is being actively controlled, false otherwise
	bool controlled;
	//these two variables are the proporitonal and derivative gains for the PD controller used to compute the torque
	double kp, kd;
	//this is the maximum absolute value torque allowed at this joint
	double maxAbsTorque;
	//the torques about the about the x, y and z axis will be scaled differently to account for the potentially different principal moments of inertia
	//of the child
	Vector3d scale;

	double strength;

	//this variable, if true, indicates that the desired orientation is specified in the custom coordinate frame
	bool relToFrame;

	//and this is the coordinate frame (and its angular velocity) that the desired orientation is specified in, if relToFrame is true
	Quaternion frame;
	Vector3d   frameAngularVelocityInWorld;

public:

	/**
		This constructor initializes the variables to some safe, default values
	*/
	ControlParams( )
	    : controlled(false), kp(0.), kd(0.), maxAbsTorque(0.),
	        scale(0., 0., 0.), strength(1.), relToFrame(false)
	{ }

	void setKp( double kp ) {
		this->kp = kp;
	}

	double getKp() const { return kp; }

	void setKd( double kd ) {
		this->kd = kd;
	}

	double getKd() const { return kd; }

	void setMaxAbsTorque( double maxAbsTorque ) {
		this->maxAbsTorque = maxAbsTorque;
	}

	double getMaxAbsTorque() const { return maxAbsTorque; }

	void setScale( const Vector3d& scale ) {
		this->scale = scale;
	}

	const Vector3d& getScale() const { return scale; }

	void setStrength( double strength ) {
		this->strength = strength;
	}

	double getStrength() const { return strength; }

	void setRelToFrame( bool relToFrame ) {
		this->relToFrame = relToFrame;
	}

	bool getRelToFrame() const { return relToFrame; }
};

/**
	A pose controller is used to have the character track a given pose.
	Each pose is given as a series of relative orientations, one for
	each parent-child pair (i.e. joint). Classes extending this one 
	have to worry about setting the desired relative orientation properly.
*/
class PoseController {
public:
	//this is the array of joint properties used to specify the 
	std::vector<ControlParams> controlParams;

public:
	/**
		Constructor - expects a character that it will work on
	*/
	PoseController();
	
	//void setDesiredPose(const ReducedCharacterState& dp)
	//    { desiredPose = dp; }

	void addControlParams( int jIndex, const ControlParams& params ) {
		controlParams[jIndex] = params;
	}

	ControlParams* getControlParams( int index ) {
		return &controlParams[index];
	}

	/**
		This method is used to compute the torques, based on the current and desired poses
	*/
	Vector3d computePDJointTorque(Character* character, int jid,
        Quaternion desiredOrientationInFrame,
        Vector3d desiredRelativeAngularVelocityInFrame, bool relToFrame);

	/**
		This method is used to compute the PD torque, given the current relative orientation of two coordinate frames (child and parent),
		the relative angular velocity, the desired values for the relative orientation and ang. vel, as well as the virtual motor's
		PD gains. The torque returned is expressed in the coordinate frame of the 'parent'.
	*/
	static Vector3d computePDTorque(const Quaternion& qRel, const Quaternion& qRelD, const Vector3d& wRel, const Vector3d& wRelD, const ControlParams* pdParams);

	/**
		This method is used to scale and apply joint limits to the torque that is passed in as a parameter. The orientation that transforms 
		the torque from the coordinate frame that it is currently stored in, to the coordinate frame of the 'child' to which the torque is 
		applied to (it wouldn't make sense to scale the torques in any other coordinate frame)  is also passed in as a parameter.
	*/
	static void scaleAndLimitTorque(Vector3d* torque, const ControlParams* pdParams, const Quaternion& qToChild);

	/**
		This method is used to apply joint limits to the torque passed in as a parameter. It is assumed that
		the torque is already represented in the correct coordinate frame
	*/
	static void limitTorque(Vector3d* torque, const ControlParams* cParams);

	/**
		sets the targets to match the current state of the character
	*/
	//void setTargetsFromState();
};


