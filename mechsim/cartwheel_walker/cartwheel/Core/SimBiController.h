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

#pragma once

#include "PoseController.h"
#include <Physics/RigidBody.h>
#include "SimBiConState.h"
#include <vector>


/**
 * A simbicon controller is a fancy PoseController. The root (i.e. pelvis or torso), as well as the two hips are controlled
 * relative to a semi-global coordinate frame (it needs only have the y-axis pointing up), but all other joints are controlled
 * exactely like in a normal PoseController. The SimBiController operates on a biped, which means that we can make special
 * assumptions about it: it must have two joints, lHip and rHip, connecting the root to the left and right upper-legs,
 * and it must also have two feet (lFoot and rFoot) as rigid bodies in the articulated linkage.
 */

class SimBiController {
protected:
	//this is the character that the controller is acting on
	Character* character;
	//this is the number of joints of the character - stored here for easy access
	int jointCount;
public:
	Character* getCharacter() const {
		return character;
	}

protected:
/**
	These are quantities that are set only once
*/
	//we will keep a reference to the left and right feet to be able to determine when the stance switches
	RigidBody* lFoot;
	RigidBody* rFoot;
	//we will also keep a reference to the root of the figure, to be able to identify the semi-global coordinate frame quickly
	RigidBody* root;

	SimBiConState state;
	

	//the root is not directly controlled by any joint, so we will store its Kp, Kd and maxTorque separated here.
	//while the pose controller does not use these values, other types of controllers may need this information
	ControlParams rootControlParams;

	double stanceHipDamping;
	double stanceHipMaxVelocity;
	double rootPredictiveTorqueScale;

	//this is the desired orientation for the root
	Quaternion qRootD;
	//this is the desired heading for the character
	double desiredHeading;


/**
	these are quantities that get updated throughout the simulation
*/
	//this value indicates which side is the stance side. 
	int stance;
	//a pointer to the swing and stance feet
	RigidBody* stanceFoot;
	RigidBody* swingFoot;
	//keep track of the swing and stance hip indices
	int stanceHipIndex;
	int swingHipIndex;
	//this is the index of the controller that is currently active
	//int FSMStateIndex;

	//this is the world relative velocity of the COM
	Vector3d comVelocity;
	//and this is the position
	Point3d comPosition;

	//this is the vector from the cm of the stance foot to the cm of the character
	Vector3d _d;
	//this is the velocity of the cm of the character, in character frame
	Vector3d _v;

	//this is the distance between the COM and the midpoint between the feet
	Vector3d doubleStanceCOMError;



	//now, for each foot (and toes if any), we will keep a summary of the contact points from the previous simulation
	Vector3d forceStanceToe;
	Vector3d forceSwingToe;
	Vector3d forceStanceHeel;
	Vector3d forceSwingHeel;
	Point3d toeSwingPos;
	Point3d toeStancePos;
	Point3d heelSwingPos;
	Point3d heelStancePos;
	bool swingToeInContact;
	bool stanceToeInContact;
	bool swingHeelInContact;
	bool stanceHeelInContact;
	bool haveToeAndHeelInformation;
	void computeToeAndHeelForces(std::vector<ContactPoint> *cfs);


	//the phase parameter, phi must have values between 0 and 1, and it indicates the progress through the current state.
	double phi;

	//this quaternion gives the current heading of the character. The complex conjugate of this orientation is used
	//to transform quantities from world coordinates into a rotation/heading-independent coordinate frame (called the character frame).
	//I will make the asumption that the character frame is a coordinate frame that is aligned with the vertical axis, but has 0 heading, and
	//the characterFrame quaternion is used to rotate vectors from the character frame to the real world frame
	Quaternion characterFrame;

	//this variable, updated everytime the controller state is advanced in time, is set to true if any body other than the feet are in contact
	//with the ground, false otherwise. A higer level process can determine if the controller failed or not, based on this information.
	bool bodyTouchedTheGround;
	
/**
	A list of private methods...
*/


	/**
		This method should be called when the controller transitions to this state.
	*/
	void transitionToState(int stateIndex);

	/**
		This method returns the net force on the body rb, acting from the ground
	*/
	Vector3d getForceOn(RigidBody* rb, std::vector<ContactPoint> *cfs);

	/**
		This method returns the net force on the body rb, acting from the ground
	*/
	Vector3d getForceOnFoot(RigidBody* foot, std::vector<ContactPoint> *cfs);

	/**
		check to see if rb is the same as whichBody or any of its children
	*/
	bool haveRelationBetween(RigidBody* rb, RigidBody* whichBody);

	/**
		This method is used to determine if the rigid body that is passed in as a parameter is a
		part of a foot
	*/
	bool isFoot(RigidBody* rb);

	/**
		This method returns true if the rigid body that is passed in as a parameter is a swing foot, false otherwise
	*/
	bool isStanceFoot(RigidBody* rb);

	/**
		This method returns true if the rigid body that is passed in as a parameter is a swing foot, false otherwise
	*/
	bool isSwingFoot(RigidBody* rb);

	/**
		This method is used to return the ratio of the weight that is supported by the stance foot.
	*/
	double getStanceFootWeightRatio(std::vector<ContactPoint> *cfs);

	/**
		This method is used to compute the torques that need to be applied to the stance and swing hips, given the
		desired orientation for the root and the swing hip. The coordinate frame that these orientations are expressed
		relative to is computed in this method. It is assumed that the stanceHipToSwingHipRatio variable is
		between 0 and 1, and it corresponds to the percentage of the total net vertical force that rests on the stance
		foot.
	*/
	void computeHipTorques(const Quaternion& qRootD, double stanceHipToSwingHipRatio, Vector3d ffRootTorque, JointTorques& torques);
	
	PoseController poseControl;

public:
	/**
		Default constructor
	*/
	SimBiController(Character* b);

	/**
		Destructor
	*/
	virtual ~SimBiController() {}
	
	void addControlParams( int jIndex, const ControlParams& params ) {
		poseControl.addControlParams(jIndex, params);
	}
	
	SimBiConState* getState() { return &state; }
	
	void setDesiredHeading(double head) { desiredHeading = head; }

	const Vector3d& getDoubleStanceCOMError() const { return doubleStanceCOMError; }

	inline void setStanceHipDamping( double damping ) {
		stanceHipDamping = damping;
	}

	inline double getStanceHipDamping() const { return stanceHipDamping; }


	inline void setStanceHipMaxVelocity( double velocity ) {
		stanceHipMaxVelocity = velocity;
	}


	inline double getStanceHipMaxVelocity() const { return stanceHipMaxVelocity; }


	/**
		This method is used to set the stance 
	*/
	void setStance(int newStance);
	
	void toggleStance();


	void setRootControlParams(const ControlParams& params) {
	    rootControlParams = params;
	}
	
	Point3d getCOMPosition() const { return comPosition; }
	Vector3d getCOMVelocity() const { return comVelocity; }

	void initControlParams();

	
	/**
		This method is used to advance the controller in time. It takes in a list of the contact points, since they might be
		used to determine when to transition to a new state. This method returns -1 if the controller does not advance to a new state,
		or the index of the state that it transitions to otherwise.
	*/
	int advanceInTime(double dt, std::vector<ContactPoint> *cfs);

	/**
		This method is used to return the value of bodyGroundContact
	*/
	inline bool isBodyInContactWithTheGround(){
		return bodyTouchedTheGround;
	}

	/**
		This method is used to return the value of the phase (phi) in the current FSM state.
	*/
	inline double getPhase(){
		return phi;
	}
	inline void setPhase( double phi ) { this->phi = phi; }

	Vector3d getV() const { return _v; }
	Vector3d getD() const { return _d; }

	/**
		This method returns the position of the CM of the stance foot, in world coordinates
	*/
	inline Point3d getStanceFootPos(){
		if (stanceFoot)
			return stanceFoot->getCMPosition();
		return Point3d(0,0,0);
	}
	
	inline Vector3d getStanceFootVel() {
	    if(stanceFoot)
	        return stanceFoot->getCMVelocity();
	    return Vector3d(0., 0., 0);
	}
	
	inline const RigidBody* getStanceFoot() const { return stanceFoot; }

	/**
		This method returns the position of the CM of the swing foot, in world coordinates
	*/
	inline Point3d getSwingFootPos(){
		if (swingFoot)
			return swingFoot->getCMPosition();
		return Point3d(0,0,0);
	}
	
	inline Vector3d getSwingFootVel(){
		if (swingFoot)
			return swingFoot->getCMVelocity();
		return Vector3d(0,0,0);
	}
	
	inline const RigidBody* getSwingFoot() const { return swingFoot; }

	/**
		This method returns the character frame orientation
	*/
	Quaternion getCharacterFrame(){
		return characterFrame;
	}

	/**
		This method is used to update the d and v parameters, as well as recompute the character coordinate frame.
	*/
	void updateDAndV();

	/**
		this method returns the stance of the character
	*/
	inline int getStance(){
		return stance;
	}

};
