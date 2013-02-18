#pragma once

#include <Physics/ArticulatedRigidBody.h>
#include <Physics/Joint.h>

#include <iostream>
#include <stdexcept>

#include "../../StaticRobotInfo.h"

/*======================================================================================================================================================================*
 * An articulated figure is composed of many articulated rigid bodies that are interconnected by joints. Characters, cars, ropes, etc, can all be viewed as articulated *
 * figures. One note is that we will only allow tree structures - no loops.                                                                                             *
 *======================================================================================================================================================================*/
class ArticulatedFigure {
protected:
	double mass;

	//keep a list of the character's joints, for easy access
	KTJoint* joints[J_MAX];

	ArticulatedRigidBody* arbs[B_MAX];


public:
	/**
		Default constructor
	*/
	ArticulatedFigure(void);

	/**
		Default destructor
	*/
	virtual ~ArticulatedFigure(void);


	/**
		Sets the root
	*/
	void setRoot( ArticulatedRigidBody* articulatedRigidBody_disown );

	/**
		returns the root of the current articulated figure.
	*/
	inline ArticulatedRigidBody* getRoot(){
		return arbs[B_PELVIS];
	}
	
	KTJoint* const* getJoints() const { return joints; }
	ArticulatedRigidBody* const* getARBs() const { return arbs; }

	/**
		This method adds one rigid body (articulated or not).
	*/
	virtual void addArticulatedRigidBody( ArticulatedRigidBody* articulatedRigidBody_disown,
	    BodyID id);
	
	ArticulatedRigidBody* getArticulatedRigidBody( int i ) { return arbs[i]; }
	int getArticulatedRigidBodyCount() const { return B_MAX; }

	/**
		This method returns an ARB that is a child of this articulated figure
	*/
	/* ArticulatedRigidBody* getARBByName(const char* name) const {
		for (uint i=0;i<arbs.size();i++)
			if (strcmp(arbs[i]->getName(), name) == 0)
				return arbs[i];
		return NULL;
	} */
	/**
		Adds a joint to the figure
		This is an empty function as the joints are not tracked
		by the ArticulatedFigure.
		This makes it possible to disown Python of the joint pointer
		so that it doesn't garbage collect it. 
		The real place where Python should be disowned is when
		Joint.setParent() is called since the parent is responsible
		for deleting the joint. However, I don't know how to force
		python to disown an object when a method is called.
	*/
	void addJoint( KTJoint* joint_disown, JointID id = J_MAX )
	{
	    if(id < 0 || id >= J_MAX)
	        throw std::runtime_error("Add invalid joint");
		else
		    joints[id] = joint_disown;
	}

	/**
		This method is used to get all the joints in the articulated figure and add them to the list of joints that is passed in as a paramter.
	*/
	//inline void addJointsToList(std::vector<Joint*> *joints);

	/**
		This method is used to compute the total mass of the articulated figure.
	*/
	void computeMass();

	/**
		This method is used to get the total mass of the articulated figure.
	*/
	double getMass();


	/**
		Returns a pointer to the character's ith joint
	*/
	const KTJoint* getJoint(int i) const {
		if (i < 0 || i >= J_MAX)
			return 0;
		return joints[i];
	}

	/**
		This method is used to return the number of joints of the character.
	*/
	int getJointCount() {
		return J_MAX;
	}
	
	ArticulatedRigidBody* getARB(int i) {
	    if(i < 0 || i >= B_MAX)
	        return 0;
	    return arbs[i];
	}
};

