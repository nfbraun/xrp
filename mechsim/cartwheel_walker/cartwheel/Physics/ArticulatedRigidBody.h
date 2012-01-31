#pragma once

#include <Physics/RigidBody.h>
#include <Physics/Joint.h>
#include <vector>
#include <iostream>

/*=======================================================================================================================================================================*
 * We will treat the articulated rigid bodies as normal rigid bodies that are connected by joints. The joints are needed to enforce constraints between the articulated  *
 * rigid bodies, but other than that, the dynamics are the same as for rigid bodies. We will assume that every articulated figure will be loop-free (tree hierarchies).  *
 *=======================================================================================================================================================================*/
class ArticulatedRigidBody : public RigidBody{
public:
	//this is the parent joint.
	KTJoint* pJoint;
	//and these are the child joints - it can have as many as it wants.
	std::vector<KTJoint*> cJoints;
	//this is the articulated figure that the rigid body belongs to
	ArticulatedFigure* AFParent;
public:
	/**
		Default constructor
	*/
	ArticulatedRigidBody() : pJoint(0), AFParent(0) {}

	/**
		returns the parent joint for the current articulated body
	*/
	inline KTJoint* getParentJoint(){return pJoint;};
	inline int getChildJointCount() const { return cJoints.size(); }
	inline KTJoint* getChildJoint(int i) { return cJoints[i]; }

	/**
		this method always returns true
	*/
	/* virtual bool isArticulated(){
		return true;
	} */

	void setAFParent(ArticulatedFigure* parent){
		AFParent = parent;
	}

	ArticulatedFigure* getAFParent(){
		return AFParent;
	}


};
