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

#include <string>

#include <MathLib/Vector3d.h>
#include <MathLib/Point3d.h>
#include <MathLib/Quaternion.h>

/*=======================================================================================================================================================================*
 * This class is responsible with the implementation of the methods that are neccessary to implement joints in an articulated system. The joints impose constraints on   *
 * the articulated rigid bodies that they connect. Each joint will be used to link a parent body to a child body. The joints that will be considered, for now at least,  *
 * are all rotationals joints with 1, 2 or 3 degrees of freedom. The default type of joint is a Ball in Socket joint with no joint limits.                               *
 *=======================================================================================================================================================================*/
class ArticulatedRigidBody;
class KTJoint {
public:
	//this is the parent link
	ArticulatedRigidBody* parent;
	
	//this is the child link
	ArticulatedRigidBody* child;
	//this is the location of the joint on the child body - expressed in the child's local coordinates 

	//this is the id of the joint...
	int id;

	/**
		This method is used to compute the relative orientation between the parent and the child rigid bodies, expressed in 
		the frame coordinate of the parent.
	*/
	void computeRelativeOrientation(Quaternion& qRel);
	
	
	//the torque applied to this joint. It should be set/reset by a controller acting on this joint.
	//Vector3d torque;
protected:

	//this is the location of the joint on the parent body - expressed in the parent's local coordinates
	Point3d pJPos;
	
	//NOTE: the locations of the parent and child joint locations must overlap in world coordinates
	Point3d cJPos;

public:
	/**
		Default constructor
	*/
	KTJoint(void)
	    : parent(0), child(0), id(-1) {}

	/**
		sets the torque
	*/
	//inline void setTorque(const Vector3d& t){torque = t;}

	/**
		retrieves the reference to the body's parent
	*/
	inline const ArticulatedRigidBody* getParent() const {return parent;}

	/**
		set the parent
	*/
	void setParent( ArticulatedRigidBody* parent );
	/**
		retrieves the reference to the child's parent
	*/
	inline const ArticulatedRigidBody* getChild() const {return child;}

	/**
		set the child
	*/
	void setChild( ArticulatedRigidBody* child );

	/**
		returns the position of the parent joint, expressed in parent's coordinates
	*/
	inline const Point3d& getParentJointPosition(){return pJPos;}
	
	/**
		sets the position of the parent joint, expressed in parent's coordinates

	*/
	inline void setParentJointPosition( const Point3d& pJPos ){
		this->pJPos = pJPos;
	}
	
	/**
		sets the position of the child joint, expressed in child's coordinates
	*/
	inline void setChildJointPosition( const Point3d& cJPos ){
		this->cJPos = cJPos;
	}
	
	/**
		returns the position of the child joint, expressed in child's coordinates

	*/
	inline const Point3d& getChildJointPosition(){return cJPos;}
};

class Joint: public KTJoint {};
