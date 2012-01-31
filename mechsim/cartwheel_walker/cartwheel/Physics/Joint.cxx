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

#include "Joint.h"
#include <Physics/Joint.h>
#include <Physics/ArticulatedFigure.h>
#include <stdexcept>

/**
	This method is used to compute the relative orientation between the parent and the child rigid bodies, expressed in 
	the frame coordinate of the parent.
*/
void KTJoint::computeRelativeOrientation(Quaternion& qRel){
	//if qp is the quaternion that gives the orientation of the parent, and qc gives the orientation of the child, then  qp^-1 * qc gives the relative
	//orientation between the child and the parent, expressed in the parent's coordinates (child to parent)
	qRel = parent->state.orientation.getComplexConjugate() * child->state.orientation;
}

void KTJoint::setParent( ArticulatedRigidBody* parent ){
	if (this->parent != NULL)
		throw std::runtime_error("This joint already has a parent");
	this->parent = parent;
	parent->cJoints.push_back(this);
}

/**
	set the chil
*/
void KTJoint::setChild( ArticulatedRigidBody* child ){
	if (this->child != NULL)
		throw std::runtime_error("This joint already has a child");
	if (child->pJoint != NULL)
		throw std::runtime_error("The child body already has a parent.");
	this->child = child;
	child->pJoint = this;
}


