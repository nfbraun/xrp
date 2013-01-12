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

#include "RigidBody.h"

/**
	This method returns the coordinates of the point that is passed in as a parameter(expressed in local coordinates), in world coordinates.
*/
Point3d RigidBody::getWorldCoordinatesForPoint(const Point3d& localPoint) const
{
	return this->state.position + getWorldCoordinatesForVector(Vector3d(localPoint));
}

/**
	This method returns the vector that is passed in as a parameter(expressed in local coordinates), in world coordinates.
*/
Vector3d RigidBody::getWorldCoordinatesForVector(const Vector3d& localVector) const
{
	//the rigid body's orientation is a unit quaternion. Using this, we can obtain the global coordinates of a local vector
	return this->state.orientation.rotate(localVector);
}

/**
	This method is used to return the local coordinates of the point that is passed in as a parameter (expressed in global coordinates)
*/
Point3d RigidBody::getLocalCoordinatesForPoint(const Point3d& globalPoint) const
{
	Vector3d v = getLocalCoordinatesForVector(Vector3d(globalPoint)) - getLocalCoordinatesForVector(Vector3d(this->state.position));
	return Point3d(0,0,0) + v;
}

/**
	This method is used to return the local coordinates of the vector that is passed in as a parameter (expressed in global coordinates)
*/
Vector3d RigidBody::getLocalCoordinatesForVector(const Vector3d& globalVector) const
{
	//the rigid body's orientation is a unit quaternion. Using this, we can obtain the global coordinates of a local vector
	return this->state.orientation.getComplexConjugate().rotate(globalVector);
}

/**
	This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in local coordinates, and the
	resulting velocity will be expressed in world coordinates.
*/
Vector3d RigidBody::getAbsoluteVelocityForLocalPoint(const Point3d& localPoint){
	//we need to compute the vector r, from the origin of the body to the point of interest
	Vector3d r(Point3d(), localPoint);
	//the velocity is given by omega x r + v. omega and v are already expressed in world coordinates, so we need to express r in world coordinates first.
	return state.angularVelocity.crossProductWith(getWorldCoordinatesForVector(r)) + state.velocity;
}

/**
	This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in global coordinates, and the
	resulting velocity will be expressed in world coordinates.
*/
Vector3d RigidBody::getAbsoluteVelocityForGlobalPoint(const Point3d& globalPoint){
	//we need to compute the vector r, from the origin of the body to the point of interest
	Vector3d r(state.position, globalPoint);
	//the velocity is given by omega x r + v. omega and v are already expressed in world coordinates, so we need to express r in world coordinates first.
	return state.angularVelocity.crossProductWith(r) + state.velocity;
}

