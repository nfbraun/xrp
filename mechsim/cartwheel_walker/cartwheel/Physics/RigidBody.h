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

	You should have received a copy of the GNU General Public License
	along with Simbicon 1.5 Controller Editor Framework. 
	If not, see <http://www.gnu.org/licenses/>.
*/

#pragma once

#include <string>
#include <Physics/RBState.h>

class ArticulatedFigure;

/*=========================================================================================================================================================================*
 | This is the implementation of a Rigid Body class. It holds all the attributes that characterize the rigid body (state information, collision detection primitives, etc).|
 | This class is used as the basis for an Articulated Rigid Body. Together with the PhysicalWorld class, this class is used to implement the dynamics of rigid bodies.     |
 | NOTE: It is assumed that the location of the center of mass of the object in local coordinates is (0,0,0) and that the principal moments of inertia are aligned to the  |
 | local coordinates x, y, and z axes!                                                                                                                                     |
 *=========================================================================================================================================================================*/

class RigidBody {

protected:
	double fMass;
public:
	int id;

	//--> this transformation matrix is used to transform points/vectors from local coordinates to global coordinates. It will be updated using the state
	//information, and is therefore redundant, but it will be used to draw the object quickly. Everytime the state is updated, this matrix must also be updated!
//	TransformationMatrix toWorld;

public:
	inline const RBState& getState() const { return state; }
	
	//--> the state of the rigid body: made up of the object's position in the world, its orientation and linear/angular velocities (stored in world coordinates)
	RBState state;

	Vector3d moi;

	/**
		This method returns the coordinates of the point that is passed in as a parameter(expressed in local coordinates), in world coordinates.
	*/
	Point3d getWorldCoordinatesForPoint(const Point3d& localPoint) const;

	/**
		This method is used to return the local coordinates of the point that is passed in as a parameter (expressed in global coordinates)
	*/
	Point3d getLocalCoordinatesForPoint(const Point3d& globalPoint);

	/**
		This method is used to return the local coordinates of the vector that is passed in as a parameter (expressed in global coordinates)
	*/
	Vector3d getLocalCoordinatesForVector(const Vector3d& globalVector);

	/**
		This method returns the vector that is passed in as a parameter(expressed in local coordinates), in world coordinates.
	*/
	Vector3d getWorldCoordinatesForVector(const Vector3d& localVector) const;

	/**
		This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in local coordinates, and the
		resulting velocity will be expressed in world coordinates.
	*/
	Vector3d getAbsoluteVelocityForLocalPoint(const Point3d& localPoint);

	/**
		This method returns the absolute velocity of a point that is passed in as a parameter. The point is expressed in global coordinates, and the
		resulting velocity will be expressed in world coordinates.
	*/
	Vector3d getAbsoluteVelocityForGlobalPoint(const Point3d& globalPoint);


	/**
		This method returns the world coordinates of the position of the center of mass of the object
	*/
	inline Point3d getCMPosition() const {
		return state.position;
	}

	/**
		This method returns the body's center of mass velocity
	*/
	inline Vector3d getCMVelocity() const {
		return state.velocity;
	}


	/**
		This method sets the rigid body mass
	*/
	void setMass( double mass ) {
		fMass = mass;
	}
	
	/**
		Returns the mass of the rigid body
	*/
	double getMass() { return fMass; }

	/**
		this method returns the body's orientation
	*/
	inline Quaternion getOrientation() const {
		return state.orientation;
	}

	/**
		this method returns the body's angular velocity
	*/
	inline Vector3d getAngularVelocity() const {
		return state.angularVelocity;
	}

};

