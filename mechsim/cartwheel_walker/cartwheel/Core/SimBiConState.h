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

#include <MathLib/Vector3d.h>
#include <MathLib/Quaternion.h>
#include "SimGlobals.h"
#include <Physics/ArticulatedRigidBody.h>
#include <stdexcept>
#include <iostream>

class SimBiController;

/**
 *	A simbicon controller is made up of a set of a number of states. Transition between states happen on foot contact, time out, user interaction, etc.
 *  Each controller state holds the trajectories for all the joints that are controlled. 
 */
class SimBiConState 
{

private:
	//this is the amount of time that it is expected the biped will spend in this state
	double fStateTime;

public:

	/**
		default constructor
	*/
	SimBiConState() : fStateTime(0.6) {}

	/**
		Returns the time we're expecting to spend in this state
	*/
	inline double getStateTime(){
		return fStateTime;
	}
	
	inline void setStateTime(double t) { fStateTime = t; }


};


