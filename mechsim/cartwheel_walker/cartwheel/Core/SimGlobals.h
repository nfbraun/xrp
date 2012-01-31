#pragma once

const int LEFT_STANCE = 0;
const int RIGHT_STANCE = 1;

/**
	This class is used as a container for all the constants that are pertinent for the physical simulations, the controllers, etc.
*/

class SimGlobals {
public:
	//and this is the desired time interval for each simulation timestep (does not apply to animations that are played back).
	static double dt;

};
