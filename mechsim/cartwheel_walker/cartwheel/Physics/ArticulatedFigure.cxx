#include "ArticulatedFigure.h"
#include <stdexcept>

/**
	Default constructor
*/
ArticulatedFigure::ArticulatedFigure(void){
	mass = 0;
}

ArticulatedFigure::~ArticulatedFigure(void){
}

/**
	Sets the root
*/
void ArticulatedFigure::setRoot( ArticulatedRigidBody* root ) {
	if(this->arbs[R_ROOT] != 0)
		throw std::runtime_error("This articulated figure already has a root");
	this->arbs[R_ROOT] = root;
}

void ArticulatedFigure::addArticulatedRigidBody( ArticulatedRigidBody* arb, ArbID id)
{
    if(id < 0 || id >= R_MAX)
        throw std::runtime_error("Add invalid ARB");
    arbs[id] = arb;
}

/**
	This method is used to compute the total mass of the articulated figure.
*/
void ArticulatedFigure::computeMass()
{
	double curMass = getRoot()->getMass();
	double totalMass = curMass;

	for (unsigned int i=0; i < J_MAX; i++){
		curMass = joints[i]->child->getMass();
		totalMass += curMass;
	}

	mass = totalMass;
}

/**
	This method is used to get the total mass of the articulated figure.
*/
double ArticulatedFigure::getMass(){
	return mass;
}

