#include "Controller.h"

/**
	Default constructor.
*/
Controller::Controller(Character* ch) {
	this->character = ch;
	jointCount = ch->getJointCount();
	//for (int i=0;i<jointCount;i++)
	//	torques.push_back(Vector3d());
	for (int i=0;i<jointCount;i++){
		character->getJoints()[i]->id = i;
	}
}
