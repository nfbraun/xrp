#pragma once

#include "Character.h"
#include <Physics/ContactPoint.h>
#include <vector>

class JointTorques {
  public:
    JointTorques() : fTorques(J_MAX) {}
    
    Vector3d get(int jid) const { return fTorques.at(jid); }
    void set(int jid, const Vector3d& torque)
        { fTorques.at(jid) = torque; }
    
    const Vector3d& at(int jid) const { return fTorques.at(jid); }
    Vector3d& at(int jid) { return fTorques.at(jid); }
  
  protected:
    std::vector<Vector3d> fTorques;
};

/**
	This class is used to provide a generic interface to a controller. A controller acts on a character - it computes torques that are
	applied to the joints of the character. The details of how the torques are computed are left up to the classes that extend this one.
*/
class Controller {
protected:
	//this is the character that the controller is acting on
	Character* character;
	//and this is the array of torques that will be computed in order for the character to match the desired pose - stored in world coordinates
	std::vector<Vector3d> _torques;
	//these are the last torques that were applied - used to place limits on how fast the torques are allowed to change
	//std::vector<Vector3d> oldTorques;
	//this is the number of joints of the character - stored here for easy access
	int jointCount;
public:
	Controller(Character* ch);
	virtual ~Controller() {}

	Character* getCharacter() const {
		return character;
	}

};
