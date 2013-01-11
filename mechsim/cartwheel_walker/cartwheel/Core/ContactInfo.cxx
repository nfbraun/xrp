#include "ContactInfo.h"
#include <Physics/RigidBody.h>

ContactInfo::ContactInfo(const std::vector<ContactPoint>& cdata)
{
    fCData = cdata;
}

/**
	This method returns the net force on the body rb, acting from the ground
*/
Vector3d ContactInfo::getForceOn(RigidBody* rb) const
{
	Vector3d fNet = Vector3d(0., 0., 0.);
	for (unsigned int i=0;i<fCData.size();i++){
		if (fCData[i].rb1 == rb)
			fNet += fCData[i].f;
		if (fCData[i].rb2 == rb)
			fNet -= fCData[i].f;
	}
	return fNet;
}

/**
	determines if there are any toe forces on the given RB
*/
bool ContactInfo::toeInContact(RigidBody* rb) const
{
	//figure out if the toe/heel are in contact...
	bool toeForce = false;
	Point3d tmpP;
	for (unsigned int i=0;i<fCData.size();i++) {
		if (fCData[i].rb1 == rb || fCData[i].rb2 == rb) {
			tmpP = rb->getLocalCoordinatesForPoint(fCData[i].cp);
			if (tmpP.z > 0) toeForce = true;
		}
	}
	
	return toeForce;
}

