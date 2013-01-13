#include "ContactInfo.h"
#include <Physics/RigidBody.h>
#include <Physics/ArticulatedFigure.h>
#include <Physics/PhysicsGlobals.h>

ContactInfo::ContactInfo(const ContactData& cdata)
{
    fCData = cdata;
}

/**
	This method returns the net force on the body rb, acting from the ground
*/
Vector3d ContactInfo::getForceOnFoot(unsigned int rb_id) const
{
    Vector3d fNet = Vector3d(0., 0., 0.);
    
    if(rb_id == R_L_FOOT) {
        for (unsigned int i=0; i<fCData.pLeft.size(); i++) {
            fNet += fCData.pLeft[i].f;
        }
    } else if(rb_id == R_R_FOOT) {
        for (unsigned int i=0; i<fCData.pRight.size(); i++) {
            fNet += fCData.pRight[i].f;
        }
    }
    
    return fNet;
}

double ContactInfo::getNormalForceOnFoot(unsigned int rb_id) const
{
    return getForceOnFoot(rb_id).dot(PhysicsGlobals::up);
}

double ContactInfo::getTangentialForceOnFoot(unsigned int rb_id) const
{
    const Vector3d f = getForceOnFoot(rb_id);
    const Vector3d n = PhysicsGlobals::up;
    
    return (f - n*(f.dot(n))).norm();
}

/**
	determines if there are any toe forces on the given RB
*/
bool ContactInfo::toeInContact(unsigned int rb_id, const RigidBody* rb) const
{
    //figure out if the toe/heel are in contact...
    bool toeForce = false;
    
    if(rb_id == R_L_FOOT) {
        for (unsigned int i=0; i<fCData.pLeft.size(); i++) {
            Point3d tmpP = rb->getLocalCoordinatesForPoint(fCData.pLeft[i].cp);
            if (tmpP.z() > 0) toeForce = true;
        }
    } else if(rb_id == R_R_FOOT) {
        for (unsigned int i=0; i<fCData.pRight.size(); i++) {
            Point3d tmpP = rb->getLocalCoordinatesForPoint(fCData.pRight[i].cp);
            if (tmpP.z() > 0) toeForce = true;
        }
    } else {
        assert(false);   // invalid argument
    }
    
    return toeForce;
}

