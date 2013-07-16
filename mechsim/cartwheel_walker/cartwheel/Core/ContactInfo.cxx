#include "ContactInfo.h"
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
    if(rb_id == B_L_FOOT) {
        return fCData.lFtot;
    } else if(rb_id == B_R_FOOT) {
        return fCData.rFtot;
    } else {
        assert(false); // invalid argument
    }
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

Vector3d ContactInfo::getCoP(unsigned int rb_id, const FullState& fstate) const
{
    double fn_tot = 0.;
    Point3d cop(0., 0., 0.);
    SE3Tr toLocal = fstate.trToLocal(rb_id);
    
    if(rb_id == B_L_FOOT) {
        for (unsigned int i=0; i<fCData.pLeft.size(); i++) {
            double fn = fCData.pLeft[i].f.dot(PhysicsGlobals::up);
            cop += fn * fCData.pLeft[i].cp;
            fn_tot += fn;
        }
    } else if(rb_id == B_R_FOOT) {
        for (unsigned int i=0; i<fCData.pRight.size(); i++) {
            double fn = fCData.pRight[i].f.dot(PhysicsGlobals::up);
            cop += fn * fCData.pRight[i].cp;
            fn_tot += fn;
        }
    } else {
        assert(false); // invalid argument
    }
    
    if(fn_tot < 0.0001) {
        return cop;
    } else {
        return toLocal.onPoint(cop.toEigen()/fn_tot);
    }
}

Vector3d ContactInfo::getCoP2(unsigned int rb_id, const FullState& fstate) const
{
    const double h = CharacterConst::footSizeZ / 2.;
    const Vector3d n = PhysicsGlobals::up;
    
    if(rb_id == B_L_FOOT) {
        if(fCData.lFtot.dot(n) < 0.0001) {
            return Vector3d(0., 0., 0.);
        } else {
            Point3d p = (n.cross(fCData.lTtot) - h*fCData.lFtot)/(fCData.lFtot.dot(n));
            
            return fstate.trToLocal(B_L_FOOT).onVector(p.toEigen());
        }
    } else if(rb_id == B_R_FOOT) {
        if(fCData.rFtot.dot(n) < 0.0001) {
            return Vector3d(0., 0., 0.);
        } else {
            Point3d p = (n.cross(fCData.rTtot) - h*fCData.rFtot)/(fCData.rFtot.dot(n));
            
            return fstate.trToLocal(B_R_FOOT).onVector(p.toEigen());
        }
    } else {
        assert(false); // invalid argument
    }
}

/**
	determines if there are any toe forces on the given RB
*/
bool ContactInfo::toeInContact(unsigned int rb_id, const FullState& fstate) const
{
    //figure out if the toe/heel are in contact...
    bool toeForce = false;
    
    if(rb_id == B_L_FOOT) {
        for (unsigned int i=0; i<fCData.pLeft.size(); i++) {
            Eigen::Vector3d tmpP = fstate.trToLocal(rb_id).onPoint(fCData.pLeft[i].cp.toEigen());
            
            if (tmpP.x() > 0) toeForce = true;
        }
    } else if(rb_id == B_R_FOOT) {
        for (unsigned int i=0; i<fCData.pRight.size(); i++) {
            Eigen::Vector3d tmpP = fstate.trToLocal(rb_id).onPoint(fCData.pRight[i].cp.toEigen());
            
            if (tmpP.x() > 0) toeForce = true;
        }
    } else {
        assert(false);   // invalid argument
    }
    
    return toeForce;
}

