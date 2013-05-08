#pragma once

#include <vector>
#include <Physics/ContactPoint.h>
#include <Physics/RigidBody.h>

class ContactInfo {
  public:
    ContactInfo(const ContactData& cdata);
    
    Vector3d getForceOnFoot(unsigned int rb_id) const;
    double getNormalForceOnFoot(unsigned int rb_id) const;
    double getTangentialForceOnFoot(unsigned int rb_id) const;
    Vector3d getCoP(unsigned int rb_id, const RigidBody* rb) const;
    Vector3d getCoP2(unsigned int rb_id, const RigidBody* rb) const;
    bool toeInContact(unsigned int rb_id, const RigidBody* rb) const;
    
  private:
    ContactData fCData;
};
