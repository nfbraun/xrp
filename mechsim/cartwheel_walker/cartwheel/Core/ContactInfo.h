#pragma once

#include "../../RobotState.h"
#include <Physics/ContactPoint.h>

class ContactInfo {
  public:
    ContactInfo(const ContactData& cdata);
    
    Vector3d getForceOnFoot(unsigned int rb_id) const;
    double getNormalForceOnFoot(unsigned int rb_id) const;
    double getTangentialForceOnFoot(unsigned int rb_id) const;
    Vector3d getCoP(unsigned int rb_id, const FullState& fstate) const;
    Vector3d getCoP2(unsigned int rb_id, const FullState& fstate) const;
    bool toeInContact(unsigned int rb_id, const FullState& fstate) const;
    
  private:
    ContactData fCData;
};
