#pragma once

#include <vector>
#include <Physics/ContactPoint.h>

class ContactInfo {
  public:
    ContactInfo(const std::vector<ContactPoint>& cdata);
    
    Vector3d getForceOn(RigidBody* rb) const;
    bool toeInContact(RigidBody* rb) const;
    
  private:
    std::vector<ContactPoint> fCData;
};
