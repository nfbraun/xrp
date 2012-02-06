#pragma once

#include "Character.h"
#include <Physics/ContactPoint.h>
#include <vector>

class JointTorques {
  public:
    JointTorques() : fTorques(J_MAX, Vector3d(0., 0., 0.)) {}
    
    Vector3d get(int jid) const { return fTorques.at(jid); }
    void set(int jid, const Vector3d& torque)
        { fTorques.at(jid) = torque; }
    
    const Vector3d& at(int jid) const { return fTorques.at(jid); }
    Vector3d& at(int jid) { return fTorques.at(jid); }
    
    void add(const JointTorques& other) {
        for(unsigned int i=0; i<J_MAX; i++) {
            fTorques.at(i) += other.at(i);
        }
    }
  
  protected:
    std::vector<Vector3d> fTorques;
};

