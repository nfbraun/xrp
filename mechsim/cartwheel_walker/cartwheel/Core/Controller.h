#pragma once

// FIXME
#include "../../StaticRobotInfo.h"
#include <Physics/ContactPoint.h>
#include <vector>

class RawTorques {
  public:
    RawTorques() : fTorques(J_MAX, Vector3d(0., 0., 0.)) {}
    
    Vector3d get(int jid) const { return fTorques.at(jid); }
    void set(int jid, const Vector3d& torque)
        { fTorques.at(jid) = torque; }
    
    const Vector3d& at(int jid) const { return fTorques.at(jid); }
    Vector3d& at(int jid) { return fTorques.at(jid); }
    
    void add(const RawTorques& other) {
        for(unsigned int i=0; i<J_MAX; i++) {
            fTorques.at(i) += other.at(i);
        }
    }
  
  protected:
    std::vector<Vector3d> fTorques;
};

class JSpTorques {
  public:
    static JSpTorques Zero() {
        JSpTorques jt;
        for(unsigned int i=0; i<DOF_MAX; i++)
            jt.t(i) = 0.;
        return jt;
    }
    
    double t(unsigned int id) const
        { assert(id < DOF_MAX); return fTorques[id]; }
    
    double& t(unsigned int id)
        { assert(id < DOF_MAX); return fTorques[id]; }
    
    double t(unsigned int side, unsigned int id) const
        { assert(side < SIDE_MAX); assert(id < RDOF_MAX); return fTorques[side*RDOF_MAX+id]; }
    
    double& t(unsigned int side, unsigned int id)
        { assert(side < SIDE_MAX); assert(id < RDOF_MAX); return fTorques[side*RDOF_MAX+id]; }
    
  private:
    double fTorques[DOF_MAX];
};

