#ifndef CW_TORQUES_H
#define CW_TORQUES_H

#include <StaticRobotInfo.h>
#include <Eigen/Dense>

class RawTorques {
  public:
    RawTorques() {
        for(unsigned int i=0; i<J_MAX; i++)
            fTorques[i].setZero();
    }
    
    const Eigen::Vector3d& get(unsigned int jid) const { return at(jid); }
    void set(unsigned int jid, const Eigen::Vector3d& torque)
        { at(jid) = torque; }
    
    const Eigen::Vector3d& at(unsigned int jid) const { assert(jid < J_MAX); return fTorques[jid]; }
    Eigen::Vector3d& at(unsigned int jid) { assert(jid < J_MAX); return fTorques[jid]; }
    
    void add(const RawTorques& other) {
        for(unsigned int i=0; i<J_MAX; i++) {
            at(i) += other.at(i);
        }
    }
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  
  protected:
    Eigen::Vector3d fTorques[J_MAX];
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
    
    JSpTorques operator+(const JSpTorques& rhs) const {
        JSpTorques result;
        for(unsigned int i=0; i<DOF_MAX; i++) {
            result.t(i) = t(i) + rhs.t(i);
        }
        return result;
    }
    
    JSpTorques operator*(double rhs) const {
        JSpTorques result;
        for(unsigned int i=0; i<DOF_MAX; i++) {
            result.t(i) = rhs*t(i);
        }
        return result;
    }
    
    void operator+=(const JSpTorques& rhs) {
        for(unsigned int i=0; i<DOF_MAX; i++) {
            t(i) += rhs.t(i);
        }
    }
    
    void operator*=(double rhs) {
        for(unsigned int i=0; i<DOF_MAX; i++) {
            t(i) *= rhs;
        }
    }
    
  private:
    double fTorques[DOF_MAX];
};

inline JSpTorques operator*(double lhs, const JSpTorques& rhs)
{
    return rhs * lhs;
}

#endif
