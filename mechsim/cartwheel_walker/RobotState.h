#ifndef CW_ROBOTSTATE_H
#define CW_ROBOTSTATE_H

#include <Eigen/Dense>

#include <StaticRobotInfo.h>
#include <kintree/SE3Tr.h>

/* Full dynamic state of a rigid body (6d position + 6d velocity) */
class BodyQ {
  public:
    BodyQ() {}
    BodyQ(Eigen::Vector3d pos, Eigen::Quaterniond rot,
          Eigen::Vector3d vel, Eigen::Vector3d avel)
        : fPos(pos), fRot(rot), fVel(vel), fAVel(avel) {}
    
    const Eigen::Vector3d& pos() const { return fPos; }
    const Eigen::Quaterniond& rot() const { return fRot; }
    const Eigen::Vector3d& vel() const { return fVel; }
    const Eigen::Vector3d& avel() const { return fAVel; }
    
    const SE3Tr trToWorld() const { return SE3Tr(rot(), pos()); }
    const SE3Tr trToLocal() const { return SE3Tr(rot(), pos()).inverse(); }
    
    Eigen::Vector3d& pos() { return fPos; }
    Eigen::Quaterniond& rot() { return fRot; }
    Eigen::Vector3d& vel() { return fVel; }
    Eigen::Vector3d& avel() { return fAVel; }
    
  private:
    Eigen::Vector3d fPos;
    Eigen::Quaterniond fRot;
    Eigen::Vector3d fVel;
    Eigen::Vector3d fAVel;
};

/* Full dynamic state of the robot, i.e. 6d positions and velocities for all its
   rigid bodies. */
class FullState {
  public:
    const BodyQ& q(unsigned int id) const
        { assert(id < B_MAX); return fQ[id]; }
    BodyQ& q(unsigned int id)
        { assert(id < B_MAX); return fQ[id]; }
    
    const BodyQ& q(unsigned int side, unsigned int id) const
        { assert(side < SIDE_MAX); assert(id < RBODY_MAX); return fQ[side*RBODY_MAX+id]; }
    BodyQ& q(unsigned int side, unsigned int id)
        { assert(side < SIDE_MAX); assert(id < RBODY_MAX); return fQ[side*RBODY_MAX+id]; }
    
    /* Convenience functions */
    const Eigen::Vector3d& pos(unsigned int id) const    { return q(id).pos(); }
    const Eigen::Quaterniond& rot(unsigned int id) const { return q(id).rot(); }
    const Eigen::Vector3d& vel(unsigned int id) const    { return q(id).vel(); }
    const Eigen::Vector3d& avel(unsigned int id) const   { return q(id).avel(); }
    
    const SE3Tr trToWorld(unsigned int id) const { return q(id).trToWorld(); }
    const SE3Tr trToLocal(unsigned int id) const { return q(id).trToLocal(); }
    
    Eigen::Vector3d& pos(unsigned int id)    { return q(id).pos(); }
    Eigen::Quaterniond& rot(unsigned int id) { return q(id).rot(); }
    Eigen::Vector3d& vel(unsigned int id)    { return q(id).vel(); }
    Eigen::Vector3d& avel(unsigned int id)   { return q(id).avel(); }
    
    const Eigen::Vector3d& pos(unsigned int side, unsigned int id) const    { return q(side, id).pos(); }
    const Eigen::Quaterniond& rot(unsigned int side, unsigned int id) const { return q(side, id).rot(); }
    const Eigen::Vector3d& vel(unsigned int side, unsigned int id) const    { return q(side, id).vel(); }
    const Eigen::Vector3d& avel(unsigned int side, unsigned int id) const   { return q(side, id).avel(); }
    
    const SE3Tr trToWorld(unsigned int side, unsigned int id) const { return q(side, id).trToWorld(); }
    const SE3Tr trToLocal(unsigned int side, unsigned int id) const { return q(side, id).trToLocal(); }
    
    Eigen::Vector3d& pos(unsigned int side, unsigned int id)    { return q(side, id).pos(); }
    Eigen::Quaterniond& rot(unsigned int side, unsigned int id) { return q(side, id).rot(); }
    Eigen::Vector3d& vel(unsigned int side, unsigned int id)    { return q(side, id).vel(); }
    Eigen::Vector3d& avel(unsigned int side, unsigned int id)   { return q(side, id).avel(); }
    
  private:
    BodyQ fQ[B_MAX];
};

/* Joint space state of the robot, i.e. generalized positions and velocities for
   all its degrees of freedom.
   Note that this contains less information than the FullState, as the 6d
   position and velocity of the system as a whole is not fixed. */
class JSpState {
  public:
    static JSpState Null() {
        JSpState jstate;
        for(unsigned int id=0; id<DOF_MAX; id++) {
            jstate.phi(id) = 0.;
            jstate.omega(id) = 0.;
        }
        return jstate;
    }
    
    double phi(unsigned int id) const
        { assert(id < DOF_MAX); return fPhi[id]; }
    double omega(unsigned int id) const
        { assert(id < DOF_MAX); return fOmega[id]; }
    
    double& phi(unsigned int id)
        { assert(id < DOF_MAX); return fPhi[id]; }
    double& omega(unsigned int id)
        { assert(id < DOF_MAX); return fOmega[id]; }
    
    double phi(unsigned int side, unsigned int id) const
        { assert(side < SIDE_MAX); assert(id < RDOF_MAX); return fPhi[side*RDOF_MAX+id]; }
    double omega(unsigned int side, unsigned int id) const
        { assert(side < SIDE_MAX); assert(id < RDOF_MAX); return fOmega[side*RDOF_MAX+id]; }
    
    double& phi(unsigned int side, unsigned int id)
        { assert(side < SIDE_MAX); assert(id < RDOF_MAX); return fPhi[side*RDOF_MAX+id]; }
    double& omega(unsigned int side, unsigned int id)
        { assert(side < SIDE_MAX); assert(id < RDOF_MAX); return fOmega[side*RDOF_MAX+id]; }
    
    double fPhi[DOF_MAX];
    double fOmega[DOF_MAX];
};

#endif
