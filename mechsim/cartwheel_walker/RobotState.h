#ifndef CW_ROBOTSTATE_H
#define CW_ROBOTSTATE_H

#include <Eigen/Dense>

#include "StaticRobotInfo.h"

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
    
    Eigen::Vector3d& pos(unsigned int id)    { return q(id).pos(); }
    Eigen::Quaterniond& rot(unsigned int id) { return q(id).rot(); }
    Eigen::Vector3d& vel(unsigned int id)    { return q(id).vel(); }
    Eigen::Vector3d& avel(unsigned int id)   { return q(id).avel(); }
    
    const Eigen::Vector3d& pos(unsigned int side, unsigned int id) const    { return q(side, id).pos(); }
    const Eigen::Quaterniond& rot(unsigned int side, unsigned int id) const { return q(side, id).rot(); }
    const Eigen::Vector3d& vel(unsigned int side, unsigned int id) const    { return q(side, id).vel(); }
    const Eigen::Vector3d& avel(unsigned int side, unsigned int id) const   { return q(side, id).avel(); }
    
    Eigen::Vector3d& pos(unsigned int side, unsigned int id)    { return q(side, id).pos(); }
    Eigen::Quaterniond& rot(unsigned int side, unsigned int id) { return q(side, id).rot(); }
    Eigen::Vector3d& vel(unsigned int side, unsigned int id)    { return q(side, id).vel(); }
    Eigen::Vector3d& avel(unsigned int side, unsigned int id)   { return q(side, id).avel(); }
    
  private:
    BodyQ fQ[B_MAX];
};

#endif
