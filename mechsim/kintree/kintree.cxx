#include <iostream>
#include <list>
#include <string>
#include <stdexcept>
#include <Eigen/Dense>
#include "KinTree.h"

/* Eigen::Vector3d GetRBPosition()
{
    while(rb != kt.rb_root()) {
        rb = rb->pjoint()->prb();
    }
}

template<typename Param, typename Matrix>
Matrix HingeJoint::R(const Param* params)
{
    return R_x<Param, Matrix>(params[0]);
} */

int main()
{
    /* RigidBody* torso = new RigidBody(0, "torso");
    
    BallJoint* l_hip = new BallJoint(torso, "l_hip");
    RigidBody* l_thigh = new RigidBody(l_hip, "l_thigh");
    HingeJoint* l_knee = new HingeJoint(l_thigh, "l_knee");
    RigidBody* l_shank = new RigidBody(l_knee, "l_shank");
    UniversalJoint* l_ankle = new UniversalJoint(l_shank, "l_ankle");
    RigidBody* l_foot = new RigidBody(l_ankle, "l_foot");
    
    BallJoint* r_hip = new BallJoint(torso, "r_hip");
    RigidBody* r_thigh = new RigidBody(r_hip, "r_thigh");
    HingeJoint* r_knee = new HingeJoint(r_thigh, "r_knee");
    RigidBody* r_shank = new RigidBody(r_knee, "r_shank");
    UniversalJoint* r_ankle = new UniversalJoint(r_shank, "r_ankle");
    RigidBody* r_foot = new RigidBody(r_ankle, "r_foot");
    
    KinTree kt(torso);
    kt.AssignParamIDs();
    
    std::cout << l_knee->pid(0) << std::endl;
    std::cout << kt.npar() << std::endl;
    */
    return 0;
}
