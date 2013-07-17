#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include <cassert>
#include <Eigen/Dense>
#include "KinTree.h"
#include "CWJoints.h"
#include "../cartwheel_walker/StaticRobotInfo.h"
#include "../cartwheel_walker/RobotState.h"
#include "Spatial.h"
#include "RNE_CRB.h"

RigidBody* construct()
{
    using namespace CharacterConst;
    using Eigen::Vector3d;
    
    /* RigidBody* lFoot = new RigidBody(0, Vector3d(0.016, legPosY_L, footPosZ), "lFoot");
    lFoot->setMass(rbMass(B_L_FOOT), rbMOI(B_L_FOOT));
    
    Joint* lAnkle = new InvAnkleJoint(lFoot, Vector3d(0., legPosY_L, anklePosZ), "lAnkle");
    
    RigidBody* lShank = new RigidBody(lAnkle, Vector3d(0., legPosY_L, shankPosZ), "lShank");
    lShank->setMass(rbMass(B_L_SHANK), rbMOI(B_L_SHANK));
    
    Joint* lKnee = new InvKneeJoint(lShank, Vector3d(0., legPosY_L, kneePosZ), "lKnee");
    
    RigidBody* lThigh = new RigidBody(lKnee, Vector3d(0., legPosY_L, thighPosZ), "lThigh");
    lThigh->setMass(rbMass(B_L_THIGH), rbMOI(B_L_THIGH));
    
    Joint* lHip = new InvHipJoint(lThigh, Vector3d(0., legPosY_L, hipPosZ), "lHip"); */
    
    RigidBody* torso = new RigidBody(0, Vector3d(0., 0., pelvisPosZ), "torso");
    torso->setMass(rbMass(B_PELVIS), rbMOI(B_PELVIS));
    
    Joint* rHip = new HipJoint(torso, Vector3d(0., legPosY_R, hipPosZ), "rHip");
    RigidBody* rThigh = new RigidBody(rHip, Vector3d(0., legPosY_R, thighPosZ), "rThigh");
    rThigh->setMass(rbMass(B_R_THIGH), rbMOI(B_R_THIGH));
    
    Joint* rKnee = new KneeJoint(rThigh, Vector3d(0., legPosY_R, kneePosZ), "rKnee");
    RigidBody* rShank = new RigidBody(rKnee, Vector3d(0., legPosY_R, shankPosZ), "rShank");
    rShank->setMass(rbMass(B_R_SHANK), rbMOI(B_R_SHANK));
    
    Joint* rAnkle = new AnkleJoint(rShank, Vector3d(0., legPosY_R, anklePosZ), "rAnkle");
    RigidBody* rFoot = new RigidBody(rAnkle, Vector3d(footPosX, legPosY_R, footPosZ), "rFoot");
    rFoot->setMass(rbMass(B_R_FOOT), rbMOI(B_R_FOOT));
    
    /* std::cout << torso->pos() << std::endl;
    std::cout << lThigh->pos() << std::endl;
    std::cout << lShank->pos() << std::endl;
    std::cout << lFoot->pos() << std::endl; */
    /* std::cout << lHip->pos() << std::endl;
    std::cout << lKnee->pos() << std::endl;
    std::cout << lAnkle->pos() << std::endl; */
    
    return torso;
}

/* Return a sort-of random JSpState for testing. */
JSpState getTestJSpState()
{
    JSpState jstate;
    
    jstate.phi(LHZ) =  0.3;
    jstate.phi(LHY) =  0.1;
    jstate.phi(LHX) =  0.5;
    jstate.phi(LKY) =  0.2;
    jstate.phi(LAY) =  0.6;
    jstate.phi(LAX) = -0.7;
    jstate.phi(RHZ) =  0.4;
    jstate.phi(RHY) = -0.8;
    jstate.phi(RHX) =  0.7;
    jstate.phi(RKY) =  0.2;
    jstate.phi(RAY) = -0.5;
    jstate.phi(RAX) =  0.9;
    
    jstate.omega(LHZ) =  1.2;
    jstate.omega(LHY) =  0.4;
    jstate.omega(LHX) = -1.5;
    jstate.omega(LKY) =  0.7;
    jstate.omega(LAY) =  0.4;
    jstate.omega(LAX) =  1.0;
    jstate.omega(RHZ) = -0.7;
    jstate.omega(RHY) =  0.2;
    jstate.omega(RHX) =  0.7;
    jstate.omega(RKY) =  -0.8;
    jstate.omega(RAY) =  -1.7;
    jstate.omega(RAX) =  0.1;
    
    return jstate;
}

int main()
{
    KinTree kt(construct());
    kt.AssignParamIDs();
    kt.setG(Eigen::Vector3d(0., 0., -9.81));
    
    assert(kt.npar() == 6);
    
    JSpState jstate = getTestJSpState();
    
    Eigen::Matrix<double, 6, 1> q;
    q << jstate.phi(RHZ), jstate.phi(RHY), jstate.phi(RHX),
         jstate.phi(RKY), jstate.phi(RAY), jstate.phi(RAX);
    Eigen::Matrix<double, 6, 1> qdot;
    qdot << jstate.omega(RHZ), jstate.omega(RHY), jstate.omega(RHX),
            jstate.omega(RKY), jstate.omega(RAY), jstate.omega(RAX);
    
    kt.calcDyn(q, qdot);
    
    std::cout << kt.M() << "\n" << std::endl;
    
    Eigen::MatrixXd M2 = CRB(q);
    std::cout << M2 << "\n" << std::endl;
    
    std::cout << "M_error = " << (kt.M() - M2).norm() << std::endl;
    
    std::cout << kt.C().transpose() << std::endl;
    
    Eigen::VectorXd C2 = RNE(q, qdot);
    
    std::cout << C2.transpose() << std::endl;
    
    std::cout << "C_error = " << (kt.C() - C2).norm() << std::endl;
    
    return 0;
}
