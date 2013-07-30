#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include "RawODESolver.h"
#include <cassert>
#include <Eigen/Dense>
#include "KinTree.h"
#include "CWJoints.h"
#include "../cartwheel_walker/StaticRobotInfo.h"
#include <gsl/gsl_errno.h>

int n_xdot_eval = 0;

struct JointSysData {
    KinTree* kt;
    unsigned int ndim;
};

int calc_xdot(double t, const double x_raw[], double xdot_raw[], void *_p)
{
    struct JointSysData* data = (struct JointSysData*) _p;
    Eigen::Map<const Eigen::VectorXd> q(x_raw, data->ndim);
    Eigen::Map<const Eigen::VectorXd> qdot(x_raw+data->ndim, data->ndim);
    
    data->kt->calcDyn(q, qdot);
    Eigen::MatrixXd M = data->kt->M();
    
    Eigen::VectorXd C = data->kt->C();
    
    Eigen::VectorXd dVdq = data->kt->dVdq();
    
    Eigen::VectorXd u(data->kt->npar());
    u.setZero();
    
    u(0) =  2.3;   // LAX
    u(1) = -1.5;  // LAY
    u(2) = -0.9;  // LKY
    u(3) = 2.5 * cos(9*t);  // LHX
    u(4) = 1.4 * cos(4*t);  // LHY
    u(5) = 1.0 * cos(6*t);   // LHZ
    
    u(6) = 0.02 * sin(4*t);   // RHZ
    u(7) = -0.5 * sin(5*t); // RHY
    u(8) = 0.1 * sin(7*t);  // RHX
    u(9) = -0.02;   // RKY
    u(10) = 0.01;  // RAY
    u(11) = 0.006; // RAX
    
    Eigen::Map<Eigen::VectorXd> dq_dt(xdot_raw, data->ndim);
    dq_dt = qdot;
    // calculate qdotdot = M^{-1} * (u - C - dVdq)
    Eigen::Map<Eigen::VectorXd> dqdot_dt(xdot_raw + data->ndim, data->ndim);
    dqdot_dt = M.ldlt().solve(u - C - dVdq);
    
    n_xdot_eval++;
    
    return GSL_SUCCESS;
}

RigidBody* construct()
{
    using namespace CharacterConst;
    using Eigen::Vector3d;
    
    RigidBody* lFoot = new RigidBody(0, Vector3d(footPosX, legPosY_L, footPosZ), "lFoot");
    lFoot->setMass(rbMass(B_L_FOOT), rbMOI(B_L_FOOT));
    
    Joint* lAnkle = new InvAnkleJoint(lFoot, Vector3d(0., legPosY_L, anklePosZ), "lAnkle");
    
    RigidBody* lShank = new RigidBody(lAnkle, Vector3d(0., legPosY_L, shankPosZ), "lShank");
    lShank->setMass(rbMass(B_L_SHANK), rbMOI(B_L_SHANK));
    
    Joint* lKnee = new InvKneeJoint(lShank, Vector3d(0., legPosY_L, kneePosZ), "lKnee");
    
    RigidBody* lThigh = new RigidBody(lKnee, Vector3d(0., legPosY_L, thighPosZ), "lThigh");
    lThigh->setMass(rbMass(B_L_THIGH), rbMOI(B_L_THIGH));
    
    Joint* lHip = new InvHipJoint(lThigh, Vector3d(0., legPosY_L, hipPosZ), "lHip");
    
    RigidBody* torso = new RigidBody(lHip, Vector3d(0., 0., pelvisPosZ), "torso");
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
    
    return lFoot;
}

double normalizeAngle(double x)
{
    if(x > 0)
        return fmod(x + M_PI, 2*M_PI) - M_PI;
    else
        return fmod(x - M_PI, 2*M_PI) + M_PI;
}

int main()
{
    struct JointSysData fData;
    
    KinTree kt(construct());
    kt.AssignParamIDs();
    // kt.setG(Eigen::Vector3d(0., 0., -9.81));
    kt.setG(Eigen::Vector3d(0., 0., 0.));
    
    std::vector<double> x_ini(2*kt.npar(), 0.0);
    const double tstep = 1./160.;
    int i;
    
    fData.kt = &kt;
    fData.ndim = kt.npar();
    
    RawODESolver solver(2*kt.npar(), calc_xdot, 0, x_ini.data(), &fData);
    
    const char* names[] = { "LAX", "LAY", "LKY", "LHX", "LHY", "LHZ", "RHZ", "RHY", "RHX", "RKY", "RAY", "RAX" };
    
    assert(kt.npar() == 12);
    
    for(unsigned int i=0; i<kt.npar(); i++)
        std::cout << "#:" << i+1 << ":p_" << names[i] << "_ana" << std::endl;
    
    for(unsigned int i=0; i<kt.npar(); i++)
        std::cout << "#:" << i+1+kt.npar() << ":o_" << names[i] << "_ana" << std::endl;
    
    for(i=0; i<4*160; ++i) {
        solver.EvolveFwd(i * tstep);
        std::cout << solver.t() << " ";
        
        for(unsigned int i=0; i<kt.npar(); i++)
            std::cout << normalizeAngle(solver.y()[i]) << " ";
        for(unsigned int i=0; i<kt.npar(); i++)
            std::cout << solver.y()[i+kt.npar()] << " ";
        std::cout << std::endl;
    }
    
    std::cerr << "n_xdot_eval: " << n_xdot_eval << std::endl;
    
    return 0;
}
