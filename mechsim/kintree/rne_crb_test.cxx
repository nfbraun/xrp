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
#include "Robot.h"

const Eigen::Vector3d g(1., 2., 3.);

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

/* Return a sort-of random VectorXd for testing. */
Eigen::VectorXd getTestVector(unsigned int set, unsigned int size)
{
    // Ugly hack to ensure determinism
    const double rnd[3][12] = {
        { 0.3, 0.1, 0.5, 0.2, 0.6, -0.7, 0.4, -0.8, 0.7, 0.2, -0.5, 0.9 },
        { 1.2, 0.4, -1.5, 0.7, 0.4, 1.0, -0.7, 0.2, 0.7, -0.8, -1.7, 0.1 },
        { 0.7, 0.6, 0.3, -0.4, 0.6, -0.1, -1.0, -0.4, -0.8, 0.6, 0.2, 0.1 }
    };
    
    assert(set < sizeof(rnd)/sizeof(rnd[0]));
    assert(size <= sizeof(rnd[set])/sizeof(rnd[set][0]));
    
    Eigen::VectorXd test(size);
    for(unsigned int i=0; i<size; i++)
        test(i) = rnd[set][i];
    
    return test;
}

Eigen::VectorXd dynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot, const Eigen::VectorXd& u)
{
    Robot rob;
    Eigen::Matrix<SE3Tr, Robot::N_DOF, 1> expSq = rob.calc_expSq(q);
    
    Eigen::VectorXd qddot_0(q.size());
    qddot_0.setZero();
    
    Eigen::VectorXd C = calc_u(rob, expSq, qdot, qddot_0, g);
    
    return calc_M(rob, expSq).ldlt().solve(u - C);
}

Eigen::VectorXd Unit(unsigned int size, unsigned int i)
{
    Eigen::VectorXd unit(size);
    unit.setZero();
    unit(i) = 1.;
    
    return unit;
}

#if 0
SpForce foo(const Eigen::VectorXd& q)
{
    SE3Tr Tr[6] = {
        SE3Tr::RotZ(q(0)),
        SE3Tr::RotY(q(1)),
        SE3Tr::RotX(q(2)),
        SE3Tr::RotY(q(3)),
        SE3Tr::RotY(q(4)),
        SE3Tr::RotX(q(5))
    };
    
    SpForce f_i(0.3, 0.2, -0.8, 1.2, -0.4, -0.5);
    
    /* for(unsigned int i=0; i<6; i++) {
        v_i = v_i.tr(Tr[i]);
        v_i += SpMot(0., 1., 0., 0., 0., 0.);
    } */
    
    return f_i.tr(Tr[1]);
}

SpForce d_foo(const Eigen::VectorXd& q)
{
    SE3Tr Tr[6] = {
        SE3Tr::RotZ(q(0)),
        SE3Tr::RotY(q(1)),
        SE3Tr::RotX(q(2)),
        SE3Tr::RotY(q(3)),
        SE3Tr::RotY(q(4)),
        SE3Tr::RotX(q(5))
    };
    
    SpForce f_i(0.3, 0.2, -0.8, 1.2, -0.4, -0.5);
    
    /* for(unsigned int i=0; i<6; i++) {
        v_i = v_i.tr(Tr[i]);
        if(i == 1)
            v_i = SpMot(0., 1., 0., 0., 0., 0.).cross(v_i);
        if(i < 1)
            v_i += SpMot(0., 1., 0., 0., 0., 0.);
    } */
    
    return SpMot(0., 1., 0., 0., 0., 0.).cross_star(f_i).tr(Tr[1]);
}
#endif

void test_dynamics(KinTree& kt)
{
    double error;
    
    Eigen::Matrix<double, 12, 1> q = getTestVector(0, 12);
    Eigen::Matrix<double, 12, 1> qdot = getTestVector(1, 12);
    Eigen::Matrix<double, 12, 1> u = getTestVector(2, 12);
    
    Robot rob;
    Eigen::Matrix<SE3Tr, Robot::N_DOF, 1> expSq = rob.calc_expSq(q);
    
    kt.calcDyn(q, qdot);
    
    error = (kt.M() - calc_M(rob, expSq)).norm();
    
    std::cout << "Mass matrix: ";
    std::cout << (error < 1e-5 ? "pass" : "FAIL");
    std::cout << " (error = " << error << ")" << std::endl;
    
    Eigen::VectorXd qddot_0(q.size());
    qddot_0.setZero();
    Eigen::VectorXd C = calc_u(rob, expSq, qdot, qddot_0, g);
    error = (kt.C() + kt.dVdq() - C).norm();
    
    std::cout << "Bias force: ";
    std::cout << (error < 1e-5 ? "pass" : "FAIL");
    std::cout << " (error = " << error << ")" << std::endl;
    
    Eigen::VectorXd qddot_kt = kt.M().ldlt().solve(u - kt.C() - kt.dVdq());
    
    error = (qddot_kt - dynamics(q, qdot, u)).norm();
    
    std::cout << "Dynamics: ";
    std::cout << (error < 1e-5 ? "pass" : "FAIL");
    std::cout << " (error = " << error << ")" << std::endl;
}

void test_deriv()
{
    double error;
    const double dt = 1e-5;
    
    Eigen::Matrix<double, 12, 1> q = getTestVector(0, 12);
    Eigen::Matrix<double, 12, 1> qdot = getTestVector(1, 12);
    Eigen::Matrix<double, 12, 1> u = getTestVector(2, 12);
    
    Eigen::Matrix<double, 12, 1> qddot;
    qddot = dynamics(q, qdot, u);
    
    Robot rob;
    Eigen::Matrix<SE3Tr, Robot::N_DOF, 1> expSq = rob.calc_expSq(q);
    
    Eigen::LDLT<Eigen::MatrixXd> mInv = calc_M(rob, expSq).ldlt();
    
    Eigen::Matrix<double, 12, 12> du_dq;
    du_dq = calc_du_dq(rob, expSq, qdot, qddot, g);
    Eigen::MatrixXd dqddot_dq = -mInv.solve(du_dq);
    
    Eigen::Matrix<double, 12, 12> dqddot_dq_num;
    for(unsigned int idx=0; idx<12; idx++) {
        dqddot_dq_num.block<12,1>(0,idx) = (dynamics(q + dt*Unit(12,idx), qdot, u) - dynamics(q - dt*Unit(12,idx), qdot, u))/(2.*dt);
    }
    
    error = (dqddot_dq_num - dqddot_dq).norm();
    
    std::cout << "dqddot_dq: ";
    std::cout << (error < 1e-5 ? "pass" : "FAIL");
    std::cout << " (error = " << error << ")" << std::endl;
    
    Eigen::Matrix<double, 12, 12> du_dqdot;
    du_dqdot = calc_du_dqdot(rob, expSq, qdot, qddot);
    Eigen::MatrixXd dqddot_dqdot = -mInv.solve(du_dqdot);
    
    Eigen::Matrix<double, 12, 12> dqddot_dqdot_num;
    for(unsigned int idx=0; idx<12; idx++) {
        dqddot_dqdot_num.block<12,1>(0,idx) = (dynamics(q, qdot + dt*Unit(12,idx), u) - dynamics(q, qdot - dt*Unit(12,idx), u))/(2.*dt);
    }
    
    error = (dqddot_dqdot_num - dqddot_dqdot).norm();
    
    std::cout << "dqddot_dqdot: ";
    std::cout << (error < 1e-5 ? "pass" : "FAIL");
    std::cout << " (error = " << error << ")" << std::endl;
    
    Eigen::MatrixXd dqddot_du = mInv.solve(Eigen::Matrix<double, 12, 12>::Identity());
    
    Eigen::Matrix<double, 12, 12> dqddot_du_num;
    for(unsigned int idx=0; idx<12; idx++) {
        dqddot_du_num.block<12,1>(0,idx) = (dynamics(q, qdot, u + dt*Unit(12,idx)) - dynamics(q, qdot, u - dt*Unit(12,idx)))/(2.*dt);
    }
    
    error = (dqddot_du_num - dqddot_du).norm();
    
    std::cout << "dqddot_du: ";
    std::cout << (error < 1e-5 ? "pass" : "FAIL");
    std::cout << " (error = " << error << ")" << std::endl;
}

void test_deriv2()
{
    const double dt = 1e-5;
    bool all_passed = true;
    
    std::cout << "dM_dq:" << std::endl;
    
    Eigen::Matrix<double, 12, 1> q = getTestVector(0, 12);
    
    for(unsigned int diff_id = 0; diff_id < Robot::N_DOF; diff_id++) {
        Eigen::Matrix<double, 12, 1> q_minus = q - Eigen::Matrix<double, 12, 1>::Unit(diff_id)*dt;
        Eigen::Matrix<double, 12, 1> q_plus = q + Eigen::Matrix<double, 12, 1>::Unit(diff_id)*dt;
        
        Robot rob;
        Eigen::Matrix<SE3Tr, Robot::N_DOF, 1> expSq = rob.calc_expSq(q);
        Eigen::Matrix<SE3Tr, Robot::N_DOF, 1> expSq_minus = rob.calc_expSq(q_minus);
        Eigen::Matrix<SE3Tr, Robot::N_DOF, 1> expSq_plus = rob.calc_expSq(q_plus);
        
        Eigen::MatrixXd M_plus = calc_M(rob, expSq_plus);
        Eigen::MatrixXd M_minus = calc_M(rob, expSq_minus);
        
        Eigen::MatrixXd dM_dq_num = (M_plus - M_minus) / (2.*dt);
        
        const double error = (dM_dq_num - calc_dM_dq(rob, expSq, diff_id)).norm();
        std::cout << "    diff_id = " << diff_id << ": ";
        if(error < 1e-8) {
            std::cout << "pass";
        } else {
            std::cout << "FAIL";
            all_passed = false;
        }
        std::cout << " (error = " << error << ")" << std::endl;
    }
    
    std::cout << "dM_dq: ";
    if(all_passed)
        std::cout << "All tests passed." << std::endl;
    else
        std::cout << "WARNING: some or all tests FAILED!" << std::endl;
}

void test_coriolis()
{
    Eigen::Matrix<double, 12, 1> q = getTestVector(0, 12);
    Eigen::Matrix<double, 12, 1> qdot = Eigen::Matrix<double, 12, 1>::Unit(3);
    
    Robot rob;
    Eigen::Matrix<SE3Tr, Robot::N_DOF, 1> expSq = rob.calc_expSq(q);
    
    Eigen::Matrix<Eigen::Matrix<double, 12, 1>, 12, 12> C;
    
    // Extract diagonal components
    for(unsigned int i=0; i<12; i++) {
        C(i,i) = calc_u(rob, expSq,
                        Eigen::Matrix<double, 12, 1>::Unit(i),
                        Eigen::Matrix<double, 12, 1>::Zero(),
                        Eigen::Vector3d::Zero());
    }
    
    // Extract non-diagonal components
    for(unsigned int i=0; i<12; i++) {
        for(unsigned int j=0; j<i; j++) {
            Eigen::Matrix<double, 12, 1> u_tilde =
                calc_u(rob, expSq,
                       Eigen::Matrix<double, 12, 1>::Unit(i)
                           + Eigen::Matrix<double, 12, 1>::Unit(j),
                       Eigen::Matrix<double, 12, 1>::Zero(),
                       Eigen::Vector3d::Zero());
            C(i,j) = (u_tilde - C(i,i) - C(j,j))/2.;
            C(j,i) = C(i,j);
        }
    }
    
    // Calculate all mass matrix derivatives directly
    Eigen::Matrix<Eigen::Matrix<double, 12, 12>, 12, 1> dM_dq;
    for(unsigned int i=0; i<12; i++) {
        dM_dq(i) = calc_dM_dq(rob, expSq, i);
    }
    
    // Calculate mass matrix derivatives from Coriolis terms
    Eigen::Matrix<Eigen::Matrix<double, 12, 12>, 12, 1> dM_dq_2;
    for(unsigned int i=0; i<12; i++) {
        for(unsigned int j=0; j<12; j++) {
            for(unsigned int k=0; k<12; k++) {
                dM_dq_2(i)(j,k) = C(i,j)(k) + C(k,i)(j);
            }
        }
    }
    
    double sqError = 0.;
    // Compare dM_dq and dM_dq_2
    for(unsigned int i=0; i<12; i++) {
        sqError += (dM_dq(i) - dM_dq_2(i)).squaredNorm();
    }
    
    std::cout << "Coriolis: ";
    std::cout << (sqrt(sqError) < 1e-13 ? "pass" : "FAIL");
    std::cout << " (error = " << sqrt(sqError) << ")" << std::endl;
}

int main()
{
    KinTree kt(construct());
    kt.AssignParamIDs();
    assert(kt.npar() == 12);
    
    kt.setG(g);
    //kt.setG(Eigen::Vector3d(0., 0., 0.));
    
    test_dynamics(kt);
    test_deriv();
    test_deriv2();
    test_coriolis();
    
    return 0;
}
