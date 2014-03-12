#include "Reaction.h"
#include <kintree/RNE_CRB.h>
#include <kintree/Robot.h>

void calcReaction(Eigen::Vector3d& F, Eigen::Vector3d& T, const FullState& fstate, const JSpState& jstate, const JSpTorques& jt, unsigned int stance)
{
    const Eigen::Vector3d g(0., 0., -9.81);
    
    assert(Robot::N_DOF == 12);
    
    Eigen::VectorXd q(Robot::N_DOF);
    if(stance == Robot::LEFT_STANCE) {
        q << jstate.phi(LAX), jstate.phi(LAY), jstate.phi(LKY),
             jstate.phi(LHX), jstate.phi(LHY), jstate.phi(LHZ),
             jstate.phi(RHZ), jstate.phi(RHY), jstate.phi(RHX),
             jstate.phi(RKY), jstate.phi(RAY), jstate.phi(RAX);
    } else {
        q << jstate.phi(RAX), jstate.phi(RAY), jstate.phi(RKY),
             jstate.phi(RHX), jstate.phi(RHY), jstate.phi(RHZ),
             jstate.phi(LHZ), jstate.phi(LHY), jstate.phi(LHX),
             jstate.phi(LKY), jstate.phi(LAY), jstate.phi(LAX);
    }
    
    Eigen::VectorXd qdot(Robot::N_DOF);
    if(stance == Robot::LEFT_STANCE) {
        qdot << jstate.omega(LAX), jstate.omega(LAY), jstate.omega(LKY),
                jstate.omega(LHX), jstate.omega(LHY), jstate.omega(LHZ),
                jstate.omega(RHZ), jstate.omega(RHY), jstate.omega(RHX),
                jstate.omega(RKY), jstate.omega(RAY), jstate.omega(RAX);
    } else {
        qdot << jstate.omega(RAX), jstate.omega(RAY), jstate.omega(RKY),
                jstate.omega(RHX), jstate.omega(RHY), jstate.omega(RHZ),
                jstate.omega(LHZ), jstate.omega(LHY), jstate.omega(LHX),
                jstate.omega(LKY), jstate.omega(LAY), jstate.omega(LAX);
    }
    
    Eigen::VectorXd tau(Robot::N_DOF);
    if(stance == Robot::LEFT_STANCE) {
        tau << jt.t(LAX), jt.t(LAY), jt.t(LKY),
               jt.t(LHX), jt.t(LHY), jt.t(LHZ),
               jt.t(RHZ), jt.t(RHY), jt.t(RHX),
               jt.t(RKY), jt.t(RAY), jt.t(RAX);
    } else {
        tau << jt.t(RAX), jt.t(RAY), jt.t(RKY),
               jt.t(RHX), jt.t(RHY), jt.t(RHZ),
               jt.t(LHZ), jt.t(LHY), jt.t(LHX),
               jt.t(LKY), jt.t(LAY), jt.t(LAX);
    }
    
    Robot rob(stance);
    Eigen::Matrix<SE3Tr, Robot::N_DOF, 1> expSq = rob.calc_expSq(q);
    
    Eigen::VectorXd qddot_0(q.size());
    qddot_0.setZero();
    
    Eigen::VectorXd C = calc_u(rob, expSq, qdot, qddot_0, g);
    Eigen::VectorXd qddot = calc_M(rob, expSq).ldlt().solve(tau - C);
    
    // Eigen::VectorXd qddot = calc_M(rob, q).ldlt().solve(tau);
    
    // calc_u(rob, q, qdot, qddot) == tau !
    
    SpForce FR = calc_react(rob, expSq, qdot, qddot, g);
    FR = FR.tr(SE3Tr::Trans(-CharacterConst::footPosX, 0., CharacterConst::footSizeZ/2.));
    
    if(stance == Robot::LEFT_STANCE)
        FR = FR.tr(SE3Tr::Rot(fstate.rot(B_L_FOOT)));
    else
        FR = FR.tr(SE3Tr::Rot(fstate.rot(B_R_FOOT)));
    
    FR += SpForce(Eigen::Vector3d::Zero(), -g*rbMass(B_L_FOOT));
    
    F = FR.lin();
    T = FR.ang();
}
