#include "DynInfo.h"
#include "StaticRobotInfo.h"

/* See DynInfo.h for function documentation. */

Eigen::Vector3d comPos(const FullState& fstate)
{
    Eigen::Vector3d r = Eigen::Vector3d::Zero();
    
    for(unsigned int b = 0; b < B_MAX; b++) {
        r += rbMass(b) * fstate.pos(b);
    }
    
    return r/totalMass();
}

Eigen::Vector3d comVel(const FullState& fstate)
{
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    
    for(unsigned int b = 0; b < B_MAX; b++) {
        v += rbMass(b) * fstate.vel(b);
    }
    
    return v;
}

double Ekin(const FullState& fstate)
{
    double Ek = 0.;
    
    for(unsigned int b=0; b<B_MAX; b++) {
        Ek += 0.5 * rbMass(b) * fstate.vel(b).squaredNorm();
        
        const Eigen::Vector3d avel =
            fstate.rot(b).conjugate()._transformVector(fstate.avel(b));
        Ek += 0.5 * avel.dot(rbMOI(b).asDiagonal() * avel);
    }
    
    return Ek;
}

double Epot(const FullState& fstate)
{
    double Ep = 0.;
    const Eigen::Vector3d g(0., 0., -9.81);
    
    for(unsigned int b = 0; b < B_MAX; b++) {
        Ep += -rbMass(b) * g.dot(fstate.pos(b));
    }
    
    return Ep;
}

double compute_Epot0()
{
    using namespace CharacterConst;
    
    const Eigen::Vector3d g(0., 0., -9.81);
    const Eigen::Vector3d cm =
          (rbMass(B_L_THIGH) * Eigen::Vector3d(0., legPosY_L, thighPosZ))
        + (rbMass(B_L_SHANK) * Eigen::Vector3d(0., legPosY_L, shankPosZ))
        + (rbMass(B_L_FOOT)  * Eigen::Vector3d(0., legPosY_L, footPosZ))
        + (rbMass(B_R_THIGH) * Eigen::Vector3d(0., legPosY_R, thighPosZ))
        + (rbMass(B_R_SHANK) * Eigen::Vector3d(0., legPosY_R, shankPosZ))
        + (rbMass(B_R_FOOT)  * Eigen::Vector3d(0., legPosY_R, footPosZ))
        + (rbMass(B_PELVIS)  * Eigen::Vector3d(0., 0., pelvisPosZ));
    const double Ep0 = -g.dot(cm);
    
    return Ep0;
}

double Epot0()
{
    static const double Ep0 = compute_Epot0();
    return Ep0;
}

double Psys(const JSpTorques& torque, const JSpState& jstate)
{
    double P = 0.;
    
    for(unsigned int dof=0; dof<DOF_MAX; dof++) {
        P += torque.t(dof) * jstate.omega(dof);
    }
    
    return P;
}

