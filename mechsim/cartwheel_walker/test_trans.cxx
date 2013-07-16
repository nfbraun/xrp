#include "Cartwheel.h"
#include "SE3Tr.h"
#include "StaticRobotInfo.h"
#include "RobotState.h"
#include "DynTransform.h"
#include <iostream>
#include <limits>

std::ostream& operator<<(std::ostream& s, const Eigen::Quaterniond& q)
{
    s << q.w() << " " << q.x() << " " << q.y() << " " << q.z();
}

void foo_leg_FullFromJoint(unsigned int side, FullState& fstate, const JSpState& jstate)
{
    using namespace CharacterConst;
    
    SE3Tr T = SE3Tr(
        Eigen::Quaterniond(sqrt(4./12.), sqrt(3./12.), sqrt(2./12.), -sqrt(3./12.)),
        Eigen::Vector3d(1., 5.,2.));
    
    Eigen::Vector3d relHipPos;
    if(side == LEFT)
        relHipPos = Eigen::Vector3d(0., legPosY_L, -pelvisSizeZ/2.);
    else
        relHipPos = Eigen::Vector3d(0., legPosY_R, -pelvisSizeZ/2.);
    
    const Eigen::Vector3d relThighPos(0., 0., -thighSizeZ/2.);
    const Eigen::Vector3d relShankPos(0., 0., -shankSizeZ/2.);
    const Eigen::Vector3d relFootPos(0.016, 0., -footSizeZ/2.);
    
    // Velocity field accumulators (v(r) = v0 + w.cross(r))
    Eigen::Vector3d w(-2., -.4, 5.);
    Eigen::Vector3d v0 = Eigen::Vector3d(-3., -.5, -.7) - w.cross(Eigen::Vector3d(1., 5.,2.));
    
    /*** Thigh ***/
    T = T * SE3Tr::Trans(relHipPos);
    const Eigen::Vector3d hipPos = T.trans();
    T = T * hipTransform(jstate.phi(side, HZ), jstate.phi(side, HY), jstate.phi(side, HX));
    fstate.pos(side, B_THIGH) = T.onPoint(relThighPos);
    fstate.rot(side, B_THIGH) = T.rot();
    
    const Eigen::Vector3d omega_H(jstate.omega(side, HX), jstate.omega(side, HY), jstate.omega(side, HZ));
    // Angular velocity difference across hip, expressed in pelvis frame
    const Eigen::Vector3d avel_H_P = hipAVelTransform(jstate.phi(side, HZ),
                                                    jstate.phi(side, HY),
                                                    jstate.phi(side, HX)) * omega_H;
    // Angular velocity difference across hip, expressed in global frame
    const Eigen::Vector3d avel_H = fstate.rot(B_PELVIS)._transformVector(avel_H_P);
    
    w += avel_H;
    v0 -= avel_H.cross(hipPos);
    
    fstate.avel(side, B_THIGH) = w;
    fstate.vel(side, B_THIGH) = v0 + w.cross(fstate.pos(side, B_THIGH));
    
    /*** Shank ***/
    T = T * SE3Tr::Trans(Eigen::Vector3d(0., 0., -thighSizeZ));
    const Eigen::Vector3d kneePos = T.trans();
    T = T * kneeTransform(jstate.phi(side, KY));
    fstate.pos(side, B_SHANK) = T.onPoint(relShankPos);
    fstate.rot(side, B_SHANK) = T.rot();
    
    // Angular velocity difference across knee, expressed in thigh frame
    const Eigen::Vector3d avel_K_T = jstate.omega(side, KY) * Eigen::Vector3d::UnitY();
    // Angular velocity difference across knee, expressed in global frame
    const Eigen::Vector3d avel_K = fstate.rot(side, B_THIGH)._transformVector(avel_K_T);
    
    w += avel_K;
    v0 -= avel_K.cross(kneePos);
    
    fstate.avel(side, B_SHANK) = w;
    fstate.vel(side, B_SHANK) = v0 + w.cross(fstate.pos(side, B_SHANK));
    
    /*** Foot ***/
    T = T * SE3Tr::Trans(Eigen::Vector3d(0., 0., -shankSizeZ));
    const Eigen::Vector3d anklePos = T.trans();
    T = T * ankleTransform(jstate.phi(side, AY), jstate.phi(side, AX));
    fstate.pos(side, B_FOOT) = T.onPoint(relFootPos);
    fstate.rot(side, B_FOOT) = T.rot();
    
    // Angular velocity difference across ankle, expressed in shank frame
    const Eigen::Vector3d avel_A_S(cos(jstate.phi(side, AY)) * jstate.omega(side, AX),
                                   jstate.omega(side, AY),
                                  -sin(jstate.phi(side, AY)) * jstate.omega(side, AX));
    const Eigen::Vector3d avel_A = fstate.rot(side, B_SHANK)._transformVector(avel_A_S);
    
    w += avel_A;
    v0 -= avel_A.cross(anklePos);
    
    fstate.avel(side, B_FOOT) = w;
    fstate.vel(side, B_FOOT) = v0 + w.cross(fstate.pos(side, B_FOOT));
}

FullState foo_fullFromJoint(const JSpState& jstate)
{
    using namespace CharacterConst;
    
    FullState fstate;
    fstate.pos(B_PELVIS) = Eigen::Vector3d(1., 5.,2.);
    fstate.rot(B_PELVIS) = Eigen::Quaterniond(sqrt(4./12.), sqrt(3./12.), sqrt(2./12.), -sqrt(3./12.));
    fstate.avel(B_PELVIS) = Eigen::Vector3d(-2., -.4, 5.);
    fstate.vel(B_PELVIS) = Eigen::Vector3d(-3., -.5, -.7);
    
    foo_leg_FullFromJoint(LEFT, fstate, jstate);
    foo_leg_FullFromJoint(RIGHT, fstate, jstate);
    
    return fstate;
}

bool compareFullStates(const FullState& s1, const FullState& s2, double tol, bool print=false)
{
    double max_pos_error = 0.;
    double max_rot_error = 0.;
    double max_vel_error = 0.;
    double max_avel_error = 0.;
    
    for(unsigned int i=0; i<B_MAX; i++) {
        const double pos_error = (s1.pos(i) - s2.pos(i)).norm();
        const double rot_error = (s1.rot(i).toRotationMatrix() - s2.rot(i).toRotationMatrix()).norm();
        const double vel_error = (s1.vel(i) - s2.vel(i)).norm();
        const double avel_error = (s1.avel(i) - s2.avel(i)).norm();
        
        if(pos_error > max_pos_error) max_pos_error = pos_error;
        if(rot_error > max_rot_error) max_rot_error = rot_error;
        if(vel_error > max_vel_error) max_vel_error = vel_error;
        if(avel_error > max_avel_error) max_avel_error = avel_error;
    }
    
    bool pass = ((max_pos_error < tol) &&
                 (max_rot_error < tol) &&
                 (max_vel_error < tol) &&
                 (max_avel_error < tol));
    
    if(print || !pass) {
        std::cout << "max_pos_error:  " << max_pos_error << std::endl;
        std::cout << "max_rot_error:  " << max_rot_error << std::endl;
        std::cout << "max_vel_error:  " << max_vel_error << std::endl;
        std::cout << "max_avel_error: " << max_avel_error << std::endl;
    }
    
    return pass;
}

bool compareJSpStates(const JSpState& s1, const JSpState& s2, double tol, bool print=false)
{
    double max_phi_error = 0.;
    double max_omega_error = 0.;
    
    for(unsigned int i=0; i<DOF_MAX; i++) {
        const double phi_error = std::abs(s1.phi(i) - s2.phi(i));
        const double omega_error = std::abs(s1.omega(i) - s2.omega(i));
        
        if(phi_error > max_phi_error) max_phi_error = phi_error;
        if(omega_error > max_omega_error) max_omega_error = omega_error;
    }
    
    bool pass = ((max_phi_error < tol) && (max_omega_error < tol));
    
    if(print || !pass) {
        std::cout << "max_phi_error:   " << max_phi_error << std::endl;
        std::cout << "max_omega_error: " << max_omega_error << std::endl;
    }
    
    return pass;
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
    jstate.omega(RKY) = -0.8;
    jstate.omega(RAY) = -1.7;
    jstate.omega(RAX) =  0.1;
    
    return jstate;
}

bool test_rotDecomp()
{
    bool pass = true;
    const double tol = 1e-15;
    
    double z, y, x;
    
    Eigen::Quaterniond qy(Eigen::AngleAxisd(-0.43, Eigen::Vector3d::UnitY()));
    decompYRot(qy, y);
    pass = pass && std::abs(-0.43 - y) < tol;
    
    Eigen::Quaterniond qyx = ( Eigen::AngleAxisd(0.84, Eigen::Vector3d::UnitY())
                             * Eigen::AngleAxisd(-0.33, Eigen::Vector3d::UnitX()));
    decompYXRot(qyx, y, x);
    pass = pass && std::abs(0.84 - y) < tol;
    pass = pass && std::abs(-0.33 - x) < tol;
    
    Eigen::Quaterniond qzyx = ( Eigen::AngleAxisd(0.2, Eigen::Vector3d::UnitZ())
                              * Eigen::AngleAxisd(0.44, Eigen::Vector3d::UnitY())
                              * Eigen::AngleAxisd(-0.9, Eigen::Vector3d::UnitX()));
    decompZYXRot(qzyx, z, y, x);
    pass = pass && std::abs(0.2 - z) < tol;
    pass = pass && std::abs(0.44 - y) < tol;
    pass = pass && std::abs(-0.9 - x) < tol;
    
    return pass;
}

/* Test if fullFromJoint() and jointFromFull() are inverse. */
bool test_inverse()
{
    bool pass = true;
    
    JSpState jstate = getTestJSpState();
    
    FullState fstate = fullFromJoint(jstate);
    JSpState jstate2 = jointFromFull(fstate);
    pass = pass && compareJSpStates(jstate, jstate2, 1e-14, false);
    
    setBodyState(fstate, B_R_THIGH, Eigen::Vector3d(1.2, 1.5, -3.),
                                    Eigen::Quaterniond(sqrt(1./2.), sqrt(1./12.), sqrt(2./12.), sqrt(3./12.)),
                                    Eigen::Vector3d(-4., 1.7, -3.2),
                                    Eigen::Vector3d(1.8, -0.6, 6.5));
    
    JSpState jstate3 = jointFromFull(fstate);
    pass = pass && compareJSpStates(jstate, jstate3, 1e-14, false);
    
    return pass;
}

bool test_setBodyState()
{
    bool pass;
    bool full_pass = true;
    const double tol = 1e-12;
    
    JSpState jstate = getTestJSpState();
    
    FullState fstate = fullFromJoint(jstate);
    
    const Eigen::Vector3d new_pos(1.2, 1.5, -3.);
    const Eigen::Quaterniond new_rot(sqrt(1./2.), sqrt(1./12.), sqrt(2./12.), sqrt(3./12.));
    const Eigen::Vector3d new_vel(-4., 1.7, -3.2);
    const Eigen::Vector3d new_avel(1.8, -0.6, 6.5);
    
    setBodyState(fstate, B_R_THIGH, new_pos, new_rot, new_vel, new_avel);
    
    const double pos_error = (fstate.pos(B_R_THIGH) - new_pos).norm();
    const double rot_error = (fstate.rot(B_R_THIGH).toRotationMatrix() - new_rot.toRotationMatrix()).norm();
    const double vel_error = (fstate.vel(B_R_THIGH) - new_vel).norm();
    const double avel_error = (fstate.avel(B_R_THIGH) - new_avel).norm();
    
    pass = (pos_error < tol && rot_error < tol && vel_error < tol && avel_error < tol);
    
    if(!pass) {
        std::cout << "setBodyState (1) FAILED!" << std::endl;
        std::cout << "pos_error:  " << pos_error << std::endl;
        std::cout << "rot_error:  " << rot_error << std::endl;
        std::cout << "vel_error:  " << vel_error << std::endl;
        std::cout << "avel_error: " << avel_error << std::endl;
    }
    
    full_pass = full_pass && pass;
    
    FullState fstate2 = fullFromJoint(jstate);
    
    setBodyState(fstate2, B_L_SHANK, Eigen::Vector3d(3., 2., 5.),
                                     Eigen::Quaterniond(sqrt(3./12.), sqrt(5./12.), sqrt(2./12.), sqrt(2./12.)),
                                     Eigen::Vector3d(1., 1.5, 0.4),
                                     Eigen::Vector3d(0.1, 0.4, 0.7));
    setBodyState(fstate2, B_R_THIGH, new_pos, new_rot, new_vel, new_avel);
    
    const double pos_error_2 = (fstate2.pos(B_R_THIGH) - new_pos).norm();
    const double rot_error_2 = (fstate2.rot(B_R_THIGH).toRotationMatrix() - new_rot.toRotationMatrix()).norm();
    const double vel_error_2 = (fstate2.vel(B_R_THIGH) - new_vel).norm();
    const double avel_error_2 = (fstate2.avel(B_R_THIGH) - new_avel).norm();
    
    pass = (pos_error < tol && rot_error < tol && vel_error < tol && avel_error < tol);
    
    if(!pass) {
        std::cout << "setBodyState (2) FAILED!" << std::endl;
        std::cout << "pos_error:  " << pos_error_2 << std::endl;
        std::cout << "rot_error:  " << rot_error_2 << std::endl;
        std::cout << "vel_error:  " << vel_error_2 << std::endl;
        std::cout << "avel_error: " << avel_error_2 << std::endl;
    }
    
    full_pass = full_pass && pass;
    
    pass = compareFullStates(fstate, fstate2, 1e-12, false);
    
    full_pass = full_pass && pass;
    
    return full_pass;
}

bool test_setBodyState2()
{
    FullState fstate = fullFromJoint(getTestJSpState());
    setBodyState(fstate, B_PELVIS,
        Eigen::Vector3d(1., 5.,2.),
        Eigen::Quaterniond(sqrt(4./12.), sqrt(3./12.), sqrt(2./12.), -sqrt(3./12.)),
        Eigen::Vector3d(-3., -.5, -.7),
        Eigen::Vector3d(-2., -.4, 5.));
    
    FullState fstate2 = foo_fullFromJoint(getTestJSpState());
    
    return compareFullStates(fstate, fstate2, 1e-10, false);
}

bool test_derivative()
{
    const double dt = 1e-6;
    
    JSpState jstate = getTestJSpState();
    
    const Eigen::Vector3d pos0(1.2, 1.5, -3.);
    const Eigen::Quaterniond rot0(sqrt(1./2.), sqrt(1./12.), sqrt(2./12.), sqrt(3./12.));
    const Eigen::Vector3d vel0(-4., 1.7, -3.2);
    const Eigen::Vector3d avel0(1.8, -0.6, 6.5);
    
    const Eigen::Quaterniond inf_rot(1., dt/4.*avel0.x(), dt/4.*avel0.y(), dt/4.*avel0.z());
    
    const Eigen::Vector3d pos_plus = pos0 + vel0*dt/2.;
    const Eigen::Vector3d pos_minus = pos0 - vel0*dt/2.;
    
    const Eigen::Quaterniond rot_plus = inf_rot * rot0;
    const Eigen::Quaterniond rot_minus = inf_rot.conjugate() * rot0;
    
    FullState fstate = fullFromJoint(jstate);
    setBodyState(fstate, B_R_FOOT, pos0, rot0, vel0, avel0);
    
    JSpState jstate_plus;
    JSpState jstate_minus;
    for(unsigned int i=0; i<DOF_MAX; i++) {
        jstate_plus.phi(i) = jstate.phi(i) + jstate.omega(i)*dt/2.;
        jstate_plus.omega(i) = std::numeric_limits<double>::quiet_NaN();
        
        jstate_minus.phi(i) = jstate.phi(i) - jstate.omega(i)*dt/2.;
        jstate_minus.omega(i) = std::numeric_limits<double>::quiet_NaN();
    }
    FullState fstate_plus = fullFromJoint(jstate_plus);
    setBodyState(fstate_plus, B_R_FOOT, pos_plus, rot_plus,
                                        Eigen::Vector3d::Zero(),
                                        Eigen::Vector3d::Zero());
    FullState fstate_minus = fullFromJoint(jstate_minus);
    setBodyState(fstate_minus, B_R_FOOT, pos_minus, rot_minus,
                                         Eigen::Vector3d::Zero(),
                                         Eigen::Vector3d::Zero());
    
    double max_vel_error = 0.;
    double max_avel_error = 0.;
    for(unsigned int b=0; b<B_MAX; b++) {
        const Eigen::Vector3d vel_ana = fstate.vel(b);
        const Eigen::Vector3d vel_num =
            (fstate_plus.pos(b) - fstate_minus.pos(b))/dt;
        
        const double vel_error = (vel_ana - vel_num).norm();
        
        const Eigen::Vector3d avel_ana = fstate.avel(b);
        Eigen::Matrix3d Omega_ana;
        Omega_ana << 0.,            -avel_ana.z(), avel_ana.y(),
                     avel_ana.z(),  0.,            -avel_ana.x(),
                     -avel_ana.y(), avel_ana.x(),  0.;
        Eigen::Matrix3d Omega_num = 
            (fstate_plus.rot(b).toRotationMatrix() - fstate_minus.rot(b).toRotationMatrix())/dt
            * fstate.rot(b).toRotationMatrix().transpose();
            
        const double avel_error = (Omega_ana - Omega_num).norm();
        
        if(vel_error > max_vel_error) max_vel_error = vel_error;
        if(avel_error > max_avel_error) max_avel_error = avel_error;
    }
    
    /* std::cout << "max_vel_error:  " << max_vel_error << std::endl;
    std::cout << "max_avel_error: " << max_avel_error << std::endl; */
    
    return (max_vel_error < 1e-8 && max_avel_error < 1e-8);
}

bool test_joint_constraint()
{
    const double tol = 1e-12;
    
    FullState fstate = foo_fullFromJoint(getTestJSpState());
    
    Eigen::Vector3d ey_T = fstate.rot(B_L_THIGH)._transformVector(Eigen::Vector3d::UnitY());
    Eigen::Vector3d ey_S = fstate.rot(B_L_SHANK)._transformVector(Eigen::Vector3d::UnitY());
    Eigen::Vector3d ex_F = fstate.rot(B_L_FOOT)._transformVector(Eigen::Vector3d::UnitX());
    
    /* std::cout << ey_T.transpose() << std::endl;
    std::cout << ey_S.transpose() << std::endl;
    std::cout << ex_F.transpose() << std::endl;
    
    std::cout << (ey_T - ey_S).norm() << std::endl;
    std::cout << ey_S.dot(ex_F) << std::endl; */
    
    return ((ey_T - ey_S).norm() < tol && std::abs(ey_S.dot(ex_F)) < tol);
}

bool test_leg_constraint(unsigned int side, FullState& fstate)
{
    using namespace CharacterConst;
    double pos_error, vel_error;
    double max_pos_error = 0., max_vel_error = 0.;
    const double tol = 1e-12;
    
    Eigen::Vector3d rHipPos1;
    if(side == LEFT) {
        rHipPos1 = fstate.pos(B_PELVIS) + 
            fstate.rot(B_PELVIS)._transformVector(Eigen::Vector3d(0., legPosY_L, -pelvisSizeZ/2.));
    } else {
        rHipPos1 = fstate.pos(B_PELVIS) + 
            fstate.rot(B_PELVIS)._transformVector(Eigen::Vector3d(0., legPosY_R, -pelvisSizeZ/2.));
    }
    Eigen::Vector3d rHipPos2 = fstate.pos(side, B_THIGH) + 
        fstate.rot(side, B_THIGH)._transformVector(Eigen::Vector3d(0., 0., thighSizeZ/2.));
    
    pos_error = (rHipPos1 - rHipPos2).norm();
    if(pos_error > max_pos_error) max_pos_error = pos_error;
    
    Eigen::Vector3d rHipVel1 = fstate.vel(B_PELVIS) + 
        fstate.avel(B_PELVIS).cross(rHipPos1 - fstate.pos(B_PELVIS));
    Eigen::Vector3d rHipVel2 = fstate.vel(side, B_THIGH) + 
        fstate.avel(side, B_THIGH).cross(rHipPos2 - fstate.pos(side, B_THIGH));
    
    vel_error = (rHipVel1 - rHipVel2).norm();
    if(vel_error > max_vel_error) max_vel_error = vel_error;
    
    Eigen::Vector3d rKneePos1 = fstate.pos(side, B_THIGH) + 
        fstate.rot(side, B_THIGH)._transformVector(Eigen::Vector3d(0., 0., -thighSizeZ/2.));
    Eigen::Vector3d rKneePos2 = fstate.pos(side, B_SHANK) + 
        fstate.rot(side, B_SHANK)._transformVector(Eigen::Vector3d(0., 0., shankSizeZ/2.));
    
    pos_error = (rKneePos1 - rKneePos2).norm();
    if(pos_error > max_pos_error) max_pos_error = pos_error;
    
    Eigen::Vector3d rKneeVel1 = fstate.vel(side, B_THIGH) + 
        fstate.avel(side, B_THIGH).cross(rKneePos1 - fstate.pos(side, B_THIGH));
    Eigen::Vector3d rKneeVel2 = fstate.vel(side, B_SHANK) + 
        fstate.avel(side, B_SHANK).cross(rKneePos2 - fstate.pos(side, B_SHANK));
    
    vel_error = (rKneeVel1 - rKneeVel2).norm();
    if(vel_error > max_vel_error) max_vel_error = vel_error;
    
    Eigen::Vector3d rAnklePos1 = fstate.pos(side, B_SHANK) + 
        fstate.rot(side, B_SHANK)._transformVector(Eigen::Vector3d(0., 0., -shankSizeZ/2.));
    Eigen::Vector3d rAnklePos2 = fstate.pos(side, B_FOOT) + 
        fstate.rot(side, B_FOOT)._transformVector(Eigen::Vector3d(-0.016, 0., footSizeZ/2.));
    
    pos_error = (rAnklePos1 - rAnklePos2).norm();
    if(pos_error > max_pos_error) max_pos_error = pos_error;
    
    Eigen::Vector3d rAnkleVel1 = fstate.vel(side, B_SHANK) + 
        fstate.avel(side, B_SHANK).cross(rAnklePos1 - fstate.pos(side, B_SHANK));
    Eigen::Vector3d rAnkleVel2 = fstate.vel(side, B_FOOT) + 
        fstate.avel(side, B_FOOT).cross(rAnklePos2 - fstate.pos(side, B_FOOT));
    
    vel_error = (rAnkleVel1 - rAnkleVel2).norm();
    if(vel_error > max_vel_error) max_vel_error = vel_error;
    
    /* std::cout << max_pos_error << std::endl;
    std::cout << max_vel_error << std::endl; */
    
    return (max_pos_error < tol && max_vel_error < tol);
}

bool test_constraint()
{
    bool pass = true;
    FullState fstate = foo_fullFromJoint(getTestJSpState());
    
    pass = pass && test_leg_constraint(LEFT, fstate);
    pass = pass && test_leg_constraint(RIGHT, fstate);
    
    return pass;
}

void test(const char* name, bool result, bool& full_result)
{
    if(result)
        std::cout << name << ": passed" << std::endl;
    else
        std::cout << name << ": FAIL" << std::endl;
    
    full_result = full_result && result;
}

int main()
{
    bool full_pass = true;
    
    test("test_rotDecomp", test_rotDecomp(), full_pass);
    test("test_inverse", test_inverse(), full_pass);
    test("test_setBodyState", test_setBodyState(), full_pass);
    test("test_setBodyState2", test_setBodyState2(), full_pass);
    test("test_derivative", test_derivative(), full_pass);
    test("test_constraint", test_constraint(), full_pass);
    test("test_joint_constraint", test_joint_constraint(), full_pass);
    
    std::cout << "\n";
    
    if(full_pass)
        std::cout << "All tests passed." << std::endl;
    else
        std::cout << "WARNING: some tests FAILED!!!" << std::endl;
    
    return 0;
}
