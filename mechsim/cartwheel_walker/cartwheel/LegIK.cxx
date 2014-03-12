#include "LegIK.h"
#include <iostream>
#include <Eigen/Dense>
#include <DynTransform.h>

/* Return knee and ankle rotations such that the distance between the ankle and the hip, measured in the foot frame, is d. */
void LegIK(const Eigen::Vector3d& d, double& hz, double& hy, double& hx, double& ky, double &ay, double &ax)
{
    using namespace CharacterConst;
    
    hz = 0., hy = 0., hx = 0., ky = 0., ay = 0., ax = 0.;
    
    const double l1 = CharacterConst::thighSizeZ;
    const double l2 = CharacterConst::shankSizeZ;
    
    Eigen::Vector3d d_clamp;
    if(d.norm() > 0.99*(l1+l2)) {
        d_clamp = d / d.norm() * (l1 + l2) * 0.99;
    } else {
        d_clamp = d;
    }
    
    // Calculate knee angle
    const double c_ky = -(l1*l1 + l2*l2 - d_clamp.squaredNorm())/(2.*l1*l2);
    ky = acos(c_ky);
    
    ax = atan2(-d_clamp.y(), -d_clamp.z());
    ay = -atan(-d_clamp.x()/sqrt(d_clamp.y()*d_clamp.y() + d_clamp.z()*d_clamp.z()))
            - asin(sin(ky) * l1/d_clamp.norm());
    
    Eigen::Quaterniond q = (Eigen::AngleAxisd(ky, Eigen::Vector3d::UnitY())
                          * Eigen::AngleAxisd(ay, Eigen::Vector3d::UnitY())
                          * Eigen::AngleAxisd(ax, Eigen::Vector3d::UnitX()));
    
    decompZYXRot(q.conjugate(), hz, hy, hx);
}

IKSwingLegTarget getSwingLegTarget(const Eigen::Vector3d& d, const Eigen::Vector3d& vd)
{
    IKSwingLegTarget target;
    
    double hz, hy, hx, ky, ay, ax;
    LegIK(d, hz, hy, hx, ky, ay, ax);
    
    Eigen::Matrix<double, 6, 6> S = swingLegFK_S(hz, hy, hx, ky, ay, ax);
    
    Eigen::Matrix<double, 6, 1> avel_vel;
    avel_vel << 0., 0., 0., vd;
    
    Eigen::Matrix<double, 6, 1> omega;
    omega = S.householderQr().solve(avel_vel);
    
    // only needed for testing with old code
    /* Eigen::Quaterniond qHip = (Eigen::AngleAxisd(hz, Eigen::Vector3d::UnitY())
                          * Eigen::AngleAxisd(hy, Eigen::Vector3d::UnitY())
                          * Eigen::AngleAxisd(hx, Eigen::Vector3d::UnitX()));
    Eigen::Quaterniond qKnee(Eigen::AngleAxisd(ky, Eigen::Vector3d::UnitY()));
    
    target.swingHipOrient = Quaternion(qHip.w(), qHip.x(), qHip.y(), qHip.z());
    target.swingKneeOrient = Quaternion(qKnee.w(), qKnee.x(), qKnee.y(), qKnee.z());
    
    target.swingHipAngVel = Vector3d(0., 0., 0.);
    target.swingKneeAngVel = Vector3d(0., 0., 0.); */
    
    target.phz = hz;
    target.phy = hy;
    target.phx = hx;
    target.pky = ky;
    target.pay = ay;
    target.pax = ax;
    
    target.ohz = omega(0);
    target.ohy = omega(1);
    target.ohx = omega(2);
    target.oky = omega(3);
    target.oay = omega(4);
    target.oax = omega(5);
    
    return target;
}

Eigen::Vector3d swingLegFK(const IKSwingLegTarget& target)
{
    SE3Tr T = hipTransform(target.phz, target.phy, target.phx)
              * SE3Tr::Trans(0., 0., -CharacterConst::thighSizeZ)
              * kneeTransform(target.pky)
              * SE3Tr::Trans(0., 0., -CharacterConst::shankSizeZ);
    
    return T.trans();
}

Eigen::Matrix3d CrossMat(const Eigen::Vector3d& v)
{
    Eigen::Matrix3d C;
    C << 0., -v.z(), v.y(),
         v.z(), 0., -v.x(),
         -v.y(), v.x(), 0.;
    return C;
}

Eigen::Matrix<double, 6, 6> swingLegFK_S(double hz, double hy, double hx, double ky, double ay, double ax)
{
    using namespace CharacterConst;
    
    const Eigen::Vector3d relThighPos(0., 0., -thighSizeZ/2.);
    const Eigen::Vector3d relShankPos(0., 0., -shankSizeZ/2.);
    const Eigen::Vector3d relFootPos(0.016, 0., -footSizeZ/2.);
    
    /*** Thigh ***/
    SE3Tr T = SE3Tr::Identity();
    const Eigen::Vector3d hipPos = T.trans();
    T = T * hipTransform(hz, hy, hx);
    Eigen::Quaterniond thighRot = T.rot();
    
    // Angular velocity difference across hip, expressed in pelvis (=global) frame
    const double cz = cos(hz);
    const double sz = sin(hz);
    const double cy = cos(hy);
    const double sy = sin(hy);
    
    Eigen::Matrix3d M_avel_H;
    M_avel_H <<  0., -sz, cz*cy,
                 0.,  cz, sz*cy,
                 1.,  0., -sy;
    
    /*** Shank ***/
    // Angular velocity difference across knee, expressed in thigh frame
    Eigen::Matrix<double, 3, 1> M_avel_K_T;
    M_avel_K_T << 0., 1., 0.;
    // Angular velocity difference across knee, expressed in global frame
    const Eigen::Matrix<double, 3, 1> M_avel_K = T.rot().toRotationMatrix() * M_avel_K_T;
    
    T = T * SE3Tr::Trans(Eigen::Vector3d(0., 0., -thighSizeZ));
    const Eigen::Vector3d kneePos = T.trans();
    T = T * kneeTransform(ky);
    
    /*** Foot ***/
    // Angular velocity difference across ankle, expressed in shank frame
    Eigen::Matrix<double, 3, 2> M_avel_A_S;
    M_avel_A_S << 0., cos(ay),
                  1., 0.,
                  0., -sin(ay);
    
    const Eigen::Matrix<double, 3, 2> M_avel_A = T.rot().toRotationMatrix() * M_avel_A_S;
    
    T = T * SE3Tr::Trans(Eigen::Vector3d(0., 0., -shankSizeZ));
    const Eigen::Vector3d anklePos = T.trans();
    
    /* Assemble matrix */
    Eigen::Matrix<double, 6, 6> S;
    S << M_avel_H, M_avel_K, M_avel_A,
        (CrossMat(hipPos - anklePos) * M_avel_H),
        (CrossMat(kneePos - anklePos) * M_avel_K),
        Eigen::Matrix<double, 3, 2>::Zero();
    
    return S;
}

/*** Tests ***/

void legik_test()
{
    double phi[RDOF_MAX];
    double phi_plus[RDOF_MAX];
    double phi_minus[RDOF_MAX];
    Eigen::Vector3d d(.6, -.5, .5);
    Eigen::Vector3d vd(-.3, -.4, .9);
    const double dt = 1e-6;
    
    Eigen::Vector3d d_plus = d + vd*dt;
    Eigen::Vector3d d_minus = d - vd*dt;
    
    LegIK(d, phi[0], phi[1], phi[2], phi[3], phi[4], phi[5]);
    LegIK(d_plus, phi_plus[0], phi_plus[1], phi_plus[2], phi_plus[3], phi_plus[4], phi_plus[5]);
    LegIK(d_minus, phi_minus[0], phi_minus[1], phi_minus[2], phi_minus[3], phi_minus[4], phi_minus[5]);
    
    Eigen::Matrix<double, 6, 6> S = 
        swingLegFK_S(phi[0], phi[1], phi[2], phi[3], phi[4], phi[5]);
    
    Eigen::Matrix<double, 6, 1> avel_vel;
    avel_vel << 0., 0., 0., vd;
    
    Eigen::Matrix<double, 6, 1> omega;
    omega = S.householderQr().solve(avel_vel);
    
    Eigen::Matrix<double, 6, 1> omega_num;
    for(unsigned int i=0; i<6; i++) {
        omega_num(i) = (phi_plus[i] - phi_minus[i]) / (2. * dt);
    }
    
    std::cout << omega_num.transpose() << std::endl;
    std::cout << (omega - omega_num).transpose() << std::endl;
}

void legik_test_2()
{
    JSpState jstate;
    jstate.phi(LHZ) = 0.6;
    jstate.phi(LHY) = -0.4;
    jstate.phi(LHX) = -0.2;
    jstate.phi(LKY) = 0.3;
    jstate.phi(LAY) = -0.5;
    jstate.phi(LAX) = 0.2;
    
    jstate.omega(LHZ) = 0.9;
    jstate.omega(LHY) = 0.8;
    jstate.omega(LHX) = -0.5;
    jstate.omega(LKY) = -0.3;
    jstate.omega(LAY) = -0.1;
    jstate.omega(LAX) = -0.7;
    
    /* Should give:
       avel = -1.09811 -0.29042 0.385789
       vel = -0.229716  -0.489826 -0.0843905
     */
    
    unsigned int side = LEFT;
    Eigen::Matrix<double, 6, 1> omega;
    omega << jstate.omega(side, HZ), jstate.omega(side, HY), jstate.omega(side, HX),
             jstate.omega(side, KY), jstate.omega(side, AY), jstate.omega(side, AX);
    
    Eigen::Matrix<double, 6, 6> S = 
        swingLegFK_S(jstate.phi(side, HZ), jstate.phi(side, HY), jstate.phi(side, HX),
                     jstate.phi(side, KY), jstate.phi(side, AY), jstate.phi(side, AX));
    
    Eigen::Vector3d avel, vel;
    avel = (S * omega).block<3,1>(0,0);
    vel = (S * omega).block<3,1>(3,0);
    
    std::cout << "-1.09811 -0.29042 0.385789" << std::endl;
    std::cout << avel.transpose() << std::endl;
    std::cout << "-0.229716  -0.489826 -0.0843905" << std::endl;
    std::cout << vel.transpose() << std::endl;
}

void legik_test_3()
{
    double hz, hy, hx, ky, ay, ax;
    LegIK(Eigen::Vector3d(.6, -.5, .5), hz, hy, hx, ky, ay, ax);
    
    SE3Tr T = hipTransform(hz, hy, hx)
              * SE3Tr::Trans(0., 0., -0.6)
              * kneeTransform(ky)
              * SE3Tr::Trans(0., 0., -0.4)
              * ankleTransform(ay, ax);
    
    std::cout << ky << " " << ay << " " << ax << std::endl;
    
    std::cout << T.trans().transpose() << std::endl;
    
    Eigen::Quaterniond q = T.inverse().rot();
    std::cout << q.w() << " " << q.x() << " " << q.y() << " " << q.z() << std::endl;
}
