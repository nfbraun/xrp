#include "DynTransform.h"
#include "StaticRobotInfo.h"

/* See DynTransform.h for function documentation. */

Eigen::Vector3d transformHipTorque(double hz, double hy, double hx, double thz, double thy, double thx)
{
    const double cz = cos(hz);
    const double sz = sin(hz);
    const double cy = cos(hy);
    const double sy = sin(hy);
    
    Eigen::Matrix3d QinvT;
    
    QinvT << cz/cy, -sz, sy*cz/cy,
             sz/cy,  cz, sy*sz/cy,
             0.,     0.,       1.;
    
    return QinvT * Eigen::Vector3d(thx, thy, thz);
}

void invTransformHipTorque(double hz, double hy, double hx, const Eigen::Vector3d& T, double& thz, double& thy, double& thx)
{
    const double cz = cos(hz);
    const double sz = sin(hz);
    const double cy = cos(hy);
    const double sy = sin(hy);
    
    thx = cy*cz*T.x() + cy*sz*T.y() - sy*T.z();
    thy = -sz*T.x() + cz*T.y();
    thz = T.z();
}

void decompYRot(const Eigen::Quaterniond q, double& y)
{
    y = 2. * atan(q.y() / q.w());
}

void decompYXRot(const Eigen::Quaterniond q, double& y, double& x)
{
    y = 2. * atan(q.y() / q.w());
    x = 2. * atan(q.x() / q.w());
}

void decompZYXRot(const Eigen::Quaterniond q, double& z, double& y, double &x)
{
    const double R00 = q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z();
    const double R10 = 2.*q.x()*q.y() + 2.*q.w()*q.z();
    const double R20 = 2.*q.x()*q.z() - 2.*q.w()*q.y();
    const double R21 = 2.*q.y()*q.z() + 2.*q.w()*q.x();
    const double R22 = q.w()*q.w() - q.x()*q.x() - q.y()*q.y() + q.z()*q.z();
    
    z = atan2(R10, R00);
    y = asin(-R20);
    x = atan2(R21, R22);
}

SE3Tr hipTransform(double hz, double hy, double hx)
{
    return SE3Tr::RotZ(hz) * SE3Tr::RotY(hy) * SE3Tr::RotX(hx);
}

SE3Tr thighTransform()
{
    return SE3Tr::Trans(0., 0., -CharacterConst::thighSizeZ);
}

SE3Tr kneeTransform(double ky)
{
    return SE3Tr::RotY(ky);
}

SE3Tr shankTransform()
{
    return SE3Tr::Trans(0., 0., -CharacterConst::shankSizeZ);
}

SE3Tr ankleTransform(double ay, double ax)
{
    return SE3Tr::RotY(ay) * SE3Tr::RotX(ax);
}

SE3Tr legTransform(double hz, double hy, double hx, double ky, double ay, double ax)
{
    return   hipTransform(hz, hy, hx)
           * thighTransform()
           * kneeTransform(ky)
           * shankTransform()
           * ankleTransform(ay, ax);
}

Eigen::Matrix3d hipAVelTransform(double hz, double hy, double hx)
{
    /* The code below is a faster (?) version of:
    
    const Eigen::Vector3d Qx =
        (SE3Tr::RotZ(hz) * SE3Tr::RotY(hy)).onVector(
            Eigen::Vector3d::UnitX());
    const Eigen::Vector3d Qy =
        SE3Tr::RotZ(hz).onVector(Eigen::Vector3d::UnitY());
    const Eigen::Vector3d Qz = Eigen::Vector3d::UnitZ();
    Eigen::Matrix3d Q;
    Q << Qx, Qy, Qz;
    return Q; */
    
    const double cz = cos(hz);
    const double sz = sin(hz);
    const double cy = cos(hy);
    const double sy = sin(hy);
    
    Eigen::Matrix3d Q;
    Q <<  cz*cy, -sz, 0.,
          sz*cy,  cz, 0.,
          -sy,    0., 1.;
    
    return Q;
}

Eigen::Matrix3d invHipAVelTransform(double hz, double hy, double hx)
{
    /* The code below is a faster version of:
    
    return hipAVelTransform(hz, hy, hx).inverse(); */
    
    const double cz = cos(hz);
    const double sz = sin(hz);
    const double cy = cos(hy);
    const double sy = sin(hy);
    
    Eigen::Matrix3d Qinv;
    
    Qinv << cz/cy,    sz/cy,    0.,
            -sz,      cz,       0.,
            sy*cz/cy, sy*sz/cy, 1.;
    
    return Qinv;
}

/* Helper function for fullFromJoint(), handling a single leg. */
void leg_FullFromJoint(unsigned int side, FullState& fstate, const JSpState& jstate)
{
    using namespace CharacterConst;
    
    Eigen::Vector3d relHipPos;
    if(side == LEFT)
        relHipPos = Eigen::Vector3d(0., legPosY_L, -pelvisSizeZ/2.);
    else
        relHipPos = Eigen::Vector3d(0., legPosY_R, -pelvisSizeZ/2.);
    
    const Eigen::Vector3d relThighPos(0., 0., -thighSizeZ/2.);
    const Eigen::Vector3d relShankPos(0., 0., -shankSizeZ/2.);
    const Eigen::Vector3d relFootPos(0.016, 0., -footSizeZ/2.);
    
    // Velocity field accumulators (v(r) = v0 + w.cross(r))
    Eigen::Vector3d v0(Eigen::Vector3d::Zero());
    Eigen::Vector3d w(Eigen::Vector3d::Zero());
    
    /*** Thigh ***/
    SE3Tr T = SE3Tr::Trans(relHipPos);
    const Eigen::Vector3d hipPos = T.trans();
    T = T * hipTransform(jstate.phi(side, HZ), jstate.phi(side, HY), jstate.phi(side, HX));
    fstate.pos(side, B_THIGH) = T.onPoint(relThighPos);
    fstate.rot(side, B_THIGH) = T.rot();
    
    const Eigen::Vector3d omega_H(jstate.omega(side, HX), jstate.omega(side, HY), jstate.omega(side, HZ));
    // Angular velocity difference across hip, expressed in pelvis (=global) frame
    const Eigen::Vector3d avel_H = hipAVelTransform(jstate.phi(side, HZ),
                                                    jstate.phi(side, HY),
                                                    jstate.phi(side, HX)) * omega_H;
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

FullState fullFromJoint(const JSpState& jstate)
{
    using namespace CharacterConst;
    
    FullState fstate;
    fstate.pos(B_PELVIS) = Eigen::Vector3d::Zero();
    fstate.rot(B_PELVIS) = Eigen::Quaterniond(1., 0., 0., 0.);
    fstate.avel(B_PELVIS) = Eigen::Vector3d::Zero();
    fstate.vel(B_PELVIS) = Eigen::Vector3d::Zero();
    
    leg_FullFromJoint(LEFT, fstate, jstate);
    leg_FullFromJoint(RIGHT, fstate, jstate);
    
    return fstate;
}

void setBodyState(FullState& fstate, unsigned int id,
                  const Eigen::Vector3d& pos,
                  const Eigen::Quaterniond& rot,
                  const Eigen::Vector3d& vel,
                  const Eigen::Vector3d& avel)
{
    Eigen::Quaterniond d_rot = rot * fstate.rot(id).conjugate();
    Eigen::Matrix3d d_Rot = d_rot.toRotationMatrix();
    
    Eigen::Vector3d d_pos = pos - d_Rot * fstate.pos(id);
    Eigen::Vector3d d_vel = vel - d_Rot * fstate.vel(id);
    Eigen::Vector3d d_avel = avel - d_Rot * fstate.avel(id);
    
    Eigen::Vector3d d_v0 = d_vel - d_avel.cross(d_Rot * fstate.pos(id) + d_pos);
    
    for(unsigned int i=0; i<B_MAX; i++) {
        fstate.pos(i) = d_Rot * fstate.pos(i) + d_pos;
        fstate.rot(i) = d_rot * fstate.rot(i);
        fstate.vel(i) = d_Rot * fstate.vel(i) + d_v0 + d_avel.cross(fstate.pos(i));
        fstate.avel(i) = d_Rot * fstate.avel(i) + d_avel;
    }
}

void setBodyState(FullState& fstate, unsigned int id, const BodyQ& q)
{
    setBodyState(fstate, id, q.pos(), q.rot(), q.vel(), q.avel());
}

/* Helper function for jointFromFull, handling a single leg. */
void leg_JointFromFull(unsigned int side, JSpState& jstate, const FullState& fstate)
{
    decompZYXRot(fstate.rot(B_PELVIS).conjugate() * fstate.rot(side, B_THIGH),
                 jstate.phi(side, HZ), jstate.phi(side, HY), jstate.phi(side, HX));
    decompYRot(fstate.rot(side, B_THIGH).conjugate() * fstate.rot(side, B_SHANK),
               jstate.phi(side, KY));
    decompYXRot(fstate.rot(side, B_SHANK).conjugate() * fstate.rot(side, B_FOOT),
                jstate.phi(side, AY), jstate.phi(side, AX));
    
    // Angular velocity difference across hip, expressed in pelvis frame
    const Eigen::Vector3d avel_H_P = fstate.rot(B_PELVIS).conjugate()._transformVector(
        fstate.avel(side, B_THIGH) - fstate.avel(B_PELVIS));
    Eigen::Vector3d omega_H = invHipAVelTransform(jstate.phi(side, HZ),
                                                  jstate.phi(side, HY),
                                                  jstate.phi(side, HX)) * avel_H_P;
    jstate.omega(side, HX) = omega_H.x();
    jstate.omega(side, HY) = omega_H.y();
    jstate.omega(side, HZ) = omega_H.z();
    
    // Angular velocity difference across knee, expressed in thigh frame
    const Eigen::Vector3d avel_K_T = fstate.rot(side, B_THIGH).conjugate()._transformVector(
        fstate.avel(side, B_SHANK) - fstate.avel(side, B_THIGH));
    jstate.omega(side, KY) = avel_K_T.y();
    
    // Angular velocity difference across ankle, expressed in shank frame
    const Eigen::Vector3d avel_A_S = fstate.rot(side, B_SHANK).conjugate()._transformVector(
        fstate.avel(side, B_FOOT) - fstate.avel(side, B_SHANK));
    jstate.omega(side, AY) = avel_A_S.y();
    jstate.omega(side, AX) = avel_A_S.x()*cos(jstate.phi(side, AY)) - avel_A_S.z()*sin(jstate.phi(side, AY));
}

JSpState jointFromFull(const FullState& fstate)
{
    JSpState jstate;
    leg_JointFromFull(LEFT, jstate, fstate);
    leg_JointFromFull(RIGHT, jstate, fstate);
    
    return jstate;
}
