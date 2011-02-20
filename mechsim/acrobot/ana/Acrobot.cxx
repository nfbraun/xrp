#include "Acrobot.h"

const double FreeAcrobot::G = 10.;
const double FreeAcrobot::M1 = 1.;
const double FreeAcrobot::M2 = 1.;
const double FreeAcrobot::LC = .5;
const double FreeAcrobot::L1 = .5;
const double FreeAcrobot::L2 = .5;
const double FreeAcrobot::I1 = 0.;
const double FreeAcrobot::I2 = 0.;
const double FreeAcrobot::GAMMA = 0.1;

GiNaC::matrix FreeAcrobot::M(const Vector& q) const
{
    GiNaC::matrix mat(ndim(), ndim());
    
    mat = M1*LC*LC + M2*L1*L1 + M2*L2*L2 + 2.*M2*L1*L2*cos(q[1]) + I1,
          M2*(L2*L2 + L1*L2*cos(q[1])),
          M2*(L2*L2 + L1*L2*cos(q[1])),
          M2*L2*L2 + I2;
    //mat = (M1 + 2*M2*(1 + cos(q[1]))), M2*(1+cos(q[1])),
    //      M2*(1+cos(q[1])),                          M2;
    
    return mat;
}

GiNaC::ex FreeAcrobot::V(const Vector& q) const
{
    return (M1*LC+M2*L1)*G*cos(q[0]) + M2*L2*G*cos(q[0]+q[1]);
}

GiNaC::ex FreeAcrobot::u(const Vector& qdot, const Vector& q) const
{
    GiNaC::matrix mat(ndim(), 1);
    mat = 0., 0.;
    // mat = -GAMMA * qdot[0], -GAMMA * qdot[1];
    return mat;
}

GiNaC::ex Acrobot::u(const Vector& qdot, const Vector& q) const
{
    GiNaC::matrix mat(ndim(), 1);
    mat = 0., u1;
    return FreeAcrobot::u(qdot, q) + mat;
}
