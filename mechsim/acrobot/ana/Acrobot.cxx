#include "Acrobot.h"

GiNaC::matrix FreeAcrobot::M(const Vector& q) const
{
    GiNaC::matrix mat(ndim(), ndim());
    
    mat = p.M1*p.LC*p.LC + p.M2*p.L1*p.L1
            + p.M2*p.L2*p.L2 + 2.*p.M2*p.L1*p.L2*cos(q[1]) + p.I1 + p.I2,
          p.M2*(p.L2*p.L2 + p.L1*p.L2*cos(q[1])) + p.I2,
          p.M2*(p.L2*p.L2 + p.L1*p.L2*cos(q[1])) + p.I2,
          p.M2*p.L2*p.L2 + p.I2;
    
    return mat;
}

GiNaC::ex FreeAcrobot::V(const Vector& q) const
{
    return (p.M1*p.LC+p.M2*p.L1)*p.G*cos(q[0]) + p.M2*p.L2*p.G*cos(q[0]+q[1]);
}

GiNaC::ex FreeAcrobot::u(const Vector& q, const Vector& qdot) const
{
    GiNaC::matrix mat(ndim(), 1);
    mat = -p.GAMMA * qdot[0], -p.GAMMA * qdot[1];
    return mat;
}

GiNaC::ex Acrobot::u(const Vector& q, const Vector& qdot) const
{
    GiNaC::matrix mat(ndim(), 1);
    mat = 0., u1;
    return FreeAcrobot::u(q, qdot) + mat;
}
