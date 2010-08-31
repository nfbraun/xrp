#include "Acrobot.h"

const double Acrobot::G = 9.81;
const double Acrobot::M1 = 1.;
const double Acrobot::M2 = 1.;
const double Acrobot::GAMMA = 0.1;

GiNaC::matrix Acrobot::M(const Vector& q) const
{
    GiNaC::matrix mat(ndim(), ndim());
    
    mat = (M1 + 2*M2*(1 + cos(q[1]))), M2*(1+cos(q[1])),
          M2*(1+cos(q[1])),                          M2;
    
    return mat;
}

GiNaC::ex Acrobot::V(const Vector& q) const
{
    return (M1+M2)*G*cos(q[0]) + M2*G*cos(q[0]+q[1]);
}

GiNaC::ex Acrobot::u(const Vector& qdot, const Vector& q) const
{
    GiNaC::matrix mat(ndim(), 1);
    mat = -GAMMA * qdot[0], -GAMMA * qdot[1];
    return mat;
}

