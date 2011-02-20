#ifndef _ACROBOT_H_
#define _ACROBOT_H_

#include "Lagrange.h"

class FreeAcrobot: public System
{
  public:
    virtual int ndim() const { return 2; }
    virtual GiNaC::matrix M(const Vector& q) const;
    virtual GiNaC::ex V(const Vector& q) const;
    virtual GiNaC::ex u(const Vector& qdot, const Vector& q) const;
    
    static const double G, M1, M2, LC, L1, L2, I1, I2, GAMMA;
};

class Acrobot: public FreeAcrobot
{
  public:
    Acrobot() : u1("u1") {}
    virtual GiNaC::ex u(const Vector& qdot, const Vector& q) const;
    GiNaC::symbol u1;
};

#endif
