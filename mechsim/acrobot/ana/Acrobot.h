#ifndef _ACROBOT_H_
#define _ACROBOT_H_

#include "Lagrange.h"

struct AcrobotParam {
    GiNaC::ex G;
    GiNaC::ex M1;
    GiNaC::ex M2;
    GiNaC::ex LC;
    GiNaC::ex L1;
    GiNaC::ex L2;
    GiNaC::ex I1;
    GiNaC::ex I2;
    GiNaC::ex GAMMA;
};

class FreeAcrobot: public System
{
  public:
    FreeAcrobot(const struct AcrobotParam& par)
        : System(), p(par) {}
    
    virtual int ndim() const { return 2; }
    virtual GiNaC::matrix M(const Vector& q) const;
    virtual GiNaC::ex V(const Vector& q) const;
    virtual GiNaC::ex u(const Vector& q, const Vector& qdot) const;
    
    struct AcrobotParam p;
};

class Acrobot: public FreeAcrobot
{
  public:
    Acrobot(const struct AcrobotParam& par)
        : FreeAcrobot(par), u1("u1") {}
    
    virtual GiNaC::ex u(const Vector& q, const Vector& qdot) const;
    GiNaC::symbol u1;
};

#endif
