#ifndef _ACROBOT_H_
#define _ACROBOT_H_

#include "Lagrange.h"

class Acrobot: public System
{
  public:
    virtual int ndim() const { return 2; }
    virtual GiNaC::matrix M(const Vector& q) const;
    virtual GiNaC::ex V(const Vector& q) const;
    virtual GiNaC::ex u(const Vector& qdot, const Vector& q) const;
    
    static const double G, M1, M2, GAMMA;
};

#endif
