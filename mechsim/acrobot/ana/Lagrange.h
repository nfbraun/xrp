#ifndef _LAGRANGE_H_
#define _LAGRANGE_H_

#include <vector>
#include <ginac/ginac.h>

class System
{
  public:
    typedef std::vector<GiNaC::symbol> Vector;
    
    virtual int ndim() const = 0;
    virtual GiNaC::matrix M(const Vector& q) const = 0;
    virtual GiNaC::ex V(const Vector& q) const = 0;
    virtual GiNaC::ex u(const Vector& q, const Vector& qdot) const
        { return 0; }
};

class Lagrange
{
  public:
    Lagrange(const System& s);
    
    GiNaC::ex qdotdot() const;
    GiNaC::matrix C() const;
    GiNaC::matrix dVdQ() const;
    
    GiNaC::ex T() const;
    GiNaC::ex E() const;
    
    const int ndim() const { return fNdim; }
    const System::Vector& q() const { return fQ; }
    const System::Vector& qdot() const { return fQdot; }
    const System::Vector& x() const { return fX; }
    const GiNaC::matrix& M() const { return fM; }
    const GiNaC::ex& V() const { return fV; }
    
    GiNaC::matrix x_vec() const;
    
  private:
    int fNdim;
    GiNaC::matrix fM;
    GiNaC::ex fV, fu;
    System::Vector fQ, fQdot, fX;
};

#endif
