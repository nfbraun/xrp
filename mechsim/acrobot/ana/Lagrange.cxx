#include "Lagrange.h"
#include <sstream>

Lagrange::Lagrange(const System& s)
    : fNdim(s.ndim())
{
    std::ostringstream name;
    for(int i=0; i<ndim(); i++) {
        name << "q" << i;
        fQ.push_back(GiNaC::symbol(name.str()));
        name.str("");
        name << "qdot" << i;
        fQdot.push_back(GiNaC::symbol(name.str()));
        name.str("");
    }
    
    fX.insert(fX.end(), fQ.begin(), fQ.end());
    fX.insert(fX.end(), fQdot.begin(), fQdot.end());
    
    fM = s.M(fQ);
    fV = s.V(fQ);
    fu = s.u(fQ, fQdot);
}

GiNaC::matrix Lagrange::C() const
{
    GiNaC::matrix C(ndim(), 1);
    
    for(int k=0; k<ndim(); k++) {
        GiNaC::ex sum = 0;
        for(int j=0; j<ndim(); j++) {
            for(int i=0; i<ndim(); i++) {
                const GiNaC::ex c = (fM(i,k).diff(fQ[j]) - (fM(i,j).diff(fQ[k]))/2);
                sum += c * fQdot[i] * fQdot[j];
            }
        }
        C(k,0) = sum;
    }
    
    return C;
}

GiNaC::matrix Lagrange::dVdQ() const
{
    GiNaC::matrix deriv(ndim(), 1);
    
    for(int k=0; k<ndim(); k++) {
        deriv(k,0) = fV.diff(fQ[k]);
    }
    
    return deriv;
}

GiNaC::ex Lagrange::qdotdot() const
{
    return (M().inverse() * (fu - C() - dVdQ())).evalm();
}

GiNaC::ex Lagrange::T() const
{
    GiNaC::ex Ekin = 0;
    for(int i=0; i<ndim(); i++) {
        for(int j=0; j<ndim(); j++) {
            Ekin += qdot()[i] * M()(i,j)/2. * qdot()[j];
        }
    }
    return Ekin;
}

GiNaC::ex Lagrange::E() const
{
    return T() + V();
}

GiNaC::matrix Lagrange::x_vec() const
{
    unsigned int n = fX.size();
    GiNaC::matrix x_vec(n, 1);
    for(unsigned int i=0; i<n; i++) {
        x_vec[i] = fX[i];
    }
    return x_vec;
}
