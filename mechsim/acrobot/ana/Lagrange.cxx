#include "Lagrange.h"
#include <sstream>

Lagrange::Lagrange(const System& s)
    : ndim(s.ndim())
{
    std::ostringstream name;
    for(int i=0; i<ndim; i++) {
        name << "q" << i;
        fQ.push_back(GiNaC::symbol(name.str()));
        name.str("");
        name << "qdot" << i;
        fQdot.push_back(GiNaC::symbol(name.str()));
        name.str("");
    }
    
    fX.insert(fX.end(), fQdot.begin(), fQdot.end());
    fX.insert(fX.end(), fQ.begin(), fQ.end());
    
    fM = s.M(fQ);
    fV = s.V(fQ);
    fu = s.u(fQdot, fQ);
}

GiNaC::matrix Lagrange::C() const
{
    GiNaC::matrix C(ndim, 1);
    
    for(int k=0; k<ndim; k++) {
        GiNaC::ex sum = 0;
        for(int j=0; j<ndim; j++) {
            for(int i=0; i<ndim; i++) {
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
    GiNaC::matrix deriv(ndim, 1);
    
    for(int k=0; k<ndim; k++) {
        deriv(k,0) = fV.diff(fQ[k]);
    }
    
    return deriv;
}

GiNaC::ex Lagrange::qdotdot() const
{
    return (M().inverse() * (fu - C() - dVdQ())).evalm();
}
