// Test the non-autonomous case
// \ddot{q} + \omega_0^2 q = A \cos(\omega t)
// Expected solution for \dot{q}(0) = q(0) = 0
// q(t) = \frac{A}{\omega_0^2 - \omega^2} (\cos(\omega t) - \cos(\omega_0 t)
//
// Here: \omega = 1, \omega_0^2 = 2, A = 1 => q(t) = cos(t) - cos(sqrt(2)*t)

#include <iostream>
#include <vector>
#include <cmath>
#include "ODESolver.h"

int main()
{
    const double qdot_ini[] = { 0.0 };
    const double q_ini[] = { 0.0 };
    const double tstep = 1./16.;
    int i;
    
    using namespace GiNaC;
    symbol t("t"), qdot0("qdot0"), q0("q0");
    ex qdotdot = cos(t) - 2*q0;
    
    std::vector<symbol> qdot, q;
    qdot.push_back(qdot0);
    q.push_back(q0);
    
    ODE2Solver solver(1, qdotdot, t, qdot, q, qdot_ini, q_ini);
    
    std::cout.precision(4);
    for(i=0; i<(16*100); ++i) {
        solver.EvolveFwd(i * tstep);
        std::cout << solver.t() << " " << solver.qdot()[0];
        std::cout << " " << solver.q()[0] << std::endl;
    }
    
    return 0;
}
