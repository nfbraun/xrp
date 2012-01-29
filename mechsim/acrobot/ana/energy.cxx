#include <iostream>
#include <cmath>
#include <ginac/ginac.h>
#include "ODESolver.h"
#include <GiJIT.h>

int main()
{
    using namespace GiNaC;
    std::vector<symbol> q, qdot;
    q.push_back(symbol("q0")); q.push_back(symbol("q1"));
    qdot.push_back(symbol("qdot0")); qdot.push_back(symbol("qdot1"));
    
    ex M00  = 1. + 2.*(1.+cos(q[1]));
    ex M01  = 1. + cos(q[1]);
    ex M11  = 1.;
    ex h0   = -sin(q[1])*pow(qdot[1],2) - 2.*sin(q[1])*qdot[0]*qdot[1];
    ex h1   =  sin(q[1])*pow(qdot[0],2);
    ex phi0 = 2.*cos(q[0]) + cos(q[0]+q[1]);
    ex phi1 = cos(q[0]+q[1]);
    
    ex k0 = 1.;
    ex k1 = .4;
    
    ex E = 1./2.*M00*pow(qdot[0],2) + M01*qdot[0]*qdot[1] + 1./2.*M11*pow(qdot[1],2)
            + 2.*sin(q[0]) + sin(q[0]+q[1]);
    ex Ec = 3;
    
    ex ubar = -k0 * atan((E-Ec)*qdot[0]);
    //ex ubar = -k0*(E-Ec)*qdot[0];
    
    ex qddot0 = -(M01*(ubar-k1*qdot[1]-k0*q[1])+h0+phi0)/M00;
    ex qddot1 = ubar - k1*qdot[1] - k0*q[1];
    
    // The free system
    //ex qddot0 = (-h0-phi0+M01/M11*(h1+phi1))/(M00-M01*M01/M11);
    //ex qddot1 = (-h1-phi1+M01/M00*(h0+phi0))/(M11-M01*M01/M00);
    
    const double q_ini[] = { -M_PI/2., 0. };
    const double qdot_ini[] = { 0.2, 0.2 };
    const double tstep = 1./16.;
    int i;
    
    typedef GiJIT::CodeGen<GiJIT::Vector, GiJIT::Vector> E_func_t;
    E_func_t::func_t E_func = E_func_t::compile(E, q, qdot);
    typedef GiJIT::CodeGen<GiJIT::Vector, GiJIT::Vector> ub_func_t;
    ub_func_t::func_t ub_func = ub_func_t::compile(ubar, q, qdot);
    
    AutODE2Solver solver(2, lst(qddot0, qddot1), q, qdot, q_ini, qdot_ini);
    
    std::cout.precision(10);
    for(i=0; i<(16*200); ++i) {
        solver.EvolveFwd(i * tstep);
        std::cout << solver.t() << " " << solver.q()[0] << " " << solver.q()[1];
        std::cout << " " << solver.qdot()[0] << " " << solver.qdot()[1];
        std::cout << " " << E_func(solver.q(), solver.qdot());
        std::cout << " " << ub_func(solver.q(), solver.qdot());
        std::cout << std::endl;
    }
    
    return 0;
}
