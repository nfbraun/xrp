#include <iostream>
#include <algorithm>
#include <cmath>
#include <ginac/ginac.h>
#include "ODESolver.h"
#include "Acrobot.h"
#include "LQR.h"
#include <GiJIT.h>

double norm_angle(double x)
{
    while(x < 0.) x += 2.*M_PI;
    return fmod(x, 2.*M_PI);
}

int main()
{
    using namespace GiNaC;
    
    struct AcrobotParam par;
    par.G = 10.;
    par.M1 = 1.;
    par.M2 = 1.;
    par.LC = .5;
    par.L1 = .5;
    par.L2 = .5;
    par.I1 = 0.;
    par.I2 = 0.;
    par.GAMMA = 0.;
    
    FreeAcrobot a(par);
    Lagrange l(a);
    
    std::vector<symbol> q = l.q(), qdot = l.qdot();
    
    GiNaC::ex qddot = l.qdotdot();
    
    const double q_ini[] = { M_PI/2., 0.0 };
    const double qdot_ini[] = { 0.0, 0.0 };
    const double tstep = 1./16.;
    int i;
    
    typedef GiJIT::CodeGen<GiJIT::Vector, GiJIT::Vector> E_func_t;
    E_func_t::func_t E_func = E_func_t::compile(l.E(), l.q(), l.qdot());
    AutODE2Solver solver(2, qddot, l.q(), l.qdot(), q_ini, qdot_ini);
    
    std::cout << "#:1:q0" << std::endl;
    std::cout << "#:2:q1" << std::endl;
    std::cout << "#:3:qdot0" << std::endl;
    std::cout << "#:4:qdot1" << std::endl;
    std::cout << "#:5:E" << std::endl;
    std::cout.precision(10);
    
    for(i=0; i<(16*10); ++i) {
        double E_val = E_func(solver.q(), solver.qdot());
        
        solver.EvolveFwd(i * tstep);
        std::cout << solver.t() << " ";
        std::cout << norm_angle(solver.q()[0]) << " ";
        std::cout << norm_angle(solver.q()[1]) << " ";
        std::cout << solver.qdot()[0] << " ";
        std::cout << solver.qdot()[1] << " ";
        std::cout << E_val << " ";
        std::cout << std::endl;
    }
    
    return 0;
}
