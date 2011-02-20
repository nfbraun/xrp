#include <iostream>
#include <algorithm>
#include <cmath>
#include <ginac/ginac.h>
#include "ODESolver.h"
#include "Acrobot.h"
#include "LQR.h"
#include <GiJIT.h>

int main()
{
    using namespace GiNaC;
    Acrobot a;
    Lagrange l(a);
    
    std::vector<symbol> q = l.q(), qdot = l.qdot();
    
    //ex M00  = l.M()(0,0);
    //ex M01  = l.M()(0,1);
    //ex M11  = l.M()(1,1);
    
    //ex h0 = l.C()(0,0);
    //ex h1 = l.C()(1,0);
    //ex phi0 = l.dVdQ()(0,0);
    //ex phi1 = l.dVdQ()(1,0);
    
    ex k0 = 1.;
    ex k1 = .4;
    
    ex E = l.E();
    ex Ec = E.subs(qdot[0]==0).subs(qdot[1]==0).subs(q[0]==0).subs(q[1]==0);
    
    ex ubar = -atan(3.*(E-Ec)/Ec*qdot[0]/sqrt(Acrobot::G/Acrobot::L1));
    
    // Partial feedback linearization
    ex qddot1 = ex_to<lst>(l.qdotdot())[1];
    ex tau1 = (- k1*qdot[1]/sqrt(Acrobot::G/Acrobot::L1) + k0*(ubar-q[1]))
        * Acrobot::G/Acrobot::L1;
    ex u1 = lsolve(qddot1 == tau1, a.u1);
    
    GiNaC::ex qddot = l.qdotdot().subs(a.u1 == u1);
    
    const double qdot_ini[] = { 0.0, 0.0 };
    const double q_ini[] = { M_PI-1., 0.0 };
    const double tstep = 1./16.;
    int i;
    
    typedef GiJIT::CodeGen<GiJIT::Vector, GiJIT::Vector> E_func_t;
    E_func_t::func_t E_func = E_func_t::compile(E, l.qdot(), l.q());
    //typedef GiJIT::CodeGen<GiJIT::Vector, GiJIT::Vector> ub_func_t;
    //ub_func_t::func_t ub_func = ub_func_t::compile(ubar, l.qdot(), l.q());
    //typedef GiJIT::CodeGen<GiJIT::Vector, GiJIT::Vector> qddot1_func_t;
    //qddot1_func_t::func_t qddot1_func = ub_func_t::compile(ex_to<lst>(qddot)[1],
    //    l.qdot(), l.q());
    // qddot = sub_matrix(ex_to<matrix>(qddot),0,2,0,1);
    AutODE2Solver solver(2, qddot, l.qdot(), l.q(), qdot_ini, q_ini);
    
    std::cout << "#:1:q0" << std::endl;
    std::cout << "#:2:q1" << std::endl;
    std::cout << "#:3:qdot0" << std::endl;
    std::cout << "#:4:qdot1" << std::endl;
    std::cout << "#:5:E/Ec" << std::endl;
    // std::cout << "#:6:ubar" << std::endl;
    // std::cout << "#:7:qddot1" << std::endl;
    std::cout.precision(10);
    double Ec_val = ex_to<numeric>(Ec).to_double();
    for(i=0; i<(16*200); ++i) {
        double E_val = E_func(solver.qdot(), solver.q());
        if(std::abs((E_val - Ec_val)/Ec_val) < .1 &&
           std::abs(solver.q()[0]) < .02 &&
           std::abs(solver.q()[1]) < .02)
            break;
        solver.EvolveFwd(i * tstep);
        std::cout << solver.t() << " ";
        std::cout << solver.q()[0] << " " << solver.q()[1] << " ";
        std::cout << solver.qdot()[0] << " ";
        std::cout << solver.qdot()[1] << " ";
        std::cout << E_val/Ec_val << " ";
        //std::cout << " " << ub_func(solver.qdot(), solver.q());
        //std::cout << " " << qddot1_func(solver.qdot(), solver.q());
        std::cout << std::endl;
    }
    
    // LQR control
    ex u1_lqr = -(LQR::lqrControl(l, a.u1) * l.x_vec()).evalm();
    
    GiNaC::ex qddot2 = l.qdotdot().subs(a.u1 == u1_lqr);
    // GiNaC::ex qddot = ((LQR::Sys_A() - LQR::Sys_B() * u) * l.x_vec()).evalm();
    //std::cout << qddot << std::endl;
    
    const double qdot_ini_2[] = { solver.qdot()[0], solver.qdot()[1] };
    const double q_ini_2[] = { solver.q()[0], solver.q()[1] };
    
    AutODE2Solver solver2(2, qddot2, l.qdot(), l.q(), qdot_ini_2, q_ini_2);
    
    const double t0 = solver.t();
    for(; i<(16*200); ++i) {
        double E_val = E_func(solver2.qdot(), solver2.q());
        solver2.EvolveFwd(i * tstep - t0);
        std::cout << solver2.t() + t0 << " " << solver2.q()[0] << " " << solver2.q()[1];
        std::cout << " " << solver2.qdot()[0] << " " << solver2.qdot()[1];
        std::cout << " " << E_val/Ec_val;
        std::cout << std::endl;
    }
    
    return 0;
}
