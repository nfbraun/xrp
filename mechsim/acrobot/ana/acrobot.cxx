#include <iostream>
#include <cmath>
#include "Lagrange.h"
#include "Acrobot.h"
#include "ODESolver.h"

double norm_angle(double x)
{
    while(x < 0) x += 2.*M_PI;
    return fmod(x, 2.*M_PI);
}

int main()
{
    const double qdot_ini[] = { M_PI/2., 0.0 };
    const double q_ini[] = { 0.0, 0.0 };
    const double tstep = 1./16.;
    int i;
    
    Acrobot a;
    Lagrange l(a);
    
    AutODE2Solver solver(2, l.qdotdot(), l.qdot(), l.q(), qdot_ini, q_ini);
    
    std::cout.precision(4);
    for(i=0; i<(16*100); ++i) {
        solver.EvolveFwd(i * tstep);
        std::cout << solver.t() << " " << norm_angle(solver.q()[0]) << " " << norm_angle(solver.q()[1]);
        std::cout << " " << solver.qdot()[0] << " " << solver.qdot()[1] << std::endl;
    }
    
    return 0;
}
