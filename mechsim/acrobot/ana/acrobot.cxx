#include <iostream>
#include <cmath>
#include "ODESolver.h"

extern ODESolver::deriv_func_t acrobot_deriv;

double norm_angle(double x)
{
    while(x < 0) x += 2.*M_PI;
    return fmod(x, 2.*M_PI);
}

int main()
{
    const double qdot_1 = 0.; //sqrt(4. * 9.81);
    const double qdot_ini[] = { M_PI/2., 0.0 };
    const double q_ini[] = { 0.0, qdot_1 };
    
    const double tstep = 1./16.;
    int i;
    
    ODESolver solver(2, &acrobot_deriv, qdot_ini, q_ini);
    
    for(i=0; i<(16*100); ++i) {
        solver.EvolveFwd(i * tstep);
        std::cout << solver.t() << " " << norm_angle(solver.q()[0]) << " " << norm_angle(solver.q()[1]);
        std::cout << " " << solver.qdot()[0] << " " << solver.qdot()[1] << std::endl;
    }
    
    return 0;
}
