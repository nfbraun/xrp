#include <iostream>
#include <algorithm>
#include <vector>
#include <cmath>
#include "RawODESolver.h"
#include <cassert>
#include <Eigen/Dense>
#include "Robot.h"
#include "RNE_CRB.h"
#include <gsl/gsl_errno.h>

int n_xdot_eval = 0;

Eigen::VectorXd dynamics(const Eigen::VectorXd& q, const Eigen::VectorXd& qdot, const Eigen::VectorXd& u)
{
    // const Eigen::Vector3d g(0., 0., -9.81);
    const Eigen::Vector3d g(0., 0., 0.);
    
    Eigen::VectorXd qddot_0(q.size());
    qddot_0.setZero();
    
    Robot rob;
    Eigen::Matrix<SE3Tr, Robot::N_DOF, 1> expSq = rob.calc_expSq(q);
    
    Eigen::VectorXd C = calc_u(rob, expSq, qdot, qddot_0, g);
    
    return calc_M(rob, expSq).ldlt().solve(u - C);
}

int calc_xdot(double t, const double x_raw[], double xdot_raw[], void *_p)
{
    Eigen::Map<const Eigen::VectorXd> q(x_raw, Robot::N_DOF);
    Eigen::Map<const Eigen::VectorXd> qdot(x_raw+Robot::N_DOF, Robot::N_DOF);
    
    Eigen::VectorXd u(Robot::N_DOF);
    u.setZero();
    
    u(0) =  2.3;   // LAX
    u(1) = -1.5;  // LAY
    u(2) = -0.9;  // LKY
    u(3) = 2.5 * cos(9*t);  // LHX
    u(4) = 1.4 * cos(4*t);  // LHY
    u(5) = 1.0 * cos(6*t);   // LHZ
    
    u(6) = 0.02 * sin(4*t);   // RHZ
    u(7) = -0.5 * sin(5*t); // RHY
    u(8) = 0.1 * sin(7*t);  // RHX
    u(9) = -0.02;   // RKY
    u(10) = 0.01;  // RAY
    u(11) = 0.006; // RAX
    
    Eigen::Map<Eigen::VectorXd> dq_dt(xdot_raw, Robot::N_DOF);
    dq_dt = qdot;
    // calculate qdotdot = M^{-1} * (u - C - dVdq)
    Eigen::Map<Eigen::VectorXd> dqdot_dt(xdot_raw+Robot::N_DOF, Robot::N_DOF);
    dqdot_dt = dynamics(q, qdot, u);
    
    n_xdot_eval++;
    
    return GSL_SUCCESS;
}

double normalizeAngle(double x)
{
    if(x > 0)
        return fmod(x + M_PI, 2*M_PI) - M_PI;
    else
        return fmod(x - M_PI, 2*M_PI) + M_PI;
}

int main()
{
    std::vector<double> x_ini(2*Robot::N_DOF, 0.0);
    const double tstep = 1./160.;
    int i;
    
    RawODESolver solver(2*Robot::N_DOF, calc_xdot, 0, x_ini.data(), 0);
    
    const char* names[] = { "LAX", "LAY", "LKY", "LHX", "LHY", "LHZ", "RHZ", "RHY", "RHX", "RKY", "RAY", "RAX" };
    
    for(unsigned int i=0; i<Robot::N_DOF; i++)
        std::cout << "#:" << i+1 << ":p_" << names[i] << "_ana" << std::endl;
    
    for(unsigned int i=0; i<Robot::N_DOF; i++)
        std::cout << "#:" << i+1+Robot::N_DOF << ":o_" << names[i] << "_ana" << std::endl;
    
    for(i=0; i<4*160; ++i) {
        solver.EvolveFwd(i * tstep);
        std::cout << solver.t() << " ";
        
        for(unsigned int i=0; i<Robot::N_DOF; i++)
            std::cout << normalizeAngle(solver.y()[i]) << " ";
        for(unsigned int i=0; i<Robot::N_DOF; i++)
            std::cout << solver.y()[i+Robot::N_DOF] << " ";
        std::cout << std::endl;
    }
    
    std::cerr << "n_xdot_eval: " << n_xdot_eval << std::endl;
    
    return 0;
}
