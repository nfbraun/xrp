#include "LagrODESolver.h"
#include <Eigen/Dense>
#include <gsl/gsl_errno.h>

int LagrODESolver::calc_xdot(double t, const double x_raw[], double xdot_raw[], void *_p)
{
    struct LagrODEData* data = (struct LagrODEData*) _p;
    Eigen::Map<const Eigen::VectorXd> q(x_raw, data->ndim);
    Eigen::Map<const Eigen::VectorXd> qdot(x_raw+data->ndim, data->ndim);
    
    Eigen::MatrixXd M(data->ndim, data->ndim);
    data->calc_M(M.data(), q.data());   // calculate M(q)
    
    Eigen::VectorXd M_qdotdot(data->ndim);
    data->calc_M_qdotdot(M_qdotdot.data(), q.data(), qdot.data());   // calculate M(q) * qdotdot(q, qdot)
    
    Eigen::Map<Eigen::VectorXd> dq_dt(xdot_raw, data->ndim);
    dq_dt = qdot;
    // calculate qdotdot = M^{-1} * (M * qdotdot)
    Eigen::Map<Eigen::VectorXd> dqdot_dt(xdot_raw + data->ndim, data->ndim);
    dqdot_dt = M.ldlt().solve(M_qdotdot);
    
    return GSL_SUCCESS;
}

LagrODESolver::LagrODESolver(int ndim, const GiNaC::ex& M_qdotdot, const GiNaC::ex& M,
              const std::vector<GiNaC::symbol>& q_vec,
              const std::vector<GiNaC::symbol>& qdot_vec,
              const double q_ini[], const double qdot_ini[],
              const gsl_odeiv_step_type* stype)
        : RawODESolver(2*ndim, calc_xdot, 0, combineIniVect(ndim, q_ini, qdot_ini),
                 (void*) &fData, stype)
{
    fData.ndim = ndim;
    fData.calc_M = M_compiler::compile(M, q_vec);
    fData.calc_M_qdotdot = M_qdotdot_compiler::compile(M_qdotdot, q_vec, qdot_vec);
}

ODEArray<double> LagrODESolver::combineIniVect(int ndim, const double ini1[],
        const double ini2[])
{
    ODEArray<double> a(2*ndim);
    for(int i=0; i<ndim; i++)
        a[i] = ini1[i];
    for(int i=0; i<ndim; i++)
        a[i+ndim] = ini2[i];
    return a;
}
