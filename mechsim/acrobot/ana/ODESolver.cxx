#include <gsl/gsl_errno.h>
#include <exception>
#include <cmath>
#include "ODESolver.h"

const double ODESolver::EPS_ABS = 1e-8;

ODESolver::ODESolver(int ndim, deriv_func_t* deriv_func,
                     const double qdot_ini[], const double q_ini[],
                     void* params)
    : fNdim(ndim), ft(0.)
{
    const gsl_odeiv_step_type *T = gsl_odeiv_step_rkf45;
    
    fODE_s = gsl_odeiv_step_alloc(T, 2*ndim);
    fODE_c = gsl_odeiv_control_y_new(EPS_ABS, 0.);
    fODE_e = gsl_odeiv_evolve_alloc(2*ndim);
    
    fODE_sys.function = deriv_func;
    fODE_sys.jacobian = NULL;
    fODE_sys.dimension = 2*ndim;
    fODE_sys.params = params;
    
    fODE_h = 1e-6;
    
    fy = new double[2*ndim];
    for(int i=0; i<ndim; i++) {
        fy[i] = qdot_ini[i];
        fy[i+ndim] = q_ini[i];
    }
}

ODESolver::~ODESolver()
{
    delete[] fy;
    gsl_odeiv_evolve_free(fODE_e);
    gsl_odeiv_control_free(fODE_c);
    gsl_odeiv_step_free(fODE_s);
}

void ODESolver::EvolveFwd(double t1)
{
    while(ft < t1) {
        int status = gsl_odeiv_evolve_apply(fODE_e, fODE_c, fODE_s, &fODE_sys, &ft, t1,
                                            &fODE_h, fy);
        if(status != GSL_SUCCESS) {
            throw std::exception();  // FIXME
        }
    }
}

