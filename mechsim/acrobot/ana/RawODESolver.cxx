#include <gsl/gsl_errno.h>
#include <iostream>
#include <stdexcept>
#include <cmath>
#include "RawODESolver.h"

const double RawODESolver::EPS_ABS = 1e-8;

RawODESolver::RawODESolver(int ndim, function_t* function,
                     jacobian_t* jacobian, const double y_ini[],
                     void* params, const gsl_odeiv_step_type* stype)
    : fNdim(ndim), ft(0.), fy(ndim, y_ini)
{
    fODE_s = gsl_odeiv_step_alloc(stype, ndim);
    fODE_c = gsl_odeiv_control_y_new(EPS_ABS, 0.);
    fODE_e = gsl_odeiv_evolve_alloc(ndim);
    
    fODE_sys.function = function;
    fODE_sys.jacobian = jacobian;
    fODE_sys.dimension = ndim;
    fODE_sys.params = params;
    
    fODE_h = 1e-6;
}

RawODESolver::~RawODESolver()
{
    gsl_odeiv_evolve_free(fODE_e);
    gsl_odeiv_control_free(fODE_c);
    gsl_odeiv_step_free(fODE_s);
}

void RawODESolver::EvolveFwd(double t1)
{
    while(ft < t1) {
        int status = gsl_odeiv_evolve_apply(fODE_e, fODE_c, fODE_s, &fODE_sys,
                                            &ft, t1, &fODE_h, fy);
        if(status != GSL_SUCCESS) {
            throw std::runtime_error("RawODESolver::EvolveFwd() failed");
        }
    }
}

