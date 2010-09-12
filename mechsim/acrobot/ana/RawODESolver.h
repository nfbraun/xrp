#ifndef _RAWODESOLVER_H_
#define _RAWODESOLVER_H_

#include "ODEArray.h"
#include <gsl/gsl_odeiv.h>

class RawODESolver {
  public:
    typedef int (function_t)(double t, const double y[], double f[], void *_p);
    typedef int (jacobian_t)(double t, const double y[], double* dfdy,
        double dfdt[], void * params);
    
    RawODESolver(int ndim, function_t function, jacobian_t jacobian,
                 const double y_ini[], void* params = NULL,
                 const gsl_odeiv_step_type* stype = gsl_odeiv_step_rkf45);
    ~RawODESolver();
    
    void EvolveFwd(double t1);
    
    double t() const { return ft; }
    const double* y() const { return fy; }
    const int ndim() const { return fNdim; }
    
    static const double EPS_ABS;
    
  private:
    int fNdim;
    
    gsl_odeiv_step* fODE_s;
    gsl_odeiv_control* fODE_c;
    gsl_odeiv_evolve* fODE_e;
    gsl_odeiv_system fODE_sys;
    double fODE_h;  // step size
    
    double ft;
    ODEArray<double> fy;
};

#endif
