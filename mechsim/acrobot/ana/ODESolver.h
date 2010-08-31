#ifndef _ODESOLVER_H_
#define _ODESOLVER_H_

#include <gsl/gsl_odeiv.h>

class ODESolver {
  public:
    typedef int (deriv_func_t)(double t, const double y[], double f[], void *_p);
    
    ODESolver(int ndim, deriv_func_t* deriv_func, const double qdot_ini[], const double q_ini[],
              void* params = NULL);
    ~ODESolver();
    
    void EvolveFwd(double t1);
    
    double t() const { return ft; }
    const double* qdot() const { return fy; }
    const double* q() const { return &fy[fNdim]; }
    
    static const double EPS_ABS;
    
  private:
    int fNdim;
    
    gsl_odeiv_step* fODE_s;
    gsl_odeiv_control* fODE_c;
    gsl_odeiv_evolve* fODE_e;
    gsl_odeiv_system fODE_sys;
    double fODE_h;  // step size
    
    double* fy;
    double ft;
};

#endif
