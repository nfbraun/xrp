#ifndef _LAGRODESOLVER_H_
#define _LAGRODESOLVER_H_

#include <ginac/ginac.h>
#include <GiJIT.h>
#include "ODEArray.h"
#include "RawODESolver.h"

class LagrODESolver: public RawODESolver
{
  public:
    LagrODESolver(int ndim, const GiNaC::ex& M_qdotdot, const GiNaC::ex& M,
              const std::vector<GiNaC::symbol>& q_vec,
              const std::vector<GiNaC::symbol>& qdot_vec,
              const double q_ini[], const double qdot_ini[],
              const gsl_odeiv_step_type* stype = gsl_odeiv_step_rkf45);
    
    const double* q() const { return y(); }
    const double* qdot() const { return y() + ndim()/2; }
    
  private:
    typedef GiJIT::CodeGenV<GiJIT::Result,
                            GiJIT::Vector > M_compiler;
    typedef GiJIT::CodeGenV<GiJIT::Result,
                            GiJIT::Vector,
                            GiJIT::Vector > M_qdotdot_compiler;
    
    struct LagrODEData {
        LagrODESolver::M_compiler::func_t calc_M;
        LagrODESolver::M_qdotdot_compiler::func_t calc_M_qdotdot;
        
        int ndim;
    };
    
    struct LagrODEData fData;
    
    static ODEArray<double> combineIniVect(int ndim, const double ini1[],
        const double ini2[]);
    
    static int calc_xdot(double t, const double x_raw[], double xdot_raw[], void *_p);
};

#endif
