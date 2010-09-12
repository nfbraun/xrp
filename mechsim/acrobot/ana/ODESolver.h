#ifndef _ODESOLVER_H_
#define _ODESOLVER_H_

#include <ginac/ginac.h>
#include <GiJIT.h>
#include "RawODESolver.h"

class ODESolver: public RawODESolver
{
  public:
    ODESolver(int ndim, const GiNaC::ex& function, const GiNaC::symbol& t,
              const std::vector<GiNaC::symbol>& y_vec,
              const double y_ini[],
              const gsl_odeiv_step_type* stype = gsl_odeiv_step_rkf45);
  private:
    typedef GiJIT::CodeGenR<GiJIT::ReturnDummyInt,
                            GiJIT::Number,
                            GiJIT::Vector,
                            GiJIT::Result,
                            GiJIT::DummyArg<void*> > func_compiler;
    typedef GiJIT::CodeGenR<GiJIT::ReturnDummyInt,
                            GiJIT::Number,
                            GiJIT::Vector,
                            GiJIT::Result,
                            GiJIT::Result,
                            GiJIT::DummyArg<void*> > jacob_compiler;
    static bool funcValid(int ndim, const GiNaC::ex& function);
    static func_compiler::func_t getFunction(int ndim,
        const GiNaC::ex& function, const GiNaC::symbol& t,
        const std::vector<GiNaC::symbol>& yvec);
    static jacob_compiler::func_t getJacobian(int ndim,
        const GiNaC::ex& function, const GiNaC::symbol& t,
        const std::vector<GiNaC::symbol>& yvec);
};

class AutODESolver: public ODESolver
{
  public:
    AutODESolver(int ndim, const GiNaC::ex& function,
              const std::vector<GiNaC::symbol>& y_vec,
              const double y_ini[],
              const gsl_odeiv_step_type* stype = gsl_odeiv_step_rkf45)
        : ODESolver(ndim, function, GiNaC::symbol(), y_vec, y_ini, stype) {}
};

class ODE2Solver: public ODESolver {
  public:
    ODE2Solver(int ndim, const GiNaC::ex& function, const GiNaC::symbol& t,
              const std::vector<GiNaC::symbol>& qdot_vec,
              const std::vector<GiNaC::symbol>& q_vec,
              const double qdot_ini[], const double q_ini[],
              const gsl_odeiv_step_type* stype = gsl_odeiv_step_rkf45)
      : ODESolver(2*ndim, getEx(ndim, function, qdot_vec),
                  t, combineSymVect(qdot_vec, q_vec),
                  combineIniVect(ndim, qdot_ini, q_ini), stype) {}
    const double* qdot() const { return y(); }
    const double* q() const { return y() + ndim()/2; }
    
  private:
    static GiNaC::ex getEx(int ndim, const GiNaC::ex& function,
        const std::vector<GiNaC::symbol>& qdot_vec);
    static std::vector<GiNaC::symbol> combineSymVect(
        const std::vector<GiNaC::symbol>& v1,
        const std::vector<GiNaC::symbol>& v2);
    static ODEArray<double> combineIniVect(int ndim, const double ini1[],
        const double ini2[]);
};

class AutODE2Solver: public ODE2Solver {
  public:
    AutODE2Solver(int ndim, const GiNaC::ex& function,
              const std::vector<GiNaC::symbol>& qdot_vec,
              const std::vector<GiNaC::symbol>& q_vec,
              const double qdot_ini[], const double q_ini[],
              const gsl_odeiv_step_type* stype = gsl_odeiv_step_rkf45)
        : ODE2Solver(ndim, function, GiNaC::symbol(), qdot_vec, q_vec,
                     qdot_ini, q_ini, stype) {}
};

#endif
