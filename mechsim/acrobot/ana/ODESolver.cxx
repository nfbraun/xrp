#include "ODESolver.h"
#include <stdexcept>
#include <gsl/gsl_errno.h>

ODESolver::ODESolver(int ndim, const GiNaC::ex& function,
              const GiNaC::symbol& t,
              const std::vector<GiNaC::symbol>& y_vec,
              const double y_ini[],
              const gsl_odeiv_step_type* stype)
    : RawODESolver(ndim, getFunction(ndim, function, t, y_vec),
            0, y_ini, 0, stype)
{ }

bool ODESolver::funcValid(int ndim, const GiNaC::ex& function)
{
    using namespace GiNaC;
    
    if(is_a<lst>(function)) {
        return (ex_to<lst>(function).nops() == (size_t)ndim);
    } else if(is_a<matrix>(function)) {
        const matrix& mat = ex_to<matrix>(function);
        return ((mat.rows() == (size_t)ndim && mat.cols() == 1) ||
                (mat.rows() == 1 && mat.cols() == (size_t)ndim));
    } else if(ndim == 1) {
        return true;
    } else {
        return false;
    }
}

ODESolver::func_compiler::func_t ODESolver::getFunction(int ndim,
        const GiNaC::ex& function, const GiNaC::symbol& t,
        const std::vector<GiNaC::symbol>& yvec)
{
    if(!funcValid(ndim, function))
        throw std::runtime_error("Function does not have expected form");
    return func_compiler::compile(GSL_SUCCESS, t, yvec, function, 0);
}

ODESolver::jacob_compiler::func_t ODESolver::getJacobian(int ndim,
        const GiNaC::ex& function, const GiNaC::symbol& t,
        const std::vector<GiNaC::symbol>& yvec)
{
    // Warning: this code is *untested*!
    GiNaC::matrix dfdy(ndim, ndim);
    GiNaC::matrix dfdt(ndim, 1);
    int i, j;
    
    if(GiNaC::is_a<GiNaC::lst>(function)) {
        const GiNaC::lst& func = GiNaC::ex_to<GiNaC::lst>(function);
        i=0;
        for(GiNaC::lst::const_iterator fi = func.begin();
            fi != func.end(); fi++) {
            j=0;
            for(std::vector<GiNaC::symbol>::const_iterator yj = yvec.begin();
                yj != yvec.end(); yj++) {
                dfdy(i,j) = (*fi).diff(*yj);
                j++;
            }
            dfdt(i,0) = (*fi).diff(t);
            i++;
        }
    }
    
    return jacob_compiler::compile(GSL_SUCCESS, t, yvec, dfdy, dfdt, 0);
}

GiNaC::ex ODE2Solver::getEx(int ndim, const GiNaC::ex& function,
        const std::vector<GiNaC::symbol>& qdot_vec)
{
    using namespace GiNaC;
    lst ex;
    
    // y[0:ndim] = qdot
    for(int i=0; i<ndim; i++)
        ex.append(qdot_vec[i]);
    
    // y[ndim:2*ndim] = qdotdot
    if(is_a<lst>(function) && ex_to<lst>(function).nops() == (size_t)ndim) {
        for(lst::const_iterator i = ex_to<lst>(function).begin();
            i != ex_to<lst>(function).end(); i++) {
            ex.append(*i);
        }
    } else if(is_a<matrix>(function)) {
        const matrix& mat = ex_to<matrix>(function);
        if((mat.rows() == (size_t)ndim && mat.cols() == 1) ||
           (mat.rows() == 1 && mat.cols() == (size_t)ndim)) {
            for(size_t i = 0; i != mat.nops(); i++) {
                ex.append(mat.op(i));
            }
        } else {
            throw std::runtime_error("Function does not have expected form");
        }
    } else if(ndim == 1) {
        ex.append(function);
    } else {
        throw std::runtime_error("Function does not have expected form");
    }
    
    return ex;
}

std::vector<GiNaC::symbol> ODE2Solver::combineSymVect(
    const std::vector<GiNaC::symbol>& v1,
    const std::vector<GiNaC::symbol>& v2)
{
    std::vector<GiNaC::symbol> v;
    v.insert(v.end(), v1.begin(), v1.end());
    v.insert(v.end(), v2.begin(), v2.end());
    return v;
}

ODEArray<double> ODE2Solver::combineIniVect(int ndim, const double ini1[],
        const double ini2[])
{
    ODEArray<double> a(2*ndim);
    for(int i=0; i<ndim; i++)
        a[i] = ini1[i];
    for(int i=0; i<ndim; i++)
        a[i+ndim] = ini2[i];
    return a;
}
