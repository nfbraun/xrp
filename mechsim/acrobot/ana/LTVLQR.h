#ifndef ACRO_LTVLQR_H
#define ACRO_LTVLQR_H

#include "Lagrange.h"
#include <Eigen/Dense>
#include <ginac/ginac.h>
#include <GiJIT.h>

class LTVLQR
{
  public:
    static GiNaC::matrix Sys_A(const Lagrange& l, const GiNaC::symbol& u1);
    static GiNaC::matrix Sys_B(const Lagrange& l, const GiNaC::symbol& u1);
    
    typedef GiJIT::CodeGenV<GiJIT::Result,
                            GiJIT::Vector,
                            GiJIT::Number> A_compiler_t;
    
    typedef GiJIT::CodeGenV<GiJIT::Result,
                            GiJIT::Vector,
                            GiJIT::Number> B_compiler_t;
    
    A_compiler_t::func_t calcA;
    B_compiler_t::func_t calcB;
    
    LTVLQR(const Lagrange& l, const GiNaC::symbol& u1);
    
    Eigen::Matrix<double, 4, 4> Q;
    Eigen::Matrix<double, 1, 1> R;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
