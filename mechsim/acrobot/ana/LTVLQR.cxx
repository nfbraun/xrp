#include "LTVLQR.h"

GiNaC::matrix LTVLQR::Sys_A(const Lagrange& l, const GiNaC::symbol& u1)
{
    using namespace GiNaC;
    
    ex qdotdot = l.qdotdot();
    System::Vector x = l.x();
    int ndim = l.ndim();
    matrix A(4, 4);
    
    for(int i=0; i<ndim; i++)
        A(i,i+ndim) = 1;
    for(int i=0; i<ndim; i++)
        for(int j=0; j<2*ndim; j++)
            A(i+ndim,j) = qdotdot[i].diff(x[j]);
    
    return A;
}

GiNaC::matrix LTVLQR::Sys_B(const Lagrange& l, const GiNaC::symbol& u1)
{
    using namespace GiNaC;
    
    ex qdotdot = l.qdotdot();
    System::Vector x = l.x();
    int ndim = l.ndim();
    matrix B(4, 1);
    
    for(int i=0; i<ndim; i++)
        for(int j=0; j<1; j++)
            B(i+ndim,j) = qdotdot[i].diff(u1);
    
    return B;
}

LTVLQR::LTVLQR(const Lagrange& l, const GiNaC::symbol& u1)
{
    calcA = A_compiler_t::compile(Sys_A(l, u1), l.x(), u1);
    calcB = B_compiler_t::compile(Sys_B(l, u1), l.x(), u1);
    
    Q << 1, 0, 0, 0,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    
    R << 1.;
}
