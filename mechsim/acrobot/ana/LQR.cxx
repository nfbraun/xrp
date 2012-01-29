#include "Octave.h"
#include "LQR.h"
#include "Acrobot.h"

GiNaC::matrix LQR::Sys_A(const Lagrange& l, const GiNaC::symbol& u1)
{
    using namespace GiNaC;
    
    ex qdotdot = l.qdotdot().subs(u1 == 0);
    System::Vector x = l.x();
    int ndim = l.ndim();
    matrix A0(4, 4);
    
    for(int i=0; i<ndim; i++)
        A0(i, i+ndim) = 1;
    for(int i=0; i<ndim; i++)
        for(int j=0; j<2*ndim; j++)
            A0(i+ndim,j) = qdotdot[i].diff(x[j]);
    
    ex A0x = A0;
    
    for(int i=0; i<2*ndim; i++)
        A0x = A0x.subs(x[i] == 0);
    
    return ex_to<matrix>(A0x);
}

GiNaC::matrix LQR::Sys_B(const Lagrange& l, const GiNaC::symbol& u1)
{
    using namespace GiNaC;
    
    ex qdotdot = l.qdotdot();
    System::Vector x = l.x();
    int ndim = l.ndim();
    matrix B0(4, 1);
    
    for(int i=0; i<ndim; i++)
        for(int j=0; j<1; j++)
            B0(i+ndim,j) = qdotdot[i].diff(u1);
    
    ex B0x = B0;
    
    for(int i=0; i<2*ndim; i++)
        B0x = B0x.subs(x[i] == 0);
    
    return ex_to<matrix>(B0x);
}

/* void QuadraticCost()
{
    using namespace GiNaC;
    
    symbol x0("x0"), x1("x1"), u("u"), k("k"), r("r");
    matrix Q(2,2);
    
    ex l = r/2*u*u + 1 - exp(k*cos(x0) + k*cos(x1) - 2*k);
    
    Q(0,0) = l.diff(x0).diff(x0).subs(x0==0).subs(x1==0);
    Q(0,1) = l.diff(x0).diff(x1).subs(x0==0).subs(x1==0);
    Q(1,0) = l.diff(x1).diff(x0).subs(x0==0).subs(x1==0);
    Q(1,1) = l.diff(x1).diff(x1).subs(x0==0).subs(x1==0);
    
    std::cout << Q << std::endl;
} */

GiNaC::matrix LQR::lqrControl(const Lagrange& l, const GiNaC::symbol& u1)
{
    const Matrix A = Octave::matTo(Sys_A(l, u1)), B = Octave::matTo(Sys_B(l, u1));
    const double r = 10., k = 10.;
    Matrix Q(4, 4, 0.);
    Q(0,0) = k;
    Q(1,1) = k;
    
    Matrix S = Octave::are(A, B * B.transpose() / r, Q);
    Matrix u = B.transpose()*S/r;
    return Octave::matFrom(u);
}

