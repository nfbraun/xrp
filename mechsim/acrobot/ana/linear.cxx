#include <iostream>
#include <ginac/ginac.h>
#include <gsl/gsl_errno.h>
#include "ODESolver.h"
#include "Octave.h"
#include "Acrobot.h"

int linear_deriv(double t, const double y[], double f[], void *_p)
{
    double* A = (double*) _p;
    
    for(int i=0; i<4; i++) {
        f[i] = 0.;
        for(int j=0; j<4; j++)
            f[i] += A[4*i+j] * y[j];
    }
    
    return GSL_SUCCESS;
}

Matrix Sys_A()
{
    using namespace GiNaC;
    
    Acrobot a;
    Lagrange l(a);
    
    ex qdotdot = l.qdotdot();
    System::Vector x = l.x();
    int ndim = a.ndim();
    matrix A0(4, 4);
    
    for(int i=0; i<ndim; i++)
        for(int j=0; j<2*ndim; j++)
            A0(i,j) = qdotdot[i].diff(x[j]);
    for(int i=0; i<ndim; i++)
        A0(i+ndim, i) = 1;
    
    ex A0x = A0;
    
    for(int i=0; i<2*ndim; i++)
        A0x = A0x.subs(x[i] == 0);
    
    A0 = ex_to<matrix>(A0x);
    
    Matrix mat(2*ndim, 2*ndim);
    for(int i=0; i<2*ndim; i++)
        for(int j=0; j<2*ndim; j++)
            mat(i,j) = ex_to<numeric>(evalf(A0(i,j))).to_double();
    
    return mat;
}

Matrix Sys_B()
{
    using namespace GiNaC;
    
    Acrobot a;
    Lagrange l(a);
    
    ex M = l.M();
    for(int i=0; i<2*a.ndim(); i++)
        M = M.subs(l.x()[i] == 0);
    
    matrix v(2, 1);
    v = 0, 1;
    
    matrix Bex = ex_to<matrix>((ex_to<matrix>(M).inverse() * v).evalm());
    
    Matrix Bn(4, 1);
    for(int i=0; i<2; i++)
        Bn(i,0) = ex_to<numeric>(evalf(Bex(i,0))).to_double();
    Bn(2,0) = 0;
    Bn(3,0) = 0;
    
    return Bn;
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

int main()
{
    const Matrix A = Sys_A(), B = Sys_B();
    const double r = .3, k = 10.;
    Matrix Q(4, 4, 0.);
    Q(2,2) = k;
    Q(3,3) = k;
    
    Matrix S = Octave::are(A, B * B.transpose() / r, Q);
    
    Matrix M = A - B*B.transpose()*S/r;
    
    double mat[16];
    for(int i=0; i<4; i++) {
        for(int j=0; j<4; j++) {
            mat[4*i+j] = M(i,j);
        }
    }
    
    const double qdot_ini[] = { 1.0, 0.0 };
    const double q_ini[] = { -1, 2 };
    
    const double tstep = 1./16.;
    int i;
    
    ODESolver solver(2, &linear_deriv, qdot_ini, q_ini, mat);
    
    for(i=0; i<50; ++i) {
        solver.EvolveFwd(i * tstep);
        std::cout << solver.t() << " " << solver.q()[0] << " " << solver.q()[1];
        std::cout << " " << solver.qdot()[0] << " " << solver.qdot()[1] << std::endl;
    }
    
    return 0;
}
