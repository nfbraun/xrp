#include "DoubleInt.h"
#include <iostream>
#include <cmath>

// see: Oskar von Stryk
// Numerical Solution of Optimal Control Problems by Direct Collocation
// http://www.sim.informatik.tu-darmstadt.de/publ/download/1991-dircol.pdf

// expected solution for l=1/9 and 10 time intervals: w(1) = 3.99338
// analytical solution: 4/(9*l) = 4

const Index DoubleInt::N_points = 10;           // number of time intervals
const Index DoubleInt::N_vars = 4*(N_points+1); // number of optimization variables
const Index DoubleInt::N_constraints = 3*(N_points) + 5; // number of constraints
// time distance between points
const double DoubleInt::dt = 1./N_points;

Index index_of_r(Index i) { return i; }
Index index_of_v(Index i) { return DoubleInt::N_points+1 + i; }
Index index_of_w(Index i) { return 2*(DoubleInt::N_points+1) + i; }
Index index_of_u(Index i) { return 3*(DoubleInt::N_points+1) + i; }

DoubleInt::DoubleInt()
    : Ipopt::TNLP()
{
}

bool DoubleInt::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
        Index& nnz_h_lag, IndexStyleEnum& index_style)
{
    n = N_vars;
    m = N_constraints;
    nnz_jac_g = 14*N_points+5;  // Number of nonzero entries in the Jacobian
    // Number of effective nonzero entries in the Hessian (16 in total, but we
    // only need the lower left corner).
    nnz_h_lag = 0;
    index_style = C_STYLE; // C or FORTRAN indexing
    
    return true;
}

bool DoubleInt::get_bounds_info(Index n, Number* x_l, Number* x_u,
        Index m, Number* g_l, Number* g_u)
{
    const double l = 1./9.;
    
    assert(n == N_vars);
    assert(m == N_constraints);
    
    for(Index i=0; i<N_vars; i++)
        x_l[i] = -1e20;
    
    for(Index i=0; i<N_vars; i++)
        x_u[i] = 1e20;
    
    for(Index i=0; i<=N_points; i++)
        x_l[index_of_r(i)] = 0.;
    
    for(Index i=0; i<=N_points; i++)
        x_u[index_of_r(i)] = l;
    
    for(Index i=0; i<N_constraints; i++)
        g_l[i] = 0.;
    
    for(Index i=0; i<N_constraints; i++)
        g_u[i] = 0.;
    
    return true;
}

bool DoubleInt::get_starting_point(Index n, bool init_x, Number* x,
        bool init_z, Number* z_L, Number* z_U,
        Index m, bool init_lambda, Number* lambda)
{
    // z and lambda initialization only needed for ``warm start''
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);
    
    for(int i=0; i<N_vars; i++) x[i] = 0.;
    
    return true;
}

bool DoubleInt::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
    assert(n == N_vars);
    
    obj_value = x[index_of_w(N_points)];
    
    return true;
}

bool DoubleInt::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
    assert(n == N_vars);
    
    for(Index i=0; i<N_vars; i++)
        grad_f[i] = 0.;
    
    grad_f[index_of_w(N_points)] = 1.;
    
    return true;
}

bool DoubleInt::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
    assert(n == N_vars);
    assert(m == N_constraints);
    
    for(Index i=0; i<N_points; i++) {
        const Number ri = x[index_of_r(i)];
        const Number ri1 = x[index_of_r(i+1)];
        const Number vi = x[index_of_v(i)];
        const Number vi1 = x[index_of_v(i+1)];
        const Number wi = x[index_of_w(i)];
        const Number wi1 = x[index_of_w(i+1)];
        const Number ui = x[index_of_u(i)];
        const Number ui1 = x[index_of_u(i+1)];
        
        g[3*i] = 3./(2.*dt)*(ri-ri1) + 3./4.*(vi + vi1) + dt/8.*(ui - ui1);
        g[3*i+1] = 3./(2.*dt)*(vi-vi1) + 3./4.*(ui+ui1);
        g[3*i+2] = (ui+ui1)*(ui+ui1)/8. + 3./(2.*dt)*(wi-wi1) + 1./8.*(ui*ui+ui1*ui1);
    }
    
    g[3*N_points] = x[index_of_r(0)];
    g[3*N_points+1] = x[index_of_r(N_points)];
    g[3*N_points+2] = x[index_of_v(0)] - 1.;
    g[3*N_points+3] = x[index_of_v(N_points)] + 1.;
    g[3*N_points+4] = x[index_of_w(0)];
    
    return true;
}

bool DoubleInt::eval_jac_g(Index n, const Number* x, bool new_x, Index m,
        Index nele_jac, Index* iRow, Index* jCol, Number* values)
{
    /* J_{ij} = \frac{d g_i}{d x_j} */
    
    if(values == 0) {
        Index idx = 0;
        for(Index i=0; i<N_points; i++) {
            iRow[idx] = 3*i; jCol[idx] = index_of_r(i); idx++;
            iRow[idx] = 3*i; jCol[idx] = index_of_r(i+1); idx++;
            iRow[idx] = 3*i; jCol[idx] = index_of_v(i); idx++;
            iRow[idx] = 3*i; jCol[idx] = index_of_v(i+1); idx++;
            iRow[idx] = 3*i; jCol[idx] = index_of_u(i); idx++;
            iRow[idx] = 3*i; jCol[idx] = index_of_u(i+1); idx++;
            
            iRow[idx] = 3*i+1; jCol[idx] = index_of_v(i); idx++;
            iRow[idx] = 3*i+1; jCol[idx] = index_of_v(i+1); idx++;
            iRow[idx] = 3*i+1; jCol[idx] = index_of_u(i); idx++;
            iRow[idx] = 3*i+1; jCol[idx] = index_of_u(i+1); idx++;
            
            iRow[idx] = 3*i+2; jCol[idx] = index_of_w(i); idx++;
            iRow[idx] = 3*i+2; jCol[idx] = index_of_w(i+1); idx++;
            iRow[idx] = 3*i+2; jCol[idx] = index_of_u(i); idx++;
            iRow[idx] = 3*i+2; jCol[idx] = index_of_u(i+1); idx++;
        }
        
        iRow[idx] = 3*N_points;   jCol[idx] = index_of_r(0); idx++;
        iRow[idx] = 3*N_points+1; jCol[idx] = index_of_r(N_points); idx++;
        iRow[idx] = 3*N_points+2; jCol[idx] = index_of_v(0); idx++;
        iRow[idx] = 3*N_points+3; jCol[idx] = index_of_v(N_points); idx++;
        iRow[idx] = 3*N_points+4; jCol[idx] = index_of_w(0); idx++;
    } else {
        Index idx = 0;
        for(Index i=0; i<N_points; i++) {
            values[idx++] = 3./(2.*dt);
            values[idx++] = -3./(2.*dt);
            values[idx++] = 3./4.;
            values[idx++] = 3./4.;
            values[idx++] = dt/8.;
            values[idx++] = -dt/8.;
            
            values[idx++] = 3./(2.*dt);
            values[idx++] = -3./(2.*dt);
            values[idx++] = 3./4.;
            values[idx++] = 3./4.;
            
            values[idx++] = 3./(2.*dt);
            values[idx++] = -3./(2.*dt);
            values[idx++] = x[index_of_u(i)]/2. + x[index_of_u(i+1)]/4.;
            values[idx++] = x[index_of_u(i+1)]/2. + x[index_of_u(i)]/4.;
        }
        values[idx++] = 1.;
        values[idx++] = 1.;
        values[idx++] = 1.;
        values[idx++] = 1.;
        values[idx++] = 1.;
    }
    
    return true;
}

// #define ANALYTIC_SECOND_DERIVATIVE
bool DoubleInt::eval_h(Index n, const Number* x, bool new_x,
        Number obj_factor, Index m, const Number* lambda,
        bool new_lambda, Index nele_hess, Index* iRow, Index* jCol,
        Number* values)
{
#ifdef ANALYTIC_SECOND_DERIVATIVE
    if(values == 0) {
        Index idx = 0;
        for(Index i=0; i<N_points; i++) {
            iRow[idx++] = 
        }
    }
#else
    return false;
#endif
}

void DoubleInt::finalize_solution(Ipopt::SolverReturn status,
        Index n, const Number* x, const Number* z_L, const Number* z_U,
        Index m, const Number* g, const Number* lambda, Number obj_value,
        const Ipopt::IpoptData*, Ipopt::IpoptCalculatedQuantities*)
{
    std::cout << "f(x*) = " << obj_value << std::endl;
    
    for(Index i=0; i<=N_points; i++) {
        std::cerr << (i*dt) << " " << x[index_of_r(i)] << " " << x[index_of_v(i)] << " ";
        std::cerr << x[index_of_w(i)] << " " << x[index_of_u(i)] << std::endl;
    }
}
