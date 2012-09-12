#include "Pendulum.h"
#include <iostream>
#include <cmath>
#include <Eigen/Dense>

const Index Pendulum::N_points = 100;           // number of time intervals
const Index Pendulum::N_vars = 4*(N_points+1); // number of optimization variables
const Index Pendulum::N_constraints = 3*(N_points) + 5; // number of constraints
// time distance between points
const double Pendulum::dt = 10./N_points;

Index index_of_phi(Index i)   { return i; }
Index index_of_omega(Index i) { return Pendulum::N_points+1 + i; }
Index index_of_w(Index i)     { return 2*(Pendulum::N_points+1) + i; }
Index index_of_u(Index i)     { return 3*(Pendulum::N_points+1) + i; }

Pendulum::Pendulum()
    : Ipopt::TNLP()
{
}

bool Pendulum::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
        Index& nnz_h_lag, IndexStyleEnum& index_style)
{
    n = N_vars;
    m = N_constraints;
    nnz_jac_g = 16*N_points+5;  // Number of nonzero entries in the Jacobian
    // Number of effective nonzero entries in the Hessian (16 in total, but we
    // only need the lower left corner).
    nnz_h_lag = 0;
    index_style = C_STYLE; // C or FORTRAN indexing
    
    return true;
}

bool Pendulum::get_bounds_info(Index n, Number* x_l, Number* x_u,
        Index m, Number* g_l, Number* g_u)
{
    assert(n == N_vars);
    assert(m == N_constraints);
    
    /* for(Index i=0; i<N_vars; i++)
        x_l[i] = -5.;
    
    for(Index i=0; i<N_vars; i++)
        x_u[i] = 5.; */
    
    for(Index i=0; i<N_vars; i++)
        x_l[i] = -1e20;
    
    for(Index i=0; i<N_vars; i++)
        x_u[i] = 1e20;
    
    for(Index i=0; i<N_constraints; i++)
        g_l[i] = 0.;
    
    for(Index i=0; i<N_constraints; i++)
        g_u[i] = 0.;
    
    return true;
}

bool Pendulum::get_starting_point(Index n, bool init_x, Number* x,
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

bool Pendulum::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
    assert(n == N_vars);
    
    obj_value = x[index_of_w(N_points)];
    
    return true;
}

bool Pendulum::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
    assert(n == N_vars);
    
    for(Index i=0; i<N_vars; i++)
        grad_f[i] = 0.;
    
    grad_f[index_of_w(N_points)] = 1.;
    
    return true;
}

Number phiddot(Number phi, Number omega, Number u)
{
    return u - 0.2*omega - sin(phi);
}

Eigen::Matrix<Number, 3, 1> dphiddot(Number phi, Number omega, Number u)
{
    Eigen::Matrix<Number, 3, 1> dp;
    dp << -cos(phi), -.2, 1.;
    
    return dp;
}

Eigen::Matrix<Number, 3, 3> ddphiddot(Number phi, Number omega, Number u)
{
    Eigen::Matrix<Number, 3, 3> ddp;
    ddp << sin(phi), 0., 0.,
                 0., 0., 0.,
                 0., 0., 0.;
    return ddp;
}

Number dphiddot_dphi(Number phi, Number omega, Number u)
{
    return -cos(phi);
}

Number dphiddot_domega(Number phi, Number omega, Number u)
{
    return -.2;
}

Number dphiddot_du(Number phi, Number omega, Number u)
{
    return 1.;
}

bool Pendulum::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
    assert(n == N_vars);
    assert(m == N_constraints);
    
    // The ODEs to solve:
    //  phi_dot = omega
    //  omega_dot = f(phi, omega, u)
    //  w_dot = u^2/2
    
    for(Index i=0; i<N_points; i++) {
        const Number phii = x[index_of_phi(i)];
        const Number phii1 = x[index_of_phi(i+1)];
        const Number omegai = x[index_of_omega(i)];
        const Number omegai1 = x[index_of_omega(i+1)];
        const Number wi = x[index_of_w(i)];
        const Number wi1 = x[index_of_w(i+1)];
        const Number ui = x[index_of_u(i)];
        const Number ui1 = x[index_of_u(i+1)];
        
        const Number phi_c = (phii + phii1)/2. + (dt/8.)*(omegai - omegai1);
        const Number omega_c = (omegai + omegai1)/2. + (dt/8.)*
            (phiddot(phii, omegai, ui) - phiddot(phii1, omegai1, ui1));
        const Number w_c = (wi + wi1)/2. + (dt/8.)*(ui*ui/2. - ui1*ui1/2.);
        const Number u_c = (ui + ui1)/2.;
        
        const Number phidot_c = -3./(2.*dt)*(phii - phii1) - (omegai + omegai1)/4.;
        const Number omegadot_c = -3./(2.*dt)*(omegai - omegai1) -
            (phiddot(phii, omegai, ui) + phiddot(phii1, omegai1, ui1))/4.;
        const Number wdot_c = -3./(2.*dt)*(wi - wi1) - (ui*ui/2. + ui1*ui1/2.)/4.;
        
        g[3*i] = omega_c - phidot_c;
        g[3*i+1] = phiddot(phi_c, phidot_c, u_c) - omegadot_c;
        g[3*i+2] = u_c*u_c/2. - wdot_c;
    }
    
    g[3*N_points] = x[index_of_phi(0)];
    g[3*N_points+1] = x[index_of_phi(N_points)] - M_PI;
    g[3*N_points+2] = x[index_of_omega(0)];
    g[3*N_points+3] = x[index_of_omega(N_points)];
    g[3*N_points+4] = x[index_of_w(0)];
    
    return true;
}

bool Pendulum::eval_jac_g(Index n, const Number* x, bool new_x, Index m,
        Index nele_jac, Index* iRow, Index* jCol, Number* values)
{
    /* J_{ij} = \frac{d g_i}{d x_j} */
    
    if(values == 0) {
        Index idx = 0;
        for(Index i=0; i<N_points; i++) {
            iRow[idx] = 3*i; jCol[idx] = index_of_phi(i); idx++;
            iRow[idx] = 3*i; jCol[idx] = index_of_phi(i+1); idx++;
            iRow[idx] = 3*i; jCol[idx] = index_of_omega(i); idx++;
            iRow[idx] = 3*i; jCol[idx] = index_of_omega(i+1); idx++;
            iRow[idx] = 3*i; jCol[idx] = index_of_u(i); idx++;
            iRow[idx] = 3*i; jCol[idx] = index_of_u(i+1); idx++;
            
            iRow[idx] = 3*i+1; jCol[idx] = index_of_phi(i); idx++;
            iRow[idx] = 3*i+1; jCol[idx] = index_of_phi(i+1); idx++;
            iRow[idx] = 3*i+1; jCol[idx] = index_of_omega(i); idx++;
            iRow[idx] = 3*i+1; jCol[idx] = index_of_omega(i+1); idx++;
            iRow[idx] = 3*i+1; jCol[idx] = index_of_u(i); idx++;
            iRow[idx] = 3*i+1; jCol[idx] = index_of_u(i+1); idx++;
            
            iRow[idx] = 3*i+2; jCol[idx] = index_of_w(i); idx++;
            iRow[idx] = 3*i+2; jCol[idx] = index_of_w(i+1); idx++;
            iRow[idx] = 3*i+2; jCol[idx] = index_of_u(i); idx++;
            iRow[idx] = 3*i+2; jCol[idx] = index_of_u(i+1); idx++;
        }
        
        iRow[idx] = 3*N_points;   jCol[idx] = index_of_phi(0); idx++;
        iRow[idx] = 3*N_points+1; jCol[idx] = index_of_phi(N_points); idx++;
        iRow[idx] = 3*N_points+2; jCol[idx] = index_of_omega(0); idx++;
        iRow[idx] = 3*N_points+3; jCol[idx] = index_of_omega(N_points); idx++;
        iRow[idx] = 3*N_points+4; jCol[idx] = index_of_w(0); idx++;
    } else {
        Index idx = 0;
        for(Index i=0; i<N_points; i++) {
            const Number phii = x[index_of_phi(i)];
            const Number phii1 = x[index_of_phi(i+1)];
            const Number omegai = x[index_of_omega(i)];
            const Number omegai1 = x[index_of_omega(i+1)];
            const Number wi = x[index_of_w(i)];
            const Number wi1 = x[index_of_w(i+1)];
            const Number ui = x[index_of_u(i)];
            const Number ui1 = x[index_of_u(i+1)];
            
            const Number phi_c = (phii + phii1)/2. + (dt/8.)*(omegai - omegai1);
            const Number omega_c = (omegai + omegai1)/2. + (dt/8.)*
                (phiddot(phii, omegai, ui) - phiddot(phii1, omegai1, ui1));
            const Number w_c = (wi + wi1)/2. + (dt/8.)*(ui*ui/2. - ui1*ui1/2.);
            const Number u_c = (ui + ui1)/2.;
            
            const Number phidot_c = -3./(2.*dt)*(phii - phii1) - (omegai + omegai1)/4.;
            const Number omegadot_c = -3./(2.*dt)*(omegai - omegai1) -
                (phiddot(phii, omegai, ui) + phiddot(phii1, omegai1, ui1))/4.;
            const Number wdot_c = -3./(2.*dt)*(wi - wi1) - (ui*ui/2. + ui1*ui1/2.)/4.;
            
            values[idx++] = dt/8.*dphiddot_dphi(phii, omegai, ui) + 3./(2.*dt);
            values[idx++] = -dt/8.*dphiddot_dphi(phii1, omegai1, ui1) - 3./(2.*dt);
            values[idx++] = 3./4. + dt/8.*dphiddot_domega(phii, omegai, ui);
            values[idx++] = 3./4. - dt/8.*dphiddot_domega(phii1, omegai1, ui1);
            values[idx++] = dt/8.*dphiddot_du(phii, omegai, ui);
            values[idx++] = -dt/8.*dphiddot_du(phii1, omegai1, ui1);
            
            values[idx++] = dphiddot_dphi(phi_c, phidot_c, u_c)/2.
                - 3./(2.*dt) * dphiddot_domega(phi_c, phidot_c, u_c)
                + dphiddot_dphi(phii, omegai, ui)/4.;
            values[idx++] = dphiddot_dphi(phi_c, phidot_c, u_c)/2.
                + 3./(2.*dt) * dphiddot_domega(phi_c, phidot_c, u_c)
                + dphiddot_dphi(phii1, omegai1, ui1)/4.;
            values[idx++] = dphiddot_dphi(phi_c, omega_c, u_c) * dt/8.
                            - dphiddot_domega(phi_c, phidot_c, u_c)/4.
                            + 3./(2.*dt)
                            + dphiddot_domega(phii, omegai, ui)/4.;
            values[idx++] = - dphiddot_dphi(phi_c, phidot_c, u_c) * dt/8.
                            - dphiddot_domega(phi_c, phidot_c, u_c)/4.
                            - 3./(2.*dt)
                            + dphiddot_domega(phii1, omegai1, ui1)/4.;
            values[idx++] = dphiddot_du(phi_c, phidot_c, u_c)/2.
                + dphiddot_du(phii, omegai, ui)/4.;
            values[idx++] = dphiddot_du(phi_c, phidot_c, u_c)/2.
                + dphiddot_du(phii1, omegai1, ui1)/4.;
            
            values[idx++] = 3./(2.*dt);
            values[idx++] = -3./(2.*dt);
            values[idx++] = u_c/2. + ui/4.;
            values[idx++] = u_c/2. + ui1/4.;
        }
        values[idx++] = 1.;
        values[idx++] = 1.;
        values[idx++] = 1.;
        values[idx++] = 1.;
        values[idx++] = 1.;
    }
    
    return true;
}

bool Pendulum::eval_h(Index n, const Number* x, bool new_x,
        Number obj_factor, Index m, const Number* lambda,
        bool new_lambda, Index nele_hess, Index* iRow, Index* jCol,
        Number* values)
{
    return false;
}

void Pendulum::finalize_solution(Ipopt::SolverReturn status,
        Index n, const Number* x, const Number* z_L, const Number* z_U,
        Index m, const Number* g, const Number* lambda, Number obj_value,
        const Ipopt::IpoptData*, Ipopt::IpoptCalculatedQuantities*)
{
    std::cout << "f(x*) = " << obj_value << std::endl;
    
    for(Index i=0; i<=N_points; i++) {
        std::cerr << (i*dt) << " " << x[index_of_phi(i)] << " " << x[index_of_omega(i)] << " ";
        std::cerr << x[index_of_w(i)] << " " << x[index_of_u(i)] << std::endl;
    }
}
