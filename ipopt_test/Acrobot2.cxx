#include "Acrobot.h"
#include <iostream>
#include <cmath>
#include <Eigen/Dense>

const Index Acrobot::N_dof = 2;      // number of degrees of freedom
const Index Acrobot::N_controls = 1; // number of controls

const Index Acrobot::N_points = 100;           // number of time intervals
const Index Acrobot::N_vars = (2*N_dof+2)*(N_points+1); // number of optimization variables
const Index Acrobot::N_constraints = N_points*(2*N_dof+1) + 9; // number of constraints
// time distance between points
const double Acrobot::dt = 5./N_points;

const Index Acrobot::N_nnz_jac_g =
      N_points * N_dof * 2 * (2 * N_dof + N_controls)   // q constraints
    + N_points * N_dof * 2 * (2 * N_dof + N_controls)   // qdot constraints
    + 4*N_points                                        // control integration constraints
    + 2*2*N_dof                                         // (q,qdot) boundary constraints
    + 1;                                                // control integration boundary constraint

Index index_of_q(Index i)
    { return Acrobot::N_dof * i; }
Index index_of_v(Index i)
    { return Acrobot::N_dof * (Acrobot::N_points + 1) + Acrobot::N_dof * i; }
Index index_of_w(Index i)
    { return 2 * Acrobot::N_dof * (Acrobot::N_points+1) + i; }
Index index_of_u(Index i)
    { return (2 * Acrobot::N_dof + 1) * (Acrobot::N_points+1) + i; }

Acrobot::Acrobot()
    : Ipopt::TNLP()
{
}

bool Acrobot::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
        Index& nnz_h_lag, IndexStyleEnum& index_style)
{
    n = N_vars;
    m = N_constraints;
    nnz_jac_g = N_nnz_jac_g;  // Number of nonzero entries in the Jacobian
    // Number of effective nonzero entries in the Hessian (16 in total, but we
    // only need the lower left corner).
    nnz_h_lag = 0;
    index_style = C_STYLE; // C or FORTRAN indexing
    
    return true;
}

bool Acrobot::get_bounds_info(Index n, Number* x_l, Number* x_u,
        Index m, Number* g_l, Number* g_u)
{
    assert(n == N_vars);
    assert(m == N_constraints);
    
    for(Index i=0; i<N_vars; i++)
        x_l[i] = -15.;
    
    for(Index i=0; i<N_vars; i++)
        x_u[i] = 15.;
        
    /* for(Index i=0; i<N_vars; i++)
        x_l[i] = -1e20;
    
    for(Index i=0; i<N_vars; i++)
        x_u[i] = 1e20; */
    
    for(Index i=0; i<N_constraints; i++)
        g_l[i] = 0.;
    
    for(Index i=0; i<N_constraints; i++)
        g_u[i] = 0.;
    
    return true;
}

bool Acrobot::get_starting_point(Index n, bool init_x, Number* x,
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

bool Acrobot::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
    assert(n == N_vars);
    
    obj_value = x[index_of_w(N_points)];
    
    return true;
}

bool Acrobot::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
    assert(n == N_vars);
    
    for(Index i=0; i<N_vars; i++)
        grad_f[i] = 0.;
    
    grad_f[index_of_w(N_points)] = 1.;
    
    return true;
}

namespace SystemConst {
    const double G = 9.81;
    const double M1 = .5;
    const double M2 = .2;
    const double LC = .3;
    const double L1 = .5;
    const double L2 = .6;
    const double I1 = 0.;
    const double I2 = 0.;
    // const double GAMMA = 0.;
}

typedef Eigen::Matrix<Number, 2, 1> Vector2N;

Vector2N phiddot(const Vector2N& q, const Vector2N& qdot, Number u)
{
    using namespace SystemConst;
    
    Eigen::Matrix<Number, 2, 2> m;
    
    m(0,0) = M1*LC*LC + M2*L1*L1 + M2*L2*L2
        + 2.*M2*L1*L2*cos(q[1]) + I1 + I2;
    m(0,1) = M2*(L2*L2 + L1*L2*cos(q[1])) + I2;
    m(1,0) = m(0,1);
    m(1,1) = M2*L2*L2 + I2;
    
    Eigen::Matrix<Number, 2, 2> mi = m.inverse();
    
    Eigen::Matrix<Number, 2, 1> c;
    c(0) = -M2*L1*L2*sin(q[1])*qdot[1]*(2*qdot[0] + qdot[1]);
    c(1) = M2*L1*L2*sin(q[1])*qdot[0]*qdot[0];
    
    Eigen::Matrix<Number, 2, 1> dV_dq;
    dV_dq(0) = -(M1*LC + M2*L1)*G*sin(q[0])
        - M2*L2*G*sin(q[0] + q[1]);
    dV_dq(1) = -M2*L2*G*sin(q[0] + q[1]);
    
    Eigen::Matrix<Number, 2, 1> b;
    b << 0, u;
    
    return mi * (b - c - dV_dq);
}

Eigen::Matrix<Number, 2, 2> dphiddot_dq(const Vector2N& q, const Vector2N& qdot, Number u)
{
    using namespace SystemConst;
    
    Eigen::Matrix<Number, 2, 2> m;
    
    m(0,0) = M1*LC*LC + M2*L1*L1 + M2*L2*L2
        + 2.*M2*L1*L2*cos(q[1]) + I1 + I2;
    m(0,1) = M2*(L2*L2 + L1*L2*cos(q[1])) + I2;
    m(1,0) = m(0,1);
    m(1,1) = M2*L2*L2 + I2;
    
    Eigen::Matrix<Number, 2, 2> dm_dq1;
    dm_dq1 << -2.*M2*L1*L2*sin(q[1]), -M2*L1*L2*sin(q[1]),
                 -M2*L1*L2*sin(q[1]),                  0.;
    
    Eigen::Matrix<Number, 2, 2> mi = m.inverse();
    
    Eigen::Matrix<Number, 2, 1> c;
    c(0) = -M2*L1*L2*sin(q[1])*qdot[1]*(2*qdot[0] + qdot[1]);
    c(1) = M2*L1*L2*sin(q[1])*qdot[0]*qdot[0];
    
    Eigen::Matrix<Number, 2, 1> dc_dq1;
    dc_dq1(0) = -M2*L1*L2*cos(q[1])*qdot[1]*(2*qdot[0] + qdot[1]);
    dc_dq1(1) = M2*L1*L2*cos(q[1])*qdot[0]*qdot[0];
    
    Eigen::Matrix<Number, 2, 1> dV_dq;
    dV_dq(0) = -(M1*LC + M2*L1)*G*sin(q[0]) - M2*L2*G*sin(q[0] + q[1]);
    dV_dq(1) = -M2*L2*G*sin(q[0] + q[1]);
    
    Eigen::Matrix<Number, 2, 1> d2V_dq0_dq;
    d2V_dq0_dq(0) = -(M1*LC + M2*L1)*G*cos(q[0]) - M2*L2*G*cos(q[0] + q[1]);
    d2V_dq0_dq(1) = -M2*L2*G*cos(q[0] + q[1]);
    
    Eigen::Matrix<Number, 2, 1> d2V_dq1_dq;
    d2V_dq1_dq(0) = -M2*L2*G*cos(q[0] + q[1]);
    d2V_dq1_dq(1) = -M2*L2*G*cos(q[0] + q[1]);
    
    Eigen::Matrix<Number, 2, 1> b;
    b << 0, u;
    
    Eigen::Matrix<Number, 2, 2> dp;
    
    // note that dm_dq0 = dc_dq0 = 0
    dp.block<2,1>(0,0) = mi * (-d2V_dq0_dq);
    dp.block<2,1>(0,1) = -mi * dm_dq1 * mi * (b - c - dV_dq) + mi * (-dc_dq1 - d2V_dq1_dq);
    
    return dp;
}

Eigen::Matrix<Number, 2, 2> dphiddot_dqdot(const Vector2N& q, const Vector2N& qdot, Number u)
{
    using namespace SystemConst;
    
    Eigen::Matrix<Number, 2, 2> m;
    
    m(0,0) = M1*LC*LC + M2*L1*L1 + M2*L2*L2
        + 2.*M2*L1*L2*cos(q[1]) + I1 + I2;
    m(0,1) = M2*(L2*L2 + L1*L2*cos(q[1])) + I2;
    m(1,0) = m(0,1);
    m(1,1) = M2*L2*L2 + I2;
    
    Eigen::Matrix<Number, 2, 2> mi = m.inverse();
    
    Eigen::Matrix<Number, 2, 1> c;
    c(0) = -M2*L1*L2*sin(q[1])*qdot[1]*(2*qdot[0] + qdot[1]);
    c(1) = M2*L1*L2*sin(q[1])*qdot[0]*qdot[0];
    
    Eigen::Matrix<Number, 2, 1> dc_dqdot0;
    dc_dqdot0(0) = -M2*L1*L2*sin(q[1])*qdot[1]*2;
    dc_dqdot0(1) = M2*L1*L2*sin(q[1])*2*qdot[0];
    
    Eigen::Matrix<Number, 2, 1> dc_dqdot1;
    dc_dqdot1(0) = -M2*L1*L2*sin(q[1])*(2*qdot[0] + 2*qdot[1]);
    dc_dqdot1(1) = 0.;
    
    Eigen::Matrix<Number, 2, 2> dp;
    
    // note that dm_dq0 = dc_dq0 = 0
    dp.block<2,1>(0,0) = mi * (-dc_dqdot0);
    dp.block<2,1>(0,1) = mi * (-dc_dqdot1);
    
    return dp;
}

Vector2N dphiddot_du(const Vector2N& q, const Vector2N& qdot, Number u)
{
    using namespace SystemConst;
    
    Eigen::Matrix<Number, 2, 2> m;
    
    m(0,0) = M1*LC*LC + M2*L1*L1 + M2*L2*L2
        + 2.*M2*L1*L2*cos(q[1]) + I1 + I2;
    m(0,1) = M2*(L2*L2 + L1*L2*cos(q[1])) + I2;
    m(1,0) = m(0,1);
    m(1,1) = M2*L2*L2 + I2;
    
    Eigen::Matrix<Number, 2, 2> mi = m.inverse();
    
    Eigen::Matrix<Number, 2, 1> b;
    b << 0, 1;
    
    return mi * b;
}

/* Eigen::Matrix<Number, 3, 3> ddphiddot(Number phi, Number omega, Number u)
{
    Eigen::Matrix<Number, 3, 3> ddp;
    ddp << sin(phi), 0., 0.,
                 0., 0., 0.,
                 0., 0., 0.;
    return ddp;
} */

void Acrobot::dphiddot_test()
{
    Vector2N q0;
    q0 << 1.4, 0.5;
    Vector2N qdot0;
    qdot0 << .3, .9;
    Number u0;
    u0 = 1.1;
    
    std::cout << phiddot(q0, qdot0, u0).transpose() << std::endl;
    std::cout << -4.58464 << " " << 38.6513 << std::endl;
    std::cout << std::endl;
    
    const double h = 1e-4;
    Vector2N he1; he1 << h, 0;
    Vector2N he2; he2 << 0, h;
    
    {
        Eigen::Matrix<Number, 2, 2> numeric;
        numeric.block<2,1>(0,0) = (phiddot(q0 + he1, qdot0, u0) - phiddot(q0 - he1, qdot0, u0))/(2*h);
        numeric.block<2,1>(0,1) = (phiddot(q0 + he2, qdot0, u0) - phiddot(q0 - he2, qdot0, u0))/(2*h);
        Eigen::Matrix<Number, 2, 2> analytic = dphiddot_dq(q0, qdot0, u0);
        
        std::cout << numeric << std::endl;
        std::cout << analytic << std::endl;
    }
    std::cout << std::endl;
    
    {
        Eigen::Matrix<Number, 2, 2> numeric;
        numeric.block<2,1>(0,0) = (phiddot(q0, qdot0 + he1, u0) - phiddot(q0, qdot0 - he1, u0))/(2*h);
        numeric.block<2,1>(0,1) = (phiddot(q0, qdot0 + he2, u0) - phiddot(q0, qdot0 - he2, u0))/(2*h);
        Eigen::Matrix<Number, 2, 2> analytic = dphiddot_dqdot(q0, qdot0, u0);
        
        std::cout << numeric << std::endl;
        std::cout << analytic << std::endl;
    }
    std::cout << std::endl;
    
    {
        Vector2N numeric;
        numeric = (phiddot(q0, qdot0, u0 + h) - phiddot(q0, qdot0, u0 - h))/(2*h);
        Vector2N analytic = dphiddot_du(q0, qdot0, u0);
        
        std::cout << numeric << std::endl;
        std::cout << analytic << std::endl;
    }
    std::cout << std::endl;
}

bool Acrobot::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
    assert(n == N_vars);
    assert(m == N_constraints);
    
    // The ODEs to solve:
    //  phi_dot = omega
    //  omega_dot = f(phi, omega, u)
    //  w_dot = u^2/2
    
    Index idx = 0;
    
    for(Index i=0; i<N_points; i++) {
        const Eigen::Map<const Vector2N> q_i(  &x[index_of_q(i)]   );
        const Eigen::Map<const Vector2N> q_i1( &x[index_of_q(i+1)] );
        const Eigen::Map<const Vector2N> v_i(  &x[index_of_v(i)]   );
        const Eigen::Map<const Vector2N> v_i1( &x[index_of_v(i+1)] );
        
        const Number w_i  = x[index_of_w(i)];
        const Number w_i1 = x[index_of_w(i+1)];
        const Number u_i  = x[index_of_u(i)];
        const Number u_i1 = x[index_of_u(i+1)];
        
        const Vector2N q_c = (q_i + q_i1)/2. + (dt/8.)*(v_i - v_i1);
        const Vector2N v_c = (v_i + v_i1)/2. + (dt/8.)*
            (phiddot(q_i, v_i, u_i) - phiddot(q_i1, v_i1, u_i1));
        const Number w_c = (w_i + w_i1)/2. + (dt/8.)*(u_i*u_i/2. - u_i1*u_i1/2.);
        const Number u_c = (u_i + u_i1)/2.;
        
        const Vector2N qdot_c = -3./(2.*dt)*(q_i - q_i1) - (v_i + v_i1)/4.;
        const Vector2N vdot_c = -3./(2.*dt)*(v_i - v_i1) -
            (phiddot(q_i, v_i, u_i) + phiddot(q_i1, v_i1, u_i1))/4.;
        const Number wdot_c = -3./(2.*dt)*(w_i - w_i1) - (u_i*u_i/2. + u_i1*u_i1/2.)/4.;
        
        // q constraints
        for(Index j=0; j<N_dof; j++)
            g[idx++] = (v_c - qdot_c)[j];
        
        // qdot constraints
        for(Index j=0; j<N_dof; j++)
            g[idx++] = (phiddot(q_c, qdot_c, u_c) - vdot_c)[j];
        
        // control integration constraints
        g[idx++] = u_c*u_c/2. - wdot_c;
    }
    
    // q boundary constraints
    g[idx++] = x[index_of_q(0) + 0] - M_PI;
    g[idx++] = x[index_of_q(0) + 1];
    g[idx++] = x[index_of_q(N_points) + 0];
    g[idx++] = x[index_of_q(N_points) + 1];
    
    // qdot boundary constraints
    g[idx++] = x[index_of_v(0) + 0];
    g[idx++] = x[index_of_v(0) + 1];
    g[idx++] = x[index_of_v(N_points) + 0];
    g[idx++] = x[index_of_v(N_points) + 1];
    
    // control integration boundary constraint
    g[idx++] = x[index_of_w(0)];
    
    assert(idx == N_constraints);
    
    return true;
}

bool Acrobot::eval_jac_g(Index n, const Number* x, bool new_x, Index m,
        Index nele_jac, Index* iRow, Index* jCol, Number* values)
{
    /* J_{ij} = \frac{d g_i}{d x_j} */
    
    if(values == 0) {
        Index idx = 0;
        Index g_idx = 0;
        for(Index i=0; i<N_points; i++) {
            // q constraints
            for(Index j=0; j<N_dof; j++) {
                for(Index k=0; k<N_dof; k++) {
                    iRow[idx] = g_idx; jCol[idx] = index_of_q(i)+k; idx++;
                    iRow[idx] = g_idx; jCol[idx] = index_of_q(i+1)+k; idx++;
                    iRow[idx] = g_idx; jCol[idx] = index_of_v(i)+k; idx++;
                    iRow[idx] = g_idx; jCol[idx] = index_of_v(i+1)+k; idx++;
                }
                iRow[idx] = g_idx; jCol[idx] = index_of_u(i); idx++;
                iRow[idx] = g_idx; jCol[idx] = index_of_u(i+1); idx++;
                g_idx++;
            }
            
            // qdot constraints
            for(Index j=0; j<N_dof; j++) {
                for(Index k=0; k<N_dof; k++) {
                    iRow[idx] = g_idx; jCol[idx] = index_of_q(i)+k; idx++;
                    iRow[idx] = g_idx; jCol[idx] = index_of_q(i+1)+k; idx++;
                    iRow[idx] = g_idx; jCol[idx] = index_of_v(i)+k; idx++;
                    iRow[idx] = g_idx; jCol[idx] = index_of_v(i+1)+k; idx++;
                }
                iRow[idx] = g_idx; jCol[idx] = index_of_u(i); idx++;
                iRow[idx] = g_idx; jCol[idx] = index_of_u(i+1); idx++;
                g_idx++;
            }
            
            // control integration constraints
            iRow[idx] = g_idx; jCol[idx] = index_of_w(i); idx++;
            iRow[idx] = g_idx; jCol[idx] = index_of_w(i+1); idx++;
            iRow[idx] = g_idx; jCol[idx] = index_of_u(i); idx++;
            iRow[idx] = g_idx; jCol[idx] = index_of_u(i+1); idx++;
            g_idx++;
        }
        
        // q boundary constraints
        iRow[idx] = g_idx; jCol[idx] = index_of_q(0)+0; idx++; g_idx++;
        iRow[idx] = g_idx; jCol[idx] = index_of_q(0)+1; idx++; g_idx++;
        iRow[idx] = g_idx; jCol[idx] = index_of_q(N_points)+0; idx++; g_idx++;
        iRow[idx] = g_idx; jCol[idx] = index_of_q(N_points)+1; idx++; g_idx++;
        
        // qdot boundary constrains
        iRow[idx] = g_idx; jCol[idx] = index_of_v(0)+0; idx++; g_idx++;
        iRow[idx] = g_idx; jCol[idx] = index_of_v(0)+1; idx++; g_idx++;
        iRow[idx] = g_idx; jCol[idx] = index_of_v(N_points)+0; idx++; g_idx++;
        iRow[idx] = g_idx; jCol[idx] = index_of_v(N_points)+1; idx++; g_idx++;
        
        // control integration boundary constraint
        iRow[idx] = g_idx; jCol[idx] = index_of_w(0); idx++; g_idx++;
        
        assert(g_idx == N_constraints);
        assert(idx == N_nnz_jac_g);
    } else {
        Index idx = 0;
        for(Index i=0; i<N_points; i++) {
            const Eigen::Map<const Vector2N> q_i(  &x[index_of_q(i)]   );
            const Eigen::Map<const Vector2N> q_i1( &x[index_of_q(i+1)] );
            const Eigen::Map<const Vector2N> v_i(  &x[index_of_v(i)]   );
            const Eigen::Map<const Vector2N> v_i1( &x[index_of_v(i+1)] );
            
            const Number w_i  = x[index_of_w(i)];
            const Number w_i1 = x[index_of_w(i+1)];
            const Number u_i  = x[index_of_u(i)];
            const Number u_i1 = x[index_of_u(i+1)];
            
            const Vector2N q_c = (q_i + q_i1)/2. + (dt/8.)*(v_i - v_i1);
            const Vector2N v_c = (v_i + v_i1)/2. + (dt/8.)*
                (phiddot(q_i, v_i, u_i) - phiddot(q_i1, v_i1, u_i1));
            const Number w_c = (w_i + w_i1)/2. + (dt/8.)*(u_i*u_i/2. - u_i1*u_i1/2.);
            const Number u_c = (u_i + u_i1)/2.;
            
            const Vector2N qdot_c = -3./(2.*dt)*(q_i - q_i1) - (v_i + v_i1)/4.;
            const Number wdot_c = -3./(2.*dt)*(w_i - w_i1) - (u_i*u_i/2. + u_i1*u_i1/2.)/4.;
            
            // q constraints
            for(Index j=0; j<N_dof; j++) {
                for(Index k=0; k<N_dof; k++) {
                    double djk = (j == k) ? 1 : 0;
                    values[idx++] = dt/8.*dphiddot_dq(q_i, v_i, u_i)(j,k) + 3./(2.*dt) * djk;
                    values[idx++] = -dt/8.*dphiddot_dq(q_i1, v_i1, u_i1)(j,k) - 3./(2.*dt) * djk;
                    values[idx++] = dt/8.*dphiddot_dqdot(q_i, v_i, u_i)(j,k) + 3./4. * djk;
                    values[idx++] = -dt/8.*dphiddot_dqdot(q_i1, v_i1, u_i1)(j,k) + 3./4. * djk;
                }
                values[idx++] = dt/8.*dphiddot_du(q_i, v_i, u_i)(j);
                values[idx++] = -dt/8.*dphiddot_du(q_i1, v_i1, u_i1)(j);
            }
            
            // qdot constraints
            for(Index j=0; j<N_dof; j++) {
                for(Index k=0; k<N_dof; k++) {
                    double djk = (j == k) ? 1 : 0;
                    values[idx++] = dphiddot_dq(q_c, qdot_c, u_c)(j,k)/2.
                        - 3./(2.*dt) * dphiddot_dqdot(q_c, qdot_c, u_c)(j,k)
                        + dphiddot_dq(q_i, v_i, u_i)(j,k)/4.;
                    values[idx++] = dphiddot_dq(q_c, qdot_c, u_c)(j,k)/2.
                        + 3./(2.*dt) * dphiddot_dqdot(q_c, qdot_c, u_c)(j,k)
                        + dphiddot_dq(q_i1, v_i1, u_i1)(j,k)/4.;
                    values[idx++] = dphiddot_dq(q_c, qdot_c, u_c)(j,k) * dt/8.
                                    - dphiddot_dqdot(q_c, qdot_c, u_c)(j,k)/4.
                                    + dphiddot_dqdot(q_i, v_i, u_i)(j,k)/4.
                                    + 3./(2.*dt) * djk;
                    values[idx++] = - dphiddot_dq(q_c, qdot_c, u_c)(j,k) * dt/8.
                                    - dphiddot_dqdot(q_c, qdot_c, u_c)(j,k)/4.
                                    + dphiddot_dqdot(q_i1, v_i1, u_i1)(j,k)/4.
                                    - 3./(2.*dt) * djk;
                }
                values[idx++] = dphiddot_du(q_c, qdot_c, u_c)(j)/2.
                    + dphiddot_du(q_i, v_i, u_i)(j)/4.;
                values[idx++] = dphiddot_du(q_c, qdot_c, u_c)(j)/2.
                    + dphiddot_du(q_i1, v_i1, u_i1)(j)/4.;
            }
            
            // control integration constraints
            values[idx++] = 3./(2.*dt);
            values[idx++] = -3./(2.*dt);
            values[idx++] = u_c/2. + u_i/4.;
            values[idx++] = u_c/2. + u_i1/4.;
        }
        // q boundary constraints
        values[idx++] = 1.;
        values[idx++] = 1.;
        values[idx++] = 1.;
        values[idx++] = 1.;
        
        // qdot boundary constraints
        values[idx++] = 1.;
        values[idx++] = 1.;
        values[idx++] = 1.;
        values[idx++] = 1.;
        
        // control integration boundary constraint
        values[idx++] = 1.;
        
        assert(idx == N_nnz_jac_g);
    }
    
    return true;
}

bool Acrobot::eval_h(Index n, const Number* x, bool new_x,
        Number obj_factor, Index m, const Number* lambda,
        bool new_lambda, Index nele_hess, Index* iRow, Index* jCol,
        Number* values)
{
    return false;
}

void Acrobot::finalize_solution(Ipopt::SolverReturn status,
        Index n, const Number* x, const Number* z_L, const Number* z_U,
        Index m, const Number* g, const Number* lambda, Number obj_value,
        const Ipopt::IpoptData*, Ipopt::IpoptCalculatedQuantities*)
{
    std::cout << "f(x*) = " << obj_value << std::endl;
    
    for(Index i=0; i<=N_points; i++) {
        std::cerr << (i*dt) << " ";
        std::cerr << x[index_of_q(i)+0] << " " << x[index_of_q(i)+1] << " ";
        std::cerr << x[index_of_v(i)+0] << " " << x[index_of_v(i)+1] << " ";
        std::cerr << x[index_of_w(i)] << " " << x[index_of_u(i)] << std::endl;
    }
}
