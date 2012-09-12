#include "Acrobot.h"
#include <iostream>
#include <cmath>
#include <Eigen/Dense>

const Index Acrobot::N_dof = 2;      // number of degrees of freedom
const Index Acrobot::N_controls = 1; // number of controls

const Index Acrobot::N_points = 30;           // number of time intervals
const Index Acrobot::N_vars = (2*N_dof+2)*(N_points+1); // number of optimization variables
const Index Acrobot::N_constraints = N_points*(2*N_dof+1) + 9; // number of constraints
// time distance between points
const double Acrobot::dt = 10./N_points;

const Index Acrobot::N_nnz_jac_g =
      N_points * N_dof * 2 * (2 * N_dof + N_controls)   // q constraints
    + N_points * N_dof * 2 * (2 * N_dof + N_controls)   // qdot constraints
    + 4*N_points                                        // control integration constraints
    + 2*2*N_dof                                         // (q,qdot) boundary constraints
    + 1;                                                // control integration boundary constraint

const Index Acrobot::N_nnz_h_lag = 40*N_points + 15;
    //(2*N_dof+N_controls)*(2*N_dof+N_controls)*N_points;

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
    // dphiddot_test();
}

bool Acrobot::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
        Index& nnz_h_lag, IndexStyleEnum& index_style)
{
    n = N_vars;
    m = N_constraints;
    nnz_jac_g = N_nnz_jac_g;  // Number of nonzero entries in the Jacobian
    // Number of effective nonzero entries in the Hessian (16 in total, but we
    // only need the lower left corner).
    nnz_h_lag = N_nnz_h_lag;
    index_style = C_STYLE; // C or FORTRAN indexing
    
    return true;
}

bool Acrobot::get_bounds_info(Index n, Number* x_l, Number* x_u,
        Index m, Number* g_l, Number* g_u)
{
    assert(n == N_vars);
    assert(m == N_constraints);
    
    for(Index i=0; i<N_vars; i++)
        x_l[i] = -30.;
    
    for(Index i=0; i<N_vars; i++)
        x_u[i] = 30.;
        
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
    
    // Note that the mass matrix only depends on q[1], not on q[0]
    Eigen::Matrix<Number, 2, 2> m;
    m(0,0) = M1*LC*LC + M2*L1*L1 + M2*L2*L2
        + 2.*M2*L1*L2*cos(q[1]) + I1 + I2;
    m(0,1) = M2*(L2*L2 + L1*L2*cos(q[1])) + I2;
    m(1,0) = m(0,1);
    m(1,1) = M2*L2*L2 + I2;
    
    // First derivative of the mass matrix (w.r.t q[1])
    Eigen::Matrix<Number, 2, 2> mp;
    mp(0,0) = -2.*M2*L1*L2*sin(q[1]);
    mp(0,1) = -M2*L1*L2*sin(q[1]);
    mp(1,0) = mp(0,1);
    mp(1,1) = 0.;
    
    // Second derivative of the mass matrix (w.r.t q[1])
    Eigen::Matrix<Number, 2, 2> mpp;
    mpp(0,0) = -2.*M2*L1*L2*cos(q[1]);
    mpp(0,1) = -M2*L1*L2*cos(q[1]);
    mpp(1,0) = mpp(0,1);
    mpp(1,1) = 0.;
    
    // Third derivative of the mass matrix (w.r.t q[1])
    Eigen::Matrix<Number, 2, 2> mppp;
    mppp(0,0) = 2.*M2*L1*L2*sin(q[1]);
    mppp(0,1) = M2*L1*L2*sin(q[1]);
    mppp(1,0) = mppp(0,1);
    mppp(1,1) = 0.;
    
    Eigen::Matrix<Number, 2, 2> mi = m.inverse();
    
    Eigen::Matrix<Number, 2, 1> c;
    c(0) = mp(0,0)*qdot[0]*qdot[1] + mp(1,0)*qdot[1]*qdot[1];
    c(1) = -mp(0,0)/2.*qdot[0]*qdot[0] + mp(1,1)/2.*qdot[1]*qdot[1];
    
    Eigen::Matrix<Number, 2, 1> dV_dq;
    dV_dq(0) = -(M1*LC + M2*L1)*G*sin(q[0])
        - M2*L2*G*sin(q[0] + q[1]);
    dV_dq(1) = -M2*L2*G*sin(q[0] + q[1]);
    
    Eigen::Matrix<Number, 2, 1> b;
    b << 0, u;
    
    return mi * (b - c - dV_dq);
}

Eigen::Matrix<Number, 2, 5> dphiddot(const Vector2N& q, const Vector2N& qdot, Number u)
{
    using namespace SystemConst;
    
    // Note that the mass matrix only depends on q[1], not on q[0]
    Eigen::Matrix<Number, 2, 2> m;
    m(0,0) = M1*LC*LC + M2*L1*L1 + M2*L2*L2
        + 2.*M2*L1*L2*cos(q[1]) + I1 + I2;
    m(0,1) = M2*(L2*L2 + L1*L2*cos(q[1])) + I2;
    m(1,0) = m(0,1);
    m(1,1) = M2*L2*L2 + I2;
    
    // First derivative of the mass matrix (w.r.t q[1])
    Eigen::Matrix<Number, 2, 2> dm_dq1;
    dm_dq1(0,0) = -2.*M2*L1*L2*sin(q[1]);
    dm_dq1(0,1) = -M2*L1*L2*sin(q[1]);
    dm_dq1(1,0) = dm_dq1(0,1);
    dm_dq1(1,1) = 0.;
    
    // Second derivative of the mass matrix (w.r.t q[1])
    Eigen::Matrix<Number, 2, 2> mpp;
    mpp(0,0) = -2.*M2*L1*L2*cos(q[1]);
    mpp(0,1) = -M2*L1*L2*cos(q[1]);
    mpp(1,0) = mpp(0,1);
    mpp(1,1) = 0.;
    
    // Third derivative of the mass matrix (w.r.t q[1])
    Eigen::Matrix<Number, 2, 2> mppp;
    mppp(0,0) = 2.*M2*L1*L2*sin(q[1]);
    mppp(0,1) = M2*L1*L2*sin(q[1]);
    mppp(1,0) = mppp(0,1);
    mppp(1,1) = 0.;
    
    Eigen::Matrix<Number, 2, 2> mi = m.inverse();
    
    Eigen::Matrix<Number, 2, 1> c;
    c(0) = dm_dq1(0,0)*qdot[0]*qdot[1] + dm_dq1(1,0)*qdot[1]*qdot[1];
    c(1) = -dm_dq1(0,0)/2.*qdot[0]*qdot[0] + dm_dq1(1,1)/2.*qdot[1]*qdot[1];
    
    Eigen::Matrix<Number, 2, 1> dc_dq1;
    dc_dq1(0) = mpp(0,0)*qdot[0]*qdot[1] + mpp(1,0)*qdot[1]*qdot[1];
    dc_dq1(1) = -mpp(0,0)/2.*qdot[0]*qdot[0] + mpp(1,1)/2.*qdot[1]*qdot[1];
    
    Eigen::Matrix<Number, 2, 1> dc_dqdot0;
    dc_dqdot0(0) = dm_dq1(0,0)*qdot[1];
    dc_dqdot0(1) = -dm_dq1(0,0)*qdot[0];
    
    Eigen::Matrix<Number, 2, 1> dc_dqdot1;
    dc_dqdot1(0) = dm_dq1(0,0)*qdot[0] + 2.*dm_dq1(1,0)*qdot[1];
    dc_dqdot1(1) = dm_dq1(1,1)*qdot[1];
    
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
    
    Eigen::Matrix<Number, 2, 2> dmi_dq0 = Eigen::Matrix<Number, 2, 2>::Zero();
    Eigen::Matrix<Number, 2, 2> dmi_dq1 = -mi * dm_dq1 * mi;
    
    Eigen::Matrix<Number, 2, 1> g = b - c - dV_dq;
    
    Eigen::Matrix<Number, 2, 1> dg_dq0 = -d2V_dq0_dq;
    Eigen::Matrix<Number, 2, 1> dg_dq1 = -dc_dq1 - d2V_dq1_dq;
    
    Eigen::Matrix<Number, 2, 5> dp;
    
    // note that dm_dq0 = dc_dq0 = 0
    
    // f = mi * g;
    
    // df_dq
    dp.block<2,1>(0,0) = dmi_dq0 * g + mi * dg_dq0;
    dp.block<2,1>(0,1) = dmi_dq1 * g + mi * dg_dq1;
    
    // df_qdot
    dp.block<2,1>(0,2) = mi * (-dc_dqdot0);
    dp.block<2,1>(0,3) = mi * (-dc_dqdot1);
    
    // df_du
    Eigen::Matrix<Number, 2, 1> bp;
    bp << 0, 1;
    
    dp.block<2,1>(0,4) = mi * bp;
    
    return dp;
}

struct DResult {
    Vector2N phiddot;
    Vector2N dphiddot[5];
    Vector2N ddphiddot[5][5];
};

DResult phiddot2(const Vector2N& q, const Vector2N& qdot, Number u)
{
    using namespace SystemConst;
    
    // Note that the mass matrix only depends on q[1], not on q[0]
    Eigen::Matrix<Number, 2, 2> m;
    m(0,0) = M1*LC*LC + M2*L1*L1 + M2*L2*L2
        + 2.*M2*L1*L2*cos(q[1]) + I1 + I2;
    m(0,1) = M2*(L2*L2 + L1*L2*cos(q[1])) + I2;
    m(1,0) = m(0,1);
    m(1,1) = M2*L2*L2 + I2;
    
    // First derivatives of the mass matrix
    Eigen::Matrix<Number, 2, 2> dm_dq[2];
    dm_dq[0].setZero(2,2);
    
    dm_dq[1](0,0) = -2.*M2*L1*L2*sin(q[1]);
    dm_dq[1](0,1) = -M2*L1*L2*sin(q[1]);
    dm_dq[1](1,0) = dm_dq[1](0,1);
    dm_dq[1](1,1) = 0.;
    
    // Second derivatives of the mass matrix
    Eigen::Matrix<Number, 2, 2> d2m_dqq[2][2];
    d2m_dqq[0][0].setZero(2,2);
    d2m_dqq[0][1].setZero(2,2);
    d2m_dqq[1][0].setZero(2,2);
    
    d2m_dqq[1][1](0,0) = -2.*M2*L1*L2*cos(q[1]);
    d2m_dqq[1][1](0,1) = -M2*L1*L2*cos(q[1]);
    d2m_dqq[1][1](1,0) = d2m_dqq[1][1](0,1);
    d2m_dqq[1][1](1,1) = 0.;
    
    // Third derivatives of the mass matrix
    Eigen::Matrix<Number, 2, 2> d3m_dqqq[2][2][2];
    d3m_dqqq[0][0][0].setZero(2,2);
    d3m_dqqq[0][0][1].setZero(2,2);
    d3m_dqqq[0][1][0].setZero(2,2);
    d3m_dqqq[0][1][1].setZero(2,2);
    d3m_dqqq[1][0][0].setZero(2,2);
    d3m_dqqq[1][0][1].setZero(2,2);
    d3m_dqqq[1][1][0].setZero(2,2);
    
    d3m_dqqq[1][1][1](0,0) = 2.*M2*L1*L2*sin(q[1]);
    d3m_dqqq[1][1][1](0,1) = M2*L1*L2*sin(q[1]);
    d3m_dqqq[1][1][1](1,0) = d3m_dqqq[1][1][1](0,1);
    d3m_dqqq[1][1][1](1,1) = 0.;
    
    // Potential energy
    Eigen::Matrix<Number, 2, 1> dV_dq;
    dV_dq(0) = -(M1*LC + M2*L1)*G*sin(q[0]) - M2*L2*G*sin(q[0] + q[1]);
    dV_dq(1) = -M2*L2*G*sin(q[0] + q[1]);
    
    Eigen::Matrix<Number, 2, 1> d2V_dqq[2];
    d2V_dqq[0](0) = -(M1*LC + M2*L1)*G*cos(q[0]) - M2*L2*G*cos(q[0] + q[1]);
    d2V_dqq[0](1) = -M2*L2*G*cos(q[0] + q[1]);
    
    d2V_dqq[1](0) = d2V_dqq[0](1);
    d2V_dqq[1](1) = -M2*L2*G*cos(q[0] + q[1]);
    
    Eigen::Matrix<Number, 2, 1> d3V_dqqq[2][2];
    d3V_dqqq[0][0](0) = (M1*LC + M2*L1)*G*sin(q[0]) + M2*L2*G*sin(q[0] + q[1]);
    d3V_dqqq[0][0](1) = M2*L2*G*sin(q[0] + q[1]);
    d3V_dqqq[0][1](1) = M2*L2*G*sin(q[0] + q[1]);
    d3V_dqqq[1][1](1) = M2*L2*G*sin(q[0] + q[1]);
    
    d3V_dqqq[0][1](0) = d3V_dqqq[0][0](1);
    d3V_dqqq[1][0](0) = d3V_dqqq[0][0](1);
    d3V_dqqq[1][0](1) = d3V_dqqq[0][1](1);
    d3V_dqqq[1][1](0) = d3V_dqqq[0][1](1);
    
    // Inverse mass matrix and its derivatives
    Eigen::Matrix<Number, 2, 2> mi = m.inverse();
    
    Eigen::Matrix<Number, 2, 2> dmi_dq[2];
    for(unsigned int a=0; a<2; a++) {
        dmi_dq[a] = -mi * dm_dq[a] * mi;
    }
    
    Eigen::Matrix<Number, 2, 2> d2mi_dqq[2][2];
    for(unsigned int a=0; a<2; a++) {
        for(unsigned int b=0; b<2; b++) {
            d2mi_dqq[a][b] = -dmi_dq[a]*dm_dq[b]*mi - dmi_dq[b]*dm_dq[a]*mi - mi*d2m_dqq[a][b]*mi;
        }
    }
    
    // Coriolis term c(q, qdot)
    Eigen::Matrix<Number, 2, 1> c;
    c.setZero(2, 1);
    for(unsigned int l=0; l<2; l++) {
        for(unsigned int i=0; i<2; i++) {
            for(unsigned int j=0; j<2; j++) {
                c(l) += (dm_dq[j](i,l) - dm_dq[l](i,j)/2.)*qdot[i]*qdot[j];
            }
        }
    }
    
    // ... and its first derivatives
    Eigen::Matrix<Number, 2, 1> dc_dq[2];
    for(unsigned int a=0; a<2; a++) {
        dc_dq[a].setZero(2,1);
        for(unsigned int l=0; l<2; l++) {
            for(unsigned int i=0; i<2; i++) {
                for(unsigned int j=0; j<2; j++) {
                    dc_dq[a](l) += (d2m_dqq[a][j](i,l) - d2m_dqq[a][l](i,j)/2.)*qdot[i]*qdot[j];
                }
            }
        }
    }
    
    Eigen::Matrix<Number, 2, 1> dc_dqdot[2];
    for(unsigned int a=0; a<2; a++) {
        dc_dqdot[a].setZero(2,1);
        for(unsigned int l=0; l<2; l++) {
            for(unsigned int j=0; j<2; j++) {
                dc_dqdot[a](l) += (dm_dq[j](a,l) - dm_dq[l](a,j)/2.) * qdot[j];
            }
            for(unsigned int i=0; i<2; i++) {
                dc_dqdot[a](l) += (dm_dq[a](i,l) - dm_dq[l](i,a)/2.) * qdot[i];
            }
        }
    }
    
    // ... and its second derivatives
    Eigen::Matrix<Number, 2, 1> d2c_dqq[2][2];
    for(unsigned int a=0; a<2; a++) {
        for(unsigned int b=0; b<2; b++) {
            d2c_dqq[a][b].setZero(2,1);
            for(unsigned int l=0; l<2; l++) {
                for(unsigned int i=0; i<2; i++) {
                    for(unsigned int j=0; j<2; j++) {
                        d2c_dqq[a][b](l) +=
                            (d3m_dqqq[a][b][j](i,l) - d3m_dqqq[a][b][l](i,j)/2.)*qdot[i]*qdot[j];
                    }
                }
            }
        }
    }
    
    // Assumes m(a,b) = m(b,a)
    Eigen::Matrix<Number, 2, 1> d2c_dqdotqdot[2][2];
    for(unsigned int a=0; a<2; a++) {
        for(unsigned int b=0; b<2; b++) {
            for(unsigned int l=0; l<2; l++) {
                d2c_dqdotqdot[a][b](l) = (dm_dq[b](a,l) + dm_dq[a](b,l) - dm_dq[l](a,b));
            }
        }
    }
    
    Eigen::Matrix<Number, 2, 1> d2c_dq_dqdot[2][2];
    for(unsigned int a=0; a<2; a++) {
        for(unsigned int b=0; b<2; b++) {
            d2c_dq_dqdot[a][b].setZero(2,1);
            for(unsigned int l=0; l<2; l++) {
                for(unsigned int j=0; j<2; j++) {
                    d2c_dq_dqdot[a][b](l) += (d2m_dqq[a][j](b,l) - d2m_dqq[a][l](b,j)/2.) * qdot[j];
                }
                for(unsigned int i=0; i<2; i++) {
                    d2c_dq_dqdot[a][b](l) += (d2m_dqq[a][b](i,l) - d2m_dqq[a][l](i,b)/2.) * qdot[i];
                }
            }
        }
    }
    
    Eigen::Matrix<Number, 2, 1> b;
    b << 0, u;
    
    // Helper g and derivatives
    Eigen::Matrix<Number, 2, 1> g = b - c - dV_dq;
    
    Eigen::Matrix<Number, 2, 1> dg_dq[2];
    for(unsigned int a=0; a<2; a++) {
        dg_dq[a] = -dc_dq[a] - d2V_dqq[a];
    }
    
    Eigen::Matrix<Number, 2, 1> d2g_dqq[2][2];
    for(unsigned int a=0; a<2; a++) {
        for(unsigned int b=0; b<2; b++) {
            d2g_dqq[a][b] = -d2c_dqq[a][b] - d3V_dqqq[a][b];
        }
    }
    
    DResult result;
    result.phiddot = mi * (b - c - dV_dq);
    
    // df_dq
    result.dphiddot[0] = dmi_dq[0] * g + mi * dg_dq[0];
    result.dphiddot[1] = dmi_dq[1] * g + mi * dg_dq[1];
    
    // df_qdot
    result.dphiddot[2] = mi * (-dc_dqdot[0]);
    result.dphiddot[3] = mi * (-dc_dqdot[1]);
    
    // df_du
    Eigen::Matrix<Number, 2, 1> bp;
    bp << 0, 1;
    
    result.dphiddot[4] = mi * bp;
    
    // d2f_dqq
    for(unsigned int a=0; a<2; a++) {
        for(unsigned int b=0; b<2; b++) {
            result.ddphiddot[a][b] = d2mi_dqq[a][b] * g + dmi_dq[a] * dg_dq[b] 
                                    + dmi_dq[b] * dg_dq[a] + mi * d2g_dqq[a][b];
        }
    }
    
    // d2f_dqdotqdot
    for(unsigned int a=0; a<2; a++) {
        for(unsigned int b=0; b<2; b++) {
            result.ddphiddot[a+2][b+2] = -mi * d2c_dqdotqdot[a][b];
        }
    }
    
    // d2f_duu
    result.ddphiddot[4][4] << 0., 0.;
    
    // d2f_dq_dqdot
    for(unsigned int a=0; a<2; a++) {
        for(unsigned int b=0; b<2; b++) {
            result.ddphiddot[a][b+2] = -dmi_dq[a] * dc_dqdot[b] - mi * d2c_dq_dqdot[a][b];
            result.ddphiddot[b+2][a] = result.ddphiddot[a][b+2];
        }
    }
    
    // d2f_dq_du
    result.ddphiddot[0][4] = dmi_dq[0] * bp;
    result.ddphiddot[1][4] = dmi_dq[1] * bp;
    
    result.ddphiddot[4][0] = result.ddphiddot[0][4];
    result.ddphiddot[4][1] = result.ddphiddot[1][4];
    
    // d2f_dqdot_du
    result.ddphiddot[2][4] << 0., 0.;
    result.ddphiddot[3][4] << 0., 0.;
    
    result.ddphiddot[4][2] = result.ddphiddot[2][4];
    result.ddphiddot[4][3] = result.ddphiddot[3][4];
    
    return result;
}

Eigen::Matrix<Number, 2, 2> dphiddot_dq(const Vector2N& q, const Vector2N& qdot, Number u)
{
    return dphiddot(q, qdot, u).block<2,2>(0,0);
}

Eigen::Matrix<Number, 2, 2> dphiddot_dqdot(const Vector2N& q, const Vector2N& qdot, Number u)
{
    return dphiddot(q, qdot, u).block<2,2>(0,2);
}

Vector2N dphiddot_du(const Vector2N& q, const Vector2N& qdot, Number u)
{
    return dphiddot(q, qdot, u).block<2,1>(0,4);
}

Vector2N phiddot_a(const Eigen::Matrix<Number, 5, 1>& arg)
{
    // return phiddot(arg.block<2,1>(0,0), arg.block<2,1>(2,0), arg(4,0));
    DResult result = phiddot2(arg.block<2,1>(0,0), arg.block<2,1>(2,0), arg(4,0));
    
    return result.phiddot;
}

Eigen::Matrix<Number, 2, 5> dphiddot_a(const Eigen::Matrix<Number, 5, 1>& arg)
{
    // return dphiddot(arg.block<2,1>(0,0), arg.block<2,1>(2,0), arg(4,0));
    
    DResult dresult = phiddot2(arg.block<2,1>(0,0), arg.block<2,1>(2,0), arg(4,0));
    
    Eigen::Matrix<Number, 2, 5> result;
    result << dresult.dphiddot[0], dresult.dphiddot[1], dresult.dphiddot[2], dresult.dphiddot[3], dresult.dphiddot[4];
    
    return result;
}

Eigen::Matrix<Number, 5, 5> ddphiddot_a_1(const Eigen::Matrix<Number, 5, 1>& arg)
{
    DResult dresult = phiddot2(arg.block<2,1>(0,0), arg.block<2,1>(2,0), arg(4,0));
    
    Eigen::Matrix<Number, 5, 5> result;
    for(unsigned int a=0; a<5; a++) {
        for(unsigned int b=0; b<5; b++) {
            result(a,b) = dresult.ddphiddot[a][b](0);
        }
    }
    
    return result;
}

Eigen::Matrix<Number, 5, 5> ddphiddot_a_2(const Eigen::Matrix<Number, 5, 1>& arg)
{
    DResult dresult = phiddot2(arg.block<2,1>(0,0), arg.block<2,1>(2,0), arg(4,0));
    
    Eigen::Matrix<Number, 5, 5> result;
    for(unsigned int a=0; a<5; a++) {
        for(unsigned int b=0; b<5; b++) {
            result(a,b) = dresult.ddphiddot[a][b](1);
        }
    }
    
    return result;
}

void Acrobot::dphiddot_test()
{
    Vector2N q0;
    q0 << 1.4, 0.5;
    Vector2N qdot0;
    qdot0 << .3, .9;
    Number u0;
    u0 = 1.1;
    
    Eigen::Matrix<Number, 5, 1> x0;
    x0 << q0, qdot0, u0;
    
    std::cout << phiddot_a(x0).transpose() << std::endl;
    std::cout << -4.58464 << " " << 38.6513 << std::endl;
    std::cout << std::endl;
    
    {
        const double h = 1e-6;
        Eigen::Matrix<Number, 2, 5> numeric;
        for(unsigned int i=0; i<5; i++) {
            Eigen::Matrix<Number,5,1> he = h * Eigen::Matrix<Number,5,1>::Unit(5,i);
            numeric.block<2,1>(0,i) = (phiddot_a(x0 + he) - phiddot_a(x0 - he))/(2*h);
        }
        
        Eigen::Matrix<Number, 2, 5> analytic = dphiddot_a(x0);
        
        std::cout << numeric << std::endl;
        std::cout << analytic << std::endl;
        std::cout << "==> " << (numeric - analytic).cwiseAbs().maxCoeff() << std::endl;
    }
    
    {
        const double h = 1e-4;
        Eigen::Matrix<Number, 5, 5> numeric1;
        Eigen::Matrix<Number, 5, 5> numeric2;
        for(unsigned int i=0; i<5; i++) {
            Eigen::Matrix<Number,5,1> he = h * Eigen::Matrix<Number,5,1>::Unit(5,i);
            numeric1.block<1,5>(i,0)
                = (dphiddot_a(x0 + he) - dphiddot_a(x0 - he)).block<1,5>(0,0)/(2*h);
            numeric2.block<1,5>(i,0)
                = (dphiddot_a(x0 + he) - dphiddot_a(x0 - he)).block<1,5>(1,0)/(2*h);
        }
        
        std::cout << std::endl;
        std::cout << numeric1 << "\n" << std::endl;
        std::cout << ddphiddot_a_1(x0) << "\n" << std::endl;
        std::cout << numeric2 << "\n" << std::endl;
        std::cout << ddphiddot_a_2(x0) << std::endl;
        
        std::cout << "==> " << (numeric1 - ddphiddot_a_1(x0)).cwiseAbs().maxCoeff() << std::endl;
        std::cout << "==> " << (numeric2 - ddphiddot_a_2(x0)).cwiseAbs().maxCoeff() << std::endl;
    }
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
        
        //for(Index j=0; j<N_dof; j++)
        //    g[idx++] = q_c.dot(q_c) + qdot_c.dot(qdot_c) + u_c*u_c;
        
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
            
            // modified qdot constraints
            /* for(Index j=0; j<N_dof; j++) {
                    values[idx++] = q_c[0] - 2 * qdot_c[0] * 3./2./dt;
                    values[idx++] = q_c[0] + 2 * qdot_c[0] * 3./2./dt;
                    values[idx++] = 2 * q_c[0] * dt/8. - 2 * qdot_c[0] * 1./4.;
                    values[idx++] = 2 * q_c[0] * -dt/8. - 2 * qdot_c[0] * 1./4.;
                    
                    values[idx++] = 2 * q_c[1] * 1./2. - 2 * qdot_c[1] * 3./2./dt;
                    values[idx++] = 2 * q_c[1] * 1./2. + 2 * qdot_c[1] * 3./2./dt;
                    values[idx++] = 2 * q_c[1] * dt/8. - 2 * qdot_c[1] * 1./4.;
                    values[idx++] = 2 * q_c[1] * -dt/8. - 2 * qdot_c[1] * 1./4.;
                    
                    values[idx++] = u_c;
                    values[idx++] = u_c;
            } */
            
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
    if(values == 0) {
        Index idx = 0;
        
        for(Index pt_idx=0; pt_idx<=N_points; pt_idx++) {
            iRow[idx] = index_of_q(pt_idx)+0; jCol[idx] = index_of_q(pt_idx)+0; idx++;
            
            iRow[idx] = index_of_q(pt_idx)+1; jCol[idx] = index_of_q(pt_idx)+0; idx++;
            iRow[idx] = index_of_q(pt_idx)+1; jCol[idx] = index_of_q(pt_idx)+1; idx++;
            
            iRow[idx] = index_of_v(pt_idx)+0; jCol[idx] = index_of_q(pt_idx)+0; idx++;
            iRow[idx] = index_of_v(pt_idx)+0; jCol[idx] = index_of_q(pt_idx)+1; idx++;
            iRow[idx] = index_of_v(pt_idx)+0; jCol[idx] = index_of_v(pt_idx)+0; idx++;
            
            iRow[idx] = index_of_v(pt_idx)+1; jCol[idx] = index_of_q(pt_idx)+0; idx++;
            iRow[idx] = index_of_v(pt_idx)+1; jCol[idx] = index_of_q(pt_idx)+1; idx++;
            iRow[idx] = index_of_v(pt_idx)+1; jCol[idx] = index_of_v(pt_idx)+0; idx++;
            iRow[idx] = index_of_v(pt_idx)+1; jCol[idx] = index_of_v(pt_idx)+1; idx++;
            
            iRow[idx] = index_of_u(pt_idx)+0; jCol[idx] = index_of_q(pt_idx)+0; idx++;
            iRow[idx] = index_of_u(pt_idx)+0; jCol[idx] = index_of_q(pt_idx)+1; idx++;
            iRow[idx] = index_of_u(pt_idx)+0; jCol[idx] = index_of_v(pt_idx)+0; idx++;
            iRow[idx] = index_of_u(pt_idx)+0; jCol[idx] = index_of_v(pt_idx)+1; idx++;
            iRow[idx] = index_of_u(pt_idx)+0; jCol[idx] = index_of_u(pt_idx)+0; idx++;
            
            if(pt_idx < N_points) {
                iRow[idx] = index_of_q(pt_idx)+0; jCol[idx] = index_of_q(pt_idx+1)+0; idx++;
                iRow[idx] = index_of_q(pt_idx)+0; jCol[idx] = index_of_q(pt_idx+1)+1; idx++;
                iRow[idx] = index_of_q(pt_idx)+0; jCol[idx] = index_of_v(pt_idx+1)+0; idx++;
                iRow[idx] = index_of_q(pt_idx)+0; jCol[idx] = index_of_v(pt_idx+1)+1; idx++;
                iRow[idx] = index_of_q(pt_idx)+0; jCol[idx] = index_of_u(pt_idx+1)+0; idx++;
                
                iRow[idx] = index_of_q(pt_idx)+1; jCol[idx] = index_of_q(pt_idx+1)+0; idx++;
                iRow[idx] = index_of_q(pt_idx)+1; jCol[idx] = index_of_q(pt_idx+1)+1; idx++;
                iRow[idx] = index_of_q(pt_idx)+1; jCol[idx] = index_of_v(pt_idx+1)+0; idx++;
                iRow[idx] = index_of_q(pt_idx)+1; jCol[idx] = index_of_v(pt_idx+1)+1; idx++;
                iRow[idx] = index_of_q(pt_idx)+1; jCol[idx] = index_of_u(pt_idx+1)+0; idx++;
                
                iRow[idx] = index_of_v(pt_idx)+0; jCol[idx] = index_of_q(pt_idx+1)+0; idx++;
                iRow[idx] = index_of_v(pt_idx)+0; jCol[idx] = index_of_q(pt_idx+1)+1; idx++;
                iRow[idx] = index_of_v(pt_idx)+0; jCol[idx] = index_of_v(pt_idx+1)+0; idx++;
                iRow[idx] = index_of_v(pt_idx)+0; jCol[idx] = index_of_v(pt_idx+1)+1; idx++;
                iRow[idx] = index_of_v(pt_idx)+0; jCol[idx] = index_of_u(pt_idx+1)+0; idx++;
                
                iRow[idx] = index_of_v(pt_idx)+1; jCol[idx] = index_of_q(pt_idx+1)+0; idx++;
                iRow[idx] = index_of_v(pt_idx)+1; jCol[idx] = index_of_q(pt_idx+1)+1; idx++;
                iRow[idx] = index_of_v(pt_idx)+1; jCol[idx] = index_of_v(pt_idx+1)+0; idx++;
                iRow[idx] = index_of_v(pt_idx)+1; jCol[idx] = index_of_v(pt_idx+1)+1; idx++;
                iRow[idx] = index_of_v(pt_idx)+1; jCol[idx] = index_of_u(pt_idx+1)+0; idx++;
                
                iRow[idx] = index_of_u(pt_idx)+0; jCol[idx] = index_of_q(pt_idx+1)+0; idx++;
                iRow[idx] = index_of_u(pt_idx)+0; jCol[idx] = index_of_q(pt_idx+1)+1; idx++;
                iRow[idx] = index_of_u(pt_idx)+0; jCol[idx] = index_of_v(pt_idx+1)+0; idx++;
                iRow[idx] = index_of_u(pt_idx)+0; jCol[idx] = index_of_v(pt_idx+1)+1; idx++;
                iRow[idx] = index_of_u(pt_idx)+0; jCol[idx] = index_of_u(pt_idx+1)+0; idx++;
            }
        }
        
        assert(idx == N_nnz_h_lag);
    } else {
        Index idx;
        
        Eigen::Matrix<double, 5, 10> C;
        /* C << 1./2., 0., dt/8., 0., 0., 1./2., 0., -dt/8., 0., 0.,
             0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
             0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
             0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
             0., 0., 0., 0., 0., 0., 0., 0., 0., 0.; */
             
        C << 1./2., 0., dt/8., 0., 0., 1./2., 0., -dt/8., 0., 0.,
             0., 1./2., 0., dt/8., 0., 0., 1./2., 0., -dt/8., 0.,
             -3./(2.*dt), 0., -1./4., 0., 0., 3./(2.*dt), 0., -1./4., 0., 0.,
             0., -3./(2.*dt), 0., -1./4., 0., 0., 3./(2.*dt), 0., -1./4., 0.,
             0., 0., 0., 0., 1./2., 0., 0., 0., 0., 1./2.;
        
        // Init Hessian to zero
        idx = 0;
        for(Index pt_idx=0; pt_idx<N_points; pt_idx++) {
            for(Index i=0; i<40; i++)
                values[idx++] = 0;
        }
        for(Index i=0; i<15; i++)
            values[idx++] = 0;
        assert(idx == N_nnz_h_lag);
        
        for(Index con_idx=0; con_idx<N_points; con_idx++) {
            const Eigen::Map<const Vector2N> q_i(  &x[index_of_q(con_idx)]   );
            const Eigen::Map<const Vector2N> q_i1( &x[index_of_q(con_idx+1)] );
            const Eigen::Map<const Vector2N> v_i(  &x[index_of_v(con_idx)]   );
            const Eigen::Map<const Vector2N> v_i1( &x[index_of_v(con_idx+1)] );
            
            const Number u_i  = x[index_of_u(con_idx)];
            const Number u_i1 = x[index_of_u(con_idx+1)];
            
            const Vector2N q_c = (q_i + q_i1)/2. + (dt/8.)*(v_i - v_i1);
            const Vector2N qdot_c = -3./(2.*dt)*(q_i - q_i1) - (v_i + v_i1)/4.;
            const Number u_c = (u_i + u_i1)/2.;
            
            DResult result_i = phiddot2(q_i, v_i, u_i);
            DResult result_i1 = phiddot2(q_i1, v_i1, u_i1);
            DResult result_c = phiddot2(q_c, qdot_c, u_c);
            
            // q: DoF 0
            idx = 40*con_idx;
            for(Index j=0; j<5; j++) {
                for(Index k=0; k<=j; k++) {
                    values[idx++] += lambda[5*con_idx] * (dt/8.) * result_i.ddphiddot[j][k][0];
                }
            }
            idx += 25; // jump over cross terms
            for(Index j=0; j<5; j++) {
                for(Index k=0; k<=j; k++) {
                    values[idx++] -= lambda[5*con_idx] * (dt/8.) * result_i1.ddphiddot[j][k][0];
                }
            }
            
            // q: DoF 1
            idx = 40*con_idx;
            for(Index j=0; j<5; j++) {
                for(Index k=0; k<=j; k++) {
                    values[idx++] += lambda[5*con_idx+1] * (dt/8.) * result_i.ddphiddot[j][k][1];
                }
            }
            idx += 25; // jump over cross terms
            for(Index j=0; j<5; j++) {
                for(Index k=0; k<=j; k++) {
                    values[idx++] -= lambda[5*con_idx+1] * (dt/8.) * result_i1.ddphiddot[j][k][1];
                }
            }
            
            // qdot: DoF 0
            idx = 40*con_idx;
            for(Index j=0; j<5; j++) {
                for(Index k=0; k<=j; k++) {
                    values[idx++] += lambda[5*con_idx+2] * (1./4.) * result_i.ddphiddot[j][k][0];
                }
            }
            idx += 25; // jump over cross terms
            for(Index j=0; j<5; j++) {
                for(Index k=0; k<=j; k++) {
                    values[idx++] += lambda[5*con_idx+2] * (1./4.) * result_i1.ddphiddot[j][k][0];
                }
            }
            idx = 40*con_idx;
            {
                Eigen::Matrix<double, 5, 5> ddphiddot;
                for(Index j=0; j<5; j++) {
                    for(Index k=0; k<5; k++) {
                        ddphiddot(j,k) = result_c.ddphiddot[j][k][0];
                        //double djk = (j == k ? 1. : 0.);
                        //ddphiddot(j,k) = 2. * djk;
                    }
                }
                Eigen::Matrix<double, 10, 10> ddx = C.transpose() * ddphiddot * C;
                /* ddx << 1./2., 0., dt/8., 0., 0., 1./2., 0., -dt/8., 0., 0.,
                       0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                       dt/8., 0., dt*dt/32., 0., 0., dt/8., 0., -dt*dt/32., 0., 0.,
                       0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                       0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                       1./2., 0., dt/8., 0., 0., 1./2., 0., -dt/8., 0., 0.,
                       0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                       -dt/8., 0., -dt*dt/32., 0., 0., -dt/8., 0., dt*dt/32., 0., 0.,
                       0., 0., 0., 0., 0., 0., 0., 0., 0., 0.,
                       0., 0., 0., 0., 0., 0., 0., 0., 0., 0.; */
                
                // Copy it
                for(Index j=0; j<5; j++) {
                    for(Index k=0; k<=j; k++) {
                        values[idx++] += lambda[5*con_idx+2] * ddx(j,k);
                    }
                }
                for(Index j=0; j<5; j++) {
                    for(Index k=0; k<5; k++) {
                        // FIXME: figure out why it is (j,5+k) and not (5+j,k)
                        values[idx++] += lambda[5*con_idx+2] * ddx(j,5+k);
                    }
                }
                for(Index j=0; j<5; j++) {
                    for(Index k=0; k<=j; k++) {
                        values[idx++] += lambda[5*con_idx+2] * ddx(5+j,5+k);
                    }
                }
            }
            
            // qdot: DoF 1
            idx = 40*con_idx;
            for(Index j=0; j<5; j++) {
                for(Index k=0; k<=j; k++) {
                    values[idx++] += lambda[5*con_idx+3] * (1./4.) * result_i.ddphiddot[j][k][1];
                }
            }
            idx += 25; // jump over cross terms
            for(Index j=0; j<5; j++) {
                for(Index k=0; k<=j; k++) {
                    values[idx++] += lambda[5*con_idx+3] * (1./4.) * result_i1.ddphiddot[j][k][1];
                }
            }
            idx = 40*con_idx;
            {
                Eigen::Matrix<double, 5, 5> ddphiddot;
                for(Index j=0; j<5; j++) {
                    for(Index k=0; k<5; k++) {
                        ddphiddot(j,k) = result_c.ddphiddot[j][k][1];
                        //double djk = (j == k ? 1. : 0.);
                        //ddphiddot(j,k) = 2. * djk;
                    }
                }
                Eigen::Matrix<double, 10, 10> ddx = C.transpose() * ddphiddot * C;
                
                // Copy it
                for(Index j=0; j<5; j++) {
                    for(Index k=0; k<=j; k++) {
                        values[idx++] += lambda[5*con_idx+3] * ddx(j,k);
                    }
                }
                for(Index j=0; j<5; j++) {
                    for(Index k=0; k<5; k++) {
                        values[idx++] += lambda[5*con_idx+3] * ddx(j,5+k);
                    }
                }
                for(Index j=0; j<5; j++) {
                    for(Index k=0; k<=j; k++) {
                        values[idx++] += lambda[5*con_idx+3] * ddx(5+j,5+k);
                    }
                }
            }
            
            // control integration
            idx = 40*con_idx;
            values[idx+14] += lambda[5*con_idx+4] * 1./2.;  // (u_i, u_i)
            values[idx+39] += lambda[5*con_idx+4] * 1./4.;  // (u_i, u_i1)
            values[idx+54] += lambda[5*con_idx+4] * 1./2.;  // (u_i1, u_i1)
        }
        
        /* Index idx = 0;
        
        for(Index i=0; i<N_points; i++) {
            for(Index j=0; j<N_dof; j++) {
                
                for(Index k=0; k<N_dof; k++) {
                    values[idx++] = 0.;
                    values[idx++] = 0.;
                    values[idx++] = 0.;
                    values[idx++] = 0.;
                }
                for(Index k=0; k<N_controls; k++) {
                    values[idx++] = 0.;
                    values[idx++] = 0.;
                }
            }
            for(Index j=0; j<N_controls; j++) {
                for(Index k=0; k<N_dof; k++) {
                    values[idx++] = 0.;
                    values[idx++] = 0.;
                }
                for(Index k=0; k<N_controls; k++) {
                    values[idx++] = 0.;
                }
            }
        }
        
        assert(idx == N_nnz_h_lag); */
        
        /* Index idx = 0;
        
        for(Index pt_idx = 0; pt_idx < N_points; pt_idx++) {
            const Eigen::Map<const Vector2N> q_i(  &x[index_of_q(pt_idx)]   );
            //const Eigen::Map<const Vector2N> q_i1( &x[index_of_q(pt_idx+1)] );
            const Eigen::Map<const Vector2N> v_i(  &x[index_of_v(pt_idx)]   );
            //const Eigen::Map<const Vector2N> v_i1( &x[index_of_v(pt_idx+1)] );
            
            const Number u_i  = x[index_of_u(pt_idx)];
            //const Number u_i1 = x[index_of_u(pt_idx+1)];
            
            DResult result_i = phiddot2(q_i, v_i, u_i);
            //DResult result_i1 = phiddot2(q_i1, v_i1, u_i1);
            
            // q constraints
            for(Index dof_idx = 0; dof_idx<N_dof; dof_idx++) {
                // g[idx++] = (v_c - qdot_c)[j];
                
                for(Index j=0; j<(N_dof+N_constraints); j++) {
                    for(Index k=0; k<(N_dof+N_constraints); k++) {
                        values[idx++] = result_i.ddphiddot[j][k][idx];
                        values[idx++] = 0.;
                        values[idx++] = 0.;
                        values[idx++] = 0.;
                    }
                    for(Index k=0; k<N_controls; k++) {
                        values[idx++] = 0.;
                        values[idx++] = 0.;
                    }
                }
                for(Index j=0; j<N_controls; j++) {
                    for(Index k=0; k<N_dof; k++) {
                        values[idx++] = 0.;
                        values[idx++] = 0.;
                    }
                    for(Index k=0; k<N_controls; k++) {
                        values[idx++] = 0.;
                    }
                }
            }
        } */
        
    }
    
    return true;
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
