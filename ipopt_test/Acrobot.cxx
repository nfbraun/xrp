#include "Acrobot.h"
#include "CSVFile.h"
#include <iostream>
#include <fstream>
#include <cmath>
#include <stdexcept>
#include <Eigen/Dense>

const double Acrobot::t_end = 6.;   // end time [s]

// number of time intervals/nodes
const Index Acrobot::N_intervals = 500;
const Index Acrobot::N_nodes = N_intervals + 1;

// number of optimization variables
const Index Acrobot::N_vars_per_node = 2*N_dof + N_ctrl;
const Index Acrobot::N_vars = N_vars_per_node*N_nodes;

// number of constraints
const Index Acrobot::N_constraints = N_intervals*(2*N_dof) + 2*2*N_dof;

// time distance between nodes
const double Acrobot::dt = t_end/N_intervals;

const Index Acrobot::N_nnz_jac_g =
      N_intervals * N_dof * 2 * (2 * N_dof + N_ctrl)   // q constraints
    + N_intervals * N_dof * 2 * (2 * N_dof + N_ctrl)   // qdot constraints
    + 2*2*N_dof;                                       // (q,qdot) boundary constraints

typedef Eigen::Matrix<Number, N_dof, 1> Vector_Ndof;
typedef Eigen::Matrix<Number, N_ctrl, 1> Vector_Nctrl;

Index calc_N_nnz_h_lag()
{
    Index N_vars = 2*N_dof + N_ctrl;
    Index on_diag_block_size = N_vars * (N_vars+1) / 2;
    Index off_diag_block_size = N_vars * N_vars;
    
    return Acrobot::N_nodes*on_diag_block_size + (Acrobot::N_nodes-1)*off_diag_block_size;
}

const Index Acrobot::N_nnz_h_lag = calc_N_nnz_h_lag();

Index index_of_q(Index node, Index dof)
    { return N_dof * node + dof; }
Index index_of_v(Index node, Index dof)
    { return N_dof * Acrobot::N_nodes + N_dof * node + dof; }
Index index_of_u(Index node, Index ctrl)
    { return (2 * N_dof) * Acrobot::N_nodes + N_ctrl * node + ctrl; }

Acrobot::Acrobot()
    : Ipopt::TNLP()
{
    Q_qq << 2., 0., 0., 2.;
    Q_qqdot.setZero();
    Q_qdotqdot.setZero();
    
    if(N_ctrl == 1) {
        R << 200.;
    } else {
        R << 200., 0., 0., 200.;
    }
}

/* void Acrobot::constraint_test()
{
    Number* x = new Number[N_vars];
    Number* g = new Number[N_constraints];
    
    get_starting_point(0, true, x, false, 0, 0, 0, false, 0);
    
    eval_g(N_vars, x, true, N_constraints, g);
    
    delete[] g;
    delete[] x;
} */

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
    
    /* for(Index i=0; i<N_vars; i++)
        x_l[i] = -30.;
    
    for(Index i=0; i<N_vars; i++)
        x_u[i] = 30.; */
        
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

double GetInterpValue(const CSVFile& dataFile, int value_id, double t, int tidx)
{
    if(tidx < 1) return dataFile.data().at(0).at(value_id);
    if(tidx >= dataFile.data().size())
        return dataFile.data().at(dataFile.data().size()-1).at(value_id);
    
    double t0 = dataFile.data().at(tidx-1).at(0);
    double t1 = dataFile.data().at(tidx).at(0);
    double x0 = dataFile.data().at(tidx-1).at(value_id);
    double x1 = dataFile.data().at(tidx).at(value_id);
    
    return x0 + (x1-x0)/(t1-t0) * (t - t0);
}

bool Acrobot::get_starting_point(Index n, bool init_x, Number* x,
        bool init_z, Number* z_L, Number* z_U,
        Index m, bool init_lambda, Number* lambda)
{
    // z and lambda initialization only needed for ``warm start''
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);
    
    // for(int i=0; i<N_vars; i++) x[i] = 0.;
    
    /* CSVFile dataFile("acrobot_psopt.dat");
    
    if(dataFile.fail())
        throw std::runtime_error("Failed to open data: acrobot.dat"); */
    
    Index file_idx = 0;
    
    for(Index node_idx=0; node_idx<N_nodes; node_idx++) {
        double t = (double) node_idx * dt;
        //while((file_idx < dataFile.data().size())
        //    && dataFile.data().at(file_idx).at(0) < t) file_idx++;
        
        for(Index i=0; i<N_dof; i++)
            x[index_of_q(node_idx, i)] = 0.0;// GetInterpValue(dataFile, 1+i, t, file_idx);
        
        for(Index i=0; i<N_dof; i++)
            x[index_of_v(node_idx, i)] = 0.0; //GetInterpValue(dataFile, 1+N_dof+i, t, file_idx);
        
        for(Index i=0; i<N_ctrl; i++)
            x[index_of_u(node_idx, i)] = 0.0; //GetInterpValue(dataFile, 1+2*N_dof+i, t, file_idx);
    }
    
    return true;
}

bool Acrobot::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
    assert(n == N_vars);
    
    Number w = 0.;
    for(Index node_idx=0; node_idx<N_nodes; node_idx++) {
        const Eigen::Map<const Vector_Ndof>  q(  &x[index_of_q(node_idx, 0)]   );
        const Eigen::Map<const Vector_Nctrl> u(  &x[index_of_u(node_idx, 0)]   );
        
        w += (q.dot(Q_qq * q)/2. + u.dot(R * u)/2.)*dt;
    }
    
    obj_value = w;
    
    return true;
}

bool Acrobot::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
    assert(n == N_vars);
    
    for(Index node_idx=0; node_idx<N_nodes; node_idx++) {
        const Eigen::Map<const Vector_Ndof>  q(  &x[index_of_q(node_idx, 0)]   );
        const Eigen::Map<const Vector_Nctrl> u(  &x[index_of_u(node_idx, 0)]   );
        
        Eigen::Map<Vector_Ndof> df_dq(&grad_f[index_of_q(node_idx, 0)]);
        Eigen::Map<Vector_Ndof> df_dv(&grad_f[index_of_v(node_idx, 0)]);
        Eigen::Map<Vector_Nctrl> df_du(&grad_f[index_of_u(node_idx, 0)]);
        
        df_dq = Q_qq * q * dt;
        df_dv.setZero();
        df_du = R * u * dt;
    }
    
    return true;
}

bool Acrobot::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
    assert(n == N_vars);
    assert(m == N_constraints);
    
    // The ODEs to solve:
    //  phi_dot = omega
    //  omega_dot = f(phi, omega, u)
    
    Index idx = 0;
    
    for(Index i=0; i<N_intervals; i++) {
        const Eigen::Map<const Vector_Ndof>  q_i(  &x[index_of_q(i, 0)]   );
        const Eigen::Map<const Vector_Ndof>  q_i1( &x[index_of_q(i+1, 0)] );
        const Eigen::Map<const Vector_Ndof>  v_i(  &x[index_of_v(i, 0)]   );
        const Eigen::Map<const Vector_Ndof>  v_i1( &x[index_of_v(i+1, 0)] );
        
        const Eigen::Map<const Vector_Nctrl> u_i(  &x[index_of_u(i, 0)]   );
        const Eigen::Map<const Vector_Nctrl> u_i1( &x[index_of_u(i+1, 0)] );
        
        // FIXME: inefficient. We do not need derivatives here.
        AcroDyn::DResult result_i = AcroDyn::qddot_full(q_i, v_i, u_i);
        AcroDyn::DResult result_i1 = AcroDyn::qddot_full(q_i1, v_i1, u_i1);
        
        const Vector_Ndof q_c = (q_i + q_i1)/2. + (dt/8.)*(v_i - v_i1);
        const Vector_Ndof v_c = (v_i + v_i1)/2. + (dt/8.)*(result_i.f - result_i1.f);
        const Vector_Nctrl u_c = (u_i + u_i1)/2.;
        
        const Vector_Ndof qdot_c = -3./(2.*dt)*(q_i - q_i1) - (v_i + v_i1)/4.;
        const Vector_Ndof vdot_c = -3./(2.*dt)*(v_i - v_i1) - (result_i.f + result_i1.f)/4.;
        
        // FIXME: inefficient (see above)
        AcroDyn::DResult result_c = AcroDyn::qddot_full(q_c, qdot_c, u_c);
        
        // q constraints
        for(Index j=0; j<N_dof; j++)
            g[idx++] = (v_c - qdot_c)[j];
        
        // qdot constraints
        for(Index j=0; j<N_dof; j++)
            g[idx++] = (result_c.f - vdot_c)[j];
    }
    
    // q boundary constraints
    g[idx++] = x[index_of_q(0, 0)] - M_PI;
    g[idx++] = x[index_of_q(0, 1)];
    g[idx++] = x[index_of_q(N_intervals, 0)];
    g[idx++] = x[index_of_q(N_intervals, 1)];
    
    // qdot boundary constraints
    g[idx++] = x[index_of_v(0, 0)];
    g[idx++] = x[index_of_v(0, 1)];
    g[idx++] = x[index_of_v(N_intervals, 0)];
    g[idx++] = x[index_of_v(N_intervals, 1)];
    
    assert(idx == N_constraints);
    
    return true;
}

void Acrobot::dump_constraints(const char* fname, Index n, const Number* x)
{
    assert(n == N_vars);
    
    std::ofstream outfile(fname);
    
    Index col_idx = 1;
    for(Index j=0; j<N_dof; j++)
        outfile << "#:" << col_idx++ << ":q_" << j << "\n";
    for(Index j=0; j<N_dof; j++)
        outfile << "#:" << col_idx++ << ":qdot_" << j << "\n";
    outfile << "#:" << col_idx++ << ":cint" << std::endl;
    
    Index idx = 0;
    
    for(Index i=0; i<N_intervals; i++) {
        outfile << i*dt << " ";
        
        const Eigen::Map<const Vector_Ndof>  q_i(  &x[index_of_q(i, 0)]   );
        const Eigen::Map<const Vector_Ndof>  q_i1( &x[index_of_q(i+1, 0)] );
        const Eigen::Map<const Vector_Ndof>  v_i(  &x[index_of_v(i, 0)]   );
        const Eigen::Map<const Vector_Ndof>  v_i1( &x[index_of_v(i+1, 0)] );
        
        const Eigen::Map<const Vector_Nctrl> u_i(  &x[index_of_u(i, 0)]   );
        const Eigen::Map<const Vector_Nctrl> u_i1( &x[index_of_u(i+1, 0)] );
        
        // FIXME: inefficient. We do not need derivatives here.
        AcroDyn::DResult result_i = AcroDyn::qddot_full(q_i, v_i, u_i);
        AcroDyn::DResult result_i1 = AcroDyn::qddot_full(q_i1, v_i1, u_i1);
        
        const Vector_Ndof q_c = (q_i + q_i1)/2. + (dt/8.)*(v_i - v_i1);
        const Vector_Ndof v_c = (v_i + v_i1)/2. + (dt/8.)*(result_i.f - result_i1.f);
        const Vector_Nctrl u_c = (u_i + u_i1)/2.;
        
        const Vector_Ndof qdot_c = -3./(2.*dt)*(q_i - q_i1) - (v_i + v_i1)/4.;
        const Vector_Ndof vdot_c = -3./(2.*dt)*(v_i - v_i1) - (result_i.f + result_i1.f)/4.;
        
        // FIXME: inefficient (see above)
        AcroDyn::DResult result_c = AcroDyn::qddot_full(q_c, qdot_c, u_c);
        
        // q constraints
        for(Index j=0; j<N_dof; j++)
            outfile << (v_c - qdot_c)[j] << " ";
        
        // qdot constraints
        for(Index j=0; j<N_dof; j++)
            outfile << (result_c.f - vdot_c)[j] << " ";
        
        outfile << std::endl;
    }
}

bool Acrobot::eval_jac_g(Index n, const Number* x, bool new_x, Index m,
        Index nele_jac, Index* iRow, Index* jCol, Number* values)
{
    /* J_{ij} = \frac{d g_i}{d x_j} */
    
    if(values == 0) {
        Index idx = 0;
        Index g_idx = 0;
        for(Index i=0; i<N_intervals; i++) {
            // q constraints
            for(Index j=0; j<N_dof; j++) {
                for(Index k=0; k<N_dof; k++) {
                    iRow[idx] = g_idx; jCol[idx] = index_of_q(i, k); idx++;
                    iRow[idx] = g_idx; jCol[idx] = index_of_q(i+1, k); idx++;
                    iRow[idx] = g_idx; jCol[idx] = index_of_v(i, k); idx++;
                    iRow[idx] = g_idx; jCol[idx] = index_of_v(i+1, k); idx++;
                }
                for(Index k=0; k<N_ctrl; k++) {
                    iRow[idx] = g_idx; jCol[idx] = index_of_u(i, k); idx++;
                    iRow[idx] = g_idx; jCol[idx] = index_of_u(i+1, k); idx++;
                }
                g_idx++;
            }
            
            // qdot constraints
            for(Index j=0; j<N_dof; j++) {
                for(Index k=0; k<N_dof; k++) {
                    iRow[idx] = g_idx; jCol[idx] = index_of_q(i, k); idx++;
                    iRow[idx] = g_idx; jCol[idx] = index_of_q(i+1, k); idx++;
                    iRow[idx] = g_idx; jCol[idx] = index_of_v(i, k); idx++;
                    iRow[idx] = g_idx; jCol[idx] = index_of_v(i+1, k); idx++;
                }
                for(Index k=0; k<N_ctrl; k++) {
                    iRow[idx] = g_idx; jCol[idx] = index_of_u(i, k); idx++;
                    iRow[idx] = g_idx; jCol[idx] = index_of_u(i+1, k); idx++;
                }
                g_idx++;
            }
        }
        
        // q boundary constraints
        iRow[idx] = g_idx; jCol[idx] = index_of_q(0, 0); idx++; g_idx++;
        iRow[idx] = g_idx; jCol[idx] = index_of_q(0, 1); idx++; g_idx++;
        iRow[idx] = g_idx; jCol[idx] = index_of_q(N_intervals, 0); idx++; g_idx++;
        iRow[idx] = g_idx; jCol[idx] = index_of_q(N_intervals, 1); idx++; g_idx++;
        
        // qdot boundary constrains
        iRow[idx] = g_idx; jCol[idx] = index_of_v(0, 0); idx++; g_idx++;
        iRow[idx] = g_idx; jCol[idx] = index_of_v(0, 1); idx++; g_idx++;
        iRow[idx] = g_idx; jCol[idx] = index_of_v(N_intervals, 0); idx++; g_idx++;
        iRow[idx] = g_idx; jCol[idx] = index_of_v(N_intervals, 1); idx++; g_idx++;
        
        assert(g_idx == N_constraints);
        assert(idx == N_nnz_jac_g);
    } else {
        Index idx = 0;
        for(Index i=0; i<N_intervals; i++) {
            const Eigen::Map<const Vector_Ndof>  q_i(  &x[index_of_q(i, 0)]   );
            const Eigen::Map<const Vector_Ndof>  q_i1( &x[index_of_q(i+1, 0)] );
            const Eigen::Map<const Vector_Ndof>  v_i(  &x[index_of_v(i, 0)]   );
            const Eigen::Map<const Vector_Ndof>  v_i1( &x[index_of_v(i+1, 0)] );
            
            const Eigen::Map<const Vector_Nctrl> u_i(  &x[index_of_u(i, 0)]   );
            const Eigen::Map<const Vector_Nctrl> u_i1( &x[index_of_u(i+1, 0)] );
            
            // FIXME: inefficient. We do not need second derivatives here.
            AcroDyn::DResult result_i = AcroDyn::qddot_full(q_i, v_i, u_i);
            AcroDyn::DResult result_i1 = AcroDyn::qddot_full(q_i1, v_i1, u_i1);
            
            const Vector_Ndof q_c = (q_i + q_i1)/2. + (dt/8.)*(v_i - v_i1);
            const Vector_Ndof v_c = (v_i + v_i1)/2. + (dt/8.)*(result_i.f - result_i1.f);
            const Vector_Nctrl u_c = (u_i + u_i1)/2.;
            
            const Vector_Ndof qdot_c = -3./(2.*dt)*(q_i - q_i1) - (v_i + v_i1)/4.;
            
            // FIXME: inefficient (see above)
            AcroDyn::DResult result_c = AcroDyn::qddot_full(q_c, qdot_c, u_c);
            
            // q constraints
            for(Index j=0; j<N_dof; j++) {
                for(Index k=0; k<N_dof; k++) {
                    double djk = (j == k) ? 1 : 0;
                    values[idx++] = dt/8.*result_i.df[k](j) + 3./(2.*dt) * djk;
                    values[idx++] = -dt/8.*result_i1.df[k](j) - 3./(2.*dt) * djk;
                    values[idx++] = dt/8.*result_i.df[N_dof+k](j) + 3./4. * djk;
                    values[idx++] = -dt/8.*result_i1.df[N_dof+k](j) + 3./4. * djk;
                }
                for(Index k=0; k<N_ctrl; k++) {
                    values[idx++] = dt/8.*result_i.df[2*N_dof+k](j);
                    values[idx++] = -dt/8.*result_i1.df[2*N_dof+k](j);
                }
            }
            
            // qdot constraints
            for(Index j=0; j<N_dof; j++) {
                for(Index k=0; k<N_dof; k++) {
                    double djk = (j == k) ? 1 : 0;
                    values[idx++] = result_c.df[k](j)/2.
                        - 3./(2.*dt) * result_c.df[N_dof+k](j)
                        + result_i.df[k](j)/4.;
                    values[idx++] = result_c.df[k](j)/2.
                        + 3./(2.*dt) * result_c.df[N_dof+k](j)
                        + result_i1.df[k](j)/4.;
                    values[idx++] = result_c.df[k](j) * dt/8.
                                    - result_c.df[N_dof+k](j)/4.
                                    + result_i.df[N_dof+k](j)/4.
                                    + 3./(2.*dt) * djk;
                    values[idx++] = - result_c.df[k](j) * dt/8.
                                    - result_c.df[N_dof+k](j)/4.
                                    + result_i1.df[N_dof+k](j)/4.
                                    - 3./(2.*dt) * djk;
                }
                for(Index k=0; k<N_ctrl; k++) {
                    values[idx++] = result_c.df[2*N_dof+k](j)/2.
                        + result_i.df[2*N_dof+k](j)/4.;
                    values[idx++] = result_c.df[2*N_dof+k](j)/2.
                        + result_i1.df[2*N_dof+k](j)/4.;
                }
            }
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
        
        assert(idx == N_nnz_jac_g);
    }
    
    return true;
}

/* Intro on sparsity structure: each constraint involves variables from two
   neighbouring nodes (points in time).
   
   There are two kinds of blocks in the Hessian: on diagonal blocks, involving
   variables from node i and node i, and off diagonal blocks, involving
   variables from node i and node (i+1). All other blocks are zero.
   
   The Hessian is symmetric. Ipopt expects to see only non-redundant elements.
*/


bool Acrobot::eval_h(Index n, const Number* x, bool new_x,
        Number obj_factor, Index m, const Number* lambda,
        bool new_lambda, Index nele_hess, Index* iRow, Index* jCol,
        Number* values)
{
    const Index block_len = 2*N_dof + N_ctrl;
    const Index on_diag_block_size = block_len * (block_len+1) / 2;
    const Index off_diag_block_size = block_len * block_len;
    const Index con_stride = on_diag_block_size + off_diag_block_size;
    
    if(values == 0) {
        Index idx = 0;
        
        for(Index node_idx=0; node_idx < N_nodes; node_idx++) {
            // List of variables
            Index block_idx[block_len];
            {
                Index b_idx = 0;
                
                for(Index i=0; i<N_dof; i++) block_idx[b_idx++] = index_of_q(node_idx, i);
                for(Index i=0; i<N_dof; i++) block_idx[b_idx++] = index_of_v(node_idx, i);
                for(Index i=0; i<N_ctrl; i++) block_idx[b_idx++] = index_of_u(node_idx, i);
                
                assert(b_idx == block_len);
            }
            
            // Construct on-diagonal block
            for(Index i=0; i<block_len; i++) {
                for(Index j=0; j<=i; j++) {
                    iRow[idx] = block_idx[i];
                    jCol[idx] = block_idx[j];
                    idx++;
                }
            }
            
            if(node_idx < N_intervals) {
                Index next_block_idx[block_len];
                {
                    Index b_idx = 0;
                    
                    for(Index i=0; i<N_dof; i++) next_block_idx[b_idx++] = index_of_q(node_idx+1, i);
                    for(Index i=0; i<N_dof; i++) next_block_idx[b_idx++] = index_of_v(node_idx+1, i);
                    for(Index i=0; i<N_ctrl; i++) next_block_idx[b_idx++] = index_of_u(node_idx+1, i);
                    
                    assert(b_idx == block_len);
                }
                
                // Construct off-diagonal block
                for(Index i=0; i<block_len; i++) {
                    for(Index j=0; j<block_len; j++) {
                        iRow[idx] = block_idx[i];
                        jCol[idx] = next_block_idx[j];
                        idx++;
                    }
                }
            }
        }
        
        assert(idx == N_nnz_h_lag);
    } else {
        Index idx;
        
        /* The matrix centMap defines a linear map from
           (q_{i,0}, ..., q_{i,N_dof-1}, qdot_{i,0}, ..., qdot_{i,N_dof-1},
            u_{i,0}, ..., u_{i,N_ctrl-1},
            q_{i+1,0}, ..., q_{i+1,N_dof-1}, qdot_{i+1,0}, ..., qdot_{i+1,N_dof-1},
            u_{i+1,0}, ..., u_{i+1,N_ctrl-1})^T
           to
           (q_{c,0}, ..., q_{c,N_dof-1}, qdot_{c,0}, ..., qdot_{c,N_dof-1},
            u_{c,0}, ..., u_{c,N_ctrl-1})^T
        */
        Eigen::Matrix<double, 2*N_dof+N_ctrl, 2*(2*N_dof+N_ctrl)> centMap;
        centMap.setZero();
        
        for(Index i=0; i<N_dof; i++) {
            centMap(i, i) = 1./2.;
            centMap(i, N_dof+i) = dt/8.;
            centMap(i, 2*N_dof+N_ctrl+i) = 1./2.;
            centMap(i, 2*N_dof+N_ctrl+N_dof+i) = -dt/8.;
        }
        
        for(Index i=0; i<N_dof; i++) {
            centMap(N_dof+i, i) = -3./(2.*dt);
            centMap(N_dof+i, N_dof+i) = -1./4.;
            centMap(N_dof+i, 2*N_dof+N_ctrl+i) = 3./(2.*dt);
            centMap(N_dof+i, 2*N_dof+N_ctrl+N_dof+i) = -1./4.;
        }
        
        for(Index i=0; i<N_ctrl; i++) {
            centMap(2*N_dof+i, 2*N_dof+i) = 1./2.;
            centMap(2*N_dof+i, 2*N_dof+N_ctrl+2*N_dof+i) = 1./2.;
        }
        
        /* The matrix costQ defines a quadratic form for the cost,
           cost = (q, qdot, u) costQ (q, qdot, u)^T
         */
        Eigen::Matrix<double, 2*N_dof+N_ctrl, 2*N_dof+N_ctrl> costQ;
        costQ.setZero();
        costQ.block<N_dof,N_dof>(0, 0) = Q_qq;
        costQ.block<N_dof,N_dof>(0, N_dof) = Q_qqdot;
        costQ.block<N_dof,N_dof>(N_dof, 0) = Q_qqdot.transpose();
        costQ.block<N_dof,N_dof>(N_dof, N_dof) = Q_qdotqdot;
        costQ.block<N_ctrl,N_ctrl>(2*N_dof, 2*N_dof) = R;
        
        // Init Hessian to zero
        idx = 0;
        for(Index node_idx = 0; node_idx < (N_nodes-1); node_idx++) {
            for(Index i=0; i<con_stride; i++)
                values[idx++] = 0;
        }
        for(Index i=0; i<on_diag_block_size; i++)
            values[idx++] = 0;
        assert(idx == N_nnz_h_lag);
        
        for(Index node_idx=0; node_idx<N_nodes; node_idx++) {
            Index idx = node_idx*con_stride;
            for(Index i=0; i<block_len; i++) {
                for(Index j=0; j<=i; j++) {
                    values[idx++] = obj_factor*costQ(i, j)*dt;
                }
            }
        }
        
        for(Index con_idx=0; con_idx<N_intervals; con_idx++) {
            const Eigen::Map<const Vector_Ndof>  q_i(  &x[index_of_q(con_idx, 0)]   );
            const Eigen::Map<const Vector_Ndof>  q_i1( &x[index_of_q(con_idx+1, 0)] );
            const Eigen::Map<const Vector_Ndof>  v_i(  &x[index_of_v(con_idx, 0)]   );
            const Eigen::Map<const Vector_Ndof>  v_i1( &x[index_of_v(con_idx+1, 0)] );
            
            const Eigen::Map<const Vector_Nctrl> u_i(  &x[index_of_u(con_idx, 0)]   );
            const Eigen::Map<const Vector_Nctrl> u_i1( &x[index_of_u(con_idx+1, 0)] );
            
            const Vector_Ndof q_c = (q_i + q_i1)/2. + (dt/8.)*(v_i - v_i1);
            const Vector_Ndof qdot_c = -3./(2.*dt)*(q_i - q_i1) - (v_i + v_i1)/4.;
            const Vector_Nctrl u_c = (u_i + u_i1)/2.;
            
            AcroDyn::DResult result_i = AcroDyn::qddot_full(q_i, v_i, u_i);
            AcroDyn::DResult result_i1 = AcroDyn::qddot_full(q_i1, v_i1, u_i1);
            AcroDyn::DResult result_c = AcroDyn::qddot_full(q_c, qdot_c, u_c);
            
            // q constraints
            for(Index dof_idx=0; dof_idx < N_dof; dof_idx++) {
                idx = con_idx*con_stride;
                for(Index j=0; j<(2*N_dof+N_ctrl); j++) {
                    for(Index k=0; k<=j; k++) {
                        values[idx++] += lambda[2*N_dof*con_idx+dof_idx] * (dt/8.) * result_i.ddf[j][k][dof_idx];
                    }
                }
                idx += off_diag_block_size; // jump over cross terms
                for(Index j=0; j<(2*N_dof+N_ctrl); j++) {
                    for(Index k=0; k<=j; k++) {
                        values[idx++] -= lambda[2*N_dof*con_idx+dof_idx] * (dt/8.) * result_i1.ddf[j][k][dof_idx];
                    }
                }
            }
            
            // qdot constraints
            for(Index dof_idx=0; dof_idx < N_dof; dof_idx++) {
                idx = con_idx*con_stride;
                for(Index j=0; j<(2*N_dof+N_ctrl); j++) {
                    for(Index k=0; k<=j; k++) {
                        values[idx++] += lambda[2*N_dof*con_idx+N_dof+dof_idx] * (1./4.) * result_i.ddf[j][k][dof_idx];
                    }
                }
                idx += off_diag_block_size; // jump over cross terms
                for(Index j=0; j<(2*N_dof+N_ctrl); j++) {
                    for(Index k=0; k<=j; k++) {
                        values[idx++] += lambda[2*N_dof*con_idx+N_dof+dof_idx] * (1./4.) * result_i1.ddf[j][k][dof_idx];
                    }
                }
                idx = con_idx*con_stride;
                {
                    Eigen::Matrix<double, 2*N_dof+N_ctrl, 2*N_dof+N_ctrl> ddphiddot;
                    for(Index j=0; j<(2*N_dof+N_ctrl); j++) {
                        for(Index k=0; k<(2*N_dof+N_ctrl); k++) {
                            ddphiddot(j,k) = result_c.ddf[j][k][dof_idx];
                        }
                    }
                    Eigen::Matrix<double, 2*(2*N_dof+N_ctrl), 2*(2*N_dof+N_ctrl)> ddx =
                        centMap.transpose() * ddphiddot * centMap;
                    // Copy it
                    for(Index j=0; j<(2*N_dof+N_ctrl); j++) {
                        for(Index k=0; k<=j; k++) {
                            values[idx++] += lambda[2*N_dof*con_idx+N_dof+dof_idx] * ddx(j,k);
                        }
                    }
                    for(Index j=0; j<(2*N_dof+N_ctrl); j++) {
                        for(Index k=0; k<(2*N_dof+N_ctrl); k++) {
                            // FIXME: figure out why it is (j,5+k) and not (5+j,k)
                            values[idx++] += lambda[2*N_dof*con_idx+N_dof+dof_idx] * ddx(j,2*N_dof+N_ctrl+k);
                        }
                    }
                    for(Index j=0; j<(2*N_dof+N_ctrl); j++) {
                        for(Index k=0; k<=j; k++) {
                            values[idx++] += lambda[2*N_dof*con_idx+N_dof+dof_idx] * ddx(2*N_dof+N_ctrl+j, 2*N_dof+N_ctrl+k);
                        }
                    }
                }
            }
        }
    }
    
    return true;
}

void Acrobot::dump_variables(const char* fname, Index n, const Number* x)
{
    std::ofstream outfile(fname);
    
    Index col_idx = 1;
    for(Index j=0; j<N_dof; j++)
        outfile << "#:" << col_idx++ << ":q_" << j << "\n";
    for(Index j=0; j<N_dof; j++)
        outfile << "#:" << col_idx++ << ":qdot_" << j << "\n";
    for(Index j=0; j<N_ctrl; j++)
        outfile << "#:" << col_idx++ << ":u_" << j << "\n";
    outfile << "#:" << col_idx++ << ":cint" << std::endl;
    
    for(Index i=0; i<=N_intervals; i++) {
        outfile << (i*dt) << " ";
        for(Index j=0; j<N_dof; j++)
            outfile << x[index_of_q(i, j)] << " ";
        for(Index j=0; j<N_dof; j++)
            outfile << x[index_of_v(i, j)] << " ";
        for(Index j=0; j<N_ctrl; j++)
            outfile << x[index_of_u(i, j)] << " ";
        outfile << std::endl;
    }
}

void Acrobot::finalize_solution(Ipopt::SolverReturn status,
        Index n, const Number* x, const Number* z_L, const Number* z_U,
        Index m, const Number* g, const Number* lambda, Number obj_value,
        const Ipopt::IpoptData*, Ipopt::IpoptCalculatedQuantities*)
{
    std::cout << "f(x*) = " << obj_value << std::endl;
    
    dump_variables("output.dat", n, x);
    dump_constraints("constraints.dat", n, x);
}
