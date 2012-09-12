#ifndef INCL_ACROBOT_H
#define INCL_ACROBOT_H

#include <coin/IpTNLP.hpp>
#include <adolc/adolc.h>

using Ipopt::Index;
using Ipopt::Number;

class Acrobot: public Ipopt::TNLP
{
  public:
    Acrobot();
    
    void dphiddot_test();
    
    virtual bool get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
        Index& nnz_h_lag, IndexStyleEnum& index_style);
    virtual bool get_bounds_info(Index n, Number* x_l, Number* x_u,
        Index m, Number* g_l, Number* g_u);
    virtual bool get_starting_point(Index n, bool init_x, Number* x,
        bool init_z, Number* z_L, Number* z_U,
        Index m, bool init_lambda, Number* lambda);
    virtual bool eval_f(Index n, const Number* x, bool new_x, Number& obj_value);
    virtual bool eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f);
    virtual bool eval_g(Index n, const Number* x, bool new_x, Index m, Number* g);
    virtual bool eval_jac_g(Index n, const Number* x, bool new_x, Index m,
        Index nele_jac, Index* iRow, Index* jCol, Number* values);
    virtual bool eval_h(Index n, const Number* x, bool new_x,
        Number obj_factor, Index m, const Number* lambda,
        bool new_lambda, Index nele_hess, Index* iRow, Index* jCol,
        Number* values);
    virtual void finalize_solution(Ipopt::SolverReturn status,
        Index n, const Number* x, const Number* z_L, const Number* z_U,
        Index m, const Number* g, const Number* lambda, Number obj_value,
        const Ipopt::IpoptData*, Ipopt::IpoptCalculatedQuantities*);
  
  public:
    /* *** */
    static const Index N_dof;
    static const Index N_controls;
    
    static const Index N_points;
    static const Index N_vars;
    static const Index N_constraints;
    
    static const Index N_nnz_jac_g;
    static const Index N_nnz_h_lag;
    
    static const double dt;
};

#endif
