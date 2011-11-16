#include "Brachistochrone.h"
#include <iostream>
#include <cmath>

const double Brachistochrone::x_F = 1.; // end x
const double Brachistochrone::H = 0.5;   // end height

const int Brachistochrone::N_points = 1000;  // number of free points
// x distance between points
const double Brachistochrone::k = x_F/(N_points+1);

Number Brachistochrone::ft(Number ym, Number yp)
{
    return 2.*sqrt(k*k+(yp-ym)*(yp-ym))/(sqrt(yp) + sqrt(ym));
}

Number Brachistochrone::fT(const Number* y)
{
    Number T;
    T = ft(0, y[0]);
    for(int i=0; i<N_points-1; i++) {
        T += ft(y[i], y[i+1]);
    }
    T += ft(y[N_points-1], H);
    return T;
}

// Derivative of t with respect to the second argument
Number Brachistochrone::dt_dp(Number ym, Number yp)
{
    Number l = sqrt(k*k + (yp-ym)*(yp-ym));
    Number v = (sqrt(yp)+sqrt(ym))/2.;
    return (yp-ym)/(v*l) - l/(4*sqrt(yp)*v*v);
}

Number Brachistochrone::ddt_dpdp(Number ym, Number yp)
{
    Number l = sqrt(k*k + (yp-ym)*(yp-ym));
    Number v = (sqrt(yp) + sqrt(ym))/2.;
    
    return 1./(v*l)
        - ((yp-ym)/(v*l*l*l) + 1./(4.*sqrt(yp)*v*v*l))*(yp-ym)
        - 1./4.*(-l/(2.*v*v*pow(yp,1.5))+((yp-ym)/(l*v*v)-l/(2*v*v*v*sqrt(yp)))/sqrt(yp));
}

Number Brachistochrone::ddt_dmdp(Number ym, Number yp)
{
    Number l = sqrt(k*k + (yp-ym)*(yp-ym));
    Number v = (sqrt(yp) + sqrt(ym))/2.;
    
    return -1./(v*l)
        - ((ym-yp)/(v*l*l*l) + 1./(4.*sqrt(ym)*v*v*l))*(yp-ym)
        - 1./(4.*sqrt(yp))*((ym-yp)/(l*v*v)-l/(2*v*v*v*sqrt(ym)));
}

adouble Brachistochrone::a_t(adouble ym, adouble yp)
{
    // Note symmetry between ym and yp!
    // Note: g=1/2 (i.e. sqrt(2*g) = 1)
    return 2.*sqrt(k*k+(yp-ym)*(yp-ym))/(sqrt(yp) + sqrt(ym));
    //return -sqrt(2.*(1.+a_i*a_i)) * 1./a_i * (sqrt(-y_i-a_i*k) - sqrt(-y_i));
    //return sqrt(k*k + (y_i1-y_i)*(y_i1-y_i))/sqrt(-y_i-y_i1);
}

adouble Brachistochrone::a_T(adouble* y)
{
    adouble T;
    T = a_t(0, y[0]);
    for(int i=0; i<N_points-1; i++) {
        T += a_t(y[i], y[i+1]);
    }
    T += a_t(y[N_points-1], H);
    return T;
}

Brachistochrone::Brachistochrone()
    : Ipopt::TNLP()
{
    adouble y[N_points];
    adouble T;
    double t;
    
    trace_on(1);
    for(int i=0; i<N_points; i++) y[i] <<= 1.;
    T = a_T(y);
    T >>= t;
    trace_off();
}

bool Brachistochrone::get_nlp_info(Index& n, Index& m, Index& nnz_jac_g,
        Index& nnz_h_lag, IndexStyleEnum& index_style)
{
    n = N_points;  // Number of variables
    m = 0;         // No constraints
    nnz_jac_g = 0;  // Number of nonzero entries in the Jacobian
    // Number of effective nonzero entries in the Hessian (16 in total, but we
    // only need the lower left corner).
    #ifdef DENSE_HESSIAN
    nnz_h_lag = N_points*(N_points+1)/2;
    #else
    nnz_h_lag = 2*N_points-1;
    #endif
    index_style = C_STYLE; // C or FORTRAN indexing
    
    return true;
}

bool Brachistochrone::get_bounds_info(Index n, Number* x_l, Number* x_u,
        Index m, Number* g_l, Number* g_u)
{
    assert(n == N_points);
    assert(m == 0);
    
    for(Index i=0; i<N_points; i++)
        x_l[i] = 0;
    
    for(Index i=0; i<N_points; i++)
        x_u[i] = 1e20;
    
    return true;
}

bool Brachistochrone::get_starting_point(Index n, bool init_x, Number* x,
        bool init_z, Number* z_L, Number* z_U,
        Index m, bool init_lambda, Number* lambda)
{
    // z and lambda initialization only needed for ``warm start''
    assert(init_x == true);
    assert(init_z == false);
    assert(init_lambda == false);
    
    for(int i=0; i<N_points; i++) x[i] = 0.;
    
    return true;
}

bool Brachistochrone::eval_f(Index n, const Number* x, bool new_x, Number& obj_value)
{
    assert(n == N_points);
    
    #ifdef USE_ADOLC_FOR_FUNC
    double result;
    double x_d[N_points];
    
    for(int i=0; i<N_points; i++) x_d[i] = x[i];
    int status = function(1, 1, N_points, x_d, &result);
    assert(status > 0);
    
    obj_value = result;
    #else
    obj_value = fT(x);
    #endif
    
    return true;
}

bool Brachistochrone::eval_grad_f(Index n, const Number* x, bool new_x, Number* grad_f)
{
    assert(n == N_points);
    
    #ifdef USE_ADOLC_FOR_GRAD
    double x_d[N_points];
    double grad_d[N_points];
    
    for(int i=0; i<N_points; i++) x_d[i] = x[i];
    int status = gradient(1, N_points, x_d, grad_d);
    assert(status > 0);
    for(int i=0; i<N_points; i++) grad_f[i] = grad_d[i];
    #else
    grad_f[0] = dt_dp(0., x[0]) + dt_dp(x[1], x[0]);
    for(int i=1; i<(N_points-1); i++)
        grad_f[i] = dt_dp(x[i-1], x[i]) + dt_dp(x[i+1], x[i]);
    grad_f[N_points-1] = dt_dp(x[N_points-2], x[N_points-1])
        + dt_dp(H, x[N_points-1]);
    #endif
    
    return true;
}

bool Brachistochrone::eval_g(Index n, const Number* x, bool new_x, Index m, Number* g)
{
    assert(n == N_points);
    assert(m == 0);
    
    return true;
}

bool Brachistochrone::eval_jac_g(Index n, const Number* x, bool new_x, Index m,
        Index nele_jac, Index* iRow, Index* jCol, Number* values)
{
    // No constraints
    
    return true;
}

#ifdef DENSE_HESSIAN
bool Brachistochrone::eval_h(Index n, const Number* x, bool new_x,
        Number obj_factor, Index m, const Number* lambda,
        bool new_lambda, Index nele_hess, Index* iRow, Index* jCol,
        Number* values)
{
    if(values == 0) {
        // Fill the lower left triangle only
        Index idx = 0;
        for(Index row=0; row<N_points; row++) {
            for(Index col=0; col<=row; col++) {
                iRow[idx] = row;
                jCol[idx] = col;
                idx++;
            }
        }
        
        assert(idx == nele_hess);
    } else {
        // Fill the lower left triangle only
        double x_d[N_points];
        double H[N_points][N_points];
        double* H_ptr[N_points];
        for(int i=0; i<N_points; i++) H_ptr[i] = H[i];
        for(int i=0; i<N_points; i++) x_d[i] = x[i];
        
        hessian(1, N_points, x_d, H_ptr);
        
        Index idx = 0;
        for(Index row=0; row<N_points; row++) {
            for(Index col=0; col<=row; col++) {
                values[idx] = obj_factor * H[row][col];
                idx++;
            }
        }
    }
    
    return true;
}
#else
bool Brachistochrone::eval_h(Index n, const Number* x, bool new_x,
        Number obj_factor, Index m, const Number* lambda,
        bool new_lambda, Index nele_hess, Index* iRow, Index* jCol,
        Number* values)
{
    if(values == 0) {
        // Fill the lower left triangle only
        Index idx = 0;
        iRow[idx] = 0; jCol[idx] = 0; idx++;
        for(Index row=1; row<N_points; row++) {
            iRow[idx] = row; jCol[idx] = row-1; idx++;
            iRow[idx] = row; jCol[idx] = row; idx++;
        }
        
        assert(idx == nele_hess);
    } else {
        // Fill the lower left triangle only
        #ifdef USE_ADOLC_FOR_HESSIAN
        double x_d[N_points];
        double H[N_points][N_points];
        double* H_ptr[N_points];
        for(int i=0; i<N_points; i++) H_ptr[i] = H[i];
        for(int i=0; i<N_points; i++) x_d[i] = x[i];
        
        hessian(1, N_points, x_d, H_ptr);
        
        Index idx = 0;
        values[idx] = obj_factor * H[0][0];
        idx++;
        for(Index row=1; row<N_points; row++) {
            values[idx] = H[row][row-1]; idx++;
            values[idx] = H[row][row]; idx++;
        }
        #else
        Index idx = 0;
        values[idx] = obj_factor * (ddt_dpdp(0., x[0])
            + ddt_dpdp(x[1], x[0]));
        idx++;
        for(Index row=1; row<(N_points-1); row++) {
            values[idx] = obj_factor * ddt_dmdp(x[row-1],x[row]);
            idx++;
            
            values[idx] = obj_factor*(ddt_dpdp(x[row-1], x[row])
                + ddt_dpdp(x[row+1], x[row]));
            idx++;
        }
        values[idx] = obj_factor * ddt_dmdp(x[N_points-2],x[N_points-1]);
        idx++;
        
        values[idx] = obj_factor * (ddt_dpdp(x[N_points-2], x[N_points-1])
            + ddt_dpdp(H, x[N_points-1]));
        idx++;
        #endif
        assert(idx == nele_hess);
    }
    
    return true;
}
#endif

void Brachistochrone::finalize_solution(Ipopt::SolverReturn status,
        Index n, const Number* x, const Number* z_L, const Number* z_U,
        Index m, const Number* g, const Number* lambda, Number obj_value,
        const Ipopt::IpoptData*, Ipopt::IpoptCalculatedQuantities*)
{
    std::cerr << "0. 0." << std::endl;
    for(Index i=0; i<n; i++)
        std::cerr << (i+1)*k << " " << -x[i] << std::endl;
    std::cerr << (N_points+1)*k << " " << -H << std::endl;
    
    /* for(Index i=0; i<n; i++)
        std::cout << "z_L[" << i << "] = " << z_L[i] << std::endl;
    
    for(Index i=0; i<n; i++)
        std::cout << "z_U[" << i << "] = " << z_U[i] << std::endl; */
    
    std::cout << "f(x*) = " << obj_value << std::endl;
}
