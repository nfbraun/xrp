/*
   M. Garcia /et al./: The Simplest Walking Model
*/

#include <stdio.h>
#include <string.h>
#include <math.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv.h>
#include <gsl/gsl_roots.h>
#include <gsl/gsl_permutation.h>
#include <gsl/gsl_eigen.h>
#include <gsl/gsl_deriv.h>
#include <gsl/gsl_linalg.h>

const double GAMMA = 0.004;

int walker_diff(double t, const double y[], double f[], void *_p)
{
    double thetadot = y[0];
    double phidot = y[1];
    double theta = y[2];
    double phi = y[3];
    
    double thetadotdot = sin(theta - GAMMA);
    double phidotdot = sin(phi) * (thetadot*thetadot - cos(theta - GAMMA))
        + sin(theta - GAMMA);

    f[0] = thetadotdot;
    f[1] = phidotdot;
    f[2] = thetadot;
    f[3] = phidot;
    
    return GSL_SUCCESS;
}

#define DIM 4
#define CACHE_SIZE 64

typedef struct {
    double t;
    double y[DIM];
} odepoint_t;

typedef struct {
    gsl_odeiv_step* s;
    gsl_odeiv_control* c;
    gsl_odeiv_evolve* e;
    gsl_odeiv_system sys;
    odepoint_t points[CACHE_SIZE];
    int cur_idx;
} odesolver_t;

void ode_alloc(odesolver_t* par)
{
    const gsl_odeiv_step_type *T = gsl_odeiv_step_rkf45;
    
    par->s = gsl_odeiv_step_alloc(T, DIM);
    par->c = gsl_odeiv_control_y_new(1e-12, 0.);
    par->e = gsl_odeiv_evolve_alloc(DIM);
    par->sys.function = walker_diff;
    par->sys.jacobian = NULL;
    par->sys.dimension = DIM;
    par->sys.params = NULL;
    
    par->points[0].t = 0.;
    par->points[0].y[0] = 0.;
    par->points[0].y[1] = 0.;
    par->points[0].y[2] = 0.;
    par->points[0].y[3] = 0.;
    par->cur_idx = 0;
}

void ode_free(odesolver_t* par)
{
    gsl_odeiv_evolve_free(par->e);
    gsl_odeiv_control_free(par->c);
    gsl_odeiv_step_free(par->s);
}

void leg_indicator_fdf(double t_end, void *_par, double* f, double* df)
{
    odesolver_t* par = (odesolver_t*) _par;
    
    // Safety check
    if(t_end < 0.) {
        fprintf(stderr, "Warning: value requested at t < 0!\n");
        *f = 0.; *df = 0.; return;
    }
    
    while(t_end < par->points[par->cur_idx].t)
        par->cur_idx--;
    
    double t = par->points[par->cur_idx].t;
    double h = 1e-6;
    
    double y[4];
    memcpy(y, par->points[par->cur_idx].y, sizeof(y));
    
    gsl_odeiv_evolve_reset(par->e);
    
    while(t < t_end)
        gsl_odeiv_evolve_apply(par->e, par->c, par->s, &par->sys, &t, t_end, &h, y);
    
    *f  = 2.*y[2] - y[3];
    *df = 2.*y[0] - y[1];
    
    par->cur_idx++;
    if(par->cur_idx >= CACHE_SIZE) {
        fprintf(stderr, "Warning: cache size too small!\n");
        par->cur_idx = CACHE_SIZE - 1;
    }
    
    par->points[par->cur_idx].t = t;
    memcpy(par->points[par->cur_idx].y, y, sizeof(y));
}

double leg_indicator_f(double t, void* par)
{
    double f, df;
    leg_indicator_fdf(t, par, &f, &df);
    return f;
}

double leg_indicator_df(double t, void* par)
{
    double f, df;
    leg_indicator_fdf(t, par, &f, &df);
    return df;

}

#define THETADOT 0
#define PHIDOT 1
#define THETA 2
#define PHI 3
#define TIME 4

typedef double walker_t[5];

double step(walker_t walker)
{
    gsl_root_fdfsolver* s;
    odesolver_t funcsolver;
    
    gsl_function_fdf fdf = 
        {.f = leg_indicator_f,
         .df = leg_indicator_df,
         .fdf = leg_indicator_fdf,
         .params = &funcsolver};
    double x0, x=4.0;
    
    ode_alloc(&funcsolver);
    
    memcpy(funcsolver.points[0].y, walker, sizeof(walker_t));
    
    s = gsl_root_fdfsolver_alloc(gsl_root_fdfsolver_steffenson);
    gsl_root_fdfsolver_set(s, &fdf, x);
    
    int status;
    do {
        gsl_root_fdfsolver_iterate(s);
        x0 = x;
        x = gsl_root_fdfsolver_root(s);
        status = gsl_root_test_delta(x, x0, 0, 1e-12);
    } while(status == GSL_CONTINUE);
    
    double tc = gsl_root_fdfsolver_root(s);
    gsl_root_fdfsolver_free(s);
    
    leg_indicator_f(tc, &funcsolver);
    
    memcpy(walker, funcsolver.points[funcsolver.cur_idx].y, sizeof(walker_t));
    walker[TIME] = tc;
    
    ode_free(&funcsolver);
    
    return tc;
}

void heelstrike(walker_t walker)
{
    double thetadot_m = walker[THETADOT];
    double theta_m = walker[THETA];
    
    walker[THETADOT] = cos(2.*theta_m) * thetadot_m;
    walker[PHIDOT] = cos(2.*theta_m) * (1. - cos(2.*theta_m)) * thetadot_m;
    walker[THETA] = -theta_m;
    walker[PHI] = -2.*theta_m;
}

typedef struct {
    double* x;
    int i, j;
} stridefunc_par_t;

double stridefunc(double x, void* _par)
{
    const stridefunc_par_t* par = (const stridefunc_par_t*) _par;
    walker_t walker;
    
    walker[0] = par->x[0];
    walker[1] = par->x[1];
    walker[2] = par->x[2];
    
    walker[par->j] += x;
    
    walker[3] = 2.*walker[2];
    
    step(walker);
    heelstrike(walker);
    
    return walker[par->i];
}

void calc_dS(double x[3], gsl_matrix* result)
{
    int i, j;
    stridefunc_par_t sfpar;
    sfpar.x = x;
    gsl_function gfunc = { .function = stridefunc, .params = &sfpar };
    
    for(i=0; i<3; ++i) {
        for(j=0; j<3; ++j) {
            double dsixj, error;
            
            sfpar.i = i; sfpar.j = j;
            gsl_deriv_central(&gfunc, 0.0, 1e-4, &dsixj, &error);
            gsl_matrix_set(result, i, j, dsixj);
        }
    }
}

void newton_step(double x_star[3], double* t)
{
    int k;
    gsl_matrix* dS = gsl_matrix_alloc(3, 3);
    calc_dS(x_star, dS);
    
    for(k=0; k<3; ++k) {
        gsl_matrix_set(dS, k, k, gsl_matrix_get(dS, k, k) - 1.);
    }
    
    gsl_vector* sxx = gsl_vector_alloc(3);
    gsl_vector* dx = gsl_vector_alloc(3);
    walker_t xp;
    
    xp[0] = x_star[0];
    xp[1] = x_star[1];
    xp[2] = x_star[2];
    xp[3] = 2.*x_star[2];
    
    step(xp);
    heelstrike(xp);
    
    for(k=0; k<3; ++k)
        gsl_vector_set(sxx, k, x_star[k] - xp[k]);
    
    gsl_permutation* p = gsl_permutation_alloc(3);
    int s;
    gsl_linalg_LU_decomp(dS, p, &s);
    gsl_linalg_LU_solve(dS, p, sxx, dx);
    
    for(k=0; k<3; ++k)
        x_star[k] += gsl_vector_get(dx, k);
    *t = xp[4];
}

void print_eigen(gsl_matrix* mat)
{
    gsl_vector_complex* eval = gsl_vector_complex_alloc(3);
    gsl_matrix_complex* evec = gsl_matrix_complex_alloc(3, 3);
    gsl_eigen_nonsymmv_workspace* w = gsl_eigen_nonsymmv_alloc(3);
    
    gsl_eigen_nonsymmv(mat, eval, evec, w);
    
    gsl_eigen_nonsymmv_free(w);
    
    int i;
    for(i=0; i<3; ++i) {
        gsl_complex eval_i = gsl_vector_complex_get(eval, i);
        printf("%.6f + %.6fi\n", GSL_REAL(eval_i), GSL_IMAG(eval_i));
    }
    
    gsl_vector_complex_free(eval);
    gsl_matrix_complex_free(evec);
}

void find_limit_cycle(double x_star[3], double* tc)
{
    int i;
    for(i=0; i<10; ++i) {
        newton_step(x_star, tc);
    }
}

// Limit cycle calculation result (cached)
const double THETADOT_STAR = -0.156080146674965;
const double PHIDOT_STAR = -0.007287197449037;
const double THETA_STAR = 0.153389564875669;

void matrix_print(gsl_matrix* mat)
{
    int i, j;
    for(i=0; i<mat->size1; ++i) {
        for(j=0; j<mat->size2; ++j) {
            printf("% 10.6f ", gsl_matrix_get(mat, i, j));
        }
        printf("\n");
    }
}

int main()
{
    /* double x_star[3];
    x_star[THETADOT] = -0.15;
    x_star[PHIDOT] = 0.;
    x_star[THETA] = 0.15;
    double tc;
    
    find_limit_cycle(x_star, &tc);
    
    printf("%.15f %.15f %.15f %.15f\n", x_star[THETADOT], x_star[PHIDOT], x_star[THETA], tc);
    */
    
    double x_star[3] = { THETADOT_STAR, PHIDOT_STAR, THETA_STAR };
    
    gsl_matrix* mat = gsl_matrix_alloc(3, 3);
    calc_dS(x_star, mat);
    // matrix_print(mat);
    print_eigen(mat);
    gsl_matrix_free(mat);
    
    return 0;
}

/* double _func(double x)
{
    return cos(x) - sin(2*x);
} */

/* void print_curve(void)
{
    const gsl_odeiv_step_type *T = gsl_odeiv_step_rkf45;
    
    gsl_odeiv_step *s = gsl_odeiv_step_alloc(T, DIM);
    gsl_odeiv_control *c = gsl_odeiv_control_y_new(1e-12, 0.);
    gsl_odeiv_evolve *e = gsl_odeiv_evolve_alloc(DIM);
    
    gsl_odeiv_system sys = { walker_diff, NULL, DIM, NULL };
    
    double t = 0.0;
    const double tstep = 0.01;
    double h = 1e-6;
    
    double y[DIM] = { -0.156080, -0.002787, 0.153390, 2.*0.153390 };
    int i;
    
    for(i=0; i<(100*5); ++i) {
        double ti = i * tstep;
        while(t < ti) {
            gsl_odeiv_evolve_apply(e, c, s, &sys, &t, ti, &h, y);
        }
        
        printf("%.6f %.6f %.6f %.6f\n", t, y[2], y[3], 2.*y[2]-y[3]);
    }
    
    gsl_odeiv_evolve_free(e);
    gsl_odeiv_control_free(c);
    gsl_odeiv_step_free(s);
}

int main()
{
    print_curve();
    return 0;
} */
