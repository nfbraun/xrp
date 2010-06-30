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
#include <gsl/gsl_multiroots.h>
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

typedef double walker_st_t[5];
typedef double step_st_t[3];

double step(walker_st_t walker_state)
{
    gsl_root_fdfsolver* s;
    odesolver_t funcsolver;
    
    gsl_function_fdf fdf = 
        {.f = leg_indicator_f,
         .df = leg_indicator_df,
         .fdf = leg_indicator_fdf,
         .params = &funcsolver};
    double tcp, tc = 4.;
    
    ode_alloc(&funcsolver);
    
    memcpy(funcsolver.points[0].y, walker_state, sizeof(walker_st_t));
    
    s = gsl_root_fdfsolver_alloc(gsl_root_fdfsolver_steffenson);
    gsl_root_fdfsolver_set(s, &fdf, tc);
    
    int status;
    do {
        gsl_root_fdfsolver_iterate(s);
        tcp = tc;
        tc = gsl_root_fdfsolver_root(s);
        status = gsl_root_test_delta(tc, tcp, 0, 1e-12);
    } while(status == GSL_CONTINUE);
    
    tc = gsl_root_fdfsolver_root(s);
    gsl_root_fdfsolver_free(s);
    
    leg_indicator_f(tc, &funcsolver);
    
    memcpy(walker_state, funcsolver.points[funcsolver.cur_idx].y, sizeof(walker_st_t));
    walker_state[TIME] = tc;
    
    ode_free(&funcsolver);
    
    return tc;
}

void heelstrike(walker_st_t walker_state)
{
    double thetadot_m = walker_state[THETADOT];
    double theta_m = walker_state[THETA];
    
    walker_state[THETADOT] = cos(2.*theta_m) * thetadot_m;
    walker_state[PHIDOT] = cos(2.*theta_m) * (1. - cos(2.*theta_m)) * thetadot_m;
    walker_state[THETA] = -theta_m;
    walker_state[PHI] = -2.*theta_m;
}

typedef struct {
    double* x;
    int i, j;
} d_stridefunc_par_t;

// Evolve state of the walker from the beginning of one step to the beginning of
// the next one
void stridefunc(step_st_t step_state, double* t)
{
    walker_st_t walker_state;
    walker_state[THETADOT] = step_state[THETADOT];
    walker_state[PHIDOT]= step_state[PHIDOT];
    walker_state[THETA] = step_state[THETA];
    walker_state[PHI] = 2. * step_state[THETA];
    
    step(walker_state);
    heelstrike(walker_state);
    
    step_state[THETADOT] = walker_state[THETADOT];
    step_state[PHIDOT] = walker_state[PHIDOT];
    step_state[THETA] = walker_state[THETA];
    if(t) *t = walker_state[TIME];
}

double diff_stridefunc(double eps, void* _par)
{
    const d_stridefunc_par_t* par = (const d_stridefunc_par_t*) _par;
    step_st_t x;
    
    memcpy(x, par->x, sizeof(step_st_t));
    
    x[par->j] += eps;
    
    stridefunc(x, 0);
    
    return x[par->i];
}

void calc_dS(step_st_t x, gsl_matrix* result)
{
    int i, j;
    d_stridefunc_par_t sfpar;
    sfpar.x = x;
    gsl_function gfunc = { .function = diff_stridefunc, .params = &sfpar };
    
    for(i=0; i<3; ++i) {
        for(j=0; j<3; ++j) {
            double dsixj, error;
            
            sfpar.i = i; sfpar.j = j;
            gsl_deriv_central(&gfunc, 0.0, 1e-8, &dsixj, &error);
            gsl_matrix_set(result, i, j, dsixj);
        }
    }
}

int stridefunc_wrap(const gsl_vector* x, void* p, gsl_vector* f)
{
    double* tc = (double*) p;
    step_st_t step_state;
    step_state[THETADOT] = gsl_vector_get(x, 0);
    step_state[PHIDOT] = gsl_vector_get(x, 1);
    step_state[THETA] = gsl_vector_get(x, 2);
    
    stridefunc(step_state, tc);
    
    gsl_vector_set(f, 0, step_state[THETADOT] - gsl_vector_get(x, 0));
    gsl_vector_set(f, 1, step_state[PHIDOT] - gsl_vector_get(x, 1));
    gsl_vector_set(f, 2, step_state[THETA] - gsl_vector_get(x, 2));
    
    return GSL_SUCCESS;
}

void find_limit_cycle(step_st_t x_star, double* tc)
{
    int iter=0;
    int status;
    
    gsl_multiroot_function f = { &stridefunc_wrap, 3, tc };
    
    gsl_vector* x = gsl_vector_alloc(3);
    gsl_vector_set(x, 0, x_star[THETADOT]);
    gsl_vector_set(x, 1, x_star[PHIDOT]);
    gsl_vector_set(x, 2, x_star[THETA]);
    
    const gsl_multiroot_fsolver_type* T = gsl_multiroot_fsolver_hybrids;
    gsl_multiroot_fsolver* s = gsl_multiroot_fsolver_alloc(T, 3);
    gsl_multiroot_fsolver_set(s, &f, x);
    
    do {
        iter++;
        status = gsl_multiroot_fsolver_iterate(s);
        if(status) break;
        status = gsl_multiroot_test_residual(s->f, 1e-12);
    } while(status == GSL_CONTINUE && iter < 100);
    
    if(status != GSL_SUCCESS) {
        fprintf(stderr, "Failed to find limit cycle!\n");
    }
    
    x_star[THETADOT] = gsl_vector_get(s->x, 0);
    x_star[PHIDOT] = gsl_vector_get(s->x, 1);
    x_star[THETA] = gsl_vector_get(s->x, 2);
    
    gsl_multiroot_fsolver_free(s);
    gsl_vector_free(x);
}

void print_eigen(gsl_matrix* mat, const char* prefix)
{
    gsl_vector_complex* eval = gsl_vector_complex_alloc(3);
    gsl_matrix_complex* evec = gsl_matrix_complex_alloc(3, 3);
    gsl_eigen_nonsymmv_workspace* w = gsl_eigen_nonsymmv_alloc(3);
    
    gsl_eigen_nonsymmv(mat, eval, evec, w);
    
    gsl_eigen_nonsymmv_free(w);
    
    int i;
    for(i=0; i<3; ++i) {
        gsl_complex eval_i = gsl_vector_complex_get(eval, i);
        printf("%s% 7.6f %+7.6fi\n", prefix, GSL_REAL(eval_i), GSL_IMAG(eval_i));
    }
    
    gsl_vector_complex_free(eval);
    gsl_matrix_complex_free(evec);
}

void matrix_print(gsl_matrix* mat, const char* prefix)
{
    int i, j;
    for(i=0; i<mat->size1; ++i) {
        printf("%s", prefix);
        for(j=0; j<mat->size2; ++j) {
            printf("% 10.6f ", gsl_matrix_get(mat, i, j));
        }
        printf("\n");
    }
}

void print_curve(step_st_t x_star, double tc)
{
    const gsl_odeiv_step_type *T = gsl_odeiv_step_rkf45;
    
    gsl_odeiv_step *s = gsl_odeiv_step_alloc(T, DIM);
    gsl_odeiv_control *c = gsl_odeiv_control_y_new(1e-12, 0.);
    gsl_odeiv_evolve *e = gsl_odeiv_evolve_alloc(DIM);
    
    gsl_odeiv_system sys = { walker_diff, NULL, DIM, NULL };
    
    double t = 0.0;
    const double tstep = 0.01;
    double h = 1e-6;
    
    double x[DIM];
    x[THETADOT] = x_star[THETADOT];
    x[PHIDOT] = x_star[PHIDOT];
    x[THETA] = x_star[THETA];
    x[PHI] = 2. * x_star[THETA];
    
    int i = 0;
    int stop = 0;
    
    do {
        double ti = i * tstep;
        
        if(ti > tc) {
            ti = tc;
            stop = 1;
        }
        
        while(t < ti) {
            gsl_odeiv_evolve_apply(e, c, s, &sys, &t, ti, &h, x);
        }
        
        printf("%.6f %.6f %.6f %.6f %.6f\n", t, x[THETADOT], x[PHIDOT], x[THETA], x[PHI]);
        i++;
    } while(!stop);
    
    gsl_odeiv_evolve_free(e);
    gsl_odeiv_control_free(c);
    gsl_odeiv_step_free(s);
}

int main(int argc, char** argv)
{
    step_st_t x_star;
    x_star[THETADOT] = -0.15;
    x_star[PHIDOT] = 0.;
    x_star[THETA] = 0.15;
    double tc;
    
    find_limit_cycle(x_star, &tc);
    
    if(argc == 1) {
        printf("Limit cycle:\n");
        printf("    theta_dot_star = % 8.6f\n", x_star[THETADOT]);
        printf("    phi_dot_star   = % 8.6f\n", x_star[PHIDOT]);
        printf("    theta_star     = % 8.6f\n", x_star[THETA]);
        printf("    T              = % 8.6f\n", tc);
        
        printf("Monodromy matrix eigenvalues:\n");
        gsl_matrix* mat = gsl_matrix_alloc(3, 3);
        calc_dS(x_star, mat);
        print_eigen(mat, "    ");
        gsl_matrix_free(mat);
    } else {
        print_curve(x_star, tc);
    }
    
    return 0;
}

