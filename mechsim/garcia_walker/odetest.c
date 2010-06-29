#include <stdio.h>
#include <math.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv.h>
#include <gsl/gsl_roots.h>

int deriv(double t, const double y[], double f[], void *_p)
{
    f[0] = -y[2];     // xdotdot
    f[1] = -4.*y[3];  // ydotdot
    f[2] = y[0];      // xdot
    f[3] = y[1];      // ydot
    
    return GSL_SUCCESS;
}

/* int jac(double t, const double y[], double *_dfdy, double dfdt[], void *params)
{
    double k = *((double *) params);
    gsl_matrix_view dfdy = gsl_matrix_view_array(_dfdy, 2, 2);
    gsl_matrix_set(&dfdy.matrix, 0, 0, );
    gsl_matrix_set(&dfdy.matrix, 0, 1, );
    gsl_matrix_set(&dfdy.matrix, 1, 0, );
    gsl_matrix_set(&dfdy.matrix, 1, 1, );
    dfdt[0] = 0.;
    dfdt[1] = 0.;
    
    return GSL_SUCCESS;
} */

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
    par->sys.function = deriv;
    par->sys.jacobian = NULL;
    par->sys.dimension = DIM;
    par->sys.params = NULL;
    
    par->points[0].t = 0.;
    par->points[0].y[0] = 0.;
    par->points[0].y[1] = 2.;
    par->points[0].y[2] = 1.;
    par->points[0].y[3] = 0.;
    par->cur_idx = 0;
}

void ode_free(odesolver_t* par)
{
    gsl_odeiv_evolve_free(par->e);
    gsl_odeiv_control_free(par->c);
    gsl_odeiv_step_free(par->s);
}

void func_fdf(double t_end, void *_par, double* f, double* df)
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
    
    //             xdot, ydot, x, y
    double y[4];
    y[0] = par->points[par->cur_idx].y[0];
    y[1] = par->points[par->cur_idx].y[1];
    y[2] = par->points[par->cur_idx].y[2];
    y[3] = par->points[par->cur_idx].y[3];
    
    gsl_odeiv_evolve_reset(par->e);
    
    while(t < t_end)
        gsl_odeiv_evolve_apply(par->e, par->c, par->s, &par->sys, &t, t_end, &h, y);
    
    *f  = y[2] - y[3];
    *df = y[0] - y[1];
    
    par->cur_idx++;
    if(par->cur_idx >= CACHE_SIZE) {
        fprintf(stderr, "Warning: cache size too small!\n");
        par->cur_idx = CACHE_SIZE - 1;
    }
    
    par->points[par->cur_idx].t = t;
    par->points[par->cur_idx].y[0] = y[0];
    par->points[par->cur_idx].y[1] = y[1];
    par->points[par->cur_idx].y[2] = y[2];
    par->points[par->cur_idx].y[3] = y[3];
}

double func(double t, void* par)
{
    double f, df;
    func_fdf(t, par, &f, &df);
    return f;
}

double dfunc(double t, void* par)
{
    double f, df;
    func_fdf(t, par, &f, &df);
    return df;

}

double intersection_point()
{
    gsl_root_fdfsolver* s;
    odesolver_t funcsolver;
    
    gsl_function_fdf fdf = 
        {.f = &func,
         .df = &dfunc,
         .fdf = &func_fdf,
         .params = &funcsolver};
    double x0, x=0.;
    
    ode_alloc(&funcsolver);
    
    s = gsl_root_fdfsolver_alloc(gsl_root_fdfsolver_steffenson);
    gsl_root_fdfsolver_set(s, &fdf, x);
    
    int status;
    do {
        gsl_root_fdfsolver_iterate(s);
        x0 = x;
        x = gsl_root_fdfsolver_root(s);
        status = gsl_root_test_delta(x, x0, 0, 1e-12);
    } while(status == GSL_CONTINUE);
    
    double root = gsl_root_fdfsolver_root(s);
    gsl_root_fdfsolver_free(s);
    
    ode_free(&funcsolver);
    
    if(status == GSL_SUCCESS)
        return root;
    else
        return -1.;
}

int main()
{
    double tcrit = intersection_point();
    printf("Result: %.12f\n", tcrit);
    printf("True:   %.12f\n", M_PI/6.);
    printf("Error:  %e\n", fabs(tcrit - M_PI/6.));
    
    return 0;
}

/* double _func(double x)
{
    return cos(x) - sin(2*x);
}

int main()
{
    odesolver_t funcsolver;
    ode_alloc(&funcsolver);
    
    int ti;
    double maxerr = 0.;
    for(ti = 0; ti < 10000; ++ti) {
        double err;
        
        err = fabs(func((ti+1) * .001, &funcsolver) - _func((ti+1) * .001));
        maxerr = fmax(maxerr, err);
        
        err = fabs(func(ti * .001, &funcsolver) - _func(ti * .001));
        maxerr = fmax(maxerr, err);
    }
    
    printf("%e\n", maxerr);
    
    ode_free(&funcsolver);
    
    return 0;
} */
