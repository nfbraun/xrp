#include <stdio.h>
#include <math.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv.h>

typedef struct {
    double m;
    double R;
    double d;
    double I;
    double gamma;
    double g;
} sysparam_t;

int func(double t, const double y[], double f[], void *_p)
{
    const sysparam_t *p = (sysparam_t*) _p;
    const double m = p->m;
    const double R = p->R;
    const double d = p->d;
    const double I = p->I;
    const double gamma = p->gamma;
    const double g = p->g;
    
    const double phidot = y[0];
    const double phi    = y[1];
    
    const double M = m*(R*R+d*d) + I + 2*m*d*R*cos(phi-gamma);
    
    double u = m*d*R*sin(phi-gamma)*phidot*phidot + m*g*d*sin(phi) + m*g*R*sin(gamma);
    f[0] = u / M;   // phidotdot
    f[1] = phidot;
    
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

int main()
{
    sysparam_t par = { .m = 0.838, .R = 0.2,
                       .d = 0.720687,
                       .I = 0.0545087,
                       .gamma = 0.0456, .g = 1. };
    const double phi0 = -0.309408;
    const double omega0 = 0.30;
    
    const gsl_odeiv_step_type *T = gsl_odeiv_step_rkf45;
    
    gsl_odeiv_step *s = gsl_odeiv_step_alloc(T, 2);
    gsl_odeiv_control *c = gsl_odeiv_control_y_new(1e-6, 0.);
    gsl_odeiv_evolve *e = gsl_odeiv_evolve_alloc(2);
    
    gsl_odeiv_system sys = { func, NULL, 2, &par };
    
    double t = 0.0;
    const double tstep = 1./16.;
    double h = 1e-6;
    //            phidot, phi
    double y[2] = {omega0, phi0};
    int i;
    
    for(i=0; i<(16*10); ++i) {
        double ti = i * tstep;
        while(t < ti) {
            gsl_odeiv_evolve_apply(e, c, s, &sys, &t, ti, &h, y);
        }
        
        printf("%.6f %.6f %.6f\n", t, y[0], y[1]);
    }
    
    gsl_odeiv_evolve_free(e);
    gsl_odeiv_control_free(c);
    gsl_odeiv_step_free(s);
    
    return 0;
}
