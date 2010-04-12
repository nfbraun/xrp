#include <stdio.h>
#include <math.h>
#include <gsl/gsl_errno.h>
#include <gsl/gsl_matrix.h>
#include <gsl/gsl_odeiv.h>

typedef struct {
    double mb, mc;
    double l;
    double g;
} sysparam_t;

int func(double t, const double y[], double f[], void *_p)
{
    const sysparam_t *p = (sysparam_t*) _p;
    const double xdot = y[0];
    const double phidot = y[1];
    const double x = y[2];
    const double phi = y[3];
    
    const double Fext = 3. * sin(t);
    
    const double a11 = p->mc + p->mb;
    const double a12 = p->l*p->mb*cos(phi);
    const double a21 = p->l*p->mb*cos(phi);
    const double a22 = p->mb*p->l*p->l;
    const double f1 = p->l*p->mb*phidot*phidot*sin(phi) + Fext;
    const double f2 = -p->mb*p->g*p->l*sin(phi);
    
    f[0] = (f2 - a22/a12*f1)/(a21 - a11*a22/a12);   // xdotdot
    f[1] = (f2 - a21/a11*f1)/(a22 - a12*a21/a11);   // phidotdot
    f[2] = xdot;
    f[3] = phidot;
    
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
    sysparam_t par = { .mb = 1., .mc = 1., .l = 1., .g = 9.81 };

    const gsl_odeiv_step_type *T = gsl_odeiv_step_rkf45;
    
    gsl_odeiv_step *s = gsl_odeiv_step_alloc(T, 4);
    gsl_odeiv_control *c = gsl_odeiv_control_y_new(1e-6, 0.);
    gsl_odeiv_evolve *e = gsl_odeiv_evolve_alloc(4);
    
    gsl_odeiv_system sys = { func, NULL, 4, &par };
    
    double t = 0.0;
    const double tstep = 0.01;
    double h = 1e-6;
    //            xdot, phidot, x, phi
    double y[4] = {0.0, 0.0, 0.0, 1.};
    int i;
    
    for(i=0; i<1000; ++i) {
        double ti = i * tstep;
        while(t < ti) {
            gsl_odeiv_evolve_apply(e, c, s, &sys, &t, ti, &h, y);
        }
        
        printf("%.6f %.6f %.6f %.6f %.6f\n", t, y[0], y[1], y[2], y[3]);
    }
    
    gsl_odeiv_evolve_free(e);
    gsl_odeiv_control_free(c);
    gsl_odeiv_step_free(s);
    
    return 0;
}
