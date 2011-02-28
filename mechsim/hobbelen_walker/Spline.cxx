#include "Spline.h"

Spline::Spline(int n, const double x[], const double y[])
{
    fAcc = gsl_interp_accel_alloc();
    fSpline = gsl_spline_alloc(gsl_interp_cspline, n);
    gsl_spline_init(fSpline, x, y, n);
}

Spline::~Spline()
{
    gsl_spline_free(fSpline);
    gsl_interp_accel_free(fAcc);
}
