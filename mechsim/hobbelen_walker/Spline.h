#ifndef __SPLINE_H__
#define __SPLINE_H__

#include <gsl/gsl_spline.h>

class Spline
{
  public:
    Spline(int n, const double x[], const double y[]);
    ~Spline();
    
    inline double eval(double x)
        { return gsl_spline_eval(fSpline, x, fAcc); }
    inline double eval_deriv(double x)
        { return gsl_spline_eval_deriv(fSpline, x, fAcc); }
    
  private:
    Spline(const Spline&) {};
    Spline& operator=(const Spline&) { return *this; }
    
    gsl_interp_accel* fAcc;
    gsl_spline* fSpline;
};

#endif
