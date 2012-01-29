#ifndef ACRO_GSLINTERP_H
#define ACRO_GSLINTERP_H

#include <vector>
#include <gsl/gsl_interp.h>

class GSLInterp {
  public:
    GSLInterp()
        : fAcc(0), fInterp(0) {}
    
    GSLInterp(const gsl_interp_type* T, const std::vector<double>& xp, const std::vector<double>& yp);
    ~GSLInterp();
    
    double operator()(double x);
    
    void init(const gsl_interp_type* T, const std::vector<double>& xp, const std::vector<double>& yp);
    
  private:
    std::vector<double> fX;
    std::vector<double> fY;
    
    gsl_interp_accel* fAcc;
    gsl_interp* fInterp;
    
    // not to be called
    GSLInterp(const GSLInterp&) {}
    GSLInterp& operator=(const GSLInterp&) { return *this; }
};

#endif
