#include "GSLInterp.h"
#include <string>
#include <stdexcept>

GSLInterp::GSLInterp(const gsl_interp_type* T, const std::vector<double>& xp, const std::vector<double>& yp)
    : fAcc(0), fInterp(0)
{
    init(T, xp, yp);
}

void GSLInterp::init(const gsl_interp_type* T, const std::vector<double>& xp, const std::vector<double>& yp)
{
    if(fAcc != 0 || fInterp != 0)
        throw std::runtime_error(std::string("in ") + __func__ + ": init() must only be called once");
    
    if(xp.empty() || yp.empty())
        throw std::runtime_error(std::string("in ") + __func__ + ": x and y vectors must not be empty");
    
    if(xp.size() != yp.size())
        throw std::runtime_error(std::string("in ") + __func__ + ": x and y vector sizes do not match");
    
    fX = xp;
    fY = yp;
    
    fAcc = gsl_interp_accel_alloc();
    fInterp = gsl_interp_alloc(T, fX.size());
    gsl_interp_init(fInterp, fX.data(), fY.data(), fX.size());
}

GSLInterp::~GSLInterp()
{
    gsl_interp_free(fInterp);
    gsl_interp_accel_free(fAcc);
}

double GSLInterp::operator()(double x)
{
    return gsl_interp_eval(fInterp, fX.data(), fY.data(), x, fAcc);
}

