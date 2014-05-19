#include "MotorModel.h"
#include <limits>
#include <algorithm>

MotorModel::MotorModel()
    : fLastTorque(0.),
      fAlpha(1.),
      fMaxTorque(std::numeric_limits<double>::infinity())
{ }

void MotorModel::setTau(double tau, double dt)
    // tau: time constant
    // dt: sampling interval
{
    fAlpha = 1./(tau/dt + 1.);
}

double MotorModel::outTorque(double inTorque)
{
    inTorque = std::max(-fMaxTorque, inTorque);
    inTorque = std::min(fMaxTorque, inTorque);
    
    fLastTorque = fAlpha*inTorque + (1.-fAlpha) * fLastTorque;
    return fLastTorque;
}
