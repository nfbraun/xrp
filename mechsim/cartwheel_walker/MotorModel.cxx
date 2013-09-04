#include "MotorModel.h"

MotorModel::MotorModel()
    : fLastTorque(0.),
      fAlpha(1.)
{ }

void MotorModel::setTau(double tau, double dt)
    // tau: time constant
    // dt: sampling interval
{
    fAlpha = 1./(tau/dt + 1.);
}

double MotorModel::outTorque(double inTorque)
{
    fLastTorque = fAlpha*inTorque + (1.-fAlpha) * fLastTorque;
    return fLastTorque;
}
