#ifndef CW_MOTORMODEL_H
#define CW_MOTORMODEL_H

class MotorModel {
  public:
    MotorModel();
    
    void setTau(double tau, double dt);
    double outTorque(double inTorque);
    
  private:
    double fLastTorque;
    double fAlpha;
};

#endif
