#ifndef CW_MOTORMODEL_H
#define CW_MOTORMODEL_H

class MotorModel {
  public:
    MotorModel();
    
    void setTau(double tau, double dt);
    void setMaxTorque(double t_max)
        { fMaxTorque = t_max; }
    
    double outTorque(double inTorque);
    
  private:
    double fLastTorque;
    double fAlpha;
    double fMaxTorque;
};

#endif
