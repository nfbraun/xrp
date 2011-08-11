#ifndef MSIM_DOUBLEPENDULUM_H
#define MSIM_DOUBLEPENDULUM_H

#include "Vector.h"
#include "SyncSimulation.h"
#include <ode/ode.h>

class DPState: public SimulationState {
  public:
    double fT;
    Vector3 fB1_pos, fB2_pos;
    double fOmega1, fOmega2;
    void Draw(int) const;
    
    virtual double GetData(int ch) const
    {
        switch(ch) {
            case 0: return atan2(fB1_pos.z(), fB1_pos.x());
            case 1: {
                Vector3 delta = fB2_pos - fB1_pos;
                return atan2(delta.z(), delta.x());
            }
            case 2: return fOmega1;
            case 3: return fOmega2;
            default: return std::numeric_limits<double>::quiet_NaN();
        }
    }
};

class DoublePendulum: public SyncSimulation<DPState> {
  public:
    DoublePendulum();
    ~DoublePendulum();
    
    double GetTimestep() { return 1./STEP_PER_SEC; }
    int GetDefaultEndTime() { return 5 * 60 * STEP_PER_SEC; }
    
    const char* GetTitle() { return TITLE; }
    int GetNDataCh() const { return 4; }
    virtual const char* GetChName(int ch) const
    {
        switch(ch) {
            case 0:  return "Phi1";
            case 1:  return "Phi2";
            case 2:  return "Omega1";
            case 3:  return "Omega2";
            default: return "<undefined>";
        }
    }
    
    void Advance();
    DPState GetCurrentState();
    
    dWorldID fWorld;
    dBodyID fBall1, fBall2;
    dJointID fJoint1, fJoint2;
    
    int fCurStep;
    
    static const int STEP_PER_SEC;
    static const int INT_PER_STEP;
    static const char TITLE[];
};

#endif
