#ifndef __DOUBLEPENDULUM_H__
#define __DOUBLEPENDULUM_H__

#include "Vector.h"
#include "CachedSimulation.h"
#include <ode/ode.h>

class DPState: public SimulationState {
  public:
    double fT;
    Vector3 fB1_pos, fB2_pos;
    void Draw() const;
};

class DoublePendulum: public CachedSimulation<DPState> {
  public:
    DoublePendulum();
    ~DoublePendulum();
    
    double GetTimestep() { return 1./STEP_PER_SEC; }
    int GetDefaultEndTime() { return 5 * 60 * STEP_PER_SEC; }
    
    const char* GetTitle() { return TITLE; }
    
    void Advance();
    DPState GetCurrentState();
    
    dWorldID fWorld;
    dBodyID fBall1, fBall2;
    
    int fCurStep;
    
    static const int STEP_PER_SEC;
    static const int INT_PER_STEP;
    static const char TITLE[];
};

#endif
