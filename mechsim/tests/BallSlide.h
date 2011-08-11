#ifndef MSIM_BALLSLIDE_H
#define MSIM_BALLSLIDE_H

#include "Vector.h"
#include "AsyncSimulation.h"
#include <ode/ode.h>

class BSState: public SimulationState {
  public:
    double fT;
    Vector3 fBPos;
    void Draw(int) const;
    Vector3 GetObjectPos() const { return fBPos; }
    
    Vector3 GetCenter() const
        { return Vector3(0., 0., 5.); }
    
    virtual double GetData(int ch) const
    {
        switch(ch) {
            case 0: return fBPos.x();
            case 1: return fBPos.y();
            case 2: return fBPos.z();
            default: return std::numeric_limits<double>::quiet_NaN();
        }
    }
};

void near_callback(void* data, dGeomID g1, dGeomID g2);

class BallSlide: public AsyncSimulation<BSState> {
  friend void near_callback(void* data, dGeomID g1, dGeomID g2);

  public:
    BallSlide();
    ~BallSlide();
    
    double GetTimestep() { return 1./STEP_PER_SEC; }
    int GetDefaultEndTime() { return 10 * STEP_PER_SEC; }
    
    const char* GetTitle() { return TITLE; }
    int GetNDataCh() const { return 3; }
    
    void Advance();
    BSState GetCurrentState();
    
    dWorldID fWorld;
    dSpaceID fSpace;
    dBodyID fBall;
    dGeomID fBallG, fSlideG, fFloorG;
    dJointGroupID fContactGroup;
    
    static const int STEP_PER_SEC;
    static const int INT_PER_STEP;
    static const char TITLE[];
};

#endif
