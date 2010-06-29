#ifndef __BALLSLIDE_H__
#define __BALLSLIDE_H__

#include "Vector.h"
#include "CachedSimulation.h"
#include <ode/ode.h>

class BSState: public SimulationState {
  public:
    double fT;
    Vector3 fBPos;
    void Draw() const;
    Vector3 GetObjectPos() const { return fBPos; }
};

void near_callback(void* data, dGeomID g1, dGeomID g2);

class BallSlide: public CachedSimulation<BSState> {
  friend void near_callback(void* data, dGeomID g1, dGeomID g2);

  public:
    BallSlide();
    ~BallSlide();
    
    double GetTimestep() { return 1./STEP_PER_SEC; }
    int GetDefaultEndTime() { return 10 * STEP_PER_SEC; }
    
    const char* GetTitle() { return TITLE; }
    
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
