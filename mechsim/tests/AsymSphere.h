#ifndef __TEST_H__
#define __TEST_H__

#include "Vector.h"
#include "Rotation.h"
#include "CachedSimulation.h"
#include <ode/ode.h>

class ASState: public SimulationState {
  public:
    double fT;
    void Draw() const;
//    Vector3 GetObjectPos() const { return fBPos; }

    Vector3  fCoG, fPos;
    Vector3  fVel, fOmega;
    Rotation fRot;
    double fPhi, fHeight;

  private:
    void DrawSlide() const;
    
    static const double DISP_SLIDEWIDTH;
    static const int DISP_SLIDELEN2;
};

class AsymSphere: public CachedSimulation<ASState> {
  public:
    AsymSphere();
    ~AsymSphere();
    
    double GetTimestep() { return 1./STEP_PER_SEC; }
    int GetDefaultEndTime() { return 10 * STEP_PER_SEC; }
    
    const char* GetTitle() { return TITLE; }
    
    void Advance();
    ASState GetCurrentState();
    
    void Collide(dGeomID g1, dGeomID g2);
    
    dWorldID fWorld;
    dJointGroupID fContactGroup;
    
    dBodyID fBody;
    dGeomID fFloorG, fBodyG;
    
    static const int STEP_PER_SEC;
    static const int INT_PER_STEP;
    static const char TITLE[];
    
    static const double GAMMA;
    static const double FLOOR_DIST;
    static const double R;
};

#endif