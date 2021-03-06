#ifndef MSIM_ASYMSPHERE_H
#define MSIM_ASYMSPHERE_H

#include <Eigen/Dense>
#include "Simulation.h"
#include <ode/ode.h>

class ASState: public SimulationState {
  public:
    double fT;
    void Draw(int) const;
//    Vector3 GetObjectPos() const { return fBPos; }

    Eigen::Vector3d  fCoG, fPos;
    Eigen::Vector3d  fVel, fOmega;
    Eigen::Quaterniond fRot;
    double fPhi, fHeight;

  private:
    void DrawSlide() const;
    
    static const double DISP_SLIDEWIDTH;
    static const int DISP_SLIDELEN2;
};

class AsymSphere: public Simulation {
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
