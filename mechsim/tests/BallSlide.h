#ifndef MSIM_BALLSLIDE_H
#define MSIM_BALLSLIDE_H

#include <Eigen/Dense>
#include "Simulation.h"
#include <ode/ode.h>

class BSState: public SimulationState {
  public:
    double fT;
    Eigen::Vector3d fBPos;
    void Draw(int) const;
    Eigen::Vector3d GetObjectPos() const { return fBPos; }
    
    Eigen::Vector3d GetCenter() const
        { return Eigen::Vector3d(0., 0., 5.); }
    
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

class BallSlide: public Simulation {
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
