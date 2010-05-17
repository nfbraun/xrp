#ifndef __MCGEER_H__
#define __MCGEER_H__

#include "Vector.h"
#include "CachedSimulation.h"
#include <ode/ode.h>

class MGState: public SimulationState {
  public:
    double fT;
    void Draw() const;
//    Vector3 GetObjectPos() const { return fBPos; }

    double fPhiLT, fPhiLS, fPhiRT, fPhiRS;

  private:
    void DrawLeg(double t, double s) const;
    void DrawSlide() const;
    
    static const double DISP_LEGDIST;
    static const double DISP_LEGWIDTH;
    static const double DISP_SLIDEWIDTH;
    static const int DISP_SLIDELEN2;
};

// void near_callback(void* data, dGeomID g1, dGeomID g2);

class McGeer: public CachedSimulation<MGState> {
//  friend void near_callback(void* data, dGeomID g1, dGeomID g2);

  public:
    McGeer();
    ~McGeer();
    
    double GetTimestep() { return 1./STEP_PER_SEC; }
    int GetDefaultEndTime() { return 60 * STEP_PER_SEC; }
    
    const char* GetTitle() { return TITLE; }
    
    void Advance();
    MGState GetCurrentState();
    
    void InitLeg(dBodyID& thigh, dBodyID& shank, double y, double iniPhiT, double iniPhiS);
    
    dWorldID fWorld;
    dSpaceID fSpace;
    dJointGroupID fContactGroup;
    
    dBodyID fLeftThigh, fLeftShank;
    dBodyID fRightThigh, fRightShank;

    dGeomID fFloorG;
    
    static const int STEP_PER_SEC;
    static const int INT_PER_STEP;
    static const char TITLE[];
    
    static const double GAMMA;
    static const double FLOOR_DIST;
    static const double M_T;
    static const double R_GYR_T;
    static const double L_T;
    static const double C_T;
    static const double W_T;
    static const double ALPHA_T;
    static const double M_S;
    static const double R_GYR_S;
    static const double L_S;
    static const double C_S;
    static const double W_S;
    static const double ALPHA_S;
    static const double R;
    static const double EPS_K;
    
    static const double INI_PHI_LT;
    static const double INI_PHI_LS;
    static const double INI_PHI_RT;
    static const double INI_PHI_RS;
};

#endif
