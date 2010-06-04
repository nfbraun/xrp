#ifndef __MCGEER_H__
#define __MCGEER_H__

#include "Vector.h"
#include "Rotation.h"
#include "CachedSimulation.h"
#include <ode/ode.h>

class McGeer;

class LegState {
  public:
    Vector3  fHPos;
    Vector3  fTPos, fSPos;
    Rotation fTRot, fSRot;
    
    inline Vector3  thighCoG() const   { return fTPos; }
    inline Rotation thighRot() const   { return fTRot; }
    inline Vector3  shankCoG() const   { return fSPos; }
    inline Rotation shankRot() const   { return fSRot; }
    
    double thighAng()      const;
    Vector3 kneePos()      const;
    double shankAng()      const;
    Vector3 footCtr()      const;
    double footClearance() const;
};

class MGState: public SimulationState {
  public:
    McGeer* fParent;
    double fT;
    void Draw() const;
//    Vector3 GetObjectPos() const { return fBPos; }

    LegState fLLeg, fRLeg;

  private:
    void DrawLeg(Vector3 thighPos, Rotation thighRot, Vector3 shankPos, Rotation shankRot) const;
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
    int GetDefaultEndTime() { return 10 * STEP_PER_SEC; }
    
    const char* GetTitle() { return TITLE; }
    
    void Advance();
    MGState GetCurrentState();
    
    void InitLeg(dBodyID& thigh, dBodyID& shank, dGeomID& footG, double y, double iniPhiT, double iniPhiS);
    void Collide(dGeomID g1, dGeomID g2);
    
    dWorldID fWorld;
    dJointGroupID fContactGroup;
    
    dBodyID fHip;
    dBodyID fLeftThigh, fLeftShank;
    dBodyID fRightThigh, fRightShank;

    dGeomID fFloorG, fLeftFootG, fRightFootG;
    
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
