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
    Vector3  fTVel, fSVel;
    Vector3  fTOme, fSOme;
    
    inline Vector3  hipCoG()   const   { return fHPos; }
    inline Vector3  thighCoG() const   { return fTPos; }
    inline Rotation thighRot() const   { return fTRot; }
    inline Vector3  shankCoG() const   { return fSPos; }
    inline Rotation shankRot() const   { return fSRot; }
    
    inline double omegaT()     const   { return fTOme.y(); }
    inline double omegaS()     const   { return fSOme.y(); }
    
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
    Vector3 GetCenter() const { return fILeg.fHPos; }

    LegState fILeg, fOLeg;

  private:
    void DrawLeg(Vector3 thighPos, Rotation thighRot, Vector3 shankPos, Rotation shankRot) const;
    void DrawSlide() const;
    
    static const double DISP_LEGWIDTH;
    static const double DISP_SLIDEWIDTH;
    static const int DISP_SLIDELEN2;
};

class McGeer: public CachedSimulation<MGState> {
  public:
    McGeer();
    ~McGeer();
    
    double GetTimestep() { return 1./STEP_PER_SEC; }
    int GetDefaultEndTime() { return 60 * STEP_PER_SEC; }
    
    const char* GetTitle() { return TITLE; }
    
    void Advance();
    MGState GetCurrentState();
    
    void InitLeg(dBodyID& thigh, dBodyID& shank, dGeomID& footG, Vector3 hipPos, double iniPhiT, double iniPhiS, double legDist);
    void Collide(dGeomID g1, dGeomID g2);
    
    dWorldID fWorld;
    dJointGroupID fContactGroup;
    
    dBodyID fHip;
    dBodyID fInnerThigh, fInnerShank;
    dBodyID fOuterThigh, fOuterShank;

    dGeomID fFloorG, fInnerFootG, fOuterFootG;
    
    static const int STEP_PER_SEC;
    static const int INT_PER_STEP;
    static const char TITLE[];
    
    static const double G;
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
    static const double KNEE_TO_FOOTCTR;
    static const double HIP_TO_FOOTCTR;
    static const double EPS_T;
    
    static const double M_H;
    static const double THETA_C;
    static const double OMEGA_C;
    static const double OMEGA_FT;
    static const double OMEGA_FS;
    
    static const double INNER_LEG_DIST;
    static const double OUTER_LEG_DIST;
};

#endif
