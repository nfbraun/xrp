#ifndef MSIM_MCGEER_H
#define MSIM_MCGEER_H

#include <Eigen/Dense>
#include "Simulation.h"
#include <ode/ode.h>

class McGeer;

class LegState {
  public:
    Eigen::Vector3d    fHPos;
    Eigen::Vector3d    fTPos, fSPos;
    Eigen::Quaterniond fTRot, fSRot;
    Eigen::Vector3d    fTVel, fSVel;
    Eigen::Vector3d    fTOme, fSOme;
    
    inline Eigen::Vector3d    hipCoG()   const   { return fHPos; }
    inline Eigen::Vector3d    thighCoG() const   { return fTPos; }
    inline Eigen::Quaterniond thighRot() const   { return fTRot; }
    inline Eigen::Vector3d    shankCoG() const   { return fSPos; }
    inline Eigen::Quaterniond shankRot() const   { return fSRot; }
    
    inline double omegaT()     const   { return fTOme.y(); }
    inline double omegaS()     const   { return fSOme.y(); }
    
    double thighAng()              const;
    Eigen::Vector3d kneePos()      const;
    double shankAng()              const;
    Eigen::Vector3d footCtr()      const;
    double footClearance()         const;
};

class MGState: public SimulationState {
  public:
    McGeer* fParent;
    double fT;
    void Draw(int) const;
    Eigen::Vector3d GetCenter() const { return fILeg.fHPos; }
    double GetData(int ch) const;
    
    LegState fILeg, fOLeg;

  private:
    void DrawRobot(bool shadowmode) const;
    void DrawLeg(Eigen::Vector3d thighPos, Eigen::Quaterniond thighRot,
                 Eigen::Vector3d shankPos, Eigen::Quaterniond shankRot) const;
    void DrawSlide(double dist, double s0, double s1) const;
    
    static const double DISP_LEGWIDTH;
    static const double DISP_SLIDEWIDTH;
};

class McGeer: public Simulation {
  public:
    McGeer();
    ~McGeer();
    
    double GetTimestep() { return 1./STEP_PER_SEC; }
    int GetDefaultEndTime() { return 15 * STEP_PER_SEC; }
    
    const char* GetTitle() { return TITLE; }
    int GetNDataCh() const { return 4; }
    
    void Advance();
    MGState GetCurrentState();
    
    void InitLeg(dBodyID& thigh, dBodyID& shank, dGeomID& footG, Eigen::Vector3d hipPos, double iniPhiT, double iniPhiS, double legDist);
    void Collide(dGeomID g1, dGeomID g2);
    
    dWorldID fWorld;
    dJointGroupID fContactGroup;
    
    dBodyID fHip;
    dBodyID fInnerThigh, fInnerShank;
    dBodyID fOuterThigh, fOuterShank;

    dGeomID fFloorG1, fFloorG2, fInnerFootG, fOuterFootG;
    
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
    
    static const double SLIDELEN_1;
    static const double SLIDELEN_2;
    static const double STEP_HEIGHT;
};

#endif
