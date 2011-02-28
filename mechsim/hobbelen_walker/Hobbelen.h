#ifndef __HOBBELEN_H__
#define __HOBBELEN_H__

#include "Vector.h"
#include "Rotation.h"
#include "BodyConf.h"
#include "ChainSegment.h"
#include "FootSegment.h"
#include "CachedSimulation.h"
#include "Spline.h"
#include <ode/ode.h>

// ** System parameters **
namespace HobbelenConst {
const double GAMMA      = 0.0; //0.0456;   // Floor slope
const double FLOOR_DIST = 1.0;   // shortest distance floor to origin

const Vector3 FloorNormal(sin(GAMMA), 0., cos(GAMMA));

//                          m,   I,      l,   c,         w   [SI units]
const ChainSegment Body    (8.5, 0.11,   0.4, 0.4 - 0.2, 0.);
const ChainSegment UpperLeg(0.9, 0.0068, 0.3, 0.15,      0.);
const ChainSegment LowerLeg(0.9, 0.0068, 0.3, 0.15,      0.);

//                     m,   I,      l,     w,      r,    h  [SI units]
const FootSegment Foot(0.1, 0.0001, 0.085, 0.0175, 0.02, 0.025);

const double K_AL = 5.;
const double K_A = 20.;

// Swing leg trajectory
const double STEP_T = 0.8;
const int SWING_SPL_N_POINTS = 6;
const double SWING_SPL_X[SWING_SPL_N_POINTS] =
    { 0.*STEP_T, 0.1*STEP_T, 0.2*STEP_T, 0.4*STEP_T, 0.7*STEP_T, 1.*STEP_T };
const double SWING_SPL_Y[SWING_SPL_N_POINTS] =
    { -0.54, -0.54, -0.4, 0.2, 0.55, 0.44 };

// These do not matter much, as long as they are large enough
const double INNER_LEG_DIST = 0.2;
const double OUTER_LEG_DIST = 0.4;
}; // end namespace HobbelenConst

class BodyQ {
  public:
    BodyQ() {}
    BodyQ(Vector3 pos, Rotation rot, Vector3 vel, Vector3 avel)
        : fPos(pos), fRot(rot), fVel(vel), fAVel(avel) {}
    static BodyQ FromODE(dBodyID id);
    void TransformGL() const;
    
    inline Vector3 pos() const { return fPos; }
    inline Rotation rot() const { return fRot; }
    inline Vector3 vel() const { return fVel; }
    inline Vector3 avel() const { return fAVel; }
    
  private:
    Vector3 fPos;
    Rotation fRot;
    Vector3 fVel;
    Vector3 fAVel;
};

class Hobbelen;

class HobState: public SimulationState {
  public:
    Hobbelen* fParent;
    double fT;
    
    BodyQ fBodyQ;
    BodyQ fIULegQ, fILLegQ, fIFootQ;
    BodyQ fOULegQ, fOLLegQ, fOFootQ;
    
    double fIHipAngle, fOHipAngle;
    double fDesiredInterLeg;
    double fIKneeAngle, fOKneeAngle;
    double fIAnkleAngle, fOAnkleAngle;
    
    double fITipClear, fIHeelClear;
    double fOTipClear, fOHeelClear;
    
    int fILegState, fOLegState;
    
    void Draw() const;
    void DrawLeg(const BodyQ& upperLegQ, const BodyQ& lowerLegQ,
                       const BodyQ& footQ) const;
    
    Vector3 GetCenter() const
        { return Vector3(fBodyQ.pos().x(), 0., 0.); }

  private:
    void DrawSlide() const;
    
    static const double DISP_LEGWIDTH;
    static const double DISP_SLIDEWIDTH;
    static const int DISP_SLIDELEN2;
};

struct HobJoint {
    dJointID id;
    double off;
};

struct HobLeg {
    dBodyID ULegB, LLegB, FootB;
    dGeomID FFootG, BFootG;
    HobJoint HipJ, KneeJ, AnkleJ;
};

enum LegType { InnerLeg, OuterLeg };
enum StepState { Pushoff, Swing, Touchdown, Stance };

class Hobbelen: public CachedSimulation<HobState> {
  public:
    Hobbelen();
    ~Hobbelen();
    
    double fT;
    double GetTimestep() { return 1./STEP_PER_SEC; }
    int GetDefaultEndTime() { return 60 * STEP_PER_SEC; }
    
    const char* GetTitle() { return TITLE; }
    
    void Advance();
    HobState GetCurrentState();
    
    double GetHeelClearance(dBodyID footB);
    double GetTipClearance(dBodyID footB);
    void SetKneeLock(const HobJoint& kneeJ, bool lock);
    void AnkleTorqueControl(const HobJoint& ankleJ, StepState state);
    void SwingHipTorqueControl(LegType leg, double t);
    void Collide(dGeomID g1, dGeomID g2);
    
    dWorldID fWorld;
    dJointGroupID fContactGroup;
    
    static const int STEP_PER_SEC;
    static const int INT_PER_STEP;
    static const char TITLE[];
    
  private:
    dBodyID BodyFromConfig(const BodyConf& conf);
    void InitLeg(HobLeg& leg, ChainSegment upperLegC, ChainSegment lowerLegC,
             FootSegment footC);
    
    dGeomID fFloorG;
    dBodyID fBodyB;
    
    HobLeg fILeg, fOLeg;
    
    Spline fSwingSpl;
    double fSwingT;
    StepState fILegState, fOLegState;
};

#endif
