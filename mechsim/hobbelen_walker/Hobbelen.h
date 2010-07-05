#ifndef __MCGEER_H__
#define __MCGEER_H__

#include "Vector.h"
#include "Rotation.h"
#include "BodyConf.h"
#include "ChainSegment.h"
#include "FootSegment.h"
#include "CachedSimulation.h"
#include <ode/ode.h>

class BodyQ {
  public:
    BodyQ() {}
    BodyQ(Vector3 pos, Rotation rot)
        : fPos(pos), fRot(rot) {}
    static BodyQ FromODE(dBodyID id);
    void TransformGL() const;
    
  private:
    Vector3 fPos;
    Rotation fRot;
};

class Hobbelen;

class HobState: public SimulationState {
  public:
    Hobbelen* fParent;
    double fT;
    
    BodyQ fBodyQ;
    BodyQ fIULegQ, fILLegQ, fIFootQ;
    BodyQ fOULegQ, fOLLegQ, fOFootQ;
    
    void Draw() const;
    void DrawLeg(const BodyQ& upperLegQ, const BodyQ& lowerLegQ,
                       const BodyQ& footQ) const;
    
    Vector3 GetCenter() const { return Vector3(0., 0., 0.); }

  private:
    void DrawSlide() const;
    
    static const double DISP_LEGWIDTH;
    static const double DISP_SLIDEWIDTH;
    static const int DISP_SLIDELEN2;
};

class Hobbelen: public CachedSimulation<HobState> {
  public:
    Hobbelen();
    ~Hobbelen();
    
    double GetTimestep() { return 1./STEP_PER_SEC; }
    int GetDefaultEndTime() { return 60 * STEP_PER_SEC; }
    
    const char* GetTitle() { return TITLE; }
    
    void Advance();
    HobState GetCurrentState();
    
    void Collide(dGeomID g1, dGeomID g2);
    
    dWorldID fWorld;
    dJointGroupID fContactGroup;
    
    static const int STEP_PER_SEC;
    static const int INT_PER_STEP;
    static const char TITLE[];
    
  private:
    dBodyID BodyFromConfig(const BodyConf& conf);
    void InitLeg(dBodyID& upperLegB, dBodyID& lowerLegB,
             dBodyID& footB, dGeomID& fFootG, dGeomID& bFootG,
             ChainSegment upperLegC, ChainSegment lowerLegC,
             FootSegment footC);
    
    dGeomID fFloorG;
    dBodyID fBodyB;
    
    // Inner leg
    dBodyID fIULegB, fILLegB, fIFootB;
    dGeomID fIFFootG, fIBFootG;
    
    // Outer leg
    dBodyID fOULegB, fOLLegB, fOFootB;
    dGeomID fOFFootG, fOBFootG;

};

#endif
