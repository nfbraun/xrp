#ifndef CW_CARTWHEEL_H
#define CW_CARTWHEEL_H

#include "Simulation.h"

#include "CWRobot.h"
#include "Controller.h"
#include <Core/TurnController.h>
#include <Physics/ArticulatedFigure.h>
#include <ode/ode.h>

#include <Eigen/Dense>

class BodyQ {
  public:
    BodyQ() {}
    BodyQ(Eigen::Vector3d pos, Eigen::Quaterniond rot,
          Eigen::Vector3d vel, Eigen::Vector3d avel)
        : fPos(pos), fRot(rot), fVel(vel), fAVel(avel) {}
    static BodyQ FromODE(dBodyID id);
    void TransformGL() const;
    
    inline Eigen::Vector3d pos() const { return fPos; }
    inline Eigen::Quaterniond rot() const { return fRot; }
    inline Eigen::Vector3d vel() const { return fVel; }
    inline Eigen::Vector3d avel() const { return fAVel; }
    
  private:
    Eigen::Vector3d fPos;
    Eigen::Quaterniond fRot;
    Eigen::Vector3d fVel;
    Eigen::Vector3d fAVel;
};

class Cartwheel;

class CartState: public SimulationState {
  public:
    Cartwheel* fParent;
    double fT;
    
    BodyQ fTorsoQ, fLowerBackQ, fPelvisQ;
    BodyQ fLUpperLegQ, fLLowerLegQ, fRUpperLegQ, fRLowerLegQ;
    BodyQ fLFootQ, fRFootQ;
    
    Eigen::Vector3d fJPos[J_MAX];
    Eigen::Vector3d fJTorques[J_MAX];
    
    Eigen::Vector3d fCoM;
    
    void Draw(int mode) const;
    void DrawRobotOutline() const;
    void DrawRobot(bool shadowmode) const;
    
    Eigen::Vector3d GetCenter() const
        { return Eigen::AngleAxis<double>(M_PI/2., Eigen::Vector3d::UnitX()) * fPelvisQ.pos(); }
    
    virtual double GetData(int ch) const {
        switch(ch) {
            case 0: return fCoM.x();
            case 1: return fCoM.y();
            case 2: return fCoM.z();
            default: return fJTorques[ch-3].norm();
            //default: return std::numeric_limits<double>::quiet_NaN();
        }
    }
    
  private:
    void DrawSlide() const;
    
    static const double DISP_LEGWIDTH;
    static const double DISP_SLIDEWIDTH;
    static const int DISP_SLIDELEN2;
};

class Cartwheel {
  public:
    Cartwheel();
    ~Cartwheel();
    
    double fT;
    double GetTimestep() { return 1./STEP_PER_SEC; }
    int GetDefaultEndTime() { return 60 * STEP_PER_SEC; }
    
    const char* GetTitle() { return TITLE; }
    
    virtual int GetNDataCh() const { return 3+J_MAX; }
    virtual const char* GetChName(int ch) const {
        switch(ch) {
            case 0:  return "CoM X";
            case 1:  return "CoM Y";
            case 2:  return "CoM Z";
            default: return "Joint";
        }
    }
    
    virtual int GetNDrawModes() const { return 2; }
    virtual const char* GetDrawModeName(int mode) {
        switch(mode) {
            case 0: return "Solid";
            case 1: return "Outline";
            default: return "";
        }
    }
    
    void Advance();
    CartState GetCurrentState();
    
    void Collide(dGeomID g1, dGeomID g2, RigidBody* rb1, RigidBody* rb2);
    
    dWorldID fWorld;
    dJointGroupID fContactGroup;
    
    static const int STEP_PER_SEC;
    static const int INT_PER_STEP;
    static const char TITLE[];
    
  private:
    /* dBodyID BodyFromConfig(const BodyConf& conf);
    void InitLeg(HobLeg& leg, ChainSegment upperLegC, ChainSegment lowerLegC,
             FootSegment footC); */
    
    void AdvanceInTime(double dt, const JointTorques& torques);
    void setRBStateFromODE(RigidBody* rb);
    
    void BodyAddTorque(dBodyID body, Vector3d torque);
    
    static const int MAX_CONTACT_FEEDBACK = 16;
    dJointFeedback fJointFeedback[MAX_CONTACT_FEEDBACK];
    int fJointFeedbackCount;
    std::vector<ContactPoint> fContactPoints;
    
    Character* fCharacter;
    IKVMCController* fLowController;
    TurnController* fHighController;
    CWRobot *fRobot;
    
    JointTorques fDebugJTorques;
    
    RigidBody* fFloorRB;
    dGeomID fFloorG;
    
    // HobLeg fLLeg, fRLeg;
};

#endif
