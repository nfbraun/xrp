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

enum PhiID { LH0, LH1, LH2, LK, LA0, LA1, RH0, RH1, RH2, RK, RA0, RA1 };
extern const char* PhiNames[];

class RobotState {
  public:
    double phi[12];
    double omega[12];
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
    
    RobotState fRobot;
    double fTorques[12];
    
    double fPhi;
    int fStance;
    Eigen::Vector3d fDesSwingPos, fDesSwingVel;
    
    bool fContactL, fContactR;
    
    void Draw(int mode) const;
    void DrawRobotOutline() const;
    void DrawRobot(bool shadowmode) const;
    
    Eigen::Vector3d GetCenter() const
        { return Eigen::AngleAxis<double>(M_PI/2., Eigen::Vector3d::UnitX()) * fPelvisQ.pos(); }
    
    virtual double GetData(int ch) const {
        switch(ch) {
            case 0: return fRobot.phi[LH0];
            case 1: return fRobot.phi[LH1];
            case 2: return fRobot.phi[LH2];
            case 3: return fRobot.phi[LK];
            case 4: return fRobot.phi[LA0];
            case 5: return fRobot.phi[LA1];
            
            case 6: return fRobot.phi[RH0];
            case 7: return fRobot.phi[RH1];
            case 8: return fRobot.phi[RH2];
            case 9: return fRobot.phi[RK];
            case 10: return fRobot.phi[RA0];
            case 11: return fRobot.phi[RA1];
            
            case 12: return fRobot.omega[LH0];
            case 13: return fRobot.omega[LH1];
            case 14: return fRobot.omega[LH2];
            case 15: return fRobot.omega[LK];
            case 16: return fRobot.omega[LA0];
            case 17: return fRobot.omega[LA1];
            
            case 18: return fRobot.omega[RH0];
            case 19: return fRobot.omega[RH1];
            case 20: return fRobot.omega[RH2];
            case 21: return fRobot.omega[RK];
            case 22: return fRobot.omega[RA0];
            case 23: return fRobot.omega[RA1];
            
            case 24: return fTorques[LH0];
            case 25: return fTorques[LH1];
            case 26: return fTorques[LH2];
            case 27: return fTorques[LK];
            case 28: return fTorques[LA0];
            case 29: return fTorques[LA1];
            
            case 30: return fTorques[RH0];
            case 31: return fTorques[RH1];
            case 32: return fTorques[RH2];
            case 33: return fTorques[RK];
            case 34: return fTorques[RA0];
            case 35: return fTorques[RA1];
            
            case 36: return fContactL;
            case 37: return fContactR;
            
            default: return std::numeric_limits<double>::quiet_NaN();
        }
    }
    
  private:
    void DrawSlide() const;
    
    static const double DISP_LEGWIDTH;
    static const double DISP_SLIDEWIDTH;
    static const int DISP_SLIDELEN2;
};

class Cartwheel: public Simulation {
  public:
    Cartwheel(unsigned int stepPerSec = 25, unsigned int intPerStep = 100);
    ~Cartwheel();
    
    double fT;
    double GetTimestep() { return 1./fStepPerSec; }
    int GetDefaultEndTime() { return 60 * fStepPerSec; }
    
    const char* GetTitle() { return TITLE; }
    
    virtual int GetNDataCh() const { return 38; }
    virtual const char* GetChName(int ch) const {
        switch(ch) {
            case 0: return "p_LH0";
            case 1: return "p_LH1";
            case 2: return "p_LH2";
            case 3: return "p_LK";
            case 4: return "p_LA0";
            case 5: return "p_LA1";
            
            case 6: return "p_RH0";
            case 7: return "p_RH1";
            case 8: return "p_RH2";
            case 9: return "p_RK";
            case 10: return "p_RA0";
            case 11: return "p_RA1";
            
            case 12: return "o_LH0";
            case 13: return "o_LH1";
            case 14: return "o_LH2";
            case 15: return "o_LK";
            case 16: return "o_LA0";
            case 17: return "o_LA1";
            
            case 18: return "o_RH0";
            case 19: return "o_RH1";
            case 20: return "o_RH2";
            case 21: return "o_RK";
            case 22: return "o_RA0";
            case 23: return "o_RA1";
            
            case 24: return "t_LH0";
            case 25: return "t_LH1";
            case 26: return "t_LH2";
            case 27: return "t_LK";
            case 28: return "t_LA0";
            case 29: return "t_LA1";
            
            case 30: return "t_RH0";
            case 31: return "t_RH1";
            case 32: return "t_RH2";
            case 33: return "t_RK";
            case 34: return "t_RA0";
            case 35: return "t_RA1";
            
            case 36: return "con_L";
            case 37: return "con_R";
            
            default: return "<undefined>";
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
    
    bool Collide(dGeomID g1, dGeomID g2, RigidBody* rb1, RigidBody* rb2);
    
    dWorldID fWorld;
    dJointGroupID fContactGroup;
    
    const unsigned int fStepPerSec;
    const unsigned int fIntPerStep;
    
    static const char TITLE[];
    
  private:
    /* dBodyID BodyFromConfig(const BodyConf& conf);
    void InitLeg(HobLeg& leg, ChainSegment upperLegC, ChainSegment lowerLegC,
             FootSegment footC); */
    
    void AdvanceInTime(double dt, const JointTorques& torques);
    void ApplyTorques(const JointTorques& torques);
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
    bool fDebugContactL, fDebugContactR;
    
    RigidBody* fFloorRB;
    dGeomID fFloorG;
};

#endif
