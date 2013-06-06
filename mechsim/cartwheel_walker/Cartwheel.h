#ifndef CW_CARTWHEEL_H
#define CW_CARTWHEEL_H

#include "Simulation.h"

#include "RobotState.h"
#include "CWRobot.h"
#include <Core/CWController.h>
#include <Core/Debug.h>
#include <Physics/ArticulatedFigure.h>
#include <ode/ode.h>

#include <Eigen/Dense>

extern const char* PhiNames[];

class Cartwheel;

class CartState: public SimulationState {
  public:
    Cartwheel* fParent;
    double fT;
    
    FullState fFState;
    JSpState fJState;
    
    Eigen::Vector3d fJPos[J_MAX];
    double fTorques[12];
    
    DebugInfo fDbg;
    
    void Draw(int mode) const;
    void DrawRobotOutline() const;
    void DrawRobot(bool shadowmode) const;
    
    Eigen::Vector3d GetCenter() const
        { return fFState.pos(B_PELVIS); }
    
    virtual double GetData(int ch) const {
        if(ch < DOF_MAX) {
            return fJState.phi(ch);
        } else if(ch < 2*DOF_MAX) {
            return fJState.omega(ch - DOF_MAX);
        } else {
            switch(ch) {
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
                
                //case 36: return fContactL;
                //case 37: return fContactR;
                
                default: return std::numeric_limits<double>::quiet_NaN();
            }
        }
    }
    
  private:
    void DrawSlide() const;
    
    static const double DISP_LEGWIDTH;
    static const double DISP_SLIDEWIDTH;
    static const int DISP_SLIDELEN2;
};

BodyQ QFromODE(dBodyID id);
void TransformGL(const BodyQ& q);

class Cartwheel: public Simulation {
  public:
    Cartwheel(unsigned int stepPerSec = 25, unsigned int intPerStep = 100);
    ~Cartwheel();
    
    double fT;
    double GetTimestep() { return 1./fStepPerSec; }
    int GetDefaultEndTime() { return 60 * fStepPerSec; }
    
    const char* GetTitle() { return TITLE; }
    
    virtual int GetNDataCh() const { return 36; }
    virtual const char* GetChName(int ch) const {
    // FIXME: make use of dofName()
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
            
            //case 36: return "con_L";
            //case 37: return "con_R";
            
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
    
    unsigned int Collide(dGeomID g1, dGeomID g2, std::vector<ContactPoint>& cps, dJointFeedback* feedback);
    
    dWorldID fWorld;
    dJointGroupID fContactGroup;
    
    const unsigned int fStepPerSec;
    const unsigned int fIntPerStep;
    
    static const char TITLE[];
    
  private:
    /* dBodyID BodyFromConfig(const BodyConf& conf);
    void InitLeg(HobLeg& leg, ChainSegment upperLegC, ChainSegment lowerLegC,
             FootSegment footC); */
    
    void AdvanceInTime(double dt, const JointSpTorques& torques);
    void ApplyTorques(const JointSpTorques& jt);
    void setRBState(RigidBody* rb, const BodyQ& q);
    
    void BodyAddTorque(dBodyID body, Vector3d torque);
    
    void LockStanceFoot(int stance);
    void SetFakeContactData(int stance);
    void SetFakeContactDataForFoot(std::vector<ContactPoint>& cpts, const Vector3d& pos);
    
    static const unsigned int MAX_CONTACTS = 4;
    dJointFeedback fLeftFeedback[MAX_CONTACTS];
    dJointFeedback fRightFeedback[MAX_CONTACTS];
    ContactData fCData;
    
    Character* fCharacter;
    CWRobot *fRobot;
    CWController *fController;
    
    JointSpTorques fDebugJTorques;
    
    RigidBody* fFloorRB;
    dGeomID fFloorG;
    
    int fStance;
    double fLastStanceSwitchTime;
    dJointID fLFootStickyJ, fRFootStickyJ;
};

#endif
