#ifndef CW_CARTWHEEL_H
#define CW_CARTWHEEL_H

#include "Simulation.h"

#include "RobotState.h"
#include "CWRobot.h"
#include <Core/CWController.h>
#include <Core/Debug.h>
#include <ode/ode.h>

#include <Eigen/Dense>

class Cartwheel;

class CartState: public SimulationState {
  public:
    Cartwheel* fParent;
    double fT;
    
    FullState fFState;
    JSpState fJState;
    
    double fPint;
    
    Eigen::Vector3d fJPos[J_MAX];
    
    JSpTorques fTorques;
    
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
        } else if(ch < 3*DOF_MAX) {
            return fTorques.t(ch - 2*DOF_MAX);
        } else {
            return std::numeric_limits<double>::quiet_NaN();
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
            case 0: return "p_LHZ";
            case 1: return "p_LHY";
            case 2: return "p_LHX";
            case 3: return "p_LKY";
            case 4: return "p_LAY";
            case 5: return "p_LAX";
            
            case 6: return "p_RHZ";
            case 7: return "p_RHY";
            case 8: return "p_RHX";
            case 9: return "p_RKY";
            case 10: return "p_RAY";
            case 11: return "p_RAX";
            
            case 12: return "o_LHZ";
            case 13: return "o_LHY";
            case 14: return "o_LHX";
            case 15: return "o_LKY";
            case 16: return "o_LAY";
            case 17: return "o_LAX";
            
            case 18: return "o_RHZ";
            case 19: return "o_RHY";
            case 20: return "o_RHX";
            case 21: return "o_RKY";
            case 22: return "o_RAY";
            case 23: return "o_RAX";
            
            case 24: return "t_LHZ";
            case 25: return "t_LHY";
            case 26: return "t_LHX";
            case 27: return "t_LKY";
            case 28: return "t_LAY";
            case 29: return "t_LAX";
            
            case 30: return "t_RHZ";
            case 31: return "t_RHY";
            case 32: return "t_RHX";
            case 33: return "t_RKY";
            case 34: return "t_RAY";
            case 35: return "t_RAX";
            
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
    void AdvanceInTime(double dt, const JSpTorques& torques);
    void ApplyTorques(const JSpTorques& jt);
    void setRBState(RigidBody* rb, const BodyQ& q);
    
    void BodyAddTorque(dBodyID body, Vector3d torque);
    
    void LockStanceFoot(int stance);
    void SetFakeContactData(int stance);
    void SetFakeContactDataForFoot(std::vector<ContactPoint>& cpts, const Vector3d& pos);
    
    static const unsigned int MAX_CONTACTS = 4;
    dJointFeedback fLeftFeedback[MAX_CONTACTS];
    dJointFeedback fRightFeedback[MAX_CONTACTS];
    ContactData fCData;
    
    CWRobot *fRobot;
    CWController *fController;
    
    JSpTorques fDebugJTorques;
    
    dGeomID fFloorG;
    
    int fStance;
    double fLastStanceSwitchTime;
    dJointID fLFootStickyJ, fRFootStickyJ;
    
    double fPint;
};

#endif
