#pragma once

#include "RobotInfo.h"
#include "WorldOracle.h"
#include "StateMachine.h"
#include "SwingController.h"
#include "TorqueController.h"
#include "TurnController.h"
#include "WalkController.h"
#include "Controller.h"
#include "Debug.h"

#define USE_WALK_CONTROLLER

class CWController {
  public:
    CWController(WorldOracle* worldOracle);
    void Init();
    JSpTorques Run(double dt, const FullState& fstate, const JSpState& jstate, const ContactData& cdata, double desiredHeading);
    
  public:
    StateMachine fStateMachine;
    TorqueController fTorqueCtrl;
    InvPendulum fInvPendCtrl;
    SwingController fSwingCtrl;
    
#ifdef USE_WALK_CONTROLLER
    WalkController fHighCtrl;
#else
    TurnController fHighCtrl;
#endif
    
    DebugInfo fDbg;
};
