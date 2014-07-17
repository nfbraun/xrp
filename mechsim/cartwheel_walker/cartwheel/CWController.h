#pragma once

#include <cartwheel/RobotInfo.h>
#include <cartwheel/WorldOracle.h>
#include <cartwheel/StateMachine.h>
#include <cartwheel/SwingController.h>
#include <cartwheel/TorqueController.h>
#include <cartwheel/TurnController.h>
#include <cartwheel/WalkController.h>
#include <Torques.h>
#include <cartwheel/Debug.h>

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
