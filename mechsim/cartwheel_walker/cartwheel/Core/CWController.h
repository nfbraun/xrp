#pragma once

#include "RobotInfo.h"
#include "WorldOracle.h"
#include "StateMachine.h"
#include "IKVMCController.h"
#include "TurnController.h"
#include "WalkController.h"
#include "Controller.h"
#include "Debug.h"

#define USE_WALK_CONTROLLER

class CWController {
  public:
    CWController(Character* character, WorldOracle* worldOracle);
    void Init();
    JointSpTorques Run(double dt, const ContactData& cdata, double desiredHeading);
    
  public:
    Character* fCharacter;
    
    StateMachine fStateMachine;
    TorqueController fTorqueCtrl;
    InvPendulum fInvPendCtrl;
    IKVMCController fIKVMCCtrl;
    
#ifdef USE_WALK_CONTROLLER
    WalkController fHighCtrl;
#else
    TurnController fHighCtrl;
#endif
    
    DebugInfo fDbg;
};
