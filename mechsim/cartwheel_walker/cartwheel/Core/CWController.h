#pragma once

#include "RobotInfo.h"
#include "StateMachine.h"
#include "IKVMCController.h"
#include "TurnController.h"
#include "Controller.h"
#include "Debug.h"

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
    TurnController fTurnCtrl;
    
    DebugInfo fDbg;
};
