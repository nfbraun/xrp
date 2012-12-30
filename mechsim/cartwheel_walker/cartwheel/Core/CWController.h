#pragma once

#include "IKVMCController.h"
#include "TurnController.h"
#include "Controller.h"

class CWController {
  public:
    CWController(Character* character, WorldOracle* worldOracle);
    
    void requestHeading(double v) { fHighController->requestHeading(v); }
    
    JointSpTorques Run(double dt, std::vector<ContactPoint> *cfs);
    
  public:
    RawTorques performPreTasks(double dt, std::vector<ContactPoint> *cfs);
    void performPostTasks(double dt, std::vector<ContactPoint> *cfs);
    
    JointSpTorques transformTorques(const RawTorques& torques);
    
    Character* fCharacter;
    IKVMCController* fLowController;
    TurnController* fHighController;
};
