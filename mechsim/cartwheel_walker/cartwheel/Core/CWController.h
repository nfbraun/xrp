#pragma once

#include "IKVMCController.h"
#include "TurnController.h"
#include "Controller.h"

class CWController {
  public:
    CWController(Character* character, WorldOracle* worldOracle);
    
    void requestHeading(double v) { fHighController->requestHeading(v); }
    
    JointTorques Run(double dt, std::vector<ContactPoint> *cfs);
    
  public:
    JointTorques performPreTasks(double dt, std::vector<ContactPoint> *cfs);
    void performPostTasks(double dt, std::vector<ContactPoint> *cfs);
    
    IKVMCController* fLowController;
    TurnController* fHighController;
};
