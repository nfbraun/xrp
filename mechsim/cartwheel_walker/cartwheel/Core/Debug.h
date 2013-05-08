#pragma once

#include "Character.h"

class DebugInfo {
  public:
    int stance;
    double phi;
  
    Vector3d desSwingPos, desSwingVel;
    
    double StanceFootWeightRatio;
    
    Vector3d virtualRootTorque, virtualRootForce;
    
    double lFootNF, lFootTF;
    double rFootNF, rFootTF;
    
    Vector3d lCoP, rCoP;
};
