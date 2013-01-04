#pragma once

#include "Character.h"

class DebugInfo {
  public:
    Vector3d desSwingPos, desSwingVel;
    
    double StanceFootWeightRatio;
    
    Vector3d virtualRootTorque, virtualRootForce;
};
