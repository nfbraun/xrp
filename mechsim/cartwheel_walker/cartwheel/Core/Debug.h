#pragma once

#include <MathLib/Vector3d.h>

class DebugInfo {
  public:
    int stance;
    double phi;
    
    Vector3d desSwingPos, desSwingVel;
    
    double desOffCoronal;
    double desVelCoronal, desVelSagittal;
    double offCoronal;
    double velCoronal, velSagittal;
    
    double StanceFootWeightRatio;
    
    Vector3d virtualRootTorque, virtualRootForce;
    
    double lFootNF, lFootTF;
    double rFootNF, rFootTF;
    
    Vector3d lCoP, rCoP;
};
