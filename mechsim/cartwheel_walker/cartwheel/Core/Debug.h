#pragma once

#include <Eigen/Dense>

class DebugInfo {
  public:
    int stance;
    double phi;
    
    Eigen::Vector3d desSwingPos, desSwingVel;
    
    double desOffCoronal;
    double desVelCoronal, desVelSagittal;
    double offCoronal;
    double velCoronal, velSagittal;
    
    double StanceFootWeightRatio;
    
    Eigen::Vector3d virtualRootTorque, virtualRootForce;
    
    double lFootNF, lFootTF;
    double rFootNF, rFootTF;
    
    Eigen::Vector3d lCoP, rCoP;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
