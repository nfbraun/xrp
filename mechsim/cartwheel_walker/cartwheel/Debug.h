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
    
    Eigen::Vector3d virtualRootTorque, virtualRootForce;
    
    double lFootNF, lFootTF;
    double rFootNF, rFootTF;
    
    Eigen::Vector3d lCoP, rCoP;
    
    double vrfGain;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
