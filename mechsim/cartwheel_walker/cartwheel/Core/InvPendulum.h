#pragma once

#include <MathLib/Trajectory.h>
#include <MathLib/Segment.h>
#include <Core/RobotInfo.h>

class InvPendulum {
  public:
    InvPendulum();
    
    // determines the desired swing foot location
    void calcDesiredSwingFootLocation(const RobotInfo& rinfo, double velDSagittal, double velDCoronal, Eigen::Vector3d& desiredPos, Eigen::Vector3d& desiredVel);
    
    // desired offset of the CM relative to the stance foot/midpoint of the feet
    double calcComOffsetCoronal(const RobotInfo& rinfo);
    
    void setSwingFootStartPos(const Eigen::Vector3d& pos) { swingFootStartPos = pos; }
    
    void setCoronalStepWidth(double corSW) {
        coronalStepWidth = corSW;
    }
    
    // this method determines the degree to which the character should be panicking
    static double getPanicLevel(const RobotInfo& rinfo, double velDSagittal, double velDCoronal);
    
  private:
    // determine the estimate desired location of the swing foot, given the etimated position of the COM, and the phase
    Eigen::Vector3d computeSwingFootLocationEstimate(const RobotInfo& rinfo, const Eigen::Vector3d& comPos, double phase, double velDSagittal, double velDCoronal);
    
    /**
        returns the required stepping location, as predicted by the inverted pendulum model. The prediction is made
        on the assumption that the character will come to a stop by taking a step at that location. The step location
        is expressed in the character's frame coordinates.
    */
    Eigen::Vector3d computeIPStepLocation(const RobotInfo& rinfo);
    
    // modify the coronal location of the step so that the desired step width results.
    double adjustCoronalStepLocation(const RobotInfo& rinfo, double IPPrediction);
    
    double getCoronalPanicLevel(const RobotInfo& rinfo);
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
  private:
    double coronalStepWidth;
    
    //this is the length of the leg of the character controlled by this controller
    double legLength;
    
    //we need to keep track of the position of the swing foot at the beginning of the step
    Eigen::Vector3d swingFootStartPos;
};

