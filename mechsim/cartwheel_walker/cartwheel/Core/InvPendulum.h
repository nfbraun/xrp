#pragma once

#include <MathLib/Trajectory.h>
#include <MathLib/Segment.h>
#include <Core/RobotInfo.h>

class InvPendulum {
  public:
    InvPendulum();
    
    // determines the desired swing foot location
    void calcDesiredSwingFootLocation(const RobotInfo& rinfo, double velDSagittal, double velDCoronal, Vector3d& desiredPos, Vector3d& desiredVel);
    
    // desired offset of the CM relative to the stance foot/midpoint of the feet
    double calcComOffsetCoronal(const RobotInfo& rinfo);
    
    void setSwingFootStartPos(const Point3d& pos) { swingFootStartPos = pos; }
    
    void setCoronalStepWidth(double corSW) {
        coronalStepWidth = corSW;
    }
    
    // this method determines the degree to which the character should be panicking
    static double getPanicLevel(const RobotInfo& rinfo, double velDSagittal, double velDCoronal);
    
  private:
    // determine the estimate desired location of the swing foot, given the etimated position of the COM, and the phase
    Vector3d computeSwingFootLocationEstimate(const RobotInfo& rinfo, const Point3d& comPos, double phase, double velDSagittal, double velDCoronal);
    
    /**
        returns the required stepping location, as predicted by the inverted pendulum model. The prediction is made
        on the assumption that the character will come to a stop by taking a step at that location. The step location
        is expressed in the character's frame coordinates.
    */
    Vector3d computeIPStepLocation(const RobotInfo& rinfo);
    
    // modify the coronal location of the step so that the desired step width results.
    double adjustCoronalStepLocation(const RobotInfo& rinfo, double IPPrediction);
    
    /**
    determines weather a leg crossing is bound to happen or not, given the predicted final desired position	of the swing foot.
    
    The suggested via point is expressed in the character frame, relative to the COM position... The via point is only suggested
    if an intersection is detected.
    */
    bool detectPossibleLegCrossing(const RobotInfo& rinfo, const Vector3d& swingFootPos, Vector3d* viaPoint);
    
    double getCoronalPanicLevel(const RobotInfo& rinfo);
    
  private:
    double coronalStepWidth;
    
    //alternate planned foot trajectory, for cases where we need to go around the stance foot...
    Trajectory3d alternateFootTraj;
    
    bool shouldPreventLegIntersections;
    
    //this is the length of the leg of the character controlled by this controller
    double legLength;
    
    //we need to keep track of the position of the swing foot at the beginning of the step
    Point3d swingFootStartPos;
};

