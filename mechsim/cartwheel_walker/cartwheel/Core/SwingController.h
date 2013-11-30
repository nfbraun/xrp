#pragma once

#include "Controller.h"
#include "RobotInfo.h"
#include "Debug.h"

class HighLevelTarget {
  public:
    double velDSagittal, velDCoronal;
    double desiredHeading;
    double swingFootHeight, swingFootHeightVel;
};

class IKSwingLegTarget {
  public:
    // only needed for testing with old code
    /* Quaternion swingHipOrient, swingKneeOrient;
    Vector3d swingHipAngVel, swingKneeAngVel; */
    
    double phz, phy, phx, pky, pay, pax;
    double ohz, ohy, ohx, oky, oay, oax;
};

class SwingController {
  public:
    DebugInfo* dbg;
    
    IKSwingLegTarget computeIKSwingLegTargets(const RobotInfo& rinfo, const Eigen::Vector3d& swingFootPos, const Eigen::Vector3d& swingFootVel, double swingFootHeight, double swingFootHeightVel);
    Eigen::Vector3d transformSwingFootTarget(const Eigen::Vector3d& step, const Eigen::Vector3d& comPos, const Eigen::Quaterniond& charFrameToWorld, double height);
    static RawTorques gravityCompensation(const RobotInfo& rinfo);
    static JSpTorques swingLegControl(const RobotInfo& rinfo, const IKSwingLegTarget& desiredPose);
    
    static void swingAnkleControl(JSpTorques& jt, const RobotInfo& rinfo);
};
