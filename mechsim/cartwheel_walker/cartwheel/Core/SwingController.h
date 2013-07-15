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
    
    IKSwingLegTarget computeIKSwingLegTargets(const RobotInfo& rinfo, const Vector3d& swingFootPos, const Vector3d& swingFootVel, double swingFootHeight, double swingFootHeightVel);
    Vector3d transformSwingFootTarget(Vector3d step, const Point3d& com, const Quaternion& charFrameToWorld, double height);
    static RawTorques gravityCompensation(const RobotInfo& rinfo);
    static void swingLegControl(JSpTorques& jt, const RobotInfo& rinfo, const IKSwingLegTarget& desiredPose);
    
    static void swingAnkleControl(JSpTorques& jt, const RobotInfo& rinfo);
};
