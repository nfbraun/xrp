#pragma once

#include <Torques.h>
#include <cartwheel/RobotInfo.h>
#include <cartwheel/Debug.h>
#include <cartwheel/SwingController.h>

class TorqueController {
  public:
    JSpTorques computeTorques(const RobotInfo& rinfo, const IKSwingLegTarget& desiredPose, double comOffsetCoronal, double velDSagittal, double velDCoronal, double desiredHeading);
    
    DebugInfo* dbg;
    
    static JSpTorques transformLegTorques(unsigned int side, const RobotInfo& rinfo, const RawTorques& torques);
    
  private:
    JSpTorques stanceLegControl(const RobotInfo& rinfo, double desiredHeading);
    
    JSpTorques rootForceControl(const RobotInfo& rinfo, double comOffsetCoronal, double velDSagittal, double velDCoronal);
    
    Eigen::Vector3d calcCoP(const RobotInfo& rinfo, const Eigen::Vector3d& T, const Eigen::Vector3d& F);
    
    double calcMaxGain(const RobotInfo& rinfo, const JSpTorques& jt0, const JSpTorques& jt);
    
    /**
    This method is used to compute torques for the stance leg that help achieve a desired speed in the sagittal and lateral planes
    */
    void COMJT(const RobotInfo& rinfo, const Eigen::Vector3d& fA, Eigen::Vector3d& stanceAnkleTorque, Eigen::Vector3d& stanceKneeTorque, Eigen::Vector3d& stanceHipTorque);
    
    /**
    This method is used to compute the force that the COM of the character should be applying.
    */
    Eigen::Vector3d computeVirtualForce(const RobotInfo& rinfo, double desOffCoronal, double desVSagittal, double desVCoronal);
    
    /**
    This method is used to compute the torques that need to be applied to the stance and swing hips, given the desired orientation for the root and the swing hip. The coordinate frame that these orientations are expressed relative to is computed in this method. It is assumed that the stanceHipToSwingHipRatio variable is between 0 and 1, and it corresponds to the percentage of the total net vertical force that rests on the stance foot.
    */
    Eigen::Vector3d computeRootTorque(const RobotInfo& rinfo, double desHeading);
};
