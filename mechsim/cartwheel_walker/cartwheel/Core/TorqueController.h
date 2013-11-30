#pragma once

#include <Core/Controller.h>
#include <Core/RobotInfo.h>
#include <Core/ContactInfo.h>
#include <Core/Debug.h>
#include "SwingController.h"

class TorqueController {
  public:
    JSpTorques computeTorques(const RobotInfo& rinfo, const ContactInfo& cfs, const IKSwingLegTarget& desiredPose, double comOffsetCoronal, double velDSagittal, double velDCoronal, double desiredHeading);
    
    DebugInfo* dbg;
    
    static JSpTorques transformLegTorques(unsigned int side, const RobotInfo& rinfo, const RawTorques& torques);
    
  private:
    JSpTorques stanceLegControl(const RobotInfo& rinfo, const ContactInfo& cfs, double comOffsetCoronal, double velDSagittal, double velDCoronal, double desiredHeading);
    
    /**
    This method is used to compute torques for the stance leg that help achieve a desired speed in the sagittal and lateral planes
    */
    void COMJT(const RobotInfo& rinfo, const Eigen::Vector3d& fA, Eigen::Vector3d& stanceAnkleTorque, Eigen::Vector3d& stanceKneeTorque, Eigen::Vector3d& stanceHipTorque);
    
    /**
    This method is used to compute the force that the COM of the character should be applying.
    */
    Eigen::Vector3d computeVirtualForce(const RobotInfo& rinfo, double desOffCoronal, double desVSagittal, double desVCoronal);
    
    /**
    This method is used to return the ratio of the weight that is supported by the stance foot.
    */
    double getStanceFootWeightRatio(const RobotInfo& rinfo, const ContactInfo& cfs);
    
    /**
    This method returns performes some pre-processing on the virtual torque. The torque is assumed to be in world coordinates, and it will remain in world coordinates.
    */
    Eigen::Vector3d preprocessAnkleVTorque(const RobotInfo& rinfo, const ContactInfo& cfs, const Eigen::Vector3d& ankleVTorque, double phi);
    
    /**
    This method is used to compute the torques that need to be applied to the stance and swing hips, given the desired orientation for the root and the swing hip. The coordinate frame that these orientations are expressed relative to is computed in this method. It is assumed that the stanceHipToSwingHipRatio variable is between 0 and 1, and it corresponds to the percentage of the total net vertical force that rests on the stance foot.
    */
    Eigen::Vector3d computeRootTorque(const RobotInfo& rinfo, double desHeading);
};
