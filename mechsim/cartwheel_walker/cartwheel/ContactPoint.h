#pragma once

#include <Eigen/Dense>

/**
    This class is mainly a container for a Contact Point. It holds information such as the world coordinates of the contact point, the normal at the contact, the rigid bodies that generated it, etc.
*/
class ContactPoint {
  public:
    // world coordinate of the origin of the contact force
    Eigen::Vector3d cp;
    
    // force applied (with f being applied to rb1, and -f to rb2)
    Eigen::Vector3d f;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

class ContactData {
  public:
    // Abuse Eigen::Matrix as alignment-safe storage class
    Eigen::Matrix<ContactPoint, Eigen::Dynamic, 1> pLeft;
    Eigen::Matrix<ContactPoint, Eigen::Dynamic, 1> pRight;
    
    // Left and right foot positions, in world coordinates
    Eigen::Vector3d lPos, rPos;
    
    // Total force and torque, in world coordinates
    Eigen::Vector3d lFtot, lTtot;
    Eigen::Vector3d rFtot, rTtot;
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};
