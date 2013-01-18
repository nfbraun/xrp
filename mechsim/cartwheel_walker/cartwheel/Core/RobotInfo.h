#pragma once

#include <Core/Character.h>

class RobotInfo {
  public:
    RobotInfo(Character* c, int st, double p): fCharacter(c), fStance(st), fPhi(p) {}
    
    const Character* character() const { return fCharacter; }
    
    int stance() const { return fStance; }
    void setStance(int st) { fStance = st; }
    
    double phi() const { return fPhi; }
    void setPhi(double p) { fPhi = p; }
    
    double totalMass() const { return fCharacter->getMass(); }
    
    Vector3d comPos() const { return fCharacter->getCOM(); }
    Vector3d comVel() const { return fCharacter->getCOMVelocity(); }
    
    /***
    This quaternion gives the current heading of the character. The complex conjugate of this orientation is used to transform quantities from world coordinates into a rotation/heading-independent coordinate frame (called the character frame).
    I will make the asumption that the character frame is a coordinate frame that is aligned with the vertical axis, but has 0 heading, and the characterFrame quaternion is used to rotate vectors from the character frame to the real world frame.
    ***/
    Quaternion characterFrame() const { return fCharacter->getHeading(); }
    double headingAngle() const { return fCharacter->getHeadingAngle(); }
    
    RigidBody* root() const { return fCharacter->getRoot(); }
    RigidBody* lFoot() const { return fCharacter->getARB(R_L_FOOT); }
    RigidBody* rFoot() const { return fCharacter->getARB(R_R_FOOT); }
    
    Quaternion rootOrient() const { return root()->getOrientation(); }
    Vector3d rootAngVel() const { return root()->getAngularVelocity(); }
    
    double rbMass(unsigned int index) const
        { assert(index < R_MAX); return fCharacter->getArticulatedRigidBody(index)->getMass(); }
    
    Vector3d rbPos(unsigned int index) const
        { assert(index < R_MAX); return fCharacter->getArticulatedRigidBody(index)->state.position; }
    
    // Return joint position in global coordinates
    Vector3d jPos(unsigned int index) const
    {
        assert(index < J_MAX);
        
        Vector3d pc = fCharacter->getJoints()[index]->child->getWorldCoordinatesForPoint(fCharacter->getJoints()[index]->getChildJointPosition());
        Vector3d pp = fCharacter->getJoints()[index]->parent->getWorldCoordinatesForPoint(fCharacter->getJoints()[index]->getParentJointPosition());
        
        assert((pp - pc).norm() < 1e-3);
        
        return pc;
    }
    
    unsigned int rootIndex() const { return R_ROOT; }
    
    unsigned int stanceThighIndex() const
        { return (fStance == LEFT_STANCE) ? (R_L_UPPER_LEG) : (R_R_UPPER_LEG); }
    unsigned int swingThighIndex() const
        { return (fStance == LEFT_STANCE) ? (R_R_UPPER_LEG) : (R_L_UPPER_LEG); }
    unsigned int stanceShankIndex() const
        { return (fStance == LEFT_STANCE) ? (R_L_LOWER_LEG) : (R_R_LOWER_LEG); }
    unsigned int swingShankIndex() const
        { return (fStance == LEFT_STANCE) ? (R_R_LOWER_LEG) : (R_L_LOWER_LEG); }
    unsigned int stanceFootIndex() const
        { return (fStance == LEFT_STANCE) ? (R_L_FOOT) : (R_R_FOOT); }
    unsigned int swingFootIndex() const
        { return (fStance == LEFT_STANCE) ? (R_R_FOOT) : (R_L_FOOT); }
    
    unsigned int stanceHipIndex() const
        { return (fStance == LEFT_STANCE) ? (J_L_HIP) : (J_R_HIP); }
    unsigned int swingHipIndex() const
        { return (fStance == LEFT_STANCE) ? (J_R_HIP) : (J_L_HIP); }
    unsigned int stanceKneeIndex() const
        { return (fStance == LEFT_STANCE) ? (J_L_KNEE) : (J_R_KNEE); }
    unsigned int swingKneeIndex() const
        { return (fStance == LEFT_STANCE) ? (J_R_KNEE) : (J_L_KNEE); }
    unsigned int stanceAnkleIndex() const
        { return (fStance == LEFT_STANCE) ? (J_L_ANKLE) : (J_R_ANKLE); }
    unsigned int swingAnkleIndex() const
        { return (fStance == LEFT_STANCE) ? (J_R_ANKLE) : (J_L_ANKLE); }

    /**
    This method returns the position of the CM of the stance foot, in world coordinates
    */
    RigidBody* stanceFoot() const
        { return (fStance == LEFT_STANCE) ? lFoot() : rFoot(); }
    Point3d stanceFootPos() const { return stanceFoot()->getCMPosition(); }
    Vector3d stanceFootVel() const { return stanceFoot()->getCMVelocity(); }

    /**
    This method returns the position of the CM of the swing foot, in world coordinates
    */
    RigidBody* swingFoot() const
        { return (fStance == LEFT_STANCE) ? rFoot() : lFoot(); }
    Point3d swingFootPos() const { return swingFoot()->getCMPosition(); }
    Vector3d swingFootVel() const { return swingFoot()->getCMVelocity(); }
    
    // this is the vector from the cm of the stance foot to the cm of the character
    Vector3d getD() const
        { return characterFrame().inverseRotate(comPos() - stanceFootPos()); }
    
    // this is the velocity of the cm of the character, in character frame
    Vector3d getV() const
        { return characterFrame().inverseRotate(comVel()); }
    
  public:
    Character* fCharacter;
    int fStance;
    double fPhi;
};
