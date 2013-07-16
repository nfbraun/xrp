#pragma once

#include <MathLib/Quaternion.h>
#include "SimGlobals.h"
#include <Physics/PhysicsGlobals.h>
#include "../../RobotState.h"
#include "../../DynInfo.h"

class RobotInfo {
  public:
    RobotInfo(const FullState& fstate, const JSpState& jstate, int st, double p): fFState(fstate), fJState(jstate), fStance(st), fPhi(p) {}
    
    int stance() const { return fStance; }
    void setStance(int st) { fStance = st; }
    
    double phi() const { return fPhi; }
    void setPhi(double p) { fPhi = p; }
    
    const FullState& fstate() const { return fFState; }
    const JSpState& jstate() const { return fJState; }
    
    Vector3d comPos() const { return ::comPos(fstate()); }
    Vector3d comVel() const { return ::comVel(fstate()); }
    
    /***
    This quaternion gives the current heading of the character. The complex conjugate of this orientation is used to transform quantities from world coordinates into a rotation/heading-independent coordinate frame (called the character frame).
    I will make the asumption that the character frame is a coordinate frame that is aligned with the vertical axis, but has 0 heading, and the characterFrame quaternion is used to rotate vectors from the character frame to the real world frame.
    ***/
    Quaternion characterFrame() const;
    double headingAngle() const;
    
    Quaternion rootOrient() const { return fstate().rot(B_PELVIS); }
    Vector3d rootAngVel() const { return fstate().avel(B_PELVIS); }
    
    // FIXME: remove once transition to Eigen is complete.
    Vector3d rbPos(unsigned int index) const { return fstate().pos(index); }
    
    // Return joint position in global coordinates
    Vector3d jPos(unsigned int index) const
        { return fstate().trToWorld(jParent(index)).onPoint(jPos_PF(index)); }
    
    unsigned int rootIndex() const { return B_PELVIS; }
    
    unsigned int stanceThighIndex() const
        { return (fStance == LEFT_STANCE) ? (B_L_THIGH) : (B_R_THIGH); }
    unsigned int swingThighIndex() const
        { return (fStance == LEFT_STANCE) ? (B_R_THIGH) : (B_L_THIGH); }
    unsigned int stanceShankIndex() const
        { return (fStance == LEFT_STANCE) ? (B_L_SHANK) : (B_R_SHANK); }
    unsigned int swingShankIndex() const
        { return (fStance == LEFT_STANCE) ? (B_R_SHANK) : (B_L_SHANK); }
    unsigned int stanceFootIndex() const
        { return (fStance == LEFT_STANCE) ? (B_L_FOOT) : (B_R_FOOT); }
    unsigned int swingFootIndex() const
        { return (fStance == LEFT_STANCE) ? (B_R_FOOT) : (B_L_FOOT); }
    
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
    Point3d stanceFootPos() const {
        return fstate().pos(stanceFootIndex());
    }
    Vector3d stanceFootVel() const {
        return fstate().vel(stanceFootIndex());
    }

    /**
    This method returns the position of the CM of the swing foot, in world coordinates
    */
    Point3d swingFootPos() const {
        return fstate().pos(swingFootIndex());
    }
    Vector3d swingFootVel() const {
        return fstate().vel(swingFootIndex());
    }
    
    // this is the vector from the cm of the stance foot to the cm of the character
    Vector3d getD() const
        { return characterFrame().inverseRotate(comPos() - stanceFootPos()); }
    
    // this is the velocity of the cm of the character, in character frame
    Vector3d getV() const
        { return characterFrame().inverseRotate(comVel()); }
    
  public:
    int fStance;
    double fPhi;
    
    FullState fFState;
    JSpState fJState;
};
