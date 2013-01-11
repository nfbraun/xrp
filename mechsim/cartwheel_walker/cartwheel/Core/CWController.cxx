#include "CWController.h"

void labelJoints(Character* character)
{
    for(unsigned int i=0; i<character->getJointCount(); i++) {
        character->getJoints()[i]->id = i;
    }
}

CWController::CWController(Character* character, WorldOracle* worldOracle)
    : fCharacter(character), fStateMachine(LEFT_STANCE), fTurnCtrl(worldOracle)
{
    labelJoints(character);
    
    fIKVMCCtrl.dbg = &fDbg;
    fTorqueCtrl.dbg = &fDbg;
    
    Init();
}

void CWController::Init()
{
    RobotInfo rinfo(fCharacter, fStateMachine.stance(), fStateMachine.phi());
    
    fTurnCtrl.requestHeading(rinfo, 0.);
    fTurnCtrl.conTransitionPlan(rinfo);
    fInvPendCtrl.setCoronalStepWidth(.3);
    fTurnCtrl.requestVelocities(.5, 0.);
}

JointSpTorques CWController::Run(double dt, const std::vector<ContactPoint>& cfs, double desiredHeading)
{
    RobotInfo rinfo(fCharacter, fStateMachine.stance(), fStateMachine.phi());
    
    bool newState = fStateMachine.advanceInTime(dt, fTurnCtrl.getStepTime(), rinfo, cfs);
    
    rinfo.setStance(fStateMachine.stance());
    rinfo.setPhi(fStateMachine.phi());
    
    fDbg.stance = fStateMachine.stance();
    fDbg.phi = fStateMachine.phi();
    
    fTurnCtrl.requestHeading(rinfo, desiredHeading);
    
    if( newState ) {
        fInvPendCtrl.setSwingFootStartPos(rinfo.swingFootPos());
        fTurnCtrl.conTransitionPlan(rinfo);
    }
    
    HighLevelTarget highTarget = fTurnCtrl.simStepPlan(rinfo, SimGlobals::dt);
    
    //compute desired swing foot location and velocity
    Vector3d desiredPos, desiredVel;
    fInvPendCtrl.calcDesiredSwingFootLocation(rinfo, highTarget.velDSagittal,
        highTarget.velDCoronal, desiredPos, desiredVel);
    
    IKSwingLegTarget swingLegTarget = fIKVMCCtrl.computeIKSwingLegTargets(rinfo,
        desiredPos, desiredVel, highTarget.swingFootHeight, highTarget.swingFootHeightVel);
    
    const double comOffsetCoronal = fInvPendCtrl.calcComOffsetCoronal(rinfo);
    return fTorqueCtrl.computeTorques(rinfo, cfs, swingLegTarget,
        comOffsetCoronal, highTarget.velDSagittal, highTarget.velDCoronal,
        highTarget.desiredHeading);
}
