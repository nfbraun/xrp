#include "CWController.h"

void labelJoints(Character* character)
{
    for(unsigned int i=0; i<character->getJointCount(); i++) {
        character->getJoints()[i]->id = i;
    }
}

CWController::CWController(Character* character, WorldOracle* worldOracle)
    : fCharacter(character), fStateMachine(LEFT_STANCE),
#ifdef USE_WALK_CONTROLLER
      fHighCtrl(.5, 0.)
#else
      fHighCtrl(worldOracle)
#endif
{
    labelJoints(character);
    
    fIKVMCCtrl.dbg = &fDbg;
    fTorqueCtrl.dbg = &fDbg;
    
    Init();
}

void CWController::Init()
{
    RobotInfo rinfo(fCharacter, fStateMachine.stance(), fStateMachine.phi());
    
#ifndef USE_WALK_CONTROLLER
    fHighCtrl.requestHeading(rinfo, 0.);
    fHighCtrl.conTransitionPlan(rinfo);
#endif
    fInvPendCtrl.setCoronalStepWidth(.3);
#ifndef USE_WALK_CONTROLLER
    fHighCtrl.requestVelocities(.5, 0.);
#endif
}

JointSpTorques CWController::Run(double dt, const ContactData& cdata, double desiredHeading)
{
    RobotInfo rinfo(fCharacter, fStateMachine.stance(), fStateMachine.phi());
    ContactInfo cinfo(cdata);
    
    bool newState = fStateMachine.advanceInTime(dt, fHighCtrl.getStepTime(), rinfo, cinfo);
    
    rinfo.setStance(fStateMachine.stance());
    rinfo.setPhi(fStateMachine.phi());
    
    fDbg.stance = fStateMachine.stance();
    fDbg.phi = fStateMachine.phi();
    
    fDbg.lFootNF = cinfo.getNormalForceOnFoot(R_L_FOOT);
    fDbg.lFootTF = cinfo.getTangentialForceOnFoot(R_L_FOOT);
    fDbg.rFootNF = cinfo.getNormalForceOnFoot(R_R_FOOT);
    fDbg.rFootTF = cinfo.getTangentialForceOnFoot(R_R_FOOT);
    
#ifndef USE_WALK_CONTROLLER
    fHighCtrl.requestHeading(rinfo, desiredHeading);
#endif
    
    if( newState ) {
        fInvPendCtrl.setSwingFootStartPos(rinfo.swingFootPos());
#ifndef USE_WALK_CONTROLLER
        fHighCtrl.conTransitionPlan(rinfo);
#endif
    }
    
    HighLevelTarget highTarget = fHighCtrl.simStepPlan(rinfo, SimGlobals::dt);
    
    //compute desired swing foot location and velocity
    Vector3d desiredPos, desiredVel;
    fInvPendCtrl.calcDesiredSwingFootLocation(rinfo, highTarget.velDSagittal,
        highTarget.velDCoronal, desiredPos, desiredVel);
    
    IKSwingLegTarget swingLegTarget = fIKVMCCtrl.computeIKSwingLegTargets(rinfo,
        desiredPos, desiredVel, highTarget.swingFootHeight, highTarget.swingFootHeightVel);
    
    const double comOffsetCoronal = fInvPendCtrl.calcComOffsetCoronal(rinfo);
    return fTorqueCtrl.computeTorques(rinfo, cinfo, swingLegTarget,
        comOffsetCoronal, highTarget.velDSagittal, highTarget.velDCoronal,
        highTarget.desiredHeading);
}
