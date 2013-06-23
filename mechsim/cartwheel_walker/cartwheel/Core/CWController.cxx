#include "CWController.h"
#include "../../DynTransform.h"

void labelJoints(Character* character)
{
    for(unsigned int i=0; i<character->getJointCount(); i++) {
        character->getJoints()[i]->id = i;
    }
}

CWController::CWController(WorldOracle* worldOracle)
    : fStateMachine(LEFT_STANCE),
#ifdef USE_WALK_CONTROLLER
      fHighCtrl(.5, 0.)
#else
      fHighCtrl(worldOracle)
#endif
{
    fCharacter = createCharacter();
    fCharacter->computeMass();
    
    labelJoints(fCharacter);
    
    fIKVMCCtrl.dbg = &fDbg;
    fTorqueCtrl.dbg = &fDbg;
    
    Init();
}

void CWController::Init()
{
    JSpState jnull = JSpState::Null();
    FullState fnull = fullFromJoint(jnull);
    
    RobotInfo rinfo(fCharacter, fnull, jnull, fStateMachine.stance(), fStateMachine.phi());
    
#ifndef USE_WALK_CONTROLLER
    fHighCtrl.requestHeading(rinfo, 0.);
    fHighCtrl.conTransitionPlan(rinfo);
#endif
    fInvPendCtrl.setCoronalStepWidth(.3);
#ifndef USE_WALK_CONTROLLER
    fHighCtrl.requestVelocities(.5, 0.);
#endif
}

ArticulatedRigidBody* createARB(unsigned int id, const Vector3d& pos)
{
    ArticulatedRigidBody* arb = new ArticulatedRigidBody();
    arb->setMass(rbMass(id));
    arb->state.position = pos;
    
    return arb;
}

Character* CWController::createCharacter()
{
    using namespace CharacterConst;
    
    Character* character = new Character();
    
    ArticulatedRigidBody* pelvis = createARB(B_PELVIS, Point3d(0, 0, pelvisPosZ));
    character->setRoot(pelvis);
    
    ArticulatedRigidBody* lThigh = createARB(B_L_THIGH,
        Point3d(0., legPosY_L, thighPosZ));
    character->addArticulatedRigidBody(lThigh, B_L_THIGH);
    
    Joint* joint2 = new Joint();
    joint2->setParentJointPosition(Point3d(0., pelvisRadius*legRelativeAnchorY, -pelvisSizeZ/2.0));
    joint2->setChildJointPosition(Point3d(0, 0, thighSizeZ/2.0));
    joint2->setParent(pelvis);
    joint2->setChild(lThigh);
    character->addJoint(joint2, J_L_HIP);
    
    ArticulatedRigidBody* rThigh = createARB(B_R_THIGH,
        Point3d(0., legPosY_R, thighPosZ));
    character->addArticulatedRigidBody(rThigh, B_R_THIGH);
    
    Joint* joint3 = new Joint();
    joint3->setParentJointPosition(Point3d(0., -pelvisRadius*legRelativeAnchorY, -pelvisSizeZ/2.0));
    joint3->setChildJointPosition(Point3d(0, 0, thighSizeZ/2.0));
    joint3->setParent(pelvis);
    joint3->setChild(rThigh);
    character->addJoint(joint3, J_R_HIP);
    
    ArticulatedRigidBody* lShank = createARB(B_L_SHANK,
        Point3d(0., legPosY_L, shankPosZ));
    character->addArticulatedRigidBody(lShank, B_L_SHANK);
    
    Joint* joint4 = new Joint();
    joint4->setParentJointPosition(Point3d(0, 0, -thighSizeZ/2.0));
    joint4->setChildJointPosition(Point3d(0, 0, shankSizeZ/2.0));
    joint4->setParent(lThigh);
    joint4->setChild(lShank);
    character->addJoint(joint4, J_L_KNEE);
    
    ArticulatedRigidBody* rShank = createARB(B_R_SHANK,
        Point3d(0., legPosY_R, shankPosZ));
    character->addArticulatedRigidBody(rShank, B_R_SHANK);
    
    Joint* joint5 = new Joint();
    joint5->setParentJointPosition(Point3d(0, 0, -thighSizeZ/2.0));
    joint5->setChildJointPosition(Point3d(0, 0, shankSizeZ/2.0));
    joint5->setParent(rThigh);
    joint5->setChild(rShank);
    character->addJoint(joint5, J_R_KNEE);
    
    ArticulatedRigidBody* lFoot = createARB(B_L_FOOT,
        Point3d(footPosX, legPosY_L, footPosZ));
    character->addArticulatedRigidBody( lFoot, B_L_FOOT );
    
    Joint* joint6 = new Joint();
    joint6->setParentJointPosition(Point3d(0, 0, -shankSizeZ/2.0));
    joint6->setChildJointPosition(Point3d(-footPosX, 0, footSizeZ/2.0));
    joint6->setParent(lShank);
    joint6->setChild(lFoot);
    character->addJoint(joint6, J_L_ANKLE);
    
    ArticulatedRigidBody* rFoot = createARB(B_R_FOOT,
        Point3d(footPosX, legPosY_R, footPosZ));
    character->addArticulatedRigidBody( rFoot, B_R_FOOT );
    
    Joint* joint7 = new Joint();
    joint7->setParentJointPosition(Point3d(0, 0, -shankSizeZ/2.0));
    joint7->setChildJointPosition(Point3d(-footPosX, 0, footSizeZ/2.0));
    joint7->setParent(rShank);
    joint7->setChild(rFoot);
    character->addJoint(joint7, J_R_ANKLE);
    
    return character;
}

void CWController::setRBState(RigidBody* rb, const BodyQ& q)
{
    rb->state.position = q.pos();
    
    rb->state.orientation.s = q.rot().w();
    rb->state.orientation.v.x() = q.rot().x();
    rb->state.orientation.v.y() = q.rot().y();
    rb->state.orientation.v.z() = q.rot().z();
    
    rb->state.velocity = q.vel();
    
    rb->state.angularVelocity = q.avel();
}

void CWController::updateCharacter(const FullState& fstate)
{
    //copy over the state of the ODE bodies to the rigid bodies...
    for(int rid=0; rid<B_MAX; rid++)
    {
        ArticulatedRigidBody* rb = fCharacter->getARBs()[rid];
        setRBState(rb, fstate.q(rid));
    }
}

JSpTorques CWController::Run(double dt, const FullState& fstate, const JSpState& jstate, const ContactData& cdata, double desiredHeading)
{
    updateCharacter(fstate);
    
    RobotInfo rinfo(fCharacter, fstate, jstate, fStateMachine.stance(), fStateMachine.phi());
    ContactInfo cinfo(cdata);
    
    bool newState = fStateMachine.advanceInTime(dt, fHighCtrl.getStepTime(), rinfo, cinfo);
    
    rinfo.setStance(fStateMachine.stance());
    rinfo.setPhi(fStateMachine.phi());
    
    fDbg.stance = fStateMachine.stance();
    fDbg.phi = fStateMachine.phi();
    
    fDbg.lFootNF = cinfo.getNormalForceOnFoot(B_L_FOOT);
    fDbg.lFootTF = cinfo.getTangentialForceOnFoot(B_L_FOOT);
    fDbg.rFootNF = cinfo.getNormalForceOnFoot(B_R_FOOT);
    fDbg.rFootTF = cinfo.getTangentialForceOnFoot(B_R_FOOT);
    
    ArticulatedRigidBody* lFootRB = rinfo.character()->getARBs()[B_L_FOOT];
    ArticulatedRigidBody* rFootRB = rinfo.character()->getARBs()[B_R_FOOT];
    
    fDbg.lCoP = cinfo.getCoP2(B_L_FOOT, lFootRB);
    fDbg.rCoP = cinfo.getCoP2(B_R_FOOT, rFootRB);
    
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
    
    fDbg.offCoronal = rinfo.getD().y();
    fDbg.velCoronal = rinfo.getV().y();
    fDbg.velSagittal = rinfo.getV().x();
    
    fDbg.desOffCoronal = comOffsetCoronal;
    fDbg.desVelCoronal = highTarget.velDCoronal;
    fDbg.desVelSagittal = highTarget.velDSagittal;
    
    return fTorqueCtrl.computeTorques(rinfo, cinfo, swingLegTarget,
        comOffsetCoronal, highTarget.velDSagittal, highTarget.velDCoronal,
        highTarget.desiredHeading);
}
