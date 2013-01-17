#include "CWRobot.h"
#include <MathLib/Vector3d.h>
#include <Physics/ArticulatedRigidBody.h>
#include <Physics/Joint.h>
#include <ode/ode.h>

dBodyID CWRobot::makeODEARB(dWorldID world, ArbID id, ArticulatedRigidBody* arb)
{
    dBodyID body = dBodyCreate(world);
    dMass ODEmass;
    ODEmass.setParameters(rbMass(id), 0, 0, 0, 
            rbMOI(id).x(), rbMOI(id).y(), rbMOI(id).z(),
            0, 0, 0);
    
    assert(rbMass(id) == arb->getMass());
    
    dBodySetMass(body, &ODEmass);
    
    dQuaternion tempQ;
    tempQ[0] = arb->getState().orientation.s;
    tempQ[1] = arb->getState().orientation.v.x();
    tempQ[2] = arb->getState().orientation.v.y();
    tempQ[3] = arb->getState().orientation.v.z();
    
    dBodySetPosition(body, arb->getState().position.x(), arb->getState().position.y(), arb->getState().position.z());
    dBodySetQuaternion(body, tempQ);

    fODEMap[arb] = body;
    
    return body;
}

dGeomID makeODEBoxGeom(double sx, double sy, double sz)
{
    dGeomID g = dCreateBox(0, sx, sy, sz);
    return g;
}

ArticulatedRigidBody* createARB(unsigned int id, const Vector3d& pos)
{
    ArticulatedRigidBody* arb = new ArticulatedRigidBody();
    arb->setMass(rbMass(id));
    arb->state.position = pos;
    
    return arb;
}

void CWRobot::create(dWorldID world)
{
    using namespace CharacterConst;
    const double unit = 1.0;
    
    fCharacter = new Character();
    
    ArticulatedRigidBody* pelvis = createARB(R_ROOT, Point3d(0, pelvisPosY, 0));
    fCharacter->setRoot( pelvis );
    
    ArticulatedRigidBody* lUpperLeg = createARB(R_L_UPPER_LEG,
        Point3d(legPosX_L, upperLegPosY, 0.));
    fCharacter->addArticulatedRigidBody( lUpperLeg, R_L_UPPER_LEG );
    
    Joint* joint2 = new Joint();
    joint2->setParentJointPosition(Point3d(pelvisRadius*legRelativeAnchorX, -pelvisSizeY/2.0, 0));
    joint2->setChildJointPosition(Point3d(0, upperLegSizeY/2.0, 0));
    joint2->setParent(pelvis);
    joint2->setChild(lUpperLeg);
    fCharacter->addJoint(joint2, J_L_HIP);
    
    ArticulatedRigidBody* rUpperLeg = createARB(R_R_UPPER_LEG,
        Point3d(legPosX_R, upperLegPosY, 0.));
    fCharacter->addArticulatedRigidBody( rUpperLeg, R_R_UPPER_LEG );
    
    Joint* joint3 = new Joint();
    joint3->setParentJointPosition(Point3d(-pelvisRadius*legRelativeAnchorX, -pelvisSizeY/2.0, 0));
    joint3->setChildJointPosition(Point3d(0, upperLegSizeY/2.0, 0));
    joint3->setParent(pelvis);
    joint3->setChild(rUpperLeg);
    fCharacter->addJoint(joint3, J_R_HIP);
    
    ArticulatedRigidBody* lLowerLeg = createARB(R_L_LOWER_LEG,
        Point3d(legPosX_L, lowerLegPosY, 0.));
    fCharacter->addArticulatedRigidBody( lLowerLeg, R_L_LOWER_LEG);
    
    Joint* joint4 = new Joint();
    joint4->setParentJointPosition(Point3d(0, -upperLegSizeY/2.0, 0));
    joint4->setChildJointPosition(Point3d(0, lowerLegSizeY/2.0, 0));
    joint4->setParent(lUpperLeg);
    joint4->setChild(lLowerLeg);
    fCharacter->addJoint(joint4, J_L_KNEE);
    
    ArticulatedRigidBody* rLowerLeg = createARB(R_R_LOWER_LEG,
        Point3d(legPosX_R, lowerLegPosY, 0.));
    fCharacter->addArticulatedRigidBody( rLowerLeg, R_R_LOWER_LEG );
    
    Joint* joint5 = new Joint();
    joint5->setParentJointPosition(Point3d(0, -upperLegSizeY/2.0, 0));
    joint5->setChildJointPosition(Point3d(0, lowerLegSizeY/2.0, 0));
    joint5->setParent(rUpperLeg);
    joint5->setChild(rLowerLeg);
    fCharacter->addJoint(joint5, J_R_KNEE);
    
    ArticulatedRigidBody* lFoot = createARB(R_L_FOOT,
        Point3d(legPosX_L, footPosY, 0.016));
    fCharacter->addArticulatedRigidBody( lFoot, R_L_FOOT );
    
    Joint* joint6 = new Joint();
    joint6->setParentJointPosition(Point3d(0, -lowerLegSizeY/2.0, 0));
    joint6->setChildJointPosition(Point3d(0, footSizeY/2.0, -footSizeZ*0.33 + legDiameter/2.0));
    joint6->setParent(lLowerLeg);
    joint6->setChild(lFoot);
    fCharacter->addJoint(joint6, J_L_ANKLE);
    
    ArticulatedRigidBody* rFoot = createARB(R_R_FOOT,
        Point3d(legPosX_R, footPosY, 0.016));
    fCharacter->addArticulatedRigidBody( rFoot, R_R_FOOT );
    
    Joint* joint7 = new Joint();
    joint7->setParentJointPosition(Point3d(0, -lowerLegSizeY/2.0, 0));
    joint7->setChildJointPosition(Point3d(0, footSizeY/2.0, -footSizeZ*0.33 + legDiameter/2.0));
    joint7->setParent(rLowerLeg);
    joint7->setChild(rFoot);
    fCharacter->addJoint(joint7, J_R_ANKLE);
    
    fPelvisB = makeODEARB(world, R_ROOT, pelvis);
    
    fLUpperLegB = makeODEARB(world, R_L_UPPER_LEG, lUpperLeg);
    fLLowerLegB = makeODEARB(world, R_L_LOWER_LEG, lLowerLeg);
    fRUpperLegB = makeODEARB(world, R_R_UPPER_LEG, rUpperLeg);
    fRLowerLegB = makeODEARB(world, R_R_LOWER_LEG, rLowerLeg);
    
    fLFootB = makeODEARB(world, R_L_FOOT, lFoot);
    fRFootB = makeODEARB(world, R_R_FOOT, rFoot);
    
    fLFootG = makeODEBoxGeom(footSizeX, footSizeY, footSizeZ);
    fRFootG = makeODEBoxGeom(footSizeX, footSizeY, footSizeZ);
    dGeomSetBody(fLFootG, fLFootB);
    dGeomSetBody(fRFootG, fRFootB);
    
    fLFootRB = lFoot;
    fRFootRB = rFoot;
    
    fLHipJ = dJointCreateBall(world, 0);
    dJointAttach(fLHipJ, fLUpperLegB, fPelvisB);
    dJointSetBallAnchor(fLHipJ, legPosX_L, hipPosY, 0.);
    
    //we'll assume that:
    //b is the twisting axis of the joint, and the joint limits will be (in magnitude) less than 90 degrees, otherwise
    //the simulation will go unstable!!!
    
    /* dJointID aMotor = dJointCreateAMotor(world, 0);
    dJointAttach(aMotor, fLUpperLegB, fPelvisB);
    dJointSetAMotorMode(aMotor, dAMotorEuler);
    
    dJointSetAMotorParam(aMotor, dParamStopCFM, 0.1);
    dJointSetAMotorParam(aMotor, dParamStopCFM2, 0.1);
    dJointSetAMotorParam(aMotor, dParamStopCFM3, 0.1);
    
    dJointSetAMotorAxis (aMotor, 0, 1, 1., 0., 0.);
    dJointSetAMotorAxis (aMotor, 2, 2, 0., 0., 1.);
    
    dJointSetAMotorParam(aMotor, dParamLoStop, -1.3);
    dJointSetAMotorParam(aMotor, dParamHiStop,  1.9);
    
    dJointSetAMotorParam(aMotor, dParamLoStop2, -1.0);
    dJointSetAMotorParam(aMotor, dParamHiStop2,  1.0);
    
    dJointSetAMotorParam(aMotor, dParamLoStop3, -1.0);
    dJointSetAMotorParam(aMotor, dParamHiStop3, 1.0); */
    
    fRHipJ = dJointCreateBall(world, 0);
    dJointAttach(fRHipJ, fRUpperLegB, fPelvisB);
    dJointSetBallAnchor(fRHipJ, legPosX_R, hipPosY, 0.);
    
    //we'll assume that:
    //b is the twisting axis of the joint, and the joint limits will be (in magnitude) less than 90 degrees, otherwise
    //the simulation will go unstable!!!
    
    /* dJointID aMotor = dJointCreateAMotor(world, 0);
    dJointAttach(aMotor, fLUpperLegB, fPelvisB);
    dJointSetAMotorMode(aMotor, dAMotorEuler);
    
    dJointSetAMotorParam(aMotor, dParamStopCFM, 0.1);
    dJointSetAMotorParam(aMotor, dParamStopCFM2, 0.1);
    dJointSetAMotorParam(aMotor, dParamStopCFM3, 0.1);
    
    dJointSetAMotorAxis (aMotor, 0, 1, 1., 0., 0.);
    dJointSetAMotorAxis (aMotor, 2, 2, 0., 0., 1.);
    
    dJointSetAMotorParam(aMotor, dParamLoStop, -1.3);
    dJointSetAMotorParam(aMotor, dParamHiStop,  1.9);
    
    dJointSetAMotorParam(aMotor, dParamLoStop2, -1.0);
    dJointSetAMotorParam(aMotor, dParamHiStop2,  1.0);
    
    dJointSetAMotorParam(aMotor, dParamLoStop3, -1.0);
    dJointSetAMotorParam(aMotor, dParamHiStop3, 1.0); */
    
    fLKneeJ = dJointCreateHinge(world, 0);
    dJointAttach(fLKneeJ, fLLowerLegB, fLUpperLegB);
    dJointSetHingeAnchor(fLKneeJ, legPosX_L, kneePosY, 0.);
    dJointSetHingeAxis(fLKneeJ, 1., 0., 0.);
    // setJointLimits(0, 2.5); //?
    
    fRKneeJ = dJointCreateHinge(world, 0);
    dJointAttach(fRKneeJ, fRLowerLegB, fRUpperLegB);
    dJointSetHingeAnchor(fRKneeJ, legPosX_R, kneePosY, 0.);
    dJointSetHingeAxis(fRKneeJ, 1., 0., 0.);
    // setJointLimits(0, 2.5); //?
    
    fLAnkleJ = dJointCreateUniversal(world, 0);
    dJointAttach(fLAnkleJ, fLFootB, fLLowerLegB);
    dJointSetUniversalAnchor(fLAnkleJ, legPosX_L, anklePosY, 0.);

    dJointSetUniversalAxis1(fLAnkleJ, 0., 0., 1.);
    dJointSetUniversalAxis2(fLAnkleJ, 1., 0., 0.);
    
    dJointSetUniversalParam(fLAnkleJ, dParamLoStop, -0.75);
    dJointSetUniversalParam(fLAnkleJ, dParamHiStop,  0.75);
    dJointSetUniversalParam(fLAnkleJ, dParamLoStop2, -0.75);
    dJointSetUniversalParam(fLAnkleJ, dParamHiStop2,  0.75);
    
    fRAnkleJ = dJointCreateUniversal(world, 0);
    dJointAttach(fRAnkleJ, fRFootB, fRLowerLegB);
    dJointSetUniversalAnchor(fRAnkleJ, legPosX_R, anklePosY, 0.);

    dJointSetUniversalAxis1(fRAnkleJ, 0., 0., -1.);
    dJointSetUniversalAxis2(fRAnkleJ, 1., 0., 0.);
    
    dJointSetUniversalParam(fRAnkleJ, dParamLoStop, -0.75);
    dJointSetUniversalParam(fRAnkleJ, dParamHiStop,  0.75);
    dJointSetUniversalParam(fRAnkleJ, dParamLoStop2, -0.75);
    dJointSetUniversalParam(fRAnkleJ, dParamHiStop2,  0.75);
    
    #ifdef DEBUG_FIXED_TORSO
    dJointID fix = dJointCreateFixed(world, 0);
    dJointAttach(fix, 0, fPelvisB);
    dJointSetFixed(fix);
    #endif
}

