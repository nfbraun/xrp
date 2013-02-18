#include "CWRobot.h"
#include <MathLib/Vector3d.h>
#include <Physics/ArticulatedRigidBody.h>
#include <Physics/Joint.h>
#include <ode/ode.h>

dBodyID CWRobot::makeODEARB(dWorldID world, BodyID id, ArticulatedRigidBody* arb)
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
    
    ArticulatedRigidBody* pelvis = createARB(B_PELVIS, Point3d(0, 0, pelvisPosZ));
    fCharacter->setRoot(pelvis);
    
    ArticulatedRigidBody* lThigh = createARB(B_L_THIGH,
        Point3d(0., legPosY_L, thighPosZ));
    fCharacter->addArticulatedRigidBody(lThigh, B_L_THIGH);
    
    Joint* joint2 = new Joint();
    joint2->setParentJointPosition(Point3d(0., pelvisRadius*legRelativeAnchorY, -pelvisSizeZ/2.0));
    joint2->setChildJointPosition(Point3d(0, 0, thighSizeZ/2.0));
    joint2->setParent(pelvis);
    joint2->setChild(lThigh);
    fCharacter->addJoint(joint2, J_L_HIP);
    
    ArticulatedRigidBody* rThigh = createARB(B_R_THIGH,
        Point3d(0., legPosY_R, thighPosZ));
    fCharacter->addArticulatedRigidBody(rThigh, B_R_THIGH);
    
    Joint* joint3 = new Joint();
    joint3->setParentJointPosition(Point3d(0., -pelvisRadius*legRelativeAnchorY, -pelvisSizeZ/2.0));
    joint3->setChildJointPosition(Point3d(0, 0, thighSizeZ/2.0));
    joint3->setParent(pelvis);
    joint3->setChild(rThigh);
    fCharacter->addJoint(joint3, J_R_HIP);
    
    ArticulatedRigidBody* lShank = createARB(B_L_SHANK,
        Point3d(0., legPosY_L, shankPosZ));
    fCharacter->addArticulatedRigidBody(lShank, B_L_SHANK);
    
    Joint* joint4 = new Joint();
    joint4->setParentJointPosition(Point3d(0, 0, -thighSizeZ/2.0));
    joint4->setChildJointPosition(Point3d(0, 0, shankSizeZ/2.0));
    joint4->setParent(lThigh);
    joint4->setChild(lShank);
    fCharacter->addJoint(joint4, J_L_KNEE);
    
    ArticulatedRigidBody* rShank = createARB(B_R_SHANK,
        Point3d(0., legPosY_R, shankPosZ));
    fCharacter->addArticulatedRigidBody(rShank, B_R_SHANK);
    
    Joint* joint5 = new Joint();
    joint5->setParentJointPosition(Point3d(0, 0, -thighSizeZ/2.0));
    joint5->setChildJointPosition(Point3d(0, 0, shankSizeZ/2.0));
    joint5->setParent(rThigh);
    joint5->setChild(rShank);
    fCharacter->addJoint(joint5, J_R_KNEE);
    
    ArticulatedRigidBody* lFoot = createARB(B_L_FOOT,
        Point3d(footPosX, legPosY_L, footPosZ));
    fCharacter->addArticulatedRigidBody( lFoot, B_L_FOOT );
    
    Joint* joint6 = new Joint();
    joint6->setParentJointPosition(Point3d(0, 0, -shankSizeZ/2.0));
    joint6->setChildJointPosition(Point3d(-footPosX, 0, footSizeZ/2.0));
    joint6->setParent(lShank);
    joint6->setChild(lFoot);
    fCharacter->addJoint(joint6, J_L_ANKLE);
    
    ArticulatedRigidBody* rFoot = createARB(B_R_FOOT,
        Point3d(footPosX, legPosY_R, footPosZ));
    fCharacter->addArticulatedRigidBody( rFoot, B_R_FOOT );
    
    Joint* joint7 = new Joint();
    joint7->setParentJointPosition(Point3d(0, 0, -shankSizeZ/2.0));
    joint7->setChildJointPosition(Point3d(-footPosX, 0, footSizeZ/2.0));
    joint7->setParent(rShank);
    joint7->setChild(rFoot);
    fCharacter->addJoint(joint7, J_R_ANKLE);
    
    fBodies[B_PELVIS] = makeODEARB(world, B_PELVIS, pelvis);
    
    fBodies[B_L_THIGH] = makeODEARB(world, B_L_THIGH, lThigh);
    fBodies[B_L_SHANK] = makeODEARB(world, B_L_SHANK, lShank);
    fBodies[B_R_THIGH] = makeODEARB(world, B_R_THIGH, rThigh);
    fBodies[B_R_SHANK] = makeODEARB(world, B_R_SHANK, rShank);
    
    fBodies[B_L_FOOT] = makeODEARB(world, B_L_FOOT, lFoot);
    fBodies[B_R_FOOT] = makeODEARB(world, B_R_FOOT, rFoot);
    
    fLFootG = makeODEBoxGeom(footSizeX, footSizeY, footSizeZ);
    fRFootG = makeODEBoxGeom(footSizeX, footSizeY, footSizeZ);
    dGeomSetBody(fLFootG, fBodies[B_L_FOOT]);
    dGeomSetBody(fRFootG, fBodies[B_R_FOOT]);
    
    fLHipJ = dJointCreateBall(world, 0);
    dJointAttach(fLHipJ, fBodies[B_L_THIGH], fBodies[B_PELVIS]);
    dJointSetBallAnchor(fLHipJ, 0., legPosY_L, hipPosZ);
    
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
    dJointAttach(fRHipJ, fBodies[B_R_THIGH], fBodies[B_PELVIS]);
    dJointSetBallAnchor(fRHipJ, 0., legPosY_R, hipPosZ);
    
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
    dJointAttach(fLKneeJ, fBodies[B_L_SHANK], fBodies[B_L_THIGH]);
    dJointSetHingeAnchor(fLKneeJ, 0., legPosY_L, kneePosZ);
    dJointSetHingeAxis(fLKneeJ, 0., 1., 0.);
    // setJointLimits(0, 2.5); //?
    
    fRKneeJ = dJointCreateHinge(world, 0);
    dJointAttach(fRKneeJ, fBodies[B_R_SHANK], fBodies[B_R_THIGH]);
    dJointSetHingeAnchor(fRKneeJ, 0., legPosY_R, kneePosZ);
    dJointSetHingeAxis(fRKneeJ, 0., 1., 0.);
    // setJointLimits(0, 2.5); //?
    
    fLAnkleJ = dJointCreateUniversal(world, 0);
    dJointAttach(fLAnkleJ, fBodies[B_L_FOOT], fBodies[B_L_SHANK]);
    dJointSetUniversalAnchor(fLAnkleJ, 0., legPosY_L, anklePosZ);

    dJointSetUniversalAxis1(fLAnkleJ, 1., 0., 0.);
    dJointSetUniversalAxis2(fLAnkleJ, 0., 1., 0.);
    
    dJointSetUniversalParam(fLAnkleJ, dParamLoStop, -0.75);
    dJointSetUniversalParam(fLAnkleJ, dParamHiStop,  0.75);
    dJointSetUniversalParam(fLAnkleJ, dParamLoStop2, -0.75);
    dJointSetUniversalParam(fLAnkleJ, dParamHiStop2,  0.75);
    
    fRAnkleJ = dJointCreateUniversal(world, 0);
    dJointAttach(fRAnkleJ, fBodies[B_R_FOOT], fBodies[B_R_SHANK]);
    dJointSetUniversalAnchor(fRAnkleJ, 0., legPosY_R, anklePosZ);

    dJointSetUniversalAxis1(fRAnkleJ, -1., 0., 0.);
    dJointSetUniversalAxis2(fRAnkleJ, 0., 1., 0.);
    
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

