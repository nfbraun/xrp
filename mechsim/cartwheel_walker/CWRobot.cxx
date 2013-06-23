#include "CWRobot.h"

//#define DEBUG_FIXED_TORSO
#define SET_LIMITS

dBodyID CWRobot::makeODEARB(dWorldID world, BodyID id, const Eigen::Vector3d& pos)
{
    dBodyID body = dBodyCreate(world);
    dMass ODEmass;
    ODEmass.setParameters(rbMass(id), 0, 0, 0, 
            rbMOI(id).x(), rbMOI(id).y(), rbMOI(id).z(),
            0, 0, 0);
    
    dBodySetMass(body, &ODEmass);
    dBodySetPosition(body, pos.x(), pos.y(), pos.z());
    
    return body;
}

void CWRobot::create(dWorldID world)
{
    using namespace CharacterConst;
    
    fBodies[B_PELVIS] = makeODEARB(world, B_PELVIS,
                                   Eigen::Vector3d(0., 0., pelvisPosZ));
    
    fBodies[B_L_THIGH] = makeODEARB(world, B_L_THIGH,
                                    Eigen::Vector3d(0., legPosY_L, thighPosZ));
    fBodies[B_L_SHANK] = makeODEARB(world, B_L_SHANK,
                                    Eigen::Vector3d(0., legPosY_L, shankPosZ));
    fBodies[B_R_THIGH] = makeODEARB(world, B_R_THIGH,
                                    Eigen::Vector3d(0., legPosY_R, thighPosZ));
    fBodies[B_R_SHANK] = makeODEARB(world, B_R_SHANK,
                                    Eigen::Vector3d(0., legPosY_R, shankPosZ));
    
    fBodies[B_L_FOOT] = makeODEARB(world, B_L_FOOT,
                                   Eigen::Vector3d(footPosX, legPosY_L, footPosZ));
    fBodies[B_R_FOOT] = makeODEARB(world, B_R_FOOT,
                                   Eigen::Vector3d(footPosX, legPosY_R, footPosZ));
    
    fLFootG = dCreateBox(0, footSizeX, footSizeY, footSizeZ);
    fRFootG = dCreateBox(0, footSizeX, footSizeY, footSizeZ);
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
    
    #ifdef SET_LIMITS
    dJointSetUniversalParam(fLAnkleJ, dParamLoStop, -0.75);
    dJointSetUniversalParam(fLAnkleJ, dParamHiStop,  0.75);
    dJointSetUniversalParam(fLAnkleJ, dParamLoStop2, -0.75);
    dJointSetUniversalParam(fLAnkleJ, dParamHiStop2,  0.75);
    #endif
    
    fRAnkleJ = dJointCreateUniversal(world, 0);
    dJointAttach(fRAnkleJ, fBodies[B_R_FOOT], fBodies[B_R_SHANK]);
    dJointSetUniversalAnchor(fRAnkleJ, 0., legPosY_R, anklePosZ);

    dJointSetUniversalAxis1(fRAnkleJ, 1., 0., 0.);
    dJointSetUniversalAxis2(fRAnkleJ, 0., 1., 0.);
    
    #ifdef SET_LIMITS
    dJointSetUniversalParam(fRAnkleJ, dParamLoStop, -0.75);
    dJointSetUniversalParam(fRAnkleJ, dParamHiStop,  0.75);
    dJointSetUniversalParam(fRAnkleJ, dParamLoStop2, -0.75);
    dJointSetUniversalParam(fRAnkleJ, dParamHiStop2,  0.75);
    #endif
    
    #ifdef DEBUG_FIXED_TORSO
    dJointID fix = dJointCreateFixed(world, 0);
    dJointAttach(fix, 0, fBodies[B_PELVIS]);
    dJointSetFixed(fix);
    #endif
}

