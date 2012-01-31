#include "CWRobot.h"
#include <MathLib/Vector3d.h>
#include <Physics/ArticulatedRigidBody.h>
#include <Physics/Joint.h>
#include <ode/ode.h>

dBodyID CWRobot::makeODEARB(dWorldID world, ArticulatedRigidBody* arb)
{
    dBodyID id = dBodyCreate(world);
    dMass ODEmass;
    ODEmass.setParameters(arb->getMass(), 0, 0, 0, 
            arb->moi.x, arb->moi.y, arb->moi.z,
            0, 0, 0);

    dBodySetMass(id, &ODEmass);
    
    dQuaternion tempQ;
    tempQ[0] = arb->getState().orientation.s;
    tempQ[1] = arb->getState().orientation.v.x;
    tempQ[2] = arb->getState().orientation.v.y;
    tempQ[3] = arb->getState().orientation.v.z;
    
    dBodySetPosition(id, arb->getState().position.x, arb->getState().position.y, arb->getState().position.z);
    dBodySetQuaternion(id, tempQ);
    //dBodySetLinearVel(id, arb->getState().velocity.x, arb->getState().velocity.y, arb->getState().velocity.z);
    //dBodySetAngularVel(id, arb->getState().angularVelocity.x, arb->getState().angularVelocity.y, arb->getState().angularVelocity.z);
    
    //arb->ODEBodyID = id;
    fODEMap[arb] = id;
    
    return id;
}

dGeomID makeODEBoxGeom(double sx, double sy, double sz)
{
    dGeomID g = dCreateBox(0, sx, sy, sz);
    return g;
}

ArticulatedRigidBody* createBox(const char* name,
                                const Point3d& pos,
                                const Vector3d& size,
                                double moiScale = 1,
                                double density=1, bool havecdp=false)
{
    const double mass = density * size.x*size.y*size.z;
    
    //std::cout << "Mass: " << mass << "kg" << std::endl;

    ArticulatedRigidBody* box = new ArticulatedRigidBody();
    box->state.position = pos;
    box->setMass(mass);
    
    box->moi = Vector3d( size.y*size.y + size.z*size.z,
                  size.x*size.x + size.z*size.z,
                  size.x*size.x + size.y*size.y ) * 1.0/12.0 * mass * moiScale;
    
    return box;
}

ArticulatedRigidBody* createCylinder(const char* name,
                                     const Point3d& pos,
                                     double basePos, double tipPos,
                                     double radius=1,
                                     double moiScale = 1,
                                     double density = 1)
{
    const double mass = density * M_PI *radius*radius* fabs(tipPos-basePos);
    
    ArticulatedRigidBody* cylinder = new ArticulatedRigidBody();
    cylinder->state.position = pos;
    cylinder->setMass(mass);

    const double height = fabs(tipPos-basePos);
    
    cylinder->moi = Vector3d(mass*(3*radius*radius + height*height) / 12.0 * moiScale,
                 mass*radius*radius / 2.0 * moiScale,
                 mass*(3*radius*radius + height*height) / 12.0 * moiScale);
    
    return cylinder;
}

void CWRobot::create(dWorldID world)
{
    using namespace CharacterConst;
    const double unit = 1.0;
    
    fCharacter = new Character();
    
    ArticulatedRigidBody* pelvis = createBox("pelvis", Point3d(0, pelvisPosY, 0),
        Vector3d(pelvisSizeX, pelvisSizeY, pelvisSizeZ),
        3., density);
    fCharacter->setRoot( pelvis );
    
    ArticulatedRigidBody* lUpperLeg = createCylinder("lUpperLeg",
        Point3d(legPosX_L, upperLegPosY, 0.),
        -upperLegSizeY/2.0, upperLegSizeY/2.0,
        legDiameter/2.0,
        1, density);
    fCharacter->addArticulatedRigidBody( lUpperLeg, R_L_UPPER_LEG );
    
    Joint* joint2 = new Joint();
    //joint2->setName("lHip");
    joint2->setParentJointPosition(Point3d(pelvisRadius*legRelativeAnchorX, -pelvisSizeY/2.0, 0));
    joint2->setChildJointPosition(Point3d(0, upperLegSizeY/2.0, 0));
    joint2->setParent(pelvis);
    joint2->setChild(lUpperLeg);
    fCharacter->addJoint(joint2, J_L_HIP);
    
    ArticulatedRigidBody* rUpperLeg = createCylinder("rUpperLeg",
        Point3d(legPosX_R, upperLegPosY, 0.),
        -upperLegSizeY/2.0, upperLegSizeY/2.0,
        legDiameter/2.0,
        1., density);
    fCharacter->addArticulatedRigidBody( rUpperLeg, R_R_UPPER_LEG );
    
    Joint* joint3 = new Joint();
    //joint3->setName("rHip");
    joint3->setParentJointPosition(Point3d(-pelvisRadius*legRelativeAnchorX, -pelvisSizeY/2.0, 0));
    joint3->setChildJointPosition(Point3d(0, upperLegSizeY/2.0, 0));
    joint3->setParent(pelvis);
    joint3->setChild(rUpperLeg);
    fCharacter->addJoint(joint3, J_R_HIP);
    
    ArticulatedRigidBody* lLowerLeg = createCylinder("lLowerLeg",
        Point3d(legPosX_L, lowerLegPosY, 0.),
        -lowerLegSizeY/2.0, lowerLegSizeY/2.0, 
        legDiameter/2.0,
        1, density);
    fCharacter->addArticulatedRigidBody( lLowerLeg, R_L_LOWER_LEG);
    
    Joint* joint4 = new Joint();
    //joint4->setName("lKnee");
    joint4->setParentJointPosition(Point3d(0, -upperLegSizeY/2.0, 0));
    joint4->setChildJointPosition(Point3d(0, lowerLegSizeY/2.0, 0));
    joint4->setParent(lUpperLeg);
    joint4->setChild(lLowerLeg);
    fCharacter->addJoint(joint4, J_L_KNEE);
    
    ArticulatedRigidBody* rLowerLeg = createCylinder("rLowerLeg",
        Point3d(legPosX_R, lowerLegPosY, 0.),
        -lowerLegSizeY/2.0, lowerLegSizeY/2.0, 
        legDiameter/2.0,
        1, density);
    fCharacter->addArticulatedRigidBody( rLowerLeg, R_R_LOWER_LEG );
    
    Joint* joint5 = new Joint();
    //joint5->setName("rKnee");
    joint5->setParentJointPosition(Point3d(0, -upperLegSizeY/2.0, 0));
    joint5->setChildJointPosition(Point3d(0, lowerLegSizeY/2.0, 0));
    joint5->setParent(rUpperLeg);
    joint5->setChild(rLowerLeg);
    fCharacter->addJoint(joint5, J_R_KNEE);
    
    ArticulatedRigidBody* lFoot = createBox("lFoot",
        Point3d(legPosX_L, footPosY, 0.016),
        Vector3d(footSizeX,footSizeY,footSizeZ),
        //# groundCoeffs = (0.0005,0.2),
        3., density, true);
    fCharacter->addArticulatedRigidBody( lFoot, R_L_FOOT );
    
    Joint* joint6 = new Joint();
    //joint6->setName("lAnkle");
    joint6->setParentJointPosition(Point3d(0, -lowerLegSizeY/2.0, 0));
    joint6->setChildJointPosition(Point3d(0, footSizeY/2.0, -footSizeZ*0.33 + legDiameter/2.0));
    joint6->setParent(lLowerLeg);
    joint6->setChild(lFoot);
    fCharacter->addJoint(joint6, J_L_ANKLE);
    
    ArticulatedRigidBody* rFoot = createBox("rFoot",
        Point3d(legPosX_R, footPosY, 0.016), 
        Vector3d(footSizeX,footSizeY,footSizeZ),
        //# groundCoeffs = (0.0005,0.2),
        3., density, true);
    fCharacter->addArticulatedRigidBody( rFoot, R_R_FOOT );
    
    Joint* joint7 = new Joint();
    //joint7->setName("rAnkle");
    joint7->setParentJointPosition(Point3d(0, -lowerLegSizeY/2.0, 0));
    joint7->setChildJointPosition(Point3d(0, footSizeY/2.0, -footSizeZ*0.33 + legDiameter/2.0));
    joint7->setParent(rLowerLeg);
    joint7->setChild(rFoot);
    fCharacter->addJoint(joint7, J_R_ANKLE);
    
    fPelvisB = makeODEARB(world, pelvis);
    
    fLUpperLegB = makeODEARB(world, lUpperLeg);
    fLLowerLegB = makeODEARB(world, lLowerLeg);
    fRUpperLegB = makeODEARB(world, rUpperLeg);
    fRLowerLegB = makeODEARB(world, rLowerLeg);
    
    fLFootB = makeODEARB(world, lFoot);
    fRFootB = makeODEARB(world, rFoot);
    
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

/* void CWRobot::applyTorques()
{
    
} */
