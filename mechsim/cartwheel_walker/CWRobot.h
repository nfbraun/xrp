#ifndef CW_CHARACTER_H
#define CW_CHARACTER_H
#include "StaticRobotInfo.h"
#include <Core/Character.h>
#include <ode/ode.h>

class CWRobot
{
public:
    dBodyID fBodies[B_MAX];
    dGeomID fLFootG, fRFootG;
    dJointID fLAnkleJ, fLKneeJ, fLHipJ;
    dJointID fRAnkleJ, fRKneeJ, fRHipJ;
    
    Character* fCharacter;
    
    CWRobot() {}
    void create(dWorldID world);
    
    dBodyID makeODEARB(dWorldID world, BodyID id, ArticulatedRigidBody* arb);
};

#endif
