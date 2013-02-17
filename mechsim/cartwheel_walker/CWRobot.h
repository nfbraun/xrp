#ifndef CW_CHARACTER_H
#define CW_CHARACTER_H
#include "StaticRobotInfo.h"
#include <Core/Character.h>
#include <ode/ode.h>
#include <map>

class CWRobot
{
public:
    dBodyID fPelvisB;
    dBodyID fLThighB, fLShankB, fRThighB, fRShankB;
    dBodyID fLFootB, fRFootB;
    dGeomID fLFootG, fRFootG;
    RigidBody* fLFootRB, *fRFootRB;
    dJointID fLAnkleJ, fLKneeJ, fLHipJ;
    dJointID fRAnkleJ, fRKneeJ, fRHipJ;
    
    Character* fCharacter;
    
    std::map<RigidBody*, dBodyID> fODEMap;
    dBodyID getODEBody(RigidBody* rb) { return fODEMap[rb]; }
    
    CWRobot() {}
    void create(dWorldID world);
    
    dBodyID makeODEARB(dWorldID world, ArbID id, ArticulatedRigidBody* arb);
};

#endif
