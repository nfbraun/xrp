#ifndef CW_CHARACTER_H
#define CW_CHARACTER_H
#include "StaticRobotInfo.h"
#include <Eigen/Dense>
#include <ode/ode.h>

class CWRobot
{
public:
    dBodyID fBodies[B_MAX];
    dGeomID fLFootG, fRFootG;
    dJointID fLAnkleJ, fLKneeJ, fLHipJ;
    dJointID fRAnkleJ, fRKneeJ, fRHipJ;
    
    CWRobot() {}
    void create(dWorldID world);
    
    dBodyID makeODEARB(dWorldID world, BodyID id, const Eigen::Vector3d& pos);
};

#endif
