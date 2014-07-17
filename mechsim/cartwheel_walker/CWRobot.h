#ifndef CW_CWROBOT_H
#define CW_CWROBOT_H
#include "StaticRobotInfo.h"
#include <RobotState.h>
#include <Torques.h>
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
    
    FullState fullState() const;
    void setFullState(const FullState& fstate);
    
    void addTorques(const JSpTorques& jt);
    
    dBodyID makeODEARB(dWorldID world, BodyID id, const Eigen::Vector3d& pos);
};

#endif
