#include "Controller.h"
#include <Core/SimBiConState.h>
#include <MathLib/Trajectory.h>
#include <MathLib/Vector3d.h>

void addControlParams(IKVMCController& controller, JointID jid,
    double kp, double kd, double tauMax, const Vector3d& scale)
{
    ControlParams params;
    //KTJoint* joint = (jid < J_MAX ? controller.getCharacter()->getJoint(jid) : 0);
    //params.setJoint(joint);
    params.setKp(kp);
    params.setKd(kd);
    params.setMaxAbsTorque(tauMax);
    params.setScale(scale);
    
    if(jid < J_MAX)
        controller.addControlParams(jid, params);
    else
        controller.setRootControlParams(params);
}

Trajectory1d makeTrajectory1d(double x0, double y0)
{
    Trajectory1d traj;
    traj.addKnot(x0, y0);
    return traj;
}

Trajectory1d makeTrajectory1d(double x0, double y0, double x1, double y1)
{
    Trajectory1d traj;
    traj.addKnot(x0, y0);
    traj.addKnot(x1, y1);
    return traj;
}

IKVMCController* createController(Character* character)
{
    IKVMCController* controller = new IKVMCController(character);
    
    addControlParams(*controller, J_MAX, 1000.0, 200.0, 200.0,
        Vector3d( 1.0, 1.0, 1.0 ) );
    //addControlParams(*controller, J_PELVIS_LOWERBACK, 75.0, 17.0, 100.0,
    //    Vector3d( 1.0, 1.0, 1.0 ) );
    //addControlParams(*controller, J_LOWERBACK_TORSO, 75.0, 17.0, 100.0,
    //    Vector3d( 1.0, 1.0, 1.0 ) );
    addControlParams(*controller, J_L_HIP, 300.0, 35.0, 100.0, Vector3d( 1.0, 1.0, 1.0 ) );
    addControlParams(*controller, J_R_HIP, 300.0, 35.0, 100.0, Vector3d( 1.0, 1.0, 1.0 ) );
    addControlParams(*controller, J_L_KNEE, 300.0, 35.0, 200.0, Vector3d( 1.0, 1.0, 1.0 ) );
    addControlParams(*controller, J_R_KNEE, 300.0, 35.0, 200.0, Vector3d( 1.0, 1.0, 1.0 ) );
    addControlParams(*controller, J_L_ANKLE, 50.0, 15.0, 50.0, Vector3d( 1.0, 0.2, 0.2 ) );
    addControlParams(*controller, J_R_ANKLE, 50.0, 15.0, 50.0, Vector3d( 1.0, 0.2, 0.2 ) );
    
    //SimBiConState* state0 = controller->getState();
    //SimBiConState* state0 = new SimBiConState();
    //state0->setDuration(0.6);
    //state0->setTransitionOnFootContact(true);
    
    return controller;
}

