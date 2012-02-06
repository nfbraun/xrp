#include "Controller.h"
#include <MathLib/Vector3d.h>

void addControlParams(IKVMCController& controller, JointID jid,
    double kp, double kd, double tauMax, const Vector3d& scale)
{
    ControlParams params;
    params.setKp(kp);
    params.setKd(kd);
    params.setMaxAbsTorque(tauMax);
    params.setScale(scale);
    
    if(jid < J_MAX)
        controller.addControlParams(jid, params);
    else
        controller.setRootControlParams(params);
}

IKVMCController* createController(Character* character)
{
    IKVMCController* controller = new IKVMCController(character);
    
    addControlParams(*controller, J_MAX, 1000.0, 200.0, 200.0,
        Vector3d( 1.0, 1.0, 1.0 ) );
    addControlParams(*controller, J_L_HIP, 300.0, 35.0, 100.0, Vector3d( 1.0, 1.0, 1.0 ) );
    addControlParams(*controller, J_R_HIP, 300.0, 35.0, 100.0, Vector3d( 1.0, 1.0, 1.0 ) );
    addControlParams(*controller, J_L_KNEE, 300.0, 35.0, 200.0, Vector3d( 1.0, 1.0, 1.0 ) );
    addControlParams(*controller, J_R_KNEE, 300.0, 35.0, 200.0, Vector3d( 1.0, 1.0, 1.0 ) );
    addControlParams(*controller, J_L_ANKLE, 50.0, 15.0, 50.0, Vector3d( 1.0, 0.2, 0.2 ) );
    addControlParams(*controller, J_R_ANKLE, 50.0, 15.0, 50.0, Vector3d( 1.0, 0.2, 0.2 ) );
    
    return controller;
}

