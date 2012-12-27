#include "CWController.h"

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

IKVMCController* createLowController(Character* character)
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

CWController::CWController(Character* character, WorldOracle* worldOracle)
{
    fLowController = createLowController(character);
    fLowController->setStance(LEFT_STANCE);
    
    fHighController = new TurnController(character, fLowController, worldOracle);
    fHighController->initializeDefaultParameters();
    
    fHighController->requestHeading(0.);
    fHighController->conTransitionPlan();
    fHighController->requestCoronalStepWidth(.3);
    fHighController->requestVelocities(.5, 0.);
}

JointTorques CWController::performPreTasks(double dt, std::vector<ContactPoint> *cfs)
{
    fHighController->simStepPlan(SimGlobals::dt);
    return fLowController->computeTorques(cfs);
}

void CWController::performPostTasks(double dt, std::vector<ContactPoint> *cfs)
{
    bool newState = (fLowController->advanceInTime(dt, cfs) != -1);
    fLowController->updateDAndV();
    if( newState ) {
        fHighController->conTransitionPlan();
    }
}

JointTorques CWController::Run(double dt, std::vector<ContactPoint> *cfs)
{
    performPostTasks(dt, cfs);
    return performPreTasks(dt, cfs);
}
