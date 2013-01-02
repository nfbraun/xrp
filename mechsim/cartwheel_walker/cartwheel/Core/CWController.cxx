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
    fCharacter = character;
    
    fLowController = createLowController(character);
    fLowController->setStance(LEFT_STANCE);
    
    fHighController = new TurnController(character, fLowController, worldOracle);
    fHighController->initializeDefaultParameters();
    
    fHighController->requestHeading(0.);
    fHighController->conTransitionPlan();
    fHighController->requestCoronalStepWidth(.3);
    fHighController->requestVelocities(.5, 0.);
}

RawTorques CWController::performPreTasks(double dt, const std::vector<ContactPoint>& cfs)
{
    fHighController->simStepPlan(SimGlobals::dt);
    return fLowController->computeTorques(cfs);
}

void CWController::performPostTasks(double dt, const std::vector<ContactPoint>& cfs)
{
    bool newState = (fLowController->advanceInTime(dt, cfs) != -1);
    fLowController->updateDAndV();
    if( newState ) {
        fHighController->conTransitionPlan();
    }
}

JointSpTorques CWController::transformTorques(const RawTorques& torques)
{
    JointSpTorques jt;
    
    /*** Left leg ***/
    Vector3d cf_lKneeAxis(1., 0., 0.);
    Eigen::Vector3d lKneeAxis = fCharacter->getARBs()[R_L_UPPER_LEG]->getOrientation().rotate(cf_lKneeAxis).toEigen();
    
    Vector3d cf_lAnkleAxis1(0., 0., 1.);
    Vector3d cf_lAnkleAxis2(1., 0., 0.);
    Eigen::Vector3d lAnkleAxis1 = fCharacter->getARBs()[R_L_FOOT]->getOrientation().rotate(cf_lAnkleAxis1).toEigen();
    Eigen::Vector3d lAnkleAxis2 = fCharacter->getARBs()[R_L_LOWER_LEG]->getOrientation().rotate(cf_lAnkleAxis2).toEigen();
    
    Vector3d lHipTorque = fCharacter->getARBs()[R_ROOT]->getOrientation().inverseRotate(torques.get(J_L_HIP));
    jt.fLeftLeg[0] = lHipTorque.x;
    jt.fLeftLeg[1] = lHipTorque.y;
    jt.fLeftLeg[2] = lHipTorque.z;
    
    jt.fLeftLeg[3] = lKneeAxis.dot(torques.get(J_L_KNEE).toEigen());
    jt.fLeftLeg[4] = lAnkleAxis1.dot(torques.get(J_L_ANKLE).toEigen());
    jt.fLeftLeg[5] = lAnkleAxis2.dot(torques.get(J_L_ANKLE).toEigen());
    
    /*** Right leg ***/
    Vector3d cf_rKneeAxis(1., 0., 0.);
    Eigen::Vector3d rKneeAxis = fCharacter->getARBs()[R_R_UPPER_LEG]->getOrientation().rotate(cf_rKneeAxis).toEigen();
    
    Vector3d cf_rAnkleAxis1(0., 0., -1.);
    Vector3d cf_rAnkleAxis2(1., 0., 0.);
    Eigen::Vector3d rAnkleAxis1 = fCharacter->getARBs()[R_R_FOOT]->getOrientation().rotate(cf_rAnkleAxis1).toEigen();
    Eigen::Vector3d rAnkleAxis2 = fCharacter->getARBs()[R_R_LOWER_LEG]->getOrientation().rotate(cf_rAnkleAxis2).toEigen();
    
    Vector3d rHipTorque = fCharacter->getARBs()[R_ROOT]->getOrientation().inverseRotate(torques.get(J_R_HIP));
    
    jt.fRightLeg[0] = rHipTorque.x;
    jt.fRightLeg[1] = rHipTorque.y;
    jt.fRightLeg[2] = rHipTorque.z;
    
    jt.fRightLeg[3] = rKneeAxis.dot(torques.get(J_R_KNEE).toEigen());
    jt.fRightLeg[4] = rAnkleAxis1.dot(torques.get(J_R_ANKLE).toEigen());
    jt.fRightLeg[5] = rAnkleAxis2.dot(torques.get(J_R_ANKLE).toEigen());
    
    return jt;
}

JointSpTorques CWController::Run(double dt, const std::vector<ContactPoint>& cfs)
{
    performPostTasks(dt, cfs);
    return transformTorques(performPreTasks(dt, cfs));
}
