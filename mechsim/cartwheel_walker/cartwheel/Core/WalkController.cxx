#include "WalkController.h"

WalkController::WalkController(double velDSagittal, double velDCoronal)
{
    ll_velDSagittal = velDSagittal;
    ll_velDCoronal = velDCoronal;
    ll_desiredHeading = 0.;
    
    //we should estimate these from the character info...
    const double ankleBaseHeight = 0.04;
    
    //now prepare the step information for the following step:
    ll_swingFootHeightTrajectory.clear();
    
    ll_swingFootHeightTrajectory.addKnot(0, ankleBaseHeight);
    ll_swingFootHeightTrajectory.addKnot(0.5, ankleBaseHeight + 0.11);
    ll_swingFootHeightTrajectory.addKnot(1, ankleBaseHeight + 0.01);
}

double WalkController::adjustStepHeight(const RobotInfo& rinfo)
{
    //if the foot is high enough, we shouldn't do much about it... also, if we're close to the start or end of the walk cycle, we don't need to do anything... the thing below is a quadratic that is 1 at 0.5, 0 at 0 and 1...
    
    double panicIntensity = -4 * rinfo.phi() * rinfo.phi() + 4 * rinfo.phi();
    panicIntensity *= InvPendulum::getPanicLevel(rinfo, ll_velDSagittal, ll_velDCoronal);
    return panicIntensity * 0.05;
}

HighLevelTarget WalkController::simStepPlan(const RobotInfo& rinfo, double dt)
{
    HighLevelTarget target;
    
    const double panicHeight = adjustStepHeight(rinfo);
    
    const double diff_dt = 0.001;
    const double hNow = ll_swingFootHeightTrajectory.evaluate_catmull_rom(rinfo.phi()) + panicHeight;
    const double hFuture = ll_swingFootHeightTrajectory.evaluate_catmull_rom(rinfo.phi()+diff_dt) + panicHeight;
    
    target.velDSagittal = ll_velDSagittal;
    target.velDCoronal = ll_velDCoronal;
    target.desiredHeading = ll_desiredHeading;
    target.swingFootHeight = hNow;
    target.swingFootHeightVel = (hFuture-hNow)/diff_dt;
    
    return target;
}
