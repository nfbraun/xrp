#include "SwingController.h"
#include "TorqueController.h"
#include "LegIK.h"
#include "../../DynTransform.h"

RawTorques SwingController::gravityCompensation(const RobotInfo& rinfo)
{
    RawTorques torques;
    
    const double m_thigh = rbMass(rinfo.swingThighIndex());
    const double m_shank = rbMass(rinfo.swingShankIndex());
    const double m_foot = rbMass(rinfo.swingFootIndex());
    
    const Eigen::Vector3d g(0., 0., 9.8);
    
    Eigen::Vector3d tmpV;
    
    Eigen::Vector3d hipT, kneeT, ankleT;
    
    tmpV = rinfo.fstate().pos(rinfo.swingThighIndex()) - rinfo.jPos(rinfo.swingHipIndex());
    hipT = -tmpV.cross(m_thigh*g);
    
    tmpV = rinfo.fstate().pos(rinfo.swingShankIndex()) - rinfo.jPos(rinfo.swingHipIndex());
    hipT += -tmpV.cross(m_shank*g);
    
    tmpV = rinfo.fstate().pos(rinfo.swingFootIndex()) - rinfo.jPos(rinfo.swingHipIndex());
    hipT += -tmpV.cross(m_foot*g);
    
    tmpV = rinfo.fstate().pos(rinfo.swingShankIndex()) - rinfo.jPos(rinfo.swingKneeIndex());
    kneeT = -tmpV.cross(m_shank*g);
    
    tmpV = rinfo.fstate().pos(rinfo.swingFootIndex()) - rinfo.jPos(rinfo.swingKneeIndex());
    kneeT += -tmpV.cross(m_foot*g);
    
    tmpV = rinfo.fstate().pos(rinfo.swingFootIndex()) - rinfo.jPos(rinfo.swingAnkleIndex());
    ankleT = -tmpV.cross(m_foot*g);
    
    torques.at(rinfo.swingHipIndex()) = hipT;
    torques.at(rinfo.swingKneeIndex()) = kneeT;
    torques.at(rinfo.swingAnkleIndex()) = ankleT;
    
    return torques;
}

/**
	This method is used to compute the target angles for the swing hip and swing knee that help 
	to ensure (approximately) precise foot-placement control.
*/
IKSwingLegTarget SwingController::computeIKSwingLegTargets(const RobotInfo& rinfo, const Eigen::Vector3d& swingFootPos, const Eigen::Vector3d& swingFootVel, double swingFootHeight, double swingFootHeightVel)
{
    const double dt = 0.001;
    
    const double hNow = swingFootHeight;
    const double hFuture = swingFootHeight + swingFootHeightVel*dt;
    
    Eigen::Vector3d pNow, pFuture;
    pNow = transformSwingFootTarget(swingFootPos, rinfo.comPos(), rinfo.characterFrame(), hNow);
    pFuture = transformSwingFootTarget(swingFootPos + swingFootVel*dt,
        rinfo.comPos(), rinfo.characterFrame(), hFuture);
    
    // pNow, pFuture are ankle positions in world coordinates
    dbg->desSwingPos = pNow;
    
    Eigen::Vector3d dNow = rinfo.fstate().rot(B_PELVIS).conjugate()._transformVector(pNow - rinfo.jPos(rinfo.swingHipIndex()));
    Eigen::Vector3d dFuture = rinfo.fstate().rot(B_PELVIS).conjugate()._transformVector(pFuture - rinfo.jPos(rinfo.swingHipIndex()));
    
    return getSwingLegTarget(dNow, (dFuture - dNow)/dt);
}

/**
	This method returns a target for the location of the swing foot, based on some state information. It is assumed that the velocity vel
	is expressed in character-relative coordinates (i.e. the sagittal component is the z-component), while the com position, and the
	initial swing foot position is expressed in world coordinates. The resulting desired foot position is also expressed in world coordinates.
*/
Eigen::Vector3d SwingController::transformSwingFootTarget(const Eigen::Vector3d& step, const Eigen::Vector3d& comPos, const Eigen::Quaterniond& charFrameToWorld, double height)
{
	//now transform this vector into world coordinates
	Eigen::Vector3d result = charFrameToWorld._transformVector(step);
	//add it to the com location
	result += comPos;
	//finally, set the desired height of the foot
	result.z() = height;

	// step += computeSwingFootDelta(t);
	
	return result;
}

void SwingController::swingAnkleControl(JSpTorques& jt, const RobotInfo& rinfo)
{
    Eigen::Quaterniond qErr = rinfo.fstate().rot(rinfo.swingFootIndex()).conjugate() * rinfo.characterFrame();
    
    Eigen::Vector3d ankleT;
    
    //qErr.v also contains information regarding the axis of rotation and the angle (sin(theta)), but I want to scale it by theta instead
    double sinTheta = qErr.vec().norm();
    if (IS_ZERO(sinTheta)) {
        //avoid the divide by close-to-zero. The orientations match, so the proportional component of the torque should be 0
        
        ankleT = Eigen::Vector3d(0., 0., 0.);
    } else {
        double absAngle = 2 * asin(sinTheta);
        ankleT = qErr.vec();
        ankleT *= 1/sinTheta * absAngle * (-50.0) * SGN(qErr.w());
    }

    ankleT += -rinfo.fstate().avel(rinfo.swingFootIndex()) * (-15.0);
    
    Eigen::Vector3d cf_AnkleAxis1(1., 0., 0.);
    Eigen::Vector3d cf_AnkleAxis2(0., 1., 0.);
    Eigen::Vector3d AnkleAxis1 = rinfo.fstate().rot(rinfo.swingFootIndex())._transformVector(cf_AnkleAxis1);
    Eigen::Vector3d AnkleAxis2 = rinfo.fstate().rot(rinfo.swingShankIndex())._transformVector(cf_AnkleAxis2);
    
    unsigned int side = (rinfo.stance() == LEFT_STANCE ? RIGHT : LEFT);
    jt.t(side, AX) = -AnkleAxis1.dot(ankleT);
    jt.t(side, AY) = -AnkleAxis2.dot(ankleT);
}

void SwingController::swingLegControl(JSpTorques& jt, const RobotInfo& rinfo, const IKSwingLegTarget& desiredPose)
{
    unsigned int side = (rinfo.stance() == LEFT_STANCE ? RIGHT : LEFT);
    
    RawTorques t_gcomp = gravityCompensation(rinfo);
    
    TorqueController::transformLegTorques(jt, side, rinfo, t_gcomp);
    
    jt.t(side, HZ) += 300.0*(desiredPose.phz - rinfo.jstate().phi(side, HZ)) + 35.0*(desiredPose.ohz - rinfo.jstate().omega(side, HZ));
    jt.t(side, HY) += 300.0*(desiredPose.phy - rinfo.jstate().phi(side, HY)) + 35.0 * (desiredPose.ohy - rinfo.jstate().omega(side, HY));
    jt.t(side, HX) += 300.0*(desiredPose.phx - rinfo.jstate().phi(side, HX)) + 35.0 * (desiredPose.ohx - rinfo.jstate().omega(side, HX));
    
    jt.t(side, KY) += 300.0*(desiredPose.pky - rinfo.jstate().phi(side, KY)) + 35.0 * (desiredPose.oky - rinfo.jstate().omega(side, KY));
    
    swingAnkleControl(jt, rinfo);
}

