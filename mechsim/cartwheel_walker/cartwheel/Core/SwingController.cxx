#include "SwingController.h"
#include "TorqueController.h"
#include "LegIK.h"
#include "../../DynTransform.h"

RawTorques SwingController::gravityCompensation(const RobotInfo& rinfo)
{
    RawTorques torques;
    
    unsigned int side = (rinfo.stance() == LEFT_STANCE ? RIGHT : LEFT);
    
    const double m_thigh = rbMass(side == LEFT ? B_L_THIGH : B_R_THIGH);
    const double m_shank = rbMass(side == LEFT ? B_L_SHANK : B_R_SHANK);
    const double m_foot = rbMass(side == LEFT ? B_L_FOOT : B_R_FOOT);
    
    const Vector3d g(0., 0., 9.8);
    
    Vector3d tmpV;
    
    Vector3d hipT, kneeT, ankleT;
    
    tmpV = rinfo.rbPos(rinfo.swingThighIndex()) - rinfo.jPos(rinfo.swingHipIndex());
    hipT = -tmpV.cross(m_thigh*g);
    
    tmpV = rinfo.rbPos(rinfo.swingShankIndex()) - rinfo.jPos(rinfo.swingHipIndex());
    hipT += -tmpV.cross(m_shank*g);
    
    tmpV = rinfo.rbPos(rinfo.swingFootIndex()) - rinfo.jPos(rinfo.swingHipIndex());
    hipT += -tmpV.cross(m_foot*g);
    
    tmpV = rinfo.rbPos(rinfo.swingShankIndex()) - rinfo.jPos(rinfo.swingKneeIndex());
    kneeT = -tmpV.cross(m_shank*g);
    
    tmpV = rinfo.rbPos(rinfo.swingFootIndex()) - rinfo.jPos(rinfo.swingKneeIndex());
    kneeT += -tmpV.cross(m_foot*g);
    
    tmpV = rinfo.rbPos(rinfo.swingFootIndex()) - rinfo.jPos(rinfo.swingAnkleIndex());
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
IKSwingLegTarget SwingController::computeIKSwingLegTargets(const RobotInfo& rinfo, const Vector3d& swingFootPos, const Vector3d& swingFootVel, double swingFootHeight, double swingFootHeightVel)
{
    const double dt = 0.001;
    
    const double hNow = swingFootHeight;
    const double hFuture = swingFootHeight + swingFootHeightVel*dt;
    
    Point3d pNow, pFuture;
    pNow = transformSwingFootTarget(swingFootPos, rinfo.comPos(), rinfo.characterFrame(), hNow);
    pFuture = transformSwingFootTarget(swingFootPos + swingFootVel*dt,
        rinfo.comPos(), rinfo.characterFrame(), hFuture);
    
    // pNow, pFuture are ankle positions in world coordinates
    dbg->desSwingPos = pNow;
    
    Eigen::Vector3d dNow = rinfo.fstate().rot(B_PELVIS).conjugate()._transformVector((pNow - rinfo.jPos(rinfo.swingHipIndex())).toEigen());
    Eigen::Vector3d dFuture = rinfo.fstate().rot(B_PELVIS).conjugate()._transformVector((pFuture - rinfo.jPos(rinfo.swingHipIndex())).toEigen());
    
    return getSwingLegTarget(dNow, (dFuture - dNow)/dt);
}

/**
	This method returns a target for the location of the swing foot, based on some state information. It is assumed that the velocity vel
	is expressed in character-relative coordinates (i.e. the sagittal component is the z-component), while the com position, and the
	initial swing foot position is expressed in world coordinates. The resulting desired foot position is also expressed in world coordinates.
*/
Vector3d SwingController::transformSwingFootTarget(Vector3d step, const Point3d& com, const Quaternion& charFrameToWorld, double height)
{
	//now transform this vector into world coordinates
	step = charFrameToWorld.rotate(step);
	//add it to the com location
	step = com + step;
	//finally, set the desired height of the foot
	step.z() = height;

	// step += computeSwingFootDelta(t);
	
	return step;
}

void SwingController::swingAnkleControl(JSpTorques& jt, const RobotInfo& rinfo)
{
    unsigned int side = (rinfo.stance() == LEFT_STANCE ? RIGHT : LEFT);
    
    Quaternion qErr = Quaternion(rinfo.fstate().rot(side, B_FOOT)).getComplexConjugate() * rinfo.characterFrame();
    
    Vector3d ankleT;
    
    //qErr.v also contains information regarding the axis of rotation and the angle (sin(theta)), but I want to scale it by theta instead
    double sinTheta = qErr.v.norm();
    if (IS_ZERO(sinTheta)) {
        //avoid the divide by close-to-zero. The orientations match, so the proportional component of the torque should be 0
        
        ankleT = Vector3d(0., 0., 0.);
    } else {
        double absAngle = 2 * asin(sinTheta);
        ankleT = qErr.v;
        ankleT *= 1/sinTheta * absAngle * (-50.0) * SGN(qErr.s);
    }

    ankleT += -Vector3d(rinfo.fstate().avel(side, B_FOOT)) * (-15.0);
    
    Vector3d cf_AnkleAxis1(1., 0., 0.);
    Vector3d cf_AnkleAxis2(0., 1., 0.);
    Vector3d AnkleAxis1 = rinfo.character()->getARBs()[rinfo.swingFootIndex()]->getOrientation().rotate(cf_AnkleAxis1);
    Vector3d AnkleAxis2 = rinfo.character()->getARBs()[rinfo.swingShankIndex()]->getOrientation().rotate(cf_AnkleAxis2);
    
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

