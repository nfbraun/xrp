#include "TorqueController.h"
#include "../../DynTransform.h"
#include "../../Reaction.h"
#include "SwingController.h"
#include "CWConfig.h"
#include <MathLib/MathLib.h>

void TorqueController::COMJT(const RobotInfo& rinfo, const Eigen::Vector3d& fA, Eigen::Vector3d& stanceAnkleTorque, Eigen::Vector3d& stanceKneeTorque, Eigen::Vector3d& stanceHipTorque)
{
	//applying a force at the COM induces the force f. The equivalent torques are given by the J' * f, where J' is
	// dp/dq, where p is the COM.

    Eigen::Vector3d anklePos = rinfo.jPos(rinfo.stanceAnkleIndex());
    Eigen::Vector3d kneePos = rinfo.jPos(rinfo.stanceKneeIndex());
    Eigen::Vector3d hipPos = rinfo.jPos(rinfo.stanceHipIndex());

	//total mass...
	// double m = lowerLeg->getMass() + upperLeg->getMass() + pelvis->getMass();
	const double pelvisMass = rbMass(rinfo.rootIndex());
	const double thighMass = rbMass(rinfo.stanceThighIndex());
	const double shankMass = rbMass(rinfo.stanceShankIndex());
	const double m = pelvisMass + thighMass + shankMass;

	Eigen::Vector3d f1 = (rinfo.fstate().pos(rinfo.stanceShankIndex()) - anklePos) * shankMass +
					(rinfo.fstate().pos(rinfo.stanceThighIndex()) - anklePos) * thighMass + 
					(rinfo.fstate().pos(rinfo.rootIndex()) - anklePos) * pelvisMass; 
	f1 /= m;
	
	Eigen::Vector3d f2 = (rinfo.fstate().pos(rinfo.stanceThighIndex()) - kneePos) * thighMass + 
					(rinfo.fstate().pos(rinfo.rootIndex()) - kneePos) * pelvisMass;
	f2 /= m;

	Eigen::Vector3d f3 = (rinfo.fstate().pos(rinfo.rootIndex()) - hipPos) * pelvisMass;
	f3 /= m;

	stanceAnkleTorque = f1.cross(fA);
	stanceKneeTorque = f2.cross(fA);
	stanceHipTorque = f3.cross(fA);
}

/**
	This method is used to compute the force that the COM of the character should be applying.
*/
Eigen::Vector3d TorqueController::computeVirtualForce(const RobotInfo& rinfo, double desOffCoronal, double desVSagittal, double desVCoronal)
{
	//this is the desired acceleration of the center of mass
	Eigen::Vector3d desA;
	desA.x() = (desVSagittal - rinfo.getV().x()) * CWConfig::VRF_SAG_D_GAIN;
	desA.y() = (desOffCoronal - rinfo.getD().y()) * CWConfig::VRF_COR_P_GAIN + (desVCoronal - rinfo.getV().y()) * CWConfig::VRF_COR_D_GAIN;
	desA.z() = 0.;
	
	/* if (doubleStanceMode == true){
	    assert(false);
		Vector3d errV = characterFrame.inverseRotate(doubleStanceCOMError*-1);
		desA.x = (-errV.x + comOffsetCoronal) * 20 + (velDCoronal - getV().x) * 9;
		desA.z = (-errV.z + comOffsetSagittal) * 10 + (velDSagittal - getV().z) * 150;
	} */

	//and this is the force that would achieve that
	Eigen::Vector3d fA = (desA) * totalMass();

	//now change this quantity to world coordinates...
	fA = rinfo.characterFrame()._transformVector(fA);
	
	return fA;
}

/**
	This method is used to compute the torques that need to be applied to the stance and swing hips, given the
	desired orientation for the root and the swing hip.
*/
/* simplified (on 2013-01-02) */
Eigen::Vector3d TorqueController::computeRootTorque(const RobotInfo& rinfo, double desHeading)
{
	//compute the total torques that should be applied to the root and swing hip, keeping in mind that
	//the desired orientations are expressed in the character frame
	Eigen::Vector3d rootTorque;
	
	//this is the desired orientation in world coordinates
	//qRootDW needs to also take into account the desired heading
	Eigen::Quaterniond qRootDW(Eigen::AngleAxisd(desHeading, CWConfig::UP));

	Eigen::Quaterniond qErr = rinfo.rootOrient().conjugate() * qRootDW;

	//qErr.v also contains information regarding the axis of rotation and the angle (sin(theta)), but I want to scale it by theta instead
	double sinTheta = qErr.vec().norm();
	if (IS_ZERO(sinTheta)){
		//avoid the divide by close-to-zero. The orientations match, so the proportional component of the torque should be 0
		rootTorque = Eigen::Vector3d::Zero();
	}else{
		double absAngle = 2 * asin(sinTheta);
		rootTorque = qErr.vec();
		rootTorque *= 1/sinTheta * absAngle * (-CWConfig::VRT_P_GAIN) * SGN(qErr.w());
	}

	//qErr represents the rotation from the desired child frame to the actual child frame, which
	//means that the torque is now expressed in child coordinates. We need to express it in parent coordinates!
	rootTorque = rinfo.rootOrient()._transformVector(rootTorque);
	//the angular velocities are stored in parent coordinates, so it is ok to add this term now
	rootTorque += rinfo.rootAngVel() * CWConfig::VRT_D_GAIN;

	//now, based on the ratio of the forces the feet exert on the ground, and the ratio of the forces between the two feet, we will compute the forces that need to be applied
	//to the two hips, to make up the root makeup torque - which means the final torque the root sees is rootTorque!

	//and done...
	// torques.at(stanceHipIndex) -= torques.get(J_L_HIP) + torques.get(J_R_HIP) + rootTorque;
	return rootTorque;
}

JSpTorques TorqueController::transformLegTorques(unsigned int side, const RobotInfo& rinfo, const RawTorques& torques)
{
    JSpTorques jt = JSpTorques::Zero();
    
    unsigned int hipId = (side == LEFT ? J_L_HIP : J_R_HIP);
    unsigned int kneeId = (side == LEFT ? J_L_KNEE : J_R_KNEE);
    unsigned int ankleId = (side == LEFT ? J_L_ANKLE : J_R_ANKLE);
    
    Eigen::Vector3d cf_KneeAxis(0., 1., 0.);
    Eigen::Vector3d KneeAxis = rinfo.fstate().rot(side, B_THIGH)._transformVector(cf_KneeAxis);
    
    Eigen::Vector3d cf_AnkleAxis1(1., 0., 0.);
    Eigen::Vector3d cf_AnkleAxis2(0., 1., 0.);
    Eigen::Vector3d AnkleAxis1 = rinfo.fstate().rot(side, B_FOOT)._transformVector(cf_AnkleAxis1);
    Eigen::Vector3d AnkleAxis2 = rinfo.fstate().rot(side, B_SHANK)._transformVector(cf_AnkleAxis2);
    
    Eigen::Vector3d hipTorque = rinfo.fstate().rot(B_PELVIS).conjugate()._transformVector(torques.get(hipId));
    
    Eigen::Quaterniond hipRot = rinfo.fstate().rot(B_PELVIS).conjugate() *
        rinfo.fstate().rot(side, B_THIGH);
    double hz, hy, hx;
    decompZYXRot(hipRot, hz, hy, hx);
    
    invTransformHipTorque(hz, hy, hx, -hipTorque, jt.t(side, HZ), jt.t(side, HY), jt.t(side, HX));
    
    jt.t(side, KY) = -KneeAxis.dot(torques.get(kneeId));
    jt.t(side, AX) = -AnkleAxis1.dot(torques.get(ankleId));
    jt.t(side, AY) = -AnkleAxis2.dot(torques.get(ankleId));
    
    return jt;
}

/* Compute the torques for the stance leg. */
JSpTorques TorqueController::stanceLegControl(const RobotInfo& rinfo, double desiredHeading)
{
    RawTorques torques;
    
    Eigen::Vector3d virtualRootTorque = computeRootTorque(rinfo, desiredHeading);
    dbg->virtualRootTorque = virtualRootTorque;
    torques.at(rinfo.stanceHipIndex()) -= virtualRootTorque;
    // FIXME: maybe re-include: - torques.at(rinfo.swingHipIndex());
    
    JSpTorques jt = transformLegTorques(rinfo.stance() == LEFT_STANCE ? LEFT : RIGHT,
                                        rinfo, torques);
    
    unsigned int side = (rinfo.stance() == LEFT_STANCE ? LEFT : RIGHT);
    jt.t(side, KY) += CWConfig::ST_KY_P_GAIN * (-rinfo.jstate().phi(side, KY))
                        + CWConfig::ST_KY_D_GAIN * (-rinfo.jstate().omega(side, KY));
    
    return jt;
}

/* Compute the torques resulting from applying a virtual force at the root. */
JSpTorques TorqueController::rootForceControl(const RobotInfo& rinfo, double comOffsetCoronal, double velDSagittal, double velDCoronal)
{
    Eigen::Vector3d virtualRootForce = computeVirtualForce(rinfo, comOffsetCoronal, velDSagittal, velDCoronal);
    dbg->virtualRootForce = virtualRootForce;
    
    Eigen::Vector3d stanceAnkleTorque, stanceKneeTorque, stanceHipTorque;
    COMJT(rinfo, virtualRootForce, stanceAnkleTorque, stanceKneeTorque, stanceHipTorque);
    
    RawTorques torques;
    torques.at(rinfo.stanceAnkleIndex()) = stanceAnkleTorque;
    torques.at(rinfo.stanceKneeIndex()) = stanceKneeTorque;
    torques.at(rinfo.stanceHipIndex()) = stanceHipTorque;
    
    return transformLegTorques(rinfo.stance() == LEFT_STANCE ? LEFT : RIGHT,
                               rinfo, torques);
}

/* Calculate the center of pressure resulting from the reaction force (T,F)
   on the stance foot. */
Eigen::Vector3d TorqueController::calcCoP(const RobotInfo& rinfo, const Eigen::Vector3d& T, const Eigen::Vector3d& F)
{
    const double h = CharacterConst::footSizeZ / 2.;
    const Eigen::Vector3d n = CWConfig::UP;
    
    if(F.dot(n) < 0.0001) {
        return Eigen::Vector3d::Zero();
    } else {
        Eigen::Vector3d p = (n.cross(T) - h*F)/(F.dot(n));
        if(rinfo.stance() == LEFT_STANCE)
            return rinfo.fstate().trToLocal(B_L_FOOT).onVector(p);
        else
            return rinfo.fstate().trToLocal(B_R_FOOT).onVector(p);
    }
}

/* Calculate the maximum gain, 0 < gain < 1, such that applying the joint space
   torques jt0 + gain*jt on the robot keeps the center of pressure inside the
   90% foot area, and the tangential component of the reaction force smaller
   than its normal component. */
double TorqueController::calcMaxGain(const RobotInfo& rinfo, const JSpTorques& jt0, const JSpTorques& jt)
{
    double gain = 0.5;
    double inc = 0.25;
    
    const double maxCoP_x = 0.9 * CharacterConst::footSizeX/2.;
    const double maxCoP_y = 0.9 * CharacterConst::footSizeY/2.;
    
    Eigen::Vector3d F0, T0;
    Eigen::Vector3d F1, T1;
    calcReaction(F0, T0, rinfo.fstate(), rinfo.jstate(), jt0, rinfo.stance());
    calcReaction(F1, T1, rinfo.fstate(), rinfo.jstate(), jt0+jt, rinfo.stance());
    
    for(unsigned int iter=0; iter<24; iter++) {
        Eigen::Vector3d CoP = calcCoP(rinfo, T0 + gain*(T1-T0), F0 + gain*(F1-F0));
        
        Eigen::Vector3d F = F0 + gain*(F1-F0);
        double Ft = sqrt(F.x()*F.x() + F.y()*F.y());
        double Fn = F.z();
        
        if(std::abs(CoP.x()) < maxCoP_x && std::abs(CoP.y()) < maxCoP_y && Ft < Fn)
            gain += inc;
        else
            gain -= inc;
        
        inc *= 0.5;
    }
    
    return gain;
}

/**
	This method is used to compute the torques
*/
JSpTorques TorqueController::computeTorques(const RobotInfo& rinfo, const IKSwingLegTarget& desiredPose, double comOffsetCoronal, double velDSagittal, double velDCoronal, double desiredHeading)
{
    JSpTorques jt_swing, jt_stance, jt_vrf;
    
    jt_swing = SwingController::swingLegControl(rinfo, desiredPose);
    jt_stance = stanceLegControl(rinfo, desiredHeading);
    jt_vrf = rootForceControl(rinfo, comOffsetCoronal, velDSagittal, velDCoronal);
    
    double vrfGain = calcMaxGain(rinfo, jt_swing + jt_stance, jt_vrf);
    dbg->vrfGain = vrfGain;
    
    return jt_swing + jt_stance + vrfGain*jt_vrf;
}
