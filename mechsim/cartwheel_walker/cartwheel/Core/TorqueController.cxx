#include "TorqueController.h"
#include "../../DynTransform.h"
#include "SwingController.h"

TorqueController::TorqueController()
{
    rootControlParams.setKp(1000.0);
    rootControlParams.setKd(200.0);
    rootControlParams.setMaxAbsTorque(200.0);
    rootControlParams.setScale(Vector3d(1.0, 1.0, 1.0));
    
    addControlParams(J_L_HIP, 300.0, 35.0, 100.0, Vector3d( 1.0, 1.0, 1.0 ) );
    addControlParams(J_R_HIP, 300.0, 35.0, 100.0, Vector3d( 1.0, 1.0, 1.0 ) );
    addControlParams(J_L_KNEE, 300.0, 35.0, 200.0, Vector3d( 1.0, 1.0, 1.0 ) );
    addControlParams(J_R_KNEE, 300.0, 35.0, 200.0, Vector3d( 1.0, 1.0, 1.0 ) );
    addControlParams(J_L_ANKLE, 50.0, 15.0, 50.0, Vector3d( 0.2, 1.0, 0.2 ) );
    addControlParams(J_R_ANKLE, 50.0, 15.0, 50.0, Vector3d( 0.2, 1.0, 0.2 ) );
}

void TorqueController::addControlParams(JointID jid, double kp, double kd, double tauMax, const Vector3d& scale)
{
    poseControl.controlParams[jid].setKp(kp);
    poseControl.controlParams[jid].setKd(kd);
    poseControl.controlParams[jid].setMaxAbsTorque(tauMax);
    poseControl.controlParams[jid].setScale(scale);
}

void TorqueController::COMJT(const RobotInfo& rinfo, const Vector3d& fA, Vector3d& stanceAnkleTorque, Vector3d& stanceKneeTorque, Vector3d& stanceHipTorque)
{
	//applying a force at the COM induces the force f. The equivalent torques are given by the J' * f, where J' is
	// dp/dq, where p is the COM.

    Vector3d anklePos = rinfo.jPos(rinfo.stanceAnkleIndex());
    Vector3d kneePos = rinfo.jPos(rinfo.stanceKneeIndex());
    Vector3d hipPos = rinfo.jPos(rinfo.stanceHipIndex());

	//total mass...
	// double m = lowerLeg->getMass() + upperLeg->getMass() + pelvis->getMass();
	const double pelvisMass = rinfo.rbMass(rinfo.rootIndex());
	const double thighMass = rinfo.rbMass(rinfo.stanceThighIndex());
	const double shankMass = rinfo.rbMass(rinfo.stanceShankIndex());
	const double m = pelvisMass + thighMass + shankMass;

	Vector3d f1 =	(rinfo.rbPos(rinfo.stanceShankIndex()) - anklePos) * shankMass +
					(rinfo.rbPos(rinfo.stanceThighIndex()) - anklePos) * thighMass + 
					(rinfo.rbPos(rinfo.rootIndex()) - anklePos) * pelvisMass; 
	f1 /= m;
	
	Vector3d f2 =	(rinfo.rbPos(rinfo.stanceThighIndex()) - kneePos) * thighMass + 
					(rinfo.rbPos(rinfo.rootIndex()) - kneePos) * pelvisMass;
	f2 /= m;

	Vector3d f3 =	(rinfo.rbPos(rinfo.rootIndex()) - hipPos) * pelvisMass;
	f3 /= m;

	stanceAnkleTorque = f1.cross(fA);
	stanceKneeTorque = f2.cross(fA);
	stanceHipTorque = f3.cross(fA);
}

/**
	This method is used to return the ratio of the weight that is supported by the stance foot.
*/
double TorqueController::getStanceFootWeightRatio(const RobotInfo& rinfo, const ContactInfo& cfs)
{
	Vector3d stanceFootForce = cfs.getForceOnFoot(rinfo.stanceFootIndex());
	Vector3d swingFootForce = cfs.getForceOnFoot(rinfo.swingFootIndex());
	double totalZForce = (stanceFootForce + swingFootForce).dot(PhysicsGlobals::up);

	if (IS_ZERO(totalZForce))
		return -1;
	else
		return stanceFootForce.dot(PhysicsGlobals::up) / totalZForce;
}

/**
	This method returns performes some pre-processing on the virtual torque. The torque is assumed to be in world coordinates,
	and it will remain in world coordinates.
*/
Vector3d TorqueController::preprocessAnkleVTorque(const RobotInfo& rinfo, const ContactInfo& cfs, Vector3d ankleVTorque, double phi)
{
	ArticulatedRigidBody* foot = rinfo.character()->getARBs()[rinfo.stanceFootIndex()];
	ankleVTorque = foot->getLocalCoordinatesForVector(ankleVTorque);
	
	if (cfs.toeInContact(rinfo.stanceFootIndex(), foot) == false || phi < 0.2 || phi > 0.8) ankleVTorque.y() = 0;

	Vector3d footRelativeAngularVel = foot->getLocalCoordinatesForVector(foot->getAngularVelocity());
	if ((footRelativeAngularVel.x() < -0.2 && ankleVTorque.x() > 0) || (footRelativeAngularVel.x() > 0.2 && ankleVTorque.x() < 0))
		ankleVTorque.x() = 0;

	if (fabs(footRelativeAngularVel.x()) > 1.0) ankleVTorque.x() = 0;
	if (fabs(footRelativeAngularVel.y()) > 1.0) ankleVTorque.y() = 0;
	
	boundToRange(&ankleVTorque.x(), -20, 20);

	ankleVTorque = foot->getWorldCoordinatesForVector(ankleVTorque);
	
	return ankleVTorque;
}

/**
	This method is used to compute the force that the COM of the character should be applying.
*/
Vector3d TorqueController::computeVirtualForce(const RobotInfo& rinfo, double desOffCoronal, double desVSagittal, double desVCoronal)
{
	//this is the desired acceleration of the center of mass
	Vector3d desA;
	desA.x() = (desVSagittal - rinfo.getV().x()) * 30;
	desA.y() = (desOffCoronal - rinfo.getD().y()) * 20 + (desVCoronal - rinfo.getV().y()) * 9;
	desA.z() = 0.;
	
	/* if (doubleStanceMode == true){
	    assert(false);
		Vector3d errV = characterFrame.inverseRotate(doubleStanceCOMError*-1);
		desA.x = (-errV.x + comOffsetCoronal) * 20 + (velDCoronal - getV().x) * 9;
		desA.z = (-errV.z + comOffsetSagittal) * 10 + (velDSagittal - getV().z) * 150;
	} */

	//and this is the force that would achieve that - make sure it's not too large...
	Vector3d fA = (desA) * rinfo.totalMass();
	boundToRange(&fA.x(), -60, 60);
	boundToRange(&fA.y(), -100, 100);

	//now change this quantity to world coordinates...
	fA = rinfo.characterFrame().rotate(fA);

	return fA;
}

/**
	This method is used to compute the torques that need to be applied to the stance and swing hips, given the
	desired orientation for the root and the swing hip.
*/
/* simplified (on 2013-01-02) */
Vector3d TorqueController::computeRootTorque(const RobotInfo& rinfo, double desHeading)
{
	//compute the total torques that should be applied to the root and swing hip, keeping in mind that
	//the desired orientations are expressed in the character frame
	Vector3d rootTorque;
	
	//this is the desired orientation in world coordinates
	Quaternion qRootDW;

	//qRootDW needs to also take into account the desired heading
	qRootDW = Quaternion::getRotationQuaternion(desHeading, PhysicsGlobals::up);

	//so this is the net torque that the root wants to see, in world coordinates
	rootTorque = poseControl.computePDTorque(rinfo.rootOrient(), qRootDW, rinfo.rootAngVel(), Vector3d(0,0,0), &rootControlParams);

	//now, based on the ratio of the forces the feet exert on the ground, and the ratio of the forces between the two feet, we will compute the forces that need to be applied
	//to the two hips, to make up the root makeup torque - which means the final torque the root sees is rootTorque!

	//and done...
	// torques.at(stanceHipIndex) -= torques.get(J_L_HIP) + torques.get(J_R_HIP) + rootTorque;
	return rootTorque;
}

void TorqueController::transformLegTorques(JSpTorques& jt, unsigned int side, const RobotInfo& rinfo, const RawTorques& torques)
{
    const Character* character = rinfo.character();
    
    unsigned int thighId = (side == LEFT ? B_L_THIGH : B_R_THIGH);
    unsigned int shankId = (side == LEFT ? B_L_SHANK : B_R_SHANK);
    unsigned int footId = (side == LEFT ? B_L_FOOT : B_R_FOOT);
    
    unsigned int hipId = (side == LEFT ? J_L_HIP : J_R_HIP);
    unsigned int kneeId = (side == LEFT ? J_L_KNEE : J_R_KNEE);
    unsigned int ankleId = (side == LEFT ? J_L_ANKLE : J_R_ANKLE);
    
    Vector3d cf_KneeAxis(0., 1., 0.);
    Eigen::Vector3d KneeAxis = character->getARBs()[thighId]->getOrientation().rotate(cf_KneeAxis).toEigen();
    
    Vector3d cf_AnkleAxis1(1., 0., 0.);
    Vector3d cf_AnkleAxis2(0., 1., 0.);
    Eigen::Vector3d AnkleAxis1 = character->getARBs()[footId]->getOrientation().rotate(cf_AnkleAxis1).toEigen();
    Eigen::Vector3d AnkleAxis2 = character->getARBs()[shankId]->getOrientation().rotate(cf_AnkleAxis2).toEigen();
    
    Eigen::Vector3d hipTorque = character->getARBs()[B_PELVIS]->getOrientation().inverseRotate(torques.get(hipId)).toEigen();
    
    Eigen::Quaterniond hipRot = character->getARBs()[B_PELVIS]->getOrientation().conjugate().toEigen() *
        character->getARBs()[thighId]->getOrientation().toEigen();
    double hz, hy, hx;
    decompZYXRot(hipRot, hz, hy, hx);
    
    invTransformHipTorque(hz, hy, hx, -hipTorque, jt.t(side, HZ), jt.t(side, HY), jt.t(side, HX));
    
    jt.t(side, KY) = -KneeAxis.dot(torques.get(kneeId).toEigen());
    jt.t(side, AX) = -AnkleAxis1.dot(torques.get(ankleId).toEigen());
    jt.t(side, AY) = -AnkleAxis2.dot(torques.get(ankleId).toEigen());
}

/* Compute the torques for the stance leg. */
void TorqueController::stanceLegControl(JSpTorques& jt, const RobotInfo& rinfo, const ContactInfo& cfs, double comOffsetCoronal, double velDSagittal, double velDCoronal, double desiredHeading)
{
    RawTorques torques;
    
    torques.at(rinfo.stanceKneeIndex()) = poseControl.computePDJointTorque(rinfo,
        rinfo.stanceKneeIndex(), Quaternion(1., 0., 0., 0.), Vector3d(0., 0., 0.), false);
    
    torques.at(rinfo.stanceAnkleIndex()) = poseControl.computePDJointTorque(rinfo,
        rinfo.stanceAnkleIndex(), Quaternion(1., 0., 0., 0.), Vector3d(0., 0., 0.), true,
        rinfo.characterFrame());
    
    //bubble-up the torques computed from the PD controllers
    torques.at(rinfo.stanceKneeIndex()) +=  torques.at(rinfo.stanceAnkleIndex());
    
    //and now separetely compute the torques for the hips - together with the feedback term, this is what defines simbicon
    dbg->StanceFootWeightRatio = getStanceFootWeightRatio(rinfo, cfs);
    
    if(getStanceFootWeightRatio(rinfo, cfs) > 0.9) {
        Vector3d virtualRootForce = computeVirtualForce(rinfo, comOffsetCoronal, velDSagittal, velDCoronal);
        Vector3d virtualRootTorque = computeRootTorque(rinfo, desiredHeading);
        
        dbg->virtualRootForce = virtualRootForce;
        dbg->virtualRootTorque = virtualRootTorque;
        
        Vector3d stanceAnkleTorque, stanceKneeTorque, stanceHipTorque;
        COMJT(rinfo, virtualRootForce, stanceAnkleTorque, stanceKneeTorque, stanceHipTorque);
        
        torques.at(rinfo.stanceAnkleIndex()) += preprocessAnkleVTorque(rinfo, cfs, stanceAnkleTorque, rinfo.phi());
        torques.at(rinfo.stanceKneeIndex()) += stanceKneeTorque;
        torques.at(rinfo.stanceHipIndex()) = stanceHipTorque - virtualRootTorque;
        // FIXME: maybe re-include: - torques.at(rinfo.swingHipIndex());
    }
    
    transformLegTorques(jt, rinfo.stance() == LEFT_STANCE ? LEFT : RIGHT,
                        rinfo, torques);
}

/**
	This method is used to compute the torques
*/
JSpTorques TorqueController::computeTorques(const RobotInfo& rinfo, const ContactInfo& cfs, const IKSwingLegTarget& desiredPose, double comOffsetCoronal, double velDSagittal, double velDCoronal, double desiredHeading)
{
    JSpTorques jt;
    
    SwingController::swingLegControl(jt, rinfo, desiredPose);
    stanceLegControl(jt, rinfo, cfs, comOffsetCoronal, velDSagittal, velDCoronal, desiredHeading);
    
    return jt;
}
