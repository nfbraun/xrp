#include "TorqueController.h"
#include "../../DynTransform.h"
#include <Core/VirtualModelController.h>

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

/**
	This method computes the torques that cancel out the effects of gravity, 
	for better tracking purposes
*/
RawTorques TorqueController::computeGravityCompensationTorques(const RobotInfo& rinfo)
{
    RawTorques torques;
    
    VirtualModelController::addJointTorquesEquivalentToForce(rinfo.character()->getJoints()[rinfo.swingHipIndex()], Point3d(), Vector3d(0, 0, rinfo.character()->getJoints()[rinfo.swingHipIndex()]->child->getMass()*9.8), NULL, torques);
    VirtualModelController::addJointTorquesEquivalentToForce(rinfo.character()->getJoints()[rinfo.swingKneeIndex()], Point3d(), Vector3d(0, 0, rinfo.character()->getJoints()[rinfo.swingKneeIndex()]->child->getMass()*9.8), NULL, torques);
    VirtualModelController::addJointTorquesEquivalentToForce(rinfo.character()->getJoints()[rinfo.swingAnkleIndex()], Point3d(), Vector3d(0, 0, rinfo.character()->getJoints()[rinfo.swingAnkleIndex()]->child->getMass()*9.8), NULL, torques);
    
    return torques;
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
	This method is used to ensure that each RB sees the net torque that the PD controller computed for it.
	Without it, an RB sees also the sum of -t of every child.
*/
void TorqueController::bubbleUpTorques(const RobotInfo& rinfo, RawTorques& torques)
{
	for (int i=J_MAX-1;i>=0;i--){
		if (i != rinfo.stanceHipIndex() && i != rinfo.stanceKneeIndex())
			if (rinfo.character()->getJoints()[i]->getParent() != rinfo.root())
				torques.at(rinfo.character()->getJoints()[i]->getParent()->pJoint->id) +=  torques.at(i);
	}
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

JSpTorques TorqueController::transformTorques(const RobotInfo& rinfo, const RawTorques& torques)
{
    JSpTorques jt;
    const Character* character = rinfo.character();
    
    /*** Left leg ***/
    Vector3d cf_lKneeAxis(0., 1., 0.);
    Eigen::Vector3d lKneeAxis = character->getARBs()[B_L_THIGH]->getOrientation().rotate(cf_lKneeAxis).toEigen();
    
    Vector3d cf_lAnkleAxis1(1., 0., 0.);
    Vector3d cf_lAnkleAxis2(0., 1., 0.);
    Eigen::Vector3d lAnkleAxis1 = character->getARBs()[B_L_FOOT]->getOrientation().rotate(cf_lAnkleAxis1).toEigen();
    Eigen::Vector3d lAnkleAxis2 = character->getARBs()[B_L_SHANK]->getOrientation().rotate(cf_lAnkleAxis2).toEigen();
    
    Eigen::Vector3d lHipTorque = character->getARBs()[B_PELVIS]->getOrientation().inverseRotate(torques.get(J_L_HIP)).toEigen();
    
    Eigen::Quaterniond lHipRot = character->getARBs()[B_PELVIS]->getOrientation().conjugate().toEigen() *
        character->getARBs()[B_L_THIGH]->getOrientation().toEigen();
    double lhz, lhy, lhx;
    decompZYXRot(lHipRot, lhz, lhy, lhx);
    
    invTransformHipTorque(lhz, lhy, lhx, -lHipTorque, jt.t(LEFT, HZ), jt.t(LEFT, HY), jt.t(LEFT, HX));
    
    jt.t(LEFT, KY) = -lKneeAxis.dot(torques.get(J_L_KNEE).toEigen());
    jt.t(LEFT, AX) = -lAnkleAxis1.dot(torques.get(J_L_ANKLE).toEigen());
    jt.t(LEFT, AY) = -lAnkleAxis2.dot(torques.get(J_L_ANKLE).toEigen());
    
    /*** Right leg ***/
    Vector3d cf_rKneeAxis(0., 1., 0.);
    Eigen::Vector3d rKneeAxis = character->getARBs()[B_R_THIGH]->getOrientation().rotate(cf_rKneeAxis).toEigen();
    
    Vector3d cf_rAnkleAxis1(1., 0., 0.);
    Vector3d cf_rAnkleAxis2(0., 1., 0.);
    Eigen::Vector3d rAnkleAxis1 = character->getARBs()[B_R_FOOT]->getOrientation().rotate(cf_rAnkleAxis1).toEigen();
    Eigen::Vector3d rAnkleAxis2 = character->getARBs()[B_R_SHANK]->getOrientation().rotate(cf_rAnkleAxis2).toEigen();
    
    Eigen::Vector3d rHipTorque = character->getARBs()[B_PELVIS]->getOrientation().inverseRotate(torques.get(J_R_HIP)).toEigen();
    
    Eigen::Quaterniond rHipRot = character->getARBs()[B_PELVIS]->getOrientation().conjugate().toEigen() *
        character->getARBs()[B_R_THIGH]->getOrientation().toEigen();
    double rhz, rhy, rhx;
    decompZYXRot(rHipRot, rhz, rhy, rhx);
    
    invTransformHipTorque(rhz, rhy, rhx, -rHipTorque, jt.t(RIGHT, HZ), jt.t(RIGHT, HY), jt.t(RIGHT, HX));
    
    jt.t(RIGHT, KY) = -rKneeAxis.dot(torques.get(J_R_KNEE).toEigen());
    jt.t(RIGHT, AX) = -rAnkleAxis1.dot(torques.get(J_R_ANKLE).toEigen());
    jt.t(RIGHT, AY) = -rAnkleAxis2.dot(torques.get(J_R_ANKLE).toEigen());
    
    return jt;
}

/**
	This method is used to compute the torques
*/
JSpTorques TorqueController::computeTorques(const RobotInfo& rinfo, const ContactInfo& cfs, const IKSwingLegTarget& desiredPose, double comOffsetCoronal, double velDSagittal, double velDCoronal, double desiredHeading)
{
	RawTorques torques;
	
	torques.at(rinfo.swingHipIndex()) = poseControl.computePDJointTorque(rinfo,
	    rinfo.swingHipIndex(), desiredPose.swingHipOrient, desiredPose.swingHipAngVel, false);
	torques.at(rinfo.swingKneeIndex()) = poseControl.computePDJointTorque(rinfo,
	    rinfo.swingKneeIndex(), desiredPose.swingKneeOrient, desiredPose.swingKneeAngVel, false);
	
	torques.at(rinfo.stanceKneeIndex()) = poseControl.computePDJointTorque(rinfo,
	    rinfo.stanceKneeIndex(), Quaternion(1., 0., 0., 0.), Vector3d(0., 0., 0.), false);
	
	torques.at(rinfo.stanceAnkleIndex()) = poseControl.computePDJointTorque(rinfo,
	    rinfo.stanceAnkleIndex(), Quaternion(1., 0., 0., 0.), Vector3d(0., 0., 0.), true,
	    rinfo.characterFrame());
	torques.at(rinfo.swingAnkleIndex()) = poseControl.computePDJointTorque(rinfo,
	    rinfo.swingAnkleIndex(), Quaternion(1., 0., 0., 0.), Vector3d(0., 0., 0.), true,
	    rinfo.characterFrame());

	//bubble-up the torques computed from the PD controllers
	bubbleUpTorques(rinfo, torques);
	
	//we'll also compute the torques that cancel out the effects of gravity, for better tracking purposes (swing leg only)
	torques.add(computeGravityCompensationTorques(rinfo));

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
	    torques.at(rinfo.stanceHipIndex()) = stanceHipTorque - virtualRootTorque - torques.at(rinfo.swingHipIndex());
	}

    return transformTorques(rinfo, torques);
}
