#include "IKVMCController.h"

#include <Core/BehaviourController.h>
#include <Core/TwoLinkIK.h>

/**
	constructor
*/
IKVMCController::IKVMCController(Character* b) : SimBiController(b), ip(this)
{
	velDSagittal = 0;
	velDCoronal = 0;

	comOffsetSagittal = 0;
	comOffsetCoronal = 0;

	panicHeight = 0;
	unplannedForHeight = 0;
	
	doubleStanceMode = false;
}

/**
	This method is used to compute the desired orientation and angular velocity for a parent RB and a child RB, relative to the grandparent RB and
	parent RB repsectively. The input is:
		- the index of the joint that connects the grandparent RB to the parent RB, and the index of the joint between parent and child

		- the distance from the parent's joint with its parent to the location of the child's joint, expressed in parent coordinates

		- two rotation normals - one that specifies the plane of rotation for the parent's joint, expressed in grandparent coords, 
		  and the other specifies the plane of rotation between the parent and the child, expressed in parent's coordinates.

	    - The position of the end effector, expressed in child's coordinate frame

		- The desired position of the end effector, expressed in world coordinates

		- an estimate of the desired position of the end effector, in world coordinates, some dt later - used to compute desired angular velocities
*/
IKSwingLegTarget IKVMCController::computeIKQandW(const Vector3d& parentAxis, const Vector3d& parentNormal, const Vector3d& childNormal, const Vector3d& childEndEffector, const Point3d& wP, bool computeAngVelocities, const Point3d& futureWP, double dt)
{
    IKSwingLegTarget desiredPose;
    
    //assert(parentJIndex == swingHipIndex);
    //assert(childJIndex == swingKneeIndex);

    //this is the joint between the grandparent RB and the parent
    KTJoint* parentJoint = character->getJoints()[swingHipIndex];
    //this is the grandparent - most calculations will be done in its coordinate frame
    ArticulatedRigidBody* gParent = parentJoint->parent;
    //this is the reduced character space where we will be setting the desired orientations and ang vels.
    //ReducedCharacterState rs(&desiredPose);

    //the desired relative orientation between parent and grandparent
    Quaternion qParent;
    //and the desired relative orientation between child and parent
    Quaternion qChild;


    TwoLinkIK::getIKOrientations(parentJoint->getParentJointPosition(), gParent->getLocalCoordinatesForPoint(wP), parentNormal, parentAxis, childNormal, childEndEffector, &qParent, &qChild);

    poseControl.controlParams[swingHipIndex].relToFrame = false;
    poseControl.controlParams[swingKneeIndex].relToFrame = false;
    //desiredPose.setJointRelativeOrientation(qChild, swingKneeIndex);
    //desiredPose.setJointRelativeOrientation(qParent, swingHipIndex);
    desiredPose.swingHipOrient = qParent;
    desiredPose.swingKneeOrient = qChild;


    Vector3d wParentD(0,0,0);
    Vector3d wChildD(0,0,0);

    if (computeAngVelocities){
        //the joint's origin will also move, so take that into account, by subbing the offset by which it moves to the
        //futureTarget (to get the same relative position to the hip)
        Vector3d velOffset = gParent->getAbsoluteVelocityForLocalPoint(parentJoint->getParentJointPosition());

        Quaternion qParentF;
        Quaternion qChildF;
        TwoLinkIK::getIKOrientations(parentJoint->getParentJointPosition(), gParent->getLocalCoordinatesForPoint(futureWP + velOffset * -dt), parentNormal, parentAxis, childNormal, childEndEffector, &qParentF, &qChildF);

        Quaternion qDiff = qParentF * qParent.getComplexConjugate();
        wParentD = qDiff.v * 2/dt;
        //the above is the desired angular velocity, were the parent not rotating already - but it usually is, so we'll account for that
        wParentD -= gParent->getLocalCoordinatesForVector(gParent->getAngularVelocity());

        qDiff = qChildF * qChild.getComplexConjugate();
        wChildD = qDiff.v * 2/dt;

        //make sure we don't go overboard with the estimates, in case there are discontinuities in the trajectories...
        boundToRange(&wChildD.x, -5, 5);
        boundToRange(&wChildD.y, -5, 5);
        boundToRange(&wChildD.z, -5, 5);
        boundToRange(&wParentD.x, -5, 5);
        boundToRange(&wParentD.y, -5, 5);
        boundToRange(&wParentD.z, -5, 5);
    }

    //desiredPose.setJointRelativeAngVelocity(wChildD, swingKneeIndex);
    //desiredPose.setJointRelativeAngVelocity(wParentD, swingHipIndex);
    desiredPose.swingHipAngVel = wParentD;
    desiredPose.swingKneeAngVel = wChildD;

    return desiredPose;
}

/**
	This method is used to compute the target angles for the swing hip and swing knee that help 
	to ensure (approximately) precise foot-placement control.
*/
IKSwingLegTarget IKVMCController::computeIKSwingLegTargets(const Vector3d& swingFootPos, const Vector3d& swingFootVel)
{
    //this is the vector that specifies the plane of rotation for the swing leg, relative to the root...
	Vector3d swingLegPlaneOfRotation = Vector3d(-1,0,0);;
    
    const double dt = 0.001;

    Point3d pNow, pFuture;
    pNow = transformSwingFootTarget(phi, swingFootPos, comPosition, characterFrame);
    pFuture = transformSwingFootTarget(phi+dt, swingFootPos + swingFootVel*dt,
        comPosition + comVelocity*dt, characterFrame);
        
    DEBUG_desSwingPos = pNow;
    
    Vector3d parentAxis(character->getJoints()[swingHipIndex]->getChildJointPosition(), character->getJoints()[swingKneeIndex]->getParentJointPosition());
    Vector3d childAxis(character->getJoints()[swingKneeIndex]->getChildJointPosition(), character->getJoints()[swingAnkleIndex]->getParentJointPosition());

    return computeIKQandW(parentAxis, swingLegPlaneOfRotation, Vector3d(-1,0,0), childAxis, pNow, true, pFuture, dt);
    // computeIKQandW(swingHipIndex, swingKneeIndex, Vector3d(0, -0.355, 0), Vector3d(1,0,0), Vector3d(1,0,0), Vector3d(0, -0.36, 0), pNow, true, pFuture, dt);
}

Vector3d IKVMCController::computeSwingFootDelta( double phiToUse, int stanceToUse )
{
    // unused for now (NB)
	/* if( phiToUse < 0 )
		phiToUse = phi;
	if( phiToUse > 1 )
		phiToUse = 1;
	if( stanceToUse < 0 )
		stanceToUse = stance;
	if( stanceToUse > 1 )
		stanceToUse = 1;

	double sign = (stanceToUse==LEFT_STANCE)?1.0:-1.0;
	Vector3d swingFootDelta(
		swingFootTrajectoryDeltaCoronal.evaluate_catmull_rom(phiToUse) * sign,
		swingFootTrajectoryDeltaHeight.evaluate_catmull_rom(phiToUse),
		swingFootTrajectoryDeltaSagittal.evaluate_catmull_rom(phiToUse) ); */
	return Vector3d(0., 0., 0.);
}

/**
	This method returns a target for the location of the swing foot, based on some state information. It is assumed that the velocity vel
	is expressed in character-relative coordinates (i.e. the sagittal component is the z-component), while the com position, and the
	initial swing foot position is expressed in world coordinates. The resulting desired foot position is also expressed in world coordinates.
*/
Vector3d IKVMCController::transformSwingFootTarget(double t, Vector3d step, const Point3d& com, const Quaternion& charFrameToWorld)
{
	//now transform this vector into world coordinates
	step = charFrameToWorld.rotate(step);
	//add it to the com location
	step = com + step;
	//finally, set the desired height of the foot
	step.y = swingFootHeightTrajectory.evaluate_catmull_rom(t) + panicHeight + unplannedForHeight;

	step += computeSwingFootDelta(t);
	
	return step;
}

/**
	This method is used to ensure that each RB sees the net torque that the PD controller computed for it.
	Without it, an RB sees also the sum of -t of every child.
*/
void IKVMCController::bubbleUpTorques(Character* character, RawTorques& torques){
	for (int i=J_MAX-1;i>=0;i--){
		if (i != stanceHipIndex && i != stanceKneeIndex)
			if (character->getJoints()[i]->getParent() != root)
				torques.at(character->getJoints()[i]->getParent()->pJoint->id) +=  torques.at(i);
	}
}

/**
	This method computes the torques that cancel out the effects of gravity, 
	for better tracking purposes
*/
RawTorques IKVMCController::computeGravityCompensationTorques(Character* character)
{
    RawTorques torques;

	for (unsigned int i=0;i<J_MAX;i++){
		if (i != stanceHipIndex && i != stanceKneeIndex && i != stanceAnkleIndex) {
			VirtualModelController::addJointTorquesEquivalentToForce(character, character->getJoints()[i], Point3d(), Vector3d(0, character->getJoints()[i]->child->getMass()*9.8, 0), NULL, torques);
		}
    }
    
    return torques;
}

/**
	This method is used to compute the force that the COM of the character should be applying.
*/
Vector3d IKVMCController::computeVirtualForce(){
	//this is the desired acceleration of the center of mass
	Vector3d desA = Vector3d();
	desA.z = (velDSagittal - getV().z) * 30;
	desA.x = (-getD().x + comOffsetCoronal) * 20 + (velDCoronal - getV().x) * 9;
	
	if (doubleStanceMode == true){
		Vector3d errV = characterFrame.inverseRotate(doubleStanceCOMError*-1);
		desA.x = (-errV.x + comOffsetCoronal) * 20 + (velDCoronal - getV().x) * 9;
		desA.z = (-errV.z + comOffsetSagittal) * 10 + (velDSagittal - getV().z) * 150;
	}

	//and this is the force that would achieve that - make sure it's not too large...
	Vector3d fA = (desA) * character->getMass();
	boundToRange(&fA.x, -100, 100);
	boundToRange(&fA.z, -60, 60);

	//now change this quantity to world coordinates...
	fA = characterFrame.rotate(fA);

	return fA;
}

/**
	This method returns performes some pre-processing on the virtual torque. The torque is assumed to be in world coordinates,
	and it will remain in world coordinates.
*/
void IKVMCController::preprocessAnkleVTorque(int ankleJointIndex, const std::vector<ContactPoint>& cfs, Vector3d *ankleVTorque){
	bool heelInContact, toeInContact;
	ArticulatedRigidBody* foot = character->getJoints()[ankleJointIndex]->child;
	getForceInfoOn(foot, cfs, &heelInContact, &toeInContact);
	*ankleVTorque = foot->getLocalCoordinatesForVector(*ankleVTorque);

	if (toeInContact == false || phi < 0.2 || phi > 0.8) ankleVTorque->x = 0;

	Vector3d footRelativeAngularVel = foot->getLocalCoordinatesForVector(foot->getAngularVelocity());
	if ((footRelativeAngularVel.z < -0.2 && ankleVTorque->z > 0) || (footRelativeAngularVel.z > 0.2 && ankleVTorque->z < 0))
		ankleVTorque->z = 0;

	if (fabs(footRelativeAngularVel.z) > 1.0) ankleVTorque->z = 0;
	if (fabs(footRelativeAngularVel.x) > 1.0) ankleVTorque->x = 0;
	
	boundToRange(&ankleVTorque->z, -20, 20);

	*ankleVTorque = foot->getWorldCoordinatesForVector(*ankleVTorque);
}

/**
	This method is used to compute torques for the stance leg that help achieve a desired speed in the sagittal and lateral planes
*/
void IKVMCController::computeLegTorques(int ankleIndex, int kneeIndex, int hipIndex, const std::vector<ContactPoint>& cfs, RawTorques& torques){
	Vector3d fA = computeVirtualForce();

	Vector3d r;

	Point3d p = comPosition;

	r.setToVectorBetween(character->getJoints()[ankleIndex]->child->getWorldCoordinatesForPoint(character->getJoints()[ankleIndex]->getChildJointPosition()), p);

	Vector3d ankleTorque = r.crossProductWith(fA);
	preprocessAnkleVTorque(ankleIndex, cfs, &ankleTorque);
	torques.at(ankleIndex) += ankleTorque;

	r.setToVectorBetween(character->getJoints()[kneeIndex]->child->getWorldCoordinatesForPoint(character->getJoints()[kneeIndex]->getChildJointPosition()), p);
	torques.at(kneeIndex) += r.crossProductWith(fA);

	r.setToVectorBetween(character->getJoints()[hipIndex]->child->getWorldCoordinatesForPoint(character->getJoints()[hipIndex]->getChildJointPosition()), p);
	torques.at(hipIndex) += r.crossProductWith(fA);

	//the torque on the stance hip is cancelled out, so pass it in as a torque that the root wants to see!
	ffRootTorque -= r.crossProductWith(fA);

	/* int lBackIndex = J_PELVIS_LOWERBACK; //character->getJointIndex("pelvis_lowerback");
	r.setToVectorBetween(character->getJoints()[lBackIndex]->child->getWorldCoordinates(character->getJoints()[lBackIndex]->cJPos), p);
	torques[lBackIndex] += r.crossProductWith(fA) / 10;

	int mBackIndex = J_LOWERBACK_TORSO; //character->getJointIndex("lowerback_torso");
	r.setToVectorBetween(character->getJoints()[mBackIndex]->child->getWorldCoordinates(character->getJoints()[mBackIndex]->cJPos), p);
	torques[mBackIndex] += r.crossProductWith(fA) / 10; */
}

RawTorques IKVMCController::COMJT(const std::vector<ContactPoint>& cfs)
{
    RawTorques torques;

	//applying a force at the COM induces the force f. The equivalent torques are given by the J' * f, where J' is
	// dp/dq, where p is the COM.

	ArticulatedRigidBody* lowerLeg = character->getJoints()[stanceAnkleIndex]->parent;
	ArticulatedRigidBody* upperLeg = character->getJoints()[stanceKneeIndex]->parent;
	ArticulatedRigidBody* pelvis = character->getJoints()[stanceHipIndex]->parent;

	Point3d anklePos = character->getJoints()[stanceAnkleIndex]->child->getWorldCoordinatesForPoint(character->getJoints()[stanceAnkleIndex]->getChildJointPosition());
	Point3d kneePos = character->getJoints()[stanceKneeIndex]->child->getWorldCoordinatesForPoint(character->getJoints()[stanceKneeIndex]->getChildJointPosition());
	Point3d hipPos = character->getJoints()[stanceHipIndex]->child->getWorldCoordinatesForPoint(character->getJoints()[stanceHipIndex]->getChildJointPosition());

	//total mass...
	double m = lowerLeg->getMass() + upperLeg->getMass() + pelvis->getMass();

	Vector3d fA = computeVirtualForce();

	Vector3d f1 =	Vector3d(anklePos, lowerLeg->state.position) * lowerLeg->getMass() +
					Vector3d(anklePos, upperLeg->state.position) * upperLeg->getMass() + 
					Vector3d(anklePos, pelvis->state.position) * pelvis->getMass(); 
	f1 /= m;
	
	Vector3d f2 =	Vector3d(kneePos, upperLeg->state.position) * upperLeg->getMass() + 
					Vector3d(kneePos, pelvis->state.position) * pelvis->getMass();
	f2 /= m;

	Vector3d f3 =	Vector3d(hipPos, pelvis->state.position) * pelvis->getMass();
	f3 /= m;

	Vector3d ankleTorque = f1.crossProductWith(fA);
	preprocessAnkleVTorque(stanceAnkleIndex, cfs, &ankleTorque);

	torques.at(stanceAnkleIndex) += ankleTorque;
	torques.at(stanceKneeIndex) += f2.crossProductWith(fA);
	torques.at(stanceHipIndex) += f3.crossProductWith(fA);

	//the torque on the stance hip is cancelled out, so pass it in as a torque that the root wants to see!
	ffRootTorque -= f3.crossProductWith(fA);
	
	return torques;
}

/**
	updates the indexes of the swing and stance hip, knees and ankles
*/
void IKVMCController::updateSwingAndStanceReferences(){
	stanceHipIndex = ((stance == LEFT_STANCE) ? (J_L_HIP) : (J_R_HIP));
	swingHipIndex = ((stance == LEFT_STANCE) ? (J_R_HIP) : (J_L_HIP));
	stanceKneeIndex = ((stance == LEFT_STANCE) ? (J_L_KNEE) : (J_R_KNEE));
	swingKneeIndex = ((stance == LEFT_STANCE) ? (J_R_KNEE) : (J_L_KNEE));
	stanceAnkleIndex = ((stance == LEFT_STANCE) ? (J_L_ANKLE) : (J_R_ANKLE));
	swingAnkleIndex = ((stance == LEFT_STANCE) ? (J_R_ANKLE) : (J_L_ANKLE));

	swingHip = character->getJoints()[swingHipIndex];
	swingKnee = character->getJoints()[swingKneeIndex];
}

/**
	This method is used to compute the torques
*/
RawTorques IKVMCController::computeTorques(const std::vector<ContactPoint>& cfs)
{
	//d and v are specified in the rotation (heading) invariant coordinate frame
	updateDAndV();

	//evaluate the target orientation for every joint, using the SIMBICON state information
	initControlParams();

	//get the correct references for the swing knee and hip, as well as an estimate of the starting position for the swing foot
	

	//now overwrite the target angles for the swing hip and the swing knee in order to ensure foot-placement control
	//if (doubleStanceMode == false)
	
	//compute desired swing foot location...
	Vector3d desiredPos, desiredVel;
	ip.calcDesiredSwingFootLocation(desiredPos, desiredVel);
	IKSwingLegTarget desiredPose = computeIKSwingLegTargets(desiredPos, desiredVel);
	
	//DEBUG_desSwingPos = desiredPos;
	//DEBUG_desSwingVel = desiredVel;

	RawTorques torques;
	
	/* for (int jid=0;jid<character->getJointCount();jid++){
	    torques.at(jid) = poseControl.computePDJointTorque(character, jid,
	        desiredPose.getJointRelativeOrientation(jid),
	        desiredPose.getJointRelativeAngVelocity(jid));
	} */
	torques.at(swingHipIndex) = poseControl.computePDJointTorque(character, swingHipIndex,
	    desiredPose.swingHipOrient, desiredPose.swingHipAngVel, false);
	torques.at(swingKneeIndex) = poseControl.computePDJointTorque(character, swingKneeIndex,
	    desiredPose.swingKneeOrient, desiredPose.swingKneeAngVel, false);
	
	torques.at(stanceHipIndex) = poseControl.computePDJointTorque(character, stanceHipIndex,
	    Quaternion(1., 0., 0., 0.), Vector3d(0., 0., 0.), false);
	torques.at(stanceKneeIndex) = poseControl.computePDJointTorque(character, stanceKneeIndex,
	    Quaternion(1., 0., 0., 0.), Vector3d(0., 0., 0.), false);
	
	torques.at(stanceAnkleIndex) = poseControl.computePDJointTorque(character, stanceAnkleIndex,
	    Quaternion(1., 0., 0., 0.), Vector3d(0., 0., 0.), true);
	torques.at(swingAnkleIndex) = poseControl.computePDJointTorque(character, swingAnkleIndex,
	    Quaternion(1., 0., 0., 0.), Vector3d(0., 0., 0.), true);

	//bubble-up the torques computed from the PD controllers
	bubbleUpTorques(character, torques);
	

	//we'll also compute the torques that cancel out the effects of gravity, for better tracking purposes
	torques.add(computeGravityCompensationTorques(character));

	ffRootTorque = Vector3d(0,0,0);

	if (cfs.size() > 0)
		torques.add(COMJT(cfs));
//		computeLegTorques(stanceAnkleIndex, stanceKneeIndex, stanceHipIndex, cfs);

	//if (doubleStanceMode && cfs->size() > 0)
	//	computeLegTorques(swingAnkleIndex, swingKneeIndex, swingHipIndex, cfs);
	

	//and now separetely compute the torques for the hips - together with the feedback term, this is what defines simbicon
	computeHipTorques(qRootD, getStanceFootWeightRatio(cfs), ffRootTorque, torques);
//	blendOutTorques();

    return torques;
}

/**
	determines if there are any heel/toe forces on the given RB
*/
void IKVMCController::getForceInfoOn(RigidBody* rb, const std::vector<ContactPoint>& cfs, bool* heelForce, bool* toeForce){
	//figure out if the toe/heel are in contact...
	*heelForce = false;
	*toeForce = false;
	Point3d tmpP;
	for (unsigned int i=0;i<cfs.size();i++){
		if (haveRelationBetween(cfs[i].rb1, rb) || haveRelationBetween(cfs[i].rb2, rb)){
			tmpP = rb->getLocalCoordinatesForPoint(cfs[i].cp);
			if (tmpP.z < 0) *heelForce = true;
			if (tmpP.z > 0) *toeForce = true;
		}
	}	
}

