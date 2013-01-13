#include "IKVMCController.h"

#include <Core/TwoLinkIK.h>

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
IKSwingLegTarget IKVMCController::computeIKQandW(const RobotInfo& rinfo, const Vector3d& parentAxis, const Vector3d& parentNormal, const Vector3d& childNormal, const Vector3d& childEndEffector, const Point3d& wP, bool computeAngVelocities, const Point3d& futureWP, double dt)
{
    IKSwingLegTarget desiredPose;
    
    //assert(parentJIndex == swingHipIndex);
    //assert(childJIndex == swingKneeIndex);

    //this is the joint between the grandparent RB and the parent
    // FIXME
    KTJoint* parentJoint = rinfo.character()->getJoints()[rinfo.swingHipIndex()];
    //this is the grandparent - most calculations will be done in its coordinate frame
    ArticulatedRigidBody* gParent = parentJoint->parent;
    //this is the reduced character space where we will be setting the desired orientations and ang vels.
    //ReducedCharacterState rs(&desiredPose);

    //the desired relative orientation between parent and grandparent
    Quaternion qParent;
    //and the desired relative orientation between child and parent
    Quaternion qChild;


    TwoLinkIK::getIKOrientations(parentJoint->getParentJointPosition(), gParent->getLocalCoordinatesForPoint(wP), parentNormal, parentAxis, childNormal, childEndEffector, &qParent, &qChild);

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
        boundToRange(&wChildD.x(), -5, 5);
        boundToRange(&wChildD.y(), -5, 5);
        boundToRange(&wChildD.z(), -5, 5);
        boundToRange(&wParentD.x(), -5, 5);
        boundToRange(&wParentD.y(), -5, 5);
        boundToRange(&wParentD.z(), -5, 5);
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
IKSwingLegTarget IKVMCController::computeIKSwingLegTargets(const RobotInfo& rinfo, const Vector3d& swingFootPos, const Vector3d& swingFootVel, double swingFootHeight, double swingFootHeightVel)
{
    //this is the vector that specifies the plane of rotation for the swing leg, relative to the root...
	Vector3d swingLegPlaneOfRotation = Vector3d(-1,0,0);;
    
    const double dt = 0.001;
    
    const double hNow = swingFootHeight;
    const double hFuture = swingFootHeight + swingFootHeightVel*dt;
    
    Point3d pNow, pFuture;
    pNow = transformSwingFootTarget(swingFootPos, rinfo.comPos(), rinfo.characterFrame(), hNow);
    pFuture = transformSwingFootTarget(swingFootPos + swingFootVel*dt,
        rinfo.comPos() + rinfo.comVel()*dt, rinfo.characterFrame(), hFuture);
        
    dbg->desSwingPos = pNow;
    
    Vector3d parentAxis = rinfo.character()->getJoints()[rinfo.swingKneeIndex()]->getParentJointPosition() - rinfo.character()->getJoints()[rinfo.swingHipIndex()]->getChildJointPosition();
    Vector3d childAxis = rinfo.character()->getJoints()[rinfo.swingAnkleIndex()]->getParentJointPosition() - rinfo.character()->getJoints()[rinfo.swingKneeIndex()]->getChildJointPosition();

    return computeIKQandW(rinfo, parentAxis, swingLegPlaneOfRotation, Vector3d(-1,0,0), childAxis, pNow, true, pFuture, dt);
    // computeIKQandW(swingHipIndex, swingKneeIndex, Vector3d(0, -0.355, 0), Vector3d(1,0,0), Vector3d(1,0,0), Vector3d(0, -0.36, 0), pNow, true, pFuture, dt);
}

/**
	This method returns a target for the location of the swing foot, based on some state information. It is assumed that the velocity vel
	is expressed in character-relative coordinates (i.e. the sagittal component is the z-component), while the com position, and the
	initial swing foot position is expressed in world coordinates. The resulting desired foot position is also expressed in world coordinates.
*/
Vector3d IKVMCController::transformSwingFootTarget(Vector3d step, const Point3d& com, const Quaternion& charFrameToWorld, double height)
{
	//now transform this vector into world coordinates
	step = charFrameToWorld.rotate(step);
	//add it to the com location
	step = com + step;
	//finally, set the desired height of the foot
	step.y() = height;

	// step += computeSwingFootDelta(t);
	
	return step;
}
