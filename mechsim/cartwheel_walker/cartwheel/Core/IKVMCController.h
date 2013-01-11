#pragma once

#include <MathLib/Trajectory.h>

#include <Core/VirtualModelController.h>

#include <Core/TorqueController.h>

#include "ContactInfo.h"

#include "Debug.h"
#include "InvPendulum.h"

/**
	Controller Inspired by Simbicon and Virtual Model Control ideas. Uses IK to control swing foot placement. It assumes
	a pretty specilized character architecture: Legs look like this: 
		root -> hips (3 dof) -> knees (1 dof, around x-axis, for now) -> ankles (doesn't matter how many dofs) -> (may or may not have toes, etc).

	An IKVMCController is a Simbicon Controller. The targets for the swing hip and swing knee are computed using IK, based on a target
	foot location (which is given by the inverted pendulum prediction), and the stance ~hip, knee and ankle torques are computed in part
	by the desired velocity of the CM. Gravity compensation can be added to any rb in the body (propagated to the root). Also, "torque bubbling"
	can be added for any chain that starts at the root.

	TODO:

		- try out some stairs, slopes
		- try some IK for the arm, while walking
		- try some static balance, arms moving...
		- get a less 'hacky' implementation of the desired foot placement position (at least in the sagittal plane).

		- test out 'external' foot placement accuracy - DONE!
		- make walk more natural - how much is simbicon, how much is T = J' * f for push-off? How much does the vertical trajectory of swing foot matter? - DONE!
		- eliminate/reduce the torso/head wobbling. - DONE!
		- stance leg/swing leg velocity/position control through T = J' * f - DONE!
		- get some decent toe-off - Pretty much DONE!
		- torque bubbling - DONE!
		- gravity compensation - DONE!
		- check to see why the crouched walk happens sometime - stance knee should be straight, shouldn't it? - DONE!
		- check to see why swing hip initially goes inwards! - DONE!
		- make sure character can walk in any direction - DONE!
		- limit the extent to which stance ankle 'rolls' on the heel - DONE!
		- how much should the strength of the root be? What works best?
			- doesn't actually matter too much. 1 seems to work best, so we'll go with that.



		- THINGS TO TRY STILL:
			- try to somehow bring the upper body into the motion - maybe simulate the force being applied
			to the CM - i.e. t = dpCM/dq * f, where CM now only takes into account the chain from the head/torso
			to the stance foot - i don't know...
			- upper body - can it be used to make it slow down/speed up/whatever faster?
			- maybe com J'?
*/

class HighLevelTarget {
  public:
    double velDSagittal, velDCoronal;
    double desiredHeading;
    double swingFootHeight, swingFootHeightVel;
};

class IKVMCController {
  public:
	/**
		This method is used to compute the target angles for the swing hip and swing knee that help 
		to ensure (approximately) precise foot-placement control.
	*/
	IKSwingLegTarget computeIKSwingLegTargets(const RobotInfo& rinfo, const Vector3d& swingFootPos, const Vector3d& swingFootVel, double swingFootHeight, double swingFootHeightVel);
	
	DebugInfo* dbg;
	
  private:
	/**
		This method computes the desired target location for the swing ankle. It also returns an estimate of the desired
		foot location some time dt later. This method returns the points expressed in world coordinates.
	*/
//	void getDesiredSwingAngleLocation(Point3d* target, Point3d* futureTarget, double dt);

	/**
		This method returns a target for the location of the swing foot, based on some state information. It is assumed that the velocity v
		is expressed in character-relative coordinates (i.e. the sagittal component is the z-component), while the com position, and the
		initial swing foot position is expressed in world coordinates. The resulting desired foot position is also expressed in world coordinates.
	*/
    Vector3d transformSwingFootTarget(Vector3d step, const Point3d& com, const Quaternion& charFrameToWorld, double height);

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
	IKSwingLegTarget computeIKQandW(const RobotInfo& rinfo, const Vector3d& parentAxis, const Vector3d& parentNormal, const Vector3d& childNormal, const Vector3d& childEndEffector, const Point3d& wP, bool computeAngVelocities, const Point3d& futureWP, double dt);
};

