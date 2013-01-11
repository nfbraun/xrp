#include "PoseController.h"
#include <MathLib/Quaternion.h>
#include <algorithm>

/**
	This method is used to compute the PD torque that aligns a child coordinate frame to a parent coordinate frame.
	Given: the current relative orientation of two coordinate frames (child and parent), the relative angular velocity,
	the desired values for the relative orientation and ang. vel, as well as the virtual motor's PD gains. The torque 
	returned is expressed in the coordinate frame of the 'parent'.
*/
Vector3d PoseController::computePDTorque(const Quaternion& qRel, const Quaternion& qRelD, const Vector3d& wRel, const Vector3d& wRelD, const ControlParams* cParams){
	Vector3d torque;
	//the torque will have the form:
	// T = kp*D(qRelD, qRel) + kd * (wRelD - wRel)

	//Note: There can be problems computing the proper torque from the quaternion part, because q and -q 
	//represent the same orientation. To make sure that we get the correct answer, we'll take into account
	//the sign of the scalar part of qErr - both this and the v part will change signs in the same way if either 
	//or both of qRel and qRelD are negative
//	Quaternion qErr = qRel.getComplexConjugate() * qRelD;
	Quaternion qErr = qRel.getComplexConjugate();
	qErr *= qRelD;

	//qErr.v also contains information regarding the axis of rotation and the angle (sin(theta)), but I want to scale it by theta instead
	double sinTheta = qErr.v.length();
	if (sinTheta>1)
		sinTheta = 1;
	if (IS_ZERO(sinTheta)){
		//avoid the divide by close-to-zero. The orientations match, so the proportional component of the torque should be 0
	}else{
		double absAngle = 2 * asin(sinTheta);
		torque = qErr.v;
		torque *= 1/sinTheta * absAngle * (-cParams->kp) * SGN(qErr.s);
//		torque = qErr.v/sinTheta * absAngle * (-cParams->kp) * SGN(qErr.s);
	}

	//qErr represents the rotation from the desired child frame to the actual child frame, which
	//means that the torque is now expressed in child coordinates. We need to express it in parent coordinates!
	torque = qRel.rotate(torque);
	//the angular velocities are stored in parent coordinates, so it is ok to add this term now
	torque += (wRelD - wRel) * (-cParams->kd);

	//now the torque is stored in parent coordinates - we need to scale it and apply torque limits
	scaleAndLimitTorque(&torque, cParams, qRel.getComplexConjugate());

	//and we're done...
	return torque;
}

/**
	This method is used to apply joint limits to the torque passed in as a parameter. It is assumed that
	the torque is already represented in the correct coordinate frame
*/
void PoseController::limitTorque(Vector3d* torque, const ControlParams* cParams){
	if (torque->x < -cParams->scale.x * cParams->maxAbsTorque) torque->x = -cParams->scale.x * cParams->maxAbsTorque;
	if (torque->x > cParams->scale.x * cParams->maxAbsTorque) torque->x = cParams->scale.x * cParams->maxAbsTorque;
	if (torque->y < -cParams->scale.y * cParams->maxAbsTorque) torque->y = -cParams->scale.y * cParams->maxAbsTorque;
	if (torque->y > cParams->scale.y * cParams->maxAbsTorque) torque->y = cParams->scale.y * cParams->maxAbsTorque;
	if (torque->z < -cParams->scale.z * cParams->maxAbsTorque) torque->z = -cParams->scale.z * cParams->maxAbsTorque;
	if (torque->z > cParams->scale.z * cParams->maxAbsTorque) torque->z = cParams->scale.z * cParams->maxAbsTorque;
}

/**
	This method is used to scale and apply joint limits to the torque that is passed in as a parameter. The orientation that transforms 
	the torque from the coordinate frame that it is currently stored in, to the coordinate frame of the 'child' to which the torque is 
	applied to (it wouldn't make sense to scale the torques in any other coordinate frame)  is also passed in as a parameter.
*/
void PoseController::scaleAndLimitTorque(Vector3d* torque, const ControlParams* cParams, const Quaternion& qToChild){
	//now change the torque to child coordinates
	*torque = qToChild.rotate(*torque);

	//and scale it differently along the main axis...
	torque->x *= cParams->scale.x;
	torque->y *= cParams->scale.y;
	torque->z *= cParams->scale.z;

	limitTorque(torque, cParams);

	// and now change it back to the original coordinates
	*torque = qToChild.getComplexConjugate().rotate(*torque);
}

/**
	This method is used to compute the torques that are to be applied at the next step.
*/
Vector3d PoseController::computePDJointTorque(const RobotInfo& rinfo, int jid,
    Quaternion desiredOrientationInFrame, Vector3d desiredRelativeAngularVelocityInFrame,
    bool relToFrame, Quaternion characterFrame)
{
    Vector3d torque;
    
    // assert(controlParams[jid].strength == 1.0);
    
    // assert(relToFrame == controlParams[jid].relToFrame);

    //ReducedCharacterState rs(&desiredPose);

    Quaternion qRelD;
    Vector3d relAngVelD;

    Quaternion qRel;
    Vector3d wRel;

    // assert(controlParams[jid].controlled);

    const RigidBody* parentRB = rinfo.character()->getJoint(jid)->getParent();
    const RigidBody* childRB = rinfo.character()->getJoint(jid)->getChild();
    Quaternion parentQworld = parentRB->getOrientation().getComplexConjugate();			

    Quaternion frameQworld;
    Vector3d frameAngularVelocityInFrame;

    if (relToFrame == false) {
        // std::cout << "relToFrame[" << i << "] = false" << std::endl;
        frameQworld = parentQworld;
        frameAngularVelocityInFrame = parentQworld.rotate( parentRB->getAngularVelocity() );
    } else {
        // std::cout << "relToFrame[" << i << "] = true" << std::endl;
        //Quaternion test = (controlParams[jid].frame * characterFrame.getComplexConjugate());
        //std::cout << test.s << " " << test.v.x << " " << test.v.y << " " << test.v.z << std::endl;
        frameQworld = characterFrame.getComplexConjugate();
        frameAngularVelocityInFrame = frameQworld.rotate( Vector3d(0., 0., 0.) );
    }

    Quaternion currentOrientationInFrame = frameQworld * childRB->getOrientation();
    // Quaternion desiredOrientationInFrame = desiredPose.getJointRelativeOrientation(jid);
    Vector3d currentAngularVelocityInFrame = frameQworld.rotate( childRB->getAngularVelocity() );
    // Vector3d desiredRelativeAngularVelocityInFrame = desiredPose.getJointRelativeAngVelocity(jid);
    Vector3d currentRelativeAngularVelocityInFrame = currentAngularVelocityInFrame - frameAngularVelocityInFrame;

    Quaternion parentQframe = parentQworld * frameQworld.getComplexConjugate();

    torque = computePDTorque(parentQframe * currentOrientationInFrame, 
                             parentQframe * desiredOrientationInFrame, 
                             parentQframe.rotate(currentRelativeAngularVelocityInFrame), 
                             parentQframe.rotate(desiredRelativeAngularVelocityInFrame), 
                             &controlParams[jid]);
    torque = parentRB->getWorldCoordinatesForVector(torque);

/*
    if (controlParams[i].relToCharFrame == false){
        //get the current relative orientation between the child and parent
        character->getRelativeOrientation(i, &qRel);
        //and the relative angular velocity, computed in parent coordinates
        character->getRelativeAngularVelocity(i, &wRel);
        //now compute the torque
        torques[i] = computePDTorque(qRel, rs.getJointRelativeOrientation(i), wRel, rs.getJointRelativeAngVelocity(i), &controlParams[i]);
        //the torque is expressed in parent coordinates, so we need to convert it to world coords now
        torques[i] = character->getJoint(i)->getParent()->getWorldCoordinates(torques[i]);
    }else{
        RigidBody* childRB = character->getJoint(i)->getChild();
        torques[i] = computePDTorque(childRB->getOrientation(), controlParams[i].charFrame * rs.getJointRelativeOrientation(i), childRB->getAngularVelocity(), rs.getJointRelativeAngVelocity(i), &controlParams[i]);
    }
*/

    return torque;
}

