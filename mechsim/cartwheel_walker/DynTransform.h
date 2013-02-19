#ifndef CW_DYNAMICS_H
#define CW_DYNAMICS_H

#include "RobotState.h"
#include "AffineTransform.h"
#include <Eigen/Dense>

/* Construct the FullState from a given (joint space) JSpState.
   NOTE: The JSpState contains less information than the FullState, because
   the 6d position and velocity of the system as a whole is not fixed.
   This function resolves the ambiguity by placing the pelvis center-of-mass
   at the origin, with zero velocity. */
FullState fullFromJoint(const JSpState& jstate);

/* Resolve the ambiguity in fullFromJoint() by specifying 6d position and
   velocity for a single (arbitrary) rigid body in the robot.
   NOTE: If this function is called more than once on the same FullState,
   the effect should be the same as if only the last call had been made. */
void setBodyState(FullState& fstate, unsigned int id,
                  const Eigen::Vector3d& pos,
                  const Eigen::Quaterniond& rot,
                  const Eigen::Vector3d& vel,
                  const Eigen::Vector3d& avel);

void setBodyState(FullState& fstate, unsigned int id, const BodyQ& q);

/* Construct the (joint space) JSpState from a given FullState.
   NOTE: This function does not check if the FullState is consistent, i.e.
   whether all joint constraints are satisfied. */
JSpState jointFromFull(const FullState& fstate);

AffineTransform hipTransform(double hz, double hy, double hx);
AffineTransform thighTransform();
AffineTransform kneeTransform(double ky);
AffineTransform shankTransform();
AffineTransform ankleTransform(double ay, double ax);

AffineTransform legTransform(double hz, double hy, double hx, double ky, double ay, double ax);

/* Returns a matrix transforming joint space generalized velocities for the
   hip into an angular velocity. */
Eigen::Matrix3d hipAVelTransform(double hz, double hy, double hx);

/* Returns the inverse matrix of hipAVelTransform(). */
Eigen::Matrix3d invHipAVelTransform(double hz, double hy, double hx);

/* Determine y such that q can be written as q = RotY(y).
   NOTE: Does not check if q is indeed of this form! */
void decompYRot(const Eigen::Quaterniond q, double& y);

/* Determine y,x such that q can be written as q = RotY(y) * Rot(x).
   NOTE: Does not check if q is indeed of this form! */
void decompYXRot(const Eigen::Quaterniond q, double& y, double& x);

/* Determine z,y,x such that q can be written as q = RotZ(z) * RotY(y) * Rot(x).
   NOTE: This should always be possible.
   FIXME: Think about behaviour for large (> M_PI/2) angles... */
void decompZYXRot(const Eigen::Quaterniond q, double&z, double& y, double& x);

#endif