#ifndef CW_DYNINFO_H
#define CW_DYNINFO_H

#include <Eigen/Dense>
#include <RobotState.h>
#include <cartwheel/Controller.h>

/* Position of robot center of mass */
Eigen::Vector3d comPos(const FullState& fstate);

/* Velocity of robot center of mass */
Eigen::Vector3d comVel(const FullState& fstate);

/* Kinetic energy of robot */
double Ekin(const FullState& fstate);

/* Potential energy of robot */
double Epot(const FullState& fstate);

/* Potential energy of robot in null position */
double Epot0();

/* Power supplied to robot by given joint space torques (generalized forces) */
double Psys(const JSpTorques& torque, const JSpState& jstate);

#endif
