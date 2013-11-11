#ifndef CW_REACTION_H
#define CW_REACTION_H

#include <Eigen/Dense>
#include "RobotState.h"
#include "cartwheel/Core/Controller.h"

void calcReaction(Eigen::Vector3d& F, Eigen::Vector3d& T, const FullState& fstate, const JSpState& jstate, const JSpTorques& jt, unsigned int stance);

#endif
