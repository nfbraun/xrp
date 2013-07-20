#pragma once

#include "TorqueController.h"

IKSwingLegTarget getSwingLegTarget(const Eigen::Vector3d& d, const Eigen::Vector3d& vd);
Eigen::Vector3d swingLegFK(const IKSwingLegTarget& target);
Eigen::Matrix<double, 6, 6> swingLegFK_S(double hz, double hy, double hx, double ky, double ay, double ax);
