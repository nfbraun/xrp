#include "CWConfig.h"

// Acceleration due to gravity
const Eigen::Vector3d CWConfig::G(0., 0., -9.81);
const Eigen::Vector3d CWConfig::UP = -CWConfig::G.normalized();

// P and D gains for calculating the virtual root force in sagittal and coronal
// direction (note: scaled by total mass)
const double CWConfig::VRF_SAG_D_GAIN = 30.0;
const double CWConfig::VRF_COR_P_GAIN = 20.0;
const double CWConfig::VRF_COR_D_GAIN = 9.0;

// P and D gain for virtual root torque
const double CWConfig::VRT_P_GAIN = 1000.0;
const double CWConfig::VRT_D_GAIN = 200.0;

// P and D gain for keeping the stance knee straight
const double CWConfig::ST_KY_P_GAIN = 300.0;
const double CWConfig::ST_KY_D_GAIN = 35.0;

// P and D gains for swing leg control
const double CWConfig::SW_H_P_GAIN = 300.0;
const double CWConfig::SW_H_D_GAIN = 35.0;
const double CWConfig::SW_KY_P_GAIN = 300.0;
const double CWConfig::SW_KY_D_GAIN = 35.0;
const double CWConfig::SW_A_P_GAIN = 50.0;
const double CWConfig::SW_A_D_GAIN = 15.0;

