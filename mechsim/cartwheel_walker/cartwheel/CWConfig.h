#pragma once
#include <Eigen/Dense>

class CWConfig {
  public:
    static const Eigen::Vector3d G;
    static const Eigen::Vector3d UP;
    
    static const double VRF_SAG_D_GAIN, VRF_COR_P_GAIN, VRF_COR_D_GAIN;
    static const double VRT_P_GAIN, VRT_D_GAIN;
    static const double ST_KY_P_GAIN, ST_KY_D_GAIN;
    static const double SW_H_P_GAIN, SW_H_D_GAIN;
    static const double SW_KY_P_GAIN, SW_KY_D_GAIN;
    static const double SW_A_P_GAIN, SW_A_D_GAIN;
};
