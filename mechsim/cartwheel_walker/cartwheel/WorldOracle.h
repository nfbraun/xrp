#pragma once
#include <Eigen/Dense>

class WorldOracle{
public:
	inline double getWorldHeightAt(const Eigen::Vector3d& worldLoc) { return 0.; }
};
