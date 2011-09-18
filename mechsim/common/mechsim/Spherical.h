#ifndef MSIM_SPHERICAL_H
#define MSIM_SPHERICAL_H

#include <limits>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>

class Spherical : public Eigen::Vector3d
{
  public:
    Spherical(double r, double theta, double phi)
     : Eigen::Vector3d(r * ::sin(theta) * ::cos(phi),
                       r * ::sin(theta) * ::sin(phi),
                       r * ::cos(theta)) {}
    
    Spherical(const Eigen::Vector3d& v) : Eigen::Vector3d(v) {}
    
    double r()     const { return norm(); }
    double phi()   const { return atan2(y(), x()); }
    double theta() const { return acos(z() / norm()); }
};

const double DEG2RAD = M_PI / 180;
const double RAD2DEG = 180. / M_PI;

#endif
