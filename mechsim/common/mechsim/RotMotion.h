#ifndef MSIM_ROTMOTION_H
#define MSIM_ROTMOTION_H

#include <Eigen/Dense>

// Velocity field: v(r) = v0 + omega x r
class RotMotion
{
  public:
    RotMotion(Eigen::Vector3d v0, Eigen::Vector3d omega)
        : fv0(v0), fOmega(omega) { }
    static RotMotion Shift(Eigen::Vector3d v)
        { return RotMotion(v, Eigen::Vector3d::Zero()); }
    static RotMotion Rotation(Eigen::Vector3d ctr, Eigen::Vector3d omega)
        { return RotMotion(-omega.cross(ctr), omega); }
    
    Eigen::Vector3d omega() const      { return fOmega; }
    Eigen::Vector3d v0() const         { return fv0; }
    Eigen::Vector3d v(Eigen::Vector3d r) const { return v0() + omega().cross(r); }
    
  private:
    Eigen::Vector3d fv0, fOmega;
};

RotMotion combine(const RotMotion& r1, const RotMotion& r2)
{
    return RotMotion(r1.v0() + r2.v0(), r1.omega() + r2.omega());
}

#endif
