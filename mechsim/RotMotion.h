#ifndef __ROTMOTION_H__
#define __ROTMOTION_H__
#include "Vector.h"

// Velocity field: v(r) = v0 + omega x r
class RotMotion
{
  public:
    RotMotion(Vector3 v0, Vector3 omega)
        : fv0(v0), fOmega(omega) { }
    static RotMotion Shift(Vector3 v)
        { return RotMotion(v, Vector3::Null); }
    static RotMotion Rotation(Vector3 ctr, Vector3 omega)
        { return RotMotion(-Vector::cross(omega, ctr), omega); }
    
    Vector3 omega() const      { return fOmega; }
    Vector3 v0() const         { return fv0; }
    Vector3 v(Vector3 r) const { return v0() + Vector::cross(omega(), r); }
    
  private:
    Vector3 fv0, fOmega;
};

RotMotion combine(const RotMotion& r1, const RotMotion& r2)
{
    return RotMotion(r1.v0() + r2.v0(), r1.omega() + r2.omega());
}

#endif
