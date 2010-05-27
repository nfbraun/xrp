#ifndef __ROTATION_H__
#define __ROTATION_H__

#include "Vector.h"

class Rotation;
Rotation operator*(const Rotation& r1, const Rotation& r2);
Vector3 operator*(const Rotation& r, const Vector3& v);

// Rotation, represented by a unit quarternion
class Rotation
{
  public:
    Rotation(double angle, Vector3 axis);
    Rotation(double a, double b, double c, double d)
        : fA(a), fB(b), fC(c), fD(d) { }
    
    inline double a() const  { return fA; }
    inline double b() const  { return fB; }
    inline double c() const  { return fC; }
    inline double d() const  { return fD; }
    
    Rotation& operator*=(const Rotation& r)
      { *this = *this * r;  return *this; }
    
    static const Rotation Unit;

  private:
    double fA, fB, fC, fD;
};

void RotateGL(const Rotation& r);

#endif
