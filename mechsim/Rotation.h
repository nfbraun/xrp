#ifndef __ROTATION_H__
#define __ROTATION_H__

#include "Vector.h"
#include "Matrix.h"

class Rotation;
Rotation operator*(const Rotation& r1, const Rotation& r2);
Vector3 operator*(const Rotation& r, const Vector3& v);

// Rotation, represented by a unit quarternion
class Rotation
{
  public:
    Rotation() {}
    Rotation(double angle, Vector3 axis);
    Rotation(double a, double b, double c, double d)
        { fQ[0] = a; fQ[1] = b; fQ[2] = c; fQ[3] = d; }
    static Rotation FromQuatArray(const double* ptr)
        { return Rotation(ptr[0], ptr[1], ptr[2], ptr[3]); }
    
    inline double a() const  { return fQ[0]; }
    inline double b() const  { return fQ[1]; }
    inline double c() const  { return fQ[2]; }
    inline double d() const  { return fQ[3]; }
    
    inline const double* quatarray() const { return fQ; }
    
    Rotation conj() const
      { return Rotation(a(), -b(), -c(), -d()); }
    
    Rotation& operator*=(const Rotation& r)
      { *this = *this * r;  return *this; }
    
    Matrix33 mat() const;
    
    static const Rotation Unit;

  private:
    double fQ[4];
};

namespace GL {
    void Rotate(const Rotation& r);
} // end namespace GL

#endif
