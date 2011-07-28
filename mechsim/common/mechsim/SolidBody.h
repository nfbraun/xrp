#ifndef MSIM_SOLIDBODY_H
#define MSIM_SOLIDBODY_H

#include "Vector.h"
#include "Matrix.h"

class SolidBody
{
  public:
    SolidBody(double m, const Vector3& CoG, const Matrix33& I)
        : fm(m), fCoG(CoG), fI(I) {}
    static SolidBody Sphere(double density, double r, Vector3 cog);
    static SolidBody SphereTotal(double m, double r, Vector3 cog);
    
    double    m() const  { return fm; }
    Vector3 cog() const  { return fCoG; }
    Matrix33  I() const  { return fI; }

  private:
    double fm;
    Vector3 fCoG;
    Matrix33 fI;
};

SolidBody combine(const SolidBody& a, const SolidBody& b);

#endif
