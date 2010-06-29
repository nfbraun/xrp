#include "SolidBody.h"
#include <cmath>

SolidBody SolidBody::SphereTotal(double m, double r, Vector3 cog)
{
    Matrix33 I = 2./5. * m * r*r * Matrix33::Unit;
    return SolidBody(m, cog, I);
}

SolidBody SolidBody::Sphere(double density, double r, Vector3 cog)
{
    double m = 4./3. * M_PI * r*r*r * density;
    return SphereTotal(m, r, cog);
}

Matrix33 SteinerShift(Vector3 v)
{
    Matrix33 s = v.mag2() * Matrix33::Unit;
    
    for(int i=0; i<3; ++i) {
        for(int j=0; j<3; ++j) {
            s(i,j) -= v(i)*v(j);
        }
    }
    
    return s;
}

SolidBody combine(const SolidBody& a, const SolidBody& b)
{
    double M = a.m() + b.m();
    Vector3 cog = (a.m()*a.cog() + b.m()*b.cog()) / M;
    Matrix33 I = a.I() + a.m() * SteinerShift(cog - a.cog())
                  + b.I() + b.m() * SteinerShift(cog - b.cog());
    
    return SolidBody(M, cog, I);
}
