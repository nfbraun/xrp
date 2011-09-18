#include "SolidBody.h"
#include <cmath>

SolidBody SolidBody::SphereTotal(double m, double r, Eigen::Vector3d cog)
{
    Eigen::Matrix3d I = 2./5. * m * r*r * Eigen::Matrix3d::Identity();
    return SolidBody(m, cog, I);
}

SolidBody SolidBody::Sphere(double density, double r, Eigen::Vector3d cog)
{
    double m = 4./3. * M_PI * r*r*r * density;
    return SphereTotal(m, r, cog);
}

Eigen::Matrix3d SteinerShift(Eigen::Vector3d v)
{
    Eigen::Matrix3d s = v.squaredNorm() * Eigen::Matrix3d::Identity();
    
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
    Eigen::Vector3d cog = (a.m()*a.cog() + b.m()*b.cog()) / M;
    Eigen::Matrix3d I = a.I() + a.m() * SteinerShift(cog - a.cog())
                  + b.I() + b.m() * SteinerShift(cog - b.cog());
    
    return SolidBody(M, cog, I);
}
