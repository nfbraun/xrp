#ifndef MSIM_SOLIDBODY_H
#define MSIM_SOLIDBODY_H

#include <Eigen/Dense>

class SolidBody
{
  public:
    SolidBody(double m, const Eigen::Vector3d& CoG, const Eigen::Matrix3d& I)
        : fm(m), fCoG(CoG), fI(I) {}
    static SolidBody Sphere(double density, double r, Eigen::Vector3d cog);
    static SolidBody SphereTotal(double m, double r, Eigen::Vector3d cog);
    
    double            m() const  { return fm; }
    Eigen::Vector3d cog() const  { return fCoG; }
    Eigen::Matrix3d   I() const  { return fI; }

  private:
    double fm;
    Eigen::Vector3d fCoG;
    Eigen::Matrix3d fI;
};

SolidBody combine(const SolidBody& a, const SolidBody& b);

#endif
