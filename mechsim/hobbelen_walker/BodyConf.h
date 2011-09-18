#ifndef HOB_BODYCONF_H
#define HOB_BODYCONF_H

#include <Eigen/Core>
#include <Eigen/Geometry>

class BodyConf
{
  public:
    BodyConf(double m, double I)
        : fm(m), fI(I) { }
    
    double m() const { return fm; }
    double I() const { return fI; }
    
    virtual Eigen::Vector3d CoG() const = 0;
    virtual Eigen::Quaterniond Rot() const = 0;
  
  protected:
    double fm, fI;
};

#endif
