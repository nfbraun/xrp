#ifndef HOB_CHAINSEGMENT_H
#define HOB_CHAINSEGMENT_H

#include <Eigen/Core>
#include <Eigen/Geometry>
#include "BodyConf.h"

class ChainSegment: public BodyConf {
  public:
    ChainSegment(double m, double I, double l, double c, double w)
      : BodyConf(m, I), fl(l), fc(c), fw(w), fPos(Eigen::Vector3d::Zero()), fTheta(0.) { }
    
    double l()     const { return fl; }
    double c()     const { return fc; }
    double w()     const { return fw; }
    double theta() const { return fTheta; }
    
    virtual Eigen::Vector3d CoG() const  { return fPos; }
    virtual Eigen::Quaterniond Rot() const
        { return Eigen::Quaterniond(Eigen::AngleAxis<double>(fTheta, Eigen::Vector3d::UnitY())); }
    
    Eigen::Vector3d p1() const;
    Eigen::Vector3d p2() const;
    Eigen::Vector3d cb() const;   // Chain segment ctr in body (unrotated) coordinates
    
    void SetCoG(Eigen::Vector3d pos, double theta);
    void SetP1(Eigen::Vector3d p1, double theta);
    void SetP2(Eigen::Vector3d p2, double theta);
    void SetP1P2(Eigen::Vector3d p1, Eigen::Vector3d p2);
    
  private:
    double fl, fc, fw;
    Eigen::Vector3d fPos;
    double fTheta;
    
    inline Eigen::Vector3d r12hat() const
        { return Eigen::Vector3d(-sin(fTheta), 0., -cos(fTheta)); }
    inline Eigen::Vector3d nor() const
        { return Eigen::Vector3d( cos(fTheta), 0., -sin(fTheta)); }
};

#endif
