#ifndef HOB_FOOTSEGMENT_H
#define HOB_FOOTSEGMENT_H

#include "BodyConf.h"
#include <Eigen/Core>
#include <Eigen/Geometry>

class FootSegment: public BodyConf
{
  public:
    FootSegment(double m, double I, double l, double w, double r, double h)
        : BodyConf(m, I), fl(l), fw(w), fr(r), fh(h),
          fPos(Eigen::Vector3d::Zero()), fTheta(0.)
           { }
    
    double l()     const { return fl; }
    double w()     const { return fw; }
    double r()     const { return fr; }
    double h()     const { return fh; }
    double theta() const { return fTheta; }
    
    virtual Eigen::Vector3d CoG() const  { return fPos; }
    virtual Eigen::Quaterniond Rot() const
        { return Eigen::Quaterniond(Eigen::AngleAxis<double>(fTheta, Eigen::Vector3d::UnitY())); }
    
    Eigen::Vector3d p1() const;   // Attach point
    Eigen::Vector3d pf() const;   // Front contact circle center
    Eigen::Vector3d pb() const;   // Back contact circle center
    Eigen::Vector3d pfb() const;  // Front contact circle ctr in body (unrotated) coordinates
    Eigen::Vector3d pbb() const;  // Back contact circle ctr in body (unrotated) coordinates
    
    void SetCoG(Eigen::Vector3d pos, double theta);
    void SetP1(Eigen::Vector3d p1, double theta);
    void SetPF(Eigen::Vector3d pf, double theta);
    void SetPB(Eigen::Vector3d pb, double theta);
    
  private:
    double fl, fw, fr, fh;
    Eigen::Vector3d fPos;
    double fTheta;
    
    inline Eigen::Vector3d rbfhat() const
        { return Eigen::Vector3d(cos(fTheta), 0., -sin(fTheta)); }
    inline Eigen::Vector3d nor() const
        { return Eigen::Vector3d(sin(fTheta), 0.,  cos(fTheta)); }
    
};

#endif
