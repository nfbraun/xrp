#ifndef CW_AFFINETRANSFORM_H
#define CW_AFFINETRANSFORM_H

#include <Eigen/Dense>

class AffineTransform {
  public:
    AffineTransform(const Eigen::Quaterniond& q, const Eigen::Vector3d& t)
        : fQ(q), fT(t) {}
    
    AffineTransform(const Eigen::AngleAxisd& aa, const Eigen::Vector3d& t)
        : fQ(aa), fT(t) {}
    
    static AffineTransform RotX(double phi)
    {
        return AffineTransform(Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX()),
                               Eigen::Vector3d::Zero());
    }
    
    static AffineTransform RotY(double phi)
    {
        return AffineTransform(Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitY()),
                               Eigen::Vector3d::Zero());
    }
    
    static AffineTransform RotZ(double phi)
    {
        return AffineTransform(Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitZ()),
                               Eigen::Vector3d::Zero());
    }
    
    static AffineTransform Translation(double x, double y, double z)
        { return Translation(Eigen::Vector3d(x, y, z)); }
    
    static AffineTransform Translation(const Eigen::Vector3d t)
        { return AffineTransform(Eigen::Quaterniond(1., 0., 0., 0.), t); }
    
    static AffineTransform Unit()
        { return AffineTransform(Eigen::Quaterniond(1., 0., 0., 0.), Eigen::Vector3d::Zero()); }
    
    Eigen::Matrix4d toMatrix() const {
        Eigen::Matrix4d M;
        M << rot().toRotationMatrix(), trans(),
             0., 0., 0.,                    1.;
        return M;
    }
    
    Eigen::Quaterniond rot() const { return fQ; }
    Eigen::Vector3d trans() const { return fT; }
    
    AffineTransform inverse() const {
        return AffineTransform(rot().conjugate(), -rot().conjugate()._transformVector(trans()));
    }
    
    AffineTransform operator*(const AffineTransform& rhs) const {
        return AffineTransform(rot() * rhs.rot(),
                               rot()._transformVector(rhs.trans()) + trans());
    }
    
    Eigen::Vector3d onPoint(const Eigen::Vector3d& p) const
    {
        return rot()._transformVector(p) + trans();
    }
    
    Eigen::Quaterniond onRotation(const Eigen::Quaterniond& q) const
    {
        return rot() * q;
    }
    
    Eigen::Vector3d onVector(const Eigen::Vector3d& v) const
    {
        return rot()._transformVector(v);
    }
    
  private:
    Eigen::Quaterniond fQ;
    Eigen::Vector3d fT;
    
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

#endif
