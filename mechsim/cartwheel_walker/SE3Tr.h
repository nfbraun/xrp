#ifndef CW_SE3TR_H
#define CW_SE3TR_H

#include <Eigen/Dense>

/* Transformation in the special Euclidean group SE(3), i.e. a combination of
   a (proper) rotation and a translation. */
class SE3Tr {
  public:
    SE3Tr() {}
    
    SE3Tr(const Eigen::Quaterniond& q, const Eigen::Vector3d& t)
        : fQ(q), fT(t) {}
    
    SE3Tr(const Eigen::AngleAxisd& aa, const Eigen::Vector3d& t)
        : fQ(aa), fT(t) {}
    
    static SE3Tr Identity()
        { return SE3Tr(Eigen::Quaterniond(1., 0., 0., 0.), Eigen::Vector3d::Zero()); }
    
    static SE3Tr Rot(const Eigen::Quaterniond& q)
        { return SE3Tr(q, Eigen::Vector3d::Zero()); }
    
    static SE3Tr Rot(const Eigen::AngleAxisd& aa)
        { return SE3Tr(aa, Eigen::Vector3d::Zero()); }
    
    static SE3Tr RotX(double phi)
        { return Rot(Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitX())); }
    
    static SE3Tr RotY(double phi)
        { return Rot(Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitY())); }
    
    static SE3Tr RotZ(double phi)
        { return Rot(Eigen::AngleAxisd(phi, Eigen::Vector3d::UnitZ())); }
    
    static SE3Tr Trans(const Eigen::Vector3d& t)
        { return SE3Tr(Eigen::Quaterniond(1., 0., 0., 0.), t); }
    
    static SE3Tr Trans(double x, double y, double z)
        { return Trans(Eigen::Vector3d(x, y, z)); }
    
    Eigen::Matrix4d toMatrix() const {
        Eigen::Matrix4d M;
        M << rot().toRotationMatrix(), trans(),
             0., 0., 0.,                    1.;
        return M;
    }
    
    Eigen::Quaterniond rot() const { return fQ; }
    Eigen::Vector3d trans() const { return fT; }
    
    SE3Tr inverse() const {
        return SE3Tr(rot().conjugate(), -rot().conjugate()._transformVector(trans()));
    }
    
    SE3Tr operator*(const SE3Tr& rhs) const {
        return SE3Tr(rot() * rhs.rot(),
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
