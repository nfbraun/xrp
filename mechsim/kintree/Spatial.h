#ifndef MSIM_SPATIAL_H
#define MSIM_SPATIAL_H

#include <Eigen/Dense>
#include <iostream>
#include "SE3Tr.h"

class SpForce;

template <typename Sp_T>
class SpVec {
  protected:
    SpVec() {}
    
    SpVec(const Eigen::Vector3d& ang, const Eigen::Vector3d& lin)
        : fAng(ang), fLin(lin) {}
    
    SpVec(double ax, double ay, double az, double lx, double ly, double lz)
        : fAng(ax, ay, az), fLin(lx, ly, lz) {}
  
  public:
    static Sp_T Zero() { return Sp_T(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()); }
    
    const Eigen::Vector3d& ang() const { return fAng; }
    const Eigen::Vector3d& lin() const { return fLin; }
    
    Sp_T operator-() const
        { return Sp_T(-ang(), -lin()); }
    
    Sp_T operator+(const Sp_T& rhs) const
        { return Sp_T(ang() + rhs.ang(), lin() + rhs.lin()); }
    
    Sp_T operator-(const Sp_T& rhs) const
        { return Sp_T(ang() - rhs.ang(), lin() - rhs.lin()); }
    
    Sp_T operator*(double rhs) const
        { return Sp_T(ang() * rhs, lin() * rhs); }
    
    friend Sp_T operator*(double lhs, const Sp_T& rhs)
        { return Sp_T(lhs*rhs.ang(), lhs*rhs.lin()); }
    
    Sp_T operator/(double rhs) const
        { return Sp_T(ang() / rhs, lin() / rhs); }
    
    void operator+=(const Sp_T& rhs)
        { fAng += rhs.ang(); fLin += rhs.lin(); }
    
    void operator-=(const Sp_T& rhs)
        { fAng -= rhs.ang(); fLin -= rhs.lin(); }
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
    Eigen::Vector3d fAng;
    Eigen::Vector3d fLin;
};

template <typename Sp_T>
std::ostream& operator<<(std::ostream& s, const SpVec<Sp_T>& vec)
{
    s << vec.ang().transpose() << " " << vec.lin().transpose();
    return s;
}

/* Spatial motion (\in M^6) */
class SpMot: public SpVec<SpMot> {
  public:
    SpMot(): SpVec<SpMot>() {}
    
    SpMot(const Eigen::Vector3d& ang, const Eigen::Vector3d& lin)
        : SpVec<SpMot>(ang, lin) {}
    
    SpMot(double ax, double ay, double az, double lx, double ly, double lz)
        : SpVec<SpMot>(ax, ay, az, lx, ly, lz) {}
    
    double dot(const SpForce& f) const;
    SpMot cross(const SpMot& rhs) const;
    SpForce cross_star(const SpForce& rhs) const;
    
    Eigen::Vector3d at(const Eigen::Vector3d& r)
        { return ang().cross(r) + lin(); }
    
    SpMot tr(const SE3Tr& t) const {
        const Eigen::Matrix3d Rot = t.rot().toRotationMatrix();
        return SpMot(Rot*ang(), t.trans().cross(Rot*ang()) + Rot*lin());
    }
};

/* Spatial force (\in F^6) */
class SpForce: public SpVec<SpForce> {
  public:
    SpForce(): SpVec<SpForce>() {}
    
    SpForce(const Eigen::Vector3d& ang, const Eigen::Vector3d& lin)
        : SpVec<SpForce>(ang, lin) {}
    
    SpForce(double ax, double ay, double az, double lx, double ly, double lz)
        : SpVec<SpForce>(ax, ay, az, lx, ly, lz) {}
    
    double dot(const SpMot& m) const { return m.dot(*this); }
    
    SpForce tr(const SE3Tr& t) const {
        const Eigen::Matrix3d Rot = t.rot().toRotationMatrix();
        return SpForce(Rot*ang() + t.trans().cross(Rot*lin()), Rot*lin());
    }
};

/* Spatial inertia (mapping from M^6 to F^6) */
class SpInertia {
  public:
    SpInertia() {}
    
    SpInertia(double m, const Eigen::Vector3d& c, const Eigen::Matrix3d& I)
        : fm(m), fc(c), fI(I) {}
    
    static SpInertia Zero()
        { return SpInertia(0., Eigen::Vector3d::Zero(), Eigen::Matrix3d::Zero()); }
    
    double m() const { return fm; }
    const Eigen::Vector3d& c() const { return fc; }
    const Eigen::Matrix3d& I() const { return fI; }
    
    SpInertia operator+(const SpInertia& rhs) const {
        const double m_tot = m() + rhs.m();
        const Eigen::Vector3d c_tot = (m()*c() + rhs.m()*rhs.c())/m_tot;
        const double mu = (m()*rhs.m())/(m()+rhs.m());
        const Eigen::Vector3d c_diff = c() - rhs.c();
        const Eigen::Matrix3d I_tot = I() + rhs.I() 
            + mu*(c_diff.dot(c_diff)*Eigen::Matrix3d::Identity() - c_diff*c_diff.transpose());
        
        return SpInertia(m_tot, c_tot, I_tot);
    }
    
    SpForce operator*(const SpMot& rhs) const
        { return SpForce(I()*rhs.ang() - m()*c().cross(c().cross(rhs.ang())) + m()*c().cross(rhs.lin()),
                         m()*rhs.lin() - m()*c().cross(rhs.ang())); }
    
    SpInertia tr(const SE3Tr& t) const {
        const Eigen::Matrix3d Rot = t.rot().toRotationMatrix();
        return SpInertia(m(), Rot*c() + t.trans(), Rot*I()*Rot.transpose());
    }
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  private:
    double fm;
    Eigen::Vector3d fc;
    Eigen::Matrix3d fI;
};

#endif
