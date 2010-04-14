#ifndef __VECTOR_H__
#define __VECTOR_H__
#include <cmath>
#include <iostream>
#include <ode/ode.h>

template<typename _Scalar_T>
class _Vector3
{
  public:
    _Vector3() { }
    _Vector3(_Scalar_T _x, _Scalar_T _y, _Scalar_T _z)
      : fX(_x), fY(_y), fZ(_z) { }
    _Vector3(const float* p)
      : fX(p[0]), fY(p[1]), fZ(p[2]) { }
    _Vector3(const double* p)
      : fX(p[0]), fY(p[1]), fZ(p[2]) { }
    _Vector3(const long double* p)
      : fX(p[0]), fY(p[1]), fZ(p[2]) { }
    
    static _Vector3<_Scalar_T> Spherical(_Scalar_T r, _Scalar_T theta, _Scalar_T phi)
      { return _Vector3<_Scalar_T>(r * sin(theta) * cos(phi), 
                                   r * sin(theta) * sin(phi),
                                   r * cos(theta)); }
                                   
    _Scalar_T x() const { return fX; }
    _Scalar_T y() const { return fY; }
    _Scalar_T z() const { return fZ; }
      
    _Scalar_T mag2() const { return x()*x() + y()*y() + z()*z(); }
    _Scalar_T mag()  const { return sqrt(mag2()); }
    
    _Scalar_T r()     const { return mag(); }
    _Scalar_T phi()   const { return atan2(fY, fX); }
    _Scalar_T theta() const { return acos(fZ / mag()); }
    
    _Vector3<_Scalar_T>& operator+=(const _Vector3<_Scalar_T>& w)
    {
        fX += w.x();
        fY += w.y();
        fZ += w.z();
    
        return *this;
    }
    
    _Vector3<_Scalar_T>& operator-=(const _Vector3<_Scalar_T>& w)
    {
        fX -= w.x();
        fY -= w.y();
        fZ -= w.z();
        
        return *this;
    }

    _Vector3<_Scalar_T>& operator*=(_Scalar_T k)
    {
        fX *= k;
        fY *= k;
        fZ *= k;
        
        return *this;
    }
    
    _Vector3<_Scalar_T>& operator/=(_Scalar_T k)
    {
        fX /= k;
        fY /= k;
        fZ /= k;
        
        return *this;
    }
    
    bool operator==(const _Vector3<_Scalar_T>& w)
    {
        return ((fX == w.x()) && (fY == w.y()) && (fZ == w.z()));
    }
    
    _Vector3<_Scalar_T> norm()
    {
        return ((*this) / mag());
    }
    
  private:    
    _Scalar_T fX, fY, fZ;
};

template<typename _Scalar_T>
_Vector3<_Scalar_T> operator+(const _Vector3<_Scalar_T>& v, const _Vector3<_Scalar_T>& w)
{
    _Vector3<_Scalar_T> u(v);
    u += w;
    return u;
}

template<typename _Scalar_T>
_Vector3<_Scalar_T> operator-(const _Vector3<_Scalar_T>& v, const _Vector3<_Scalar_T>& w)
{
    _Vector3<_Scalar_T> u(v);
    u -= w;
    return u;
}

template<typename _Scalar_T>
_Vector3<_Scalar_T> operator*(const _Vector3<_Scalar_T>& v, _Scalar_T k)
{
    _Vector3<_Scalar_T> u(v);
    u *= k;
    return u;
}

template<typename _Scalar_T>
_Vector3<_Scalar_T> operator*(_Scalar_T k, const _Vector3<_Scalar_T>& v)
{
    return v*k;
}

template<typename _Scalar_T>
std::ostream& operator<< (std::ostream& out, const _Vector3<_Scalar_T>& v)
{
    out << '(' << v.x << ", " << v.y << ", " << v.z << ')';
    return out;
}

typedef _Vector3<double> Vector3;

namespace Vector {
    template<typename _Scalar_T>
    _Scalar_T dot(const _Vector3<_Scalar_T>& v, const _Vector3<_Scalar_T>& w)
    {
        return (v.x()*w.x() + v.y()*w.y() + v.z()*w.z());
    }

    template<typename _Scalar_T>
    _Vector3<_Scalar_T> cross(const _Vector3<_Scalar_T>& v, const _Vector3<_Scalar_T>& w)
    {
        _Vector3<_Scalar_T> u(v.y()*w.z() - v.z()*w.y(),
                              v.z()*w.x() - v.x()*w.z(),
                              v.x()*w.y() - v.y()*w.x());
        return u;
    }
    
    const Vector3 Null = Vector3(0., 0., 0.);
} // end namespace Vector

const double DEG2RAD = M_PI / 180;
const double RAD2DEG = 180. / M_PI;

#endif
