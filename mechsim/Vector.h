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
       { fC[0] = _x; fC[1] = _y; fC[2] = _z; }
    _Vector3(const float* c)
       { fC[0] = c[0]; fC[1] = c[1]; fC[2] = c[2]; }
    _Vector3(const double* c)
       { fC[0] = c[0]; fC[1] = c[1]; fC[2] = c[2]; }
    _Vector3(const long double* c)
       { fC[0] = c[0]; fC[1] = c[1]; fC[2] = c[2]; }
    
    operator const _Scalar_T*() const { return fC; }
    
    static _Vector3<_Scalar_T> Spherical(_Scalar_T r, _Scalar_T theta, _Scalar_T phi)
      { return _Vector3<_Scalar_T>(r * sin(theta) * cos(phi), 
                                   r * sin(theta) * sin(phi),
                                   r * cos(theta)); }
                                   
    _Scalar_T x() const { return fC[0]; }
    _Scalar_T y() const { return fC[1]; }
    _Scalar_T z() const { return fC[2]; }
      
    _Scalar_T mag2() const { return x()*x() + y()*y() + z()*z(); }
    _Scalar_T mag()  const { return sqrt(mag2()); }
    
    _Scalar_T r()     const { return mag(); }
    _Scalar_T phi()   const { return atan2(y(), x()); }
    _Scalar_T theta() const { return acos(z() / mag()); }
    
    _Vector3<_Scalar_T>& operator+=(const _Vector3<_Scalar_T>& w)
    {
        fC[0] += w.x();
        fC[1] += w.y();
        fC[2] += w.z();
    
        return *this;
    }
    
    _Vector3<_Scalar_T>& operator-=(const _Vector3<_Scalar_T>& w)
    {
        fC[0] -= w.x();
        fC[1] -= w.y();
        fC[2] -= w.z();
        
        return *this;
    }

    _Vector3<_Scalar_T>& operator*=(_Scalar_T k)
    {
        fC[0] *= k;
        fC[1] *= k;
        fC[2] *= k;
        
        return *this;
    }
    
    _Vector3<_Scalar_T>& operator/=(_Scalar_T k)
    {
        fC[0] /= k;
        fC[1] /= k;
        fC[2] /= k;
        
        return *this;
    }
    
    bool operator==(const _Vector3<_Scalar_T>& w)
    {
        return ((x() == w.x()) && (y() == w.y()) && (z() == w.z()));
    }
    
    _Vector3<_Scalar_T> norm()
    {
        return ((*this) / mag());
    }
    
  private:    
    _Scalar_T fC[3];
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
_Vector3<_Scalar_T> operator/(const _Vector3<_Scalar_T>& v, _Scalar_T k)
{
    _Vector3<_Scalar_T> u(v);
    u /= k;
    return u;
}

template<typename _Scalar_T>
std::ostream& operator<< (std::ostream& out, const _Vector3<_Scalar_T>& v)
{
    out << '(' << v.x() << ", " << v.y() << ", " << v.z() << ')';
    return out;
}

typedef _Vector3<double> Vector3;
typedef _Vector3<float> Vector3f;

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
