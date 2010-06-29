#ifndef __VECTOR_H__
#define __VECTOR_H__
#include <limits>
#include <cmath>
#include <iostream>
#include <GL/gl.h>
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
    
    static const unsigned int SIZE = 3;
    static unsigned int size() { return SIZE; }
    
    _Scalar_T  operator()(unsigned int i) const { return fC[i]; }
    _Scalar_T& operator()(unsigned int i)       { return fC[i]; }
    
    _Scalar_T  at(unsigned int i) const {
        if(i >= size())
            std::numeric_limits<_Scalar_T>::quiet_NaN();
        return (*this)(i);
    }
    
    _Scalar_T& at(unsigned int i) {
        if(i >= size())
            std::numeric_limits<_Scalar_T>::quiet_NaN();
        return (*this)(i);
    }
    
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
    
    _Vector3<_Scalar_T> operator-() const
    {
        return _Vector3<_Scalar_T>(-x(), -y(), -z());
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
    
    _Vector3<_Scalar_T> norm() const
    {
        return ((*this) / mag());
    }
    
    static const _Vector3<_Scalar_T> Null;
    static const _Vector3<_Scalar_T> eX;
    static const _Vector3<_Scalar_T> eY;
    static const _Vector3<_Scalar_T> eZ;
    
  private:    
    _Scalar_T fC[SIZE];
};

template<typename _Scalar_T>
const _Vector3<_Scalar_T> _Vector3<_Scalar_T>::Null = _Vector3<_Scalar_T>(0., 0., 0.);

template<typename _Scalar_T>
const _Vector3<_Scalar_T> _Vector3<_Scalar_T>::eX = _Vector3<_Scalar_T>(1., 0., 0.);

template<typename _Scalar_T>
const _Vector3<_Scalar_T> _Vector3<_Scalar_T>::eY = _Vector3<_Scalar_T>(0., 1., 0.);

template<typename _Scalar_T>
const _Vector3<_Scalar_T> _Vector3<_Scalar_T>::eZ = _Vector3<_Scalar_T>(0., 0., 1.);

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
} // end namespace Vector

namespace GL {
    inline void Translate(const Vector3& v)
        { glTranslated(v.x(), v.y(), v.z()); }
    inline void Translate(const Vector3f& v)
        { glTranslatef(v.x(), v.y(), v.z()); }
} // end namespace GL

namespace ODE {
    inline void BodySetLinearVel(dBodyID id, const Vector3& v)
        { dBodySetLinearVel(id, v.x(), v.y(), v.z()); }
    inline void BodySetAngularVel(dBodyID id, const Vector3& o)
        { dBodySetAngularVel(id, o.x(), o.y(), o.z()); }
} // end namespace ODE

const double DEG2RAD = M_PI / 180;
const double RAD2DEG = 180. / M_PI;

#endif
