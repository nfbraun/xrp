#ifndef __VECTOR_H__
#define __VECTOR_H__
#include <cmath>

template<typename _Scalar_T>
class _Vector3
{
  public:
    _Vector3(double _x, double _y, double _z)
      : x(_x), y(_y), z(_z) { }
      
    _Scalar_T mag2() const { return x*x + y*y + z*z; }
    _Scalar_T mag() const { return sqrt(mag2()); }
    
    _Vector3<_Scalar_T>& operator+=(const _Vector3<_Scalar_T>& w)
    {
        x += w.x;
        y += w.y;
        z += w.z;
    
        return *this;
    }
    
    _Vector3<_Scalar_T>& operator-=(const _Vector3<_Scalar_T>& w)
    {
        x -= w.x;
        y -= w.y;
        z -= w.z;
        
        return *this;
    }

    _Vector3<_Scalar_T>& operator*=(_Scalar_T k)
    {
        x *= k;
        y *= k;
        z *= k;
        
        return *this;
    }
    
    _Vector3<_Scalar_T>& operator/=(_Scalar_T k)
    {
        x /= k;
        y /= k;
        z /= k;
        
        return *this;
    }
    
    bool operator==(const _Vector3<_Scalar_T>& w)
    {
        return ((x == w.x) && (y == w.y) && (z == w.z));
    }
    
    _Vector3<_Scalar_T> norm()
    {
        return ((*this) / mag());
    }
          
    _Scalar_T x, y, z;
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

namespace Vector {
    template<typename _Scalar_T>
    _Scalar_T dot(const _Vector3<_Scalar_T>& v, const _Vector3<_Scalar_T>& w)
    {
        return (v.x*w.x + v.y*w.y + v.z*w.z);
    }

    template<typename _Scalar_T>
    _Vector3<_Scalar_T> cross(const _Vector3<_Scalar_T>& v, const _Vector3<_Scalar_T>& w)
    {
        _Vector3<_Scalar_T> u(v.y*w.z - v.z*w.y,
                              v.z*w.x - v.x*w.z,
                              v.x*w.y - v.y*w.x);
        return u;
    }
} // end namespace Vector

typedef _Vector3<double> Vector3;

#endif
