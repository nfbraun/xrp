#ifndef MSIM_MATRIX_H
#define MSIM_MATRIX_H

#include "Vector.h"
#include <limits>

template<typename _Scalar_T>
class _Matrix33
{
  public:
    _Matrix33() {}
    _Matrix33(const float* a)
       { for(int i=0; i<size(); ++i) fA[i] = a[i]; }
    _Matrix33(const double* a)
       { for(int i=0; i<size(); ++i) fA[i] = a[i]; }
    _Matrix33(_Scalar_T a11, _Scalar_T a12, _Scalar_T a13,
              _Scalar_T a21, _Scalar_T a22, _Scalar_T a23,
              _Scalar_T a31, _Scalar_T a32, _Scalar_T a33) {
            fA[0] = a11; fA[1] = a12; fA[2] = a13;
            fA[3] = a21; fA[4] = a22; fA[5] = a23;
            fA[6] = a31; fA[7] = a32; fA[8] = a33;
        }
    
    static _Matrix33<_Scalar_T> Diag(_Scalar_T a11, _Scalar_T a22, _Scalar_T a33)
    { return _Matrix33<_Scalar_T>(a11, 0., 0., 0., a22, 0., 0. , 0., a33); }
    
    static _Matrix33<_Scalar_T> Sym(_Scalar_T a11, _Scalar_T a22, _Scalar_T a33,
                                    _Scalar_T a12, _Scalar_T a13, _Scalar_T a23)
    { return _Matrix33<_Scalar_T>(a11, a12, a13, a12, a22, a23, a13, a23, a33);}
    
    static _Matrix33<_Scalar_T> Asym(_Scalar_T a12, _Scalar_T a13, _Scalar_T a23)
    { return _Matrix33<_Scalar_T>(0., a12, a13, -a12, 0., a23, -a13, -a23, 0.);}
    
    static const unsigned int ROWS = 3;
    static const unsigned int COLS = 3;
    static const unsigned int SIZE = ROWS * COLS;
    
    static unsigned int rows() { return ROWS; }
    static unsigned int cols() { return COLS; }
    static unsigned int size() { return SIZE; }
    
    _Scalar_T  operator()(unsigned int i, unsigned int j) const
        { return fA[i*cols()+j]; }
    _Scalar_T& operator()(unsigned int i, unsigned int j)
        { return fA[i*cols()+j]; }
    
    _Scalar_T  at(unsigned int i, unsigned int j) const {
        if(i>=rows() || j>=cols())
            return std::numeric_limits<_Scalar_T>::quiet_NaN();
        return (*this)(i,j);
    }
    
    _Scalar_T& at(unsigned int i, unsigned int j) {
        if(i>=rows() || j>=cols())
            return std::numeric_limits<_Scalar_T>::quiet_NaN();
        return (*this)(i,j);
    }
    
    _Matrix33<_Scalar_T>& operator+=(const _Matrix33<_Scalar_T>& b)
    {
        for(unsigned int i=0; i<size(); ++i)
            fA[i] += b.fA[i];
        
        return *this;
    }
    
    _Matrix33<_Scalar_T> operator-() const
    {
        _Matrix33<_Scalar_T> c;
        for(unsigned int i=0; i<c.size(); ++i)
            c.fA[i] = -fA[i];
        return c;
    }
    
    _Matrix33<_Scalar_T>& operator-=(const _Matrix33<_Scalar_T>& b)
    {
        for(unsigned int i=0; i<size(); ++i)
            fA[i] -= b.fA[i];
        
        return *this;
    }

    _Matrix33<_Scalar_T>& operator*=(_Scalar_T k)
    {
        for(unsigned int i=0; i<size(); ++i)
            fA[i] *= k;
        
        return *this;
    }
    
    _Scalar_T trace()
    {
        _Scalar_T t=0.;
        for(int i=0; i<cols(); ++i)
            t += (*this)(i,i);
        return t;
    }
    
    _Scalar_T det()
    {
        return (   (*this)(0,0) * (*this)(1,1) * (*this)(2,2)
                 + (*this)(0,1) * (*this)(1,2) * (*this)(2,0)
                 + (*this)(0,2) * (*this)(1,0) * (*this)(2,1)
                 - (*this)(0,0) * (*this)(1,2) * (*this)(2,1)
                 - (*this)(0,1) * (*this)(1,0) * (*this)(2,2)
                 - (*this)(0,2) * (*this)(1,1) * (*this)(2,0) );
    }
    
    _Scalar_T quadratic(const _Vector3<_Scalar_T>& v)
    // Calculate  v^T A v
    {
        double x = 0.;
        for(int i=0; i<rows(); ++i) {
            for(int j=0; j<cols(); ++j) {
                x += (*this)(i,j) * v(i) * v(j);
            }
        }
        return x;
    }
    
    static const _Matrix33<_Scalar_T> Unit;
    
  private:
    _Scalar_T fA[SIZE];
};

template<typename _Scalar_T>
_Matrix33<_Scalar_T> operator+(const _Matrix33<_Scalar_T>& a, const _Matrix33<_Scalar_T>& b)
{
    _Matrix33<_Scalar_T> c(a);
    c += b;
    return c;
}

template<typename _Scalar_T>
_Matrix33<_Scalar_T> operator-(const _Matrix33<_Scalar_T>& a, const _Matrix33<_Scalar_T>& b)
{
    _Matrix33<_Scalar_T> c(a);
    c -= b;
    return c;
}

template<typename _Scalar_T>
_Matrix33<_Scalar_T> operator*(const _Matrix33<_Scalar_T>& a, _Scalar_T k)
{
    _Matrix33<_Scalar_T> b(a);
    b *= k;
    return b;
}

template<typename _Scalar_T>
_Matrix33<_Scalar_T> operator*(_Scalar_T k, const _Matrix33<_Scalar_T>& a)
{
   return a * k;
}

template<typename _Scalar_T>
_Matrix33<_Scalar_T> operator*(const _Matrix33<_Scalar_T>& a, const _Matrix33<_Scalar_T>& b)
{
    _Matrix33<_Scalar_T> c;
    for(unsigned int i=0; i<c.rows(); ++i) {
        for(unsigned int j=0; j<c.cols(); ++j) {
            _Scalar_T s=0.;
            for(unsigned int k=0; k<a.cols(); ++k) {
                k += a(i,k) * b(k,j);
            }
            c(i,j) = s;
        }
    }
    return c;
}

template<typename _Scalar_T>
_Vector3<_Scalar_T> operator*(const _Matrix33<_Scalar_T>& a, const _Vector3<_Scalar_T>& v)
{
    _Vector3<_Scalar_T> w;
    for(unsigned int i=0; i<a.rows(); ++i) {
        _Scalar_T s = 0.;
        for(unsigned int j=0; j<a.cols(); ++j) {
            s += a(i,j) * v(j);
        }
        w(i) = s;
    }
    return w;
}

template<typename _Scalar_T>
std::ostream& operator<< (std::ostream& out, const _Matrix33<_Scalar_T>& a)
{
    out << '(';
    for(unsigned int i=0; i<a.rows(); ++i) {
        unsigned int j;
        out << '(';
        for(j=0; j<a.cols()-1; ++j)
            out << a(i,j) << " ";
        out << a(i,j) << ')';
    }
    out << ')';
    return out;
}

template<typename _Scalar_T>
const _Matrix33<_Scalar_T> _Matrix33<_Scalar_T>::Unit =
    _Matrix33<_Scalar_T>::Diag(1., 1., 1.);

typedef _Matrix33<double> Matrix33;
typedef _Matrix33<float> Matrix33f;

#endif
