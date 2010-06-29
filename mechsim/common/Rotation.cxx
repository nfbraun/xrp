#include "Rotation.h"
#include <GL/gl.h>

const Rotation Rotation::Unit = Rotation(1., 0., 0., 0.);

Rotation::Rotation(double angle, Vector3 axis)
{
    Vector3 naxis = axis.norm() * sin(angle/2.);

    fQ[0] = cos(angle/2.);
    fQ[1] = naxis.x();
    fQ[2] = naxis.y();
    fQ[3] = naxis.z();
}

Rotation operator*(const Rotation& r1, const Rotation& r2)
{
    double a1 = r1.a(), b1 = r1.b(), c1 = r1.c(), d1 = r1.d();
    double a2 = r2.a(), b2 = r2.b(), c2 = r2.c(), d2 = r2.d();
    
    double a = a1*a2 - b1*b2 - c1*c2 - d1*d2;
    double b = a1*b2 + b1*a2 + c1*d2 - d1*c2;
    double c = a1*c2 - b1*d2 + c1*a2 + d1*b2;
    double d = a1*d2 + b1*c2 - c1*b2 + d1*a2;
    double norm = sqrt(a*a + b*b + c*c + d*d);
    
    return Rotation(a/norm, b/norm, c/norm, d/norm);
}

Vector3 operator*(const Rotation& r, const Vector3& v)
{
    double a = r.a(), b = r.b(), c = r.c(), d = r.d();
    
    double x = 2.*( (0.5-c*c-d*d)*v.x() + (    b*c-a*d)*v.y() + (    a*c+b*d)*v.z() );
    double y = 2.*( (    a*d+b*c)*v.x() + (0.5-b*b-d*d)*v.y() + (    c*d-a*b)*v.z() );
    double z = 2.*( (    b*d-a*c)*v.x() + (    a*b+c*d)*v.y() + (0.5-b*b-c*c)*v.z() );
    
    return Vector3(x,y,z);
}

Matrix33 Rotation::mat() const
{
    Matrix33 m;
    
    m(0,0) = 1. - 2.*c()*c() - 2.*d()*d();
    m(0,1) = 2.*b()*c() - 2.*a()*d();
    m(0,2) = 2.*b()*d() + 2.*a()*c();
    
    m(1,0) = 2.*b()*c() + 2.*a()*d();
    m(1,1) = 1. - 2.*b()*b() - 2.*d()*d();
    m(1,2) = 2.*c()*d() - 2.*a()*b();
    
    m(2,0) = 2.*b()*d() - 2.*a()*c();
    m(2,1) = 2.*c()*d() + 2.*a()*b();
    m(2,2) = 1. - 2.*b()*b() - 2.*c()*c();
    
    return m;
}

