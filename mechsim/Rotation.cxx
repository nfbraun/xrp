#include "Rotation.h"
#include <GL/gl.h>

const Rotation Rotation::Unit = Rotation(0., 1., 0., 0.);

Rotation::Rotation(double angle, Vector3 axis)
{
    Vector3 naxis = axis.norm() * sin(angle/2.);

    fA = cos(angle/2.);
    fB = naxis.x();
    fC = naxis.y();
    fD = naxis.z();
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

void RotateGL(const Rotation& r)
{
    double mat[4][4];
    double a = r.a(), b = r.b(), c = r.c(), d = r.d();
    
    // see http://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    mat[0][0] = a*a + b*b - c*c - d*d;
    mat[0][1] = 2.*b*c - 2.*a*d;
    mat[0][2] = 2.*b*d + 2.*a*c;
    mat[0][3] = 0.;
    
    mat[1][0] = 2.*b*c + 2.*a*d;
    mat[1][1] = a*a - b*b + c*c - d*d;
    mat[1][2] = 2.*c*d - 2.*a*b;
    mat[1][3] = 0.;
    
    mat[2][0] = 2.*b*d - 2.*a*c;
    mat[2][1] = 2.*c*d + 2.*a*b;
    mat[2][2] = a*a - b*b - c*c + d*d;
    mat[2][3] = 0.;
    
    mat[3][0] = 0.;
    mat[3][1] = 0.;
    mat[3][2] = 0.;
    mat[3][3] = 1.;
    
    glMultMatrixd(&mat[0][0]);
}
