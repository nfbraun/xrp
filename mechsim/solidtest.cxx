#include "Vector.h"
#include "RotMotion.h"
#include "SolidBody.h"
#include <iostream>

int main()
{
    Vector3 r0(1., .5, .2);
    Vector3 omega(3., 2., -3.);
    Vector3 v0(1., 0., -.5);
    RotMotion mot = combine(RotMotion::Rotation(r0, omega), RotMotion::Shift(v0));
    
    Vector3 r1(3., 1., 3.), r2(1., 2., -.5);
    
    double m1 = 2., m2 = 3.;
    Matrix33 I1 = Matrix33::Sym(1., .5, 1., .2, 1., .8);
    Matrix33 I2 = Matrix33::Sym(.5, .4, 1., .5, .8, 1.);
    
    Vector3 v1 = mot.v(r1);
    Vector3 v2 = mot.v(r2);
    
    double T = .5*m1*v1.mag2() + .5*m2*v2.mag2() 
            + .5*I1.quadratic(mot.omega()) + .5*I2.quadratic(mot.omega());
    
    SolidBody s1(m1, r1, I1);
    SolidBody s2(m2, r2, I2);
    SolidBody s = combine(s1, s2);
    
    Vector3 vc = mot.v(s.cog());
    
    double Tp = .5*s.m()*vc.mag2() + .5*s.I().quadratic(mot.omega());
    
    std::cout << T << std::endl;
    std::cout << Tp << std::endl;
    
    return 0;
}
