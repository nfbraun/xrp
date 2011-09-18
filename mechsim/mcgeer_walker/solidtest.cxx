#include <Eigen/Dense>
#include "RotMotion.h"
#include "SolidBody.h"
#include <iostream>

int main()
{
    Eigen::Vector3d r0(1., .5, .2);
    Eigen::Vector3d omega(3., 2., -3.);
    Eigen::Vector3d v0(1., 0., -.5);
    RotMotion mot = combine(RotMotion::Rotation(r0, omega), RotMotion::Shift(v0));
    
    Eigen::Vector3d r1(3., 1., 3.), r2(1., 2., -.5);
    
    double m1 = 2., m2 = 3.;
    Eigen::Matrix3d I1;
    I1 << 1.0, 0.2, 1.0,
          0.2, 0.5, 0.8,
          1.0, 0.8, 1.0;
    Eigen::Matrix3d I2;
    I2 << 0.5, 0.5, 0.8,
          0.5, 0.4, 1.0,
          0.8, 1.0, 1.0;
    
    Eigen::Vector3d v1 = mot.v(r1);
    Eigen::Vector3d v2 = mot.v(r2);
    
    double T = .5*m1*v1.squaredNorm() + .5*m2*v2.squaredNorm()
             + .5*mot.omega().dot(I1*mot.omega())
             + .5*mot.omega().dot(I2*mot.omega());
    
    SolidBody s1(m1, r1, I1);
    SolidBody s2(m2, r2, I2);
    SolidBody s = combine(s1, s2);
    
    Eigen::Vector3d vc = mot.v(s.cog());
    
    double Tp = .5*s.m()*vc.squaredNorm()
              + .5*mot.omega().dot(s.I() * mot.omega());
    
    std::cout << T << std::endl;
    std::cout << Tp << std::endl;
    
    return 0;
}
