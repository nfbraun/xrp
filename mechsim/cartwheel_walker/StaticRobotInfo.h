#ifndef CW_STATICROBOTINFO_H
#define CW_STATICROBOTINFO_H

#include <Eigen/Dense>

enum JointID { J_L_HIP, J_R_HIP, J_L_KNEE, J_R_KNEE,
    J_L_ANKLE, J_R_ANKLE, J_MAX };

enum ArbID { R_ROOT, R_L_THIGH, R_L_SHANK, R_R_THIGH,
    R_R_SHANK, R_L_FOOT, R_R_FOOT, R_MAX };

enum PhiID { LH0, LH1, LH2, LK, LA0, LA1, RH0, RH1, RH2, RK, RA0, RA1 };

namespace CharacterConst {
    const double density = 900;
    
    const double footSizeX = 0.2;
    const double footSizeY = 0.12;
    const double footSizeZ = 0.05;
    const double legDiameter = 0.1;
    const double shankDiameter = 0.1;
    const double thighDiameter = 0.1;
    const double legSizeZ = 1.0;
    const double kneeRelativePosZ = 0.5;
    const double legRelativeAnchorY = 0.6;
    
    const double pelvisSizeX = 0.25;
    const double pelvisSizeY = 0.45;
    const double pelvisSizeZ = 0.3;
    
    const double pelvisDiameter = 0.45;
    const double pelvisRadius = pelvisDiameter/2.0;
    
    const double shankSizeZ = legSizeZ * kneeRelativePosZ;
    const double thighSizeZ = legSizeZ - shankSizeZ;
    
    const double footPosZ = footSizeZ/2.;
    const double anklePosZ = footSizeZ;
    const double shankPosZ = anklePosZ + shankSizeZ/2.;
    const double kneePosZ = anklePosZ + legSizeZ * kneeRelativePosZ;
    const double thighPosZ = kneePosZ + thighSizeZ/2.;
    const double hipPosZ = anklePosZ + legSizeZ;
    const double pelvisPosZ = hipPosZ + pelvisSizeZ/2.;
    
    const double legPosY_L = pelvisSizeY/2.0*legRelativeAnchorY;
    const double legPosY_R = -pelvisSizeY/2.0*legRelativeAnchorY;
} // end namespace CharacterConst

inline double boxMass(double sx, double sy, double sz)
{
    return CharacterConst::density * sx * sy * sz;
}

inline Eigen::Vector3d boxMOI(double sx, double sy, double sz)
{
    return (1./12.) * boxMass(sx, sy, sz) *
        Eigen::Vector3d( sy*sy + sz*sz,
                         sx*sx + sz*sz,
                         sx*sx + sy*sy );
}

inline double cylinderMass(double h, double r)
{
    return CharacterConst::density * M_PI * r * r * h;
}

inline Eigen::Vector3d cylinderMOI(double h, double r)
{
    return cylinderMass(h, r) *
        Eigen::Vector3d( (3.*r*r + h*h) / 12.,
                         r*r / 2.,
                         (3.*r*r + h*h) / 12.);
}

inline double rbMass(unsigned int id)
{
    using namespace CharacterConst;
    
    const double masses[R_MAX] =
    { boxMass(pelvisSizeX, pelvisSizeY, pelvisSizeZ), // R_ROOT
      cylinderMass(thighSizeZ, thighDiameter/2.),     // R_L_THIGH
      cylinderMass(shankSizeZ, shankDiameter/2.),     // R_L_SHANK
      cylinderMass(thighSizeZ, thighDiameter/2.),     // R_R_THIGH
      cylinderMass(shankSizeZ, shankDiameter/2.),     // R_R_SHANK
      boxMass(footSizeX, footSizeY, footSizeZ),       // R_L_FOOT
      boxMass(footSizeX, footSizeY, footSizeZ)        // R_R_FOOT
    };
    
    assert(id < R_MAX);
    return masses[id];
}

inline Eigen::Vector3d rbMOI(unsigned int id)
{
    using namespace CharacterConst;
    
    const Eigen::Vector3d mois[R_MAX] =
    { 3.*boxMOI(pelvisSizeX, pelvisSizeY, pelvisSizeZ), // R_ROOT
         cylinderMOI(thighSizeZ, thighDiameter/2.),     // R_L_THIGH
         cylinderMOI(shankSizeZ, shankDiameter/2.),     // R_L_SHANK
         cylinderMOI(thighSizeZ, thighDiameter/2.),     // R_R_THIGH
         cylinderMOI(shankSizeZ, shankDiameter/2.),     // R_R_SHANK
      3.*boxMOI(footSizeX, footSizeY, footSizeZ),       // R_L_FOOT
      3.*boxMOI(footSizeX, footSizeY, footSizeZ)        // R_R_FOOT
    };
    
    assert(id < R_MAX);
    return mois[id];

}

#endif
