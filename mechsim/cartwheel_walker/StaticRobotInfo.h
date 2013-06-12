#ifndef CW_STATICROBOTINFO_H
#define CW_STATICROBOTINFO_H

#include <Eigen/Dense>

enum SideID { LEFT=0, RIGHT=1, SIDE_MAX=2 };

enum JointID { J_L_HIP, J_R_HIP, J_L_KNEE, J_R_KNEE,
    J_L_ANKLE, J_R_ANKLE, J_MAX };

enum BodyID { B_L_THIGH, B_L_SHANK, B_L_FOOT, B_R_THIGH, B_R_SHANK, B_R_FOOT, B_PELVIS, B_MAX };
enum RBodyID { B_THIGH, B_SHANK, B_FOOT, RBODY_MAX };

enum DoFID { LHZ, LHY, LHX, LKY, LAY, LAX, RHZ, RHY, RHX, RKY, RAY, RAX, DOF_MAX };
enum RDoFID { HZ, HY, HX, KY, AY, AX, RDOF_MAX };

inline const char* dofName(unsigned int id)
{
    static const char* names[] = { "LHZ", "LHY", "LHX", "LKY", "LAY", "LAX",
                                   "RHZ", "RHY", "RHX", "RKY", "RAY", "RAX" };
    assert(id < DOF_MAX);
    
    return names[id];
}

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
    
    const double footPosX = footSizeX*0.33 - legDiameter/2.;
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
                         (3.*r*r + h*h) / 12.,
                         r*r / 2.);
}

inline double rbMass(unsigned int id)
{
    using namespace CharacterConst;
    
    const double masses[B_MAX] =
    { cylinderMass(thighSizeZ, thighDiameter/2.),     // B_L_THIGH
      cylinderMass(shankSizeZ, shankDiameter/2.),     // B_L_SHANK
      boxMass(footSizeX, footSizeY, footSizeZ),       // B_L_FOOT
      cylinderMass(thighSizeZ, thighDiameter/2.),     // B_R_THIGH
      cylinderMass(shankSizeZ, shankDiameter/2.),     // B_R_SHANK
      boxMass(footSizeX, footSizeY, footSizeZ),       // B_R_FOOT
      boxMass(pelvisSizeX, pelvisSizeY, pelvisSizeZ)  // B_PELVIS
    };
    
    assert(id < B_MAX);
    return masses[id];
}

inline Eigen::Vector3d rbMOI(unsigned int id)
{
    using namespace CharacterConst;
    
    const Eigen::Vector3d mois[B_MAX] =
    {    cylinderMOI(thighSizeZ, thighDiameter/2.),     // B_L_THIGH
         cylinderMOI(shankSizeZ, shankDiameter/2.),     // B_L_SHANK
      3.*boxMOI(footSizeX, footSizeY, footSizeZ),       // B_L_FOOT
         cylinderMOI(thighSizeZ, thighDiameter/2.),     // B_R_THIGH
         cylinderMOI(shankSizeZ, shankDiameter/2.),     // B_R_SHANK
      3.*boxMOI(footSizeX, footSizeY, footSizeZ),       // B_R_FOOT
      3.*boxMOI(pelvisSizeX, pelvisSizeY, pelvisSizeZ)  // B_PELVIS
    };
    
    assert(id < B_MAX);
    return mois[id];
}

#endif
