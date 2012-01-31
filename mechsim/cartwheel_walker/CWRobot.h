#ifndef CW_CHARACTER_H
#define CW_CHARACTER_H
#include <Core/Character.h>
#include <ode/ode.h>
#include <map>

class CWRobot
{
public:
    dBodyID fTorsoB, fLowerBackB, fPelvisB;
    dBodyID fLUpperLegB, fLLowerLegB, fRUpperLegB, fRLowerLegB;
    dBodyID fLFootB, fRFootB;
    dGeomID fLFootG, fRFootG;
    RigidBody* fLFootRB, *fRFootRB;
    dJointID fLAnkleJ, fLKneeJ, fLHipJ;
    dJointID fRAnkleJ, fRKneeJ, fRHipJ;
    
    Character* fCharacter;
    
    std::map<RigidBody*, dBodyID> fODEMap;
    dBodyID getODEBody(RigidBody* rb) { return fODEMap[rb]; }
    
    CWRobot() {}
    void create(dWorldID world);
    
    dBodyID makeODEARB(dWorldID world, ArticulatedRigidBody* arb);
};

namespace CharacterConst {
    const double density = 900;
    
    const double footSizeX = 0.12;
    const double footSizeY = 0.05;
    const double footSizeZ = 0.2;
    const double legDiameter = 0.1;
    const double lowerLegDiameter = 0.1;
    const double upperLegDiameter = 0.1;
    const double legSizeY = 1.0;
    const double kneeRelativePosY = 0.5;
    const double legRelativeAnchorX = 0.6;
    
    const double pelvisSizeX = 0.45;
    const double pelvisSizeY = 0.3;
    const double pelvisSizeZ = 0.25;
    
    const double pelvisDiameter = 0.45;
    const double pelvisRadius = pelvisDiameter/2.0;
    
    const double lowerLegSizeY = legSizeY * kneeRelativePosY;
    const double upperLegSizeY = legSizeY - lowerLegSizeY;
    
    const double footPosY = footSizeY/2.;
    const double anklePosY = footSizeY;
    const double lowerLegPosY = anklePosY + lowerLegSizeY/2.;
    const double kneePosY = anklePosY + legSizeY * kneeRelativePosY;
    const double upperLegPosY = kneePosY + upperLegSizeY/2.;
    const double hipPosY = anklePosY + legSizeY;
    const double pelvisPosY = hipPosY + pelvisSizeY/2.;
    
    const double legPosX_L = pelvisSizeX/2.0*legRelativeAnchorX;
    //const double kneePosY_L = anklePosY + (hipPosY - anklePosY) * kneeRelativePosY;
    
    const double legPosX_R = -pelvisSizeX/2.0*legRelativeAnchorX;
    //const double kneePosY_R = anklePosY + (hipPosY - anklePosY) * kneeRelativePosY;
    
    //const double pelvisBottomPos = -pelvisSizeY/2.0-legSizeY*0.1;
    //const double pelvisTopPos = pelvisSizeY/2.0;
    //const double rootPosY = hipPosY + pelvisSizeY/2.0 + 0.007;
} // end namespace CharacterConst

#endif
