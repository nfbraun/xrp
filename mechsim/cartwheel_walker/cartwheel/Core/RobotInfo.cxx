#include "RobotInfo.h"

Eigen::Quaterniond RobotInfo::characterFrame() const
{
    return Quaternion(rootOrient()).getComplexConjugate().decomposeRotation(PhysicsGlobals::up).getComplexConjugate().toEigen();
}

/**
    this method is used to return the current heading of the character, specified as an angle measured in radians
*/
double RobotInfo::headingAngle() const
{
    //first we need to get the current heading of the character. Also, note that q and -q represent the same rotation
    Quaternion q = characterFrame();
    if (q.s<0){
        q.s = -q.s;
        q.v = -q.v;
    }
    double currentHeading = 2 * safeACOS(q.s);
    if (q.v.dot(PhysicsGlobals::up) < 0)
        currentHeading = -currentHeading;
    
    return currentHeading;
}

