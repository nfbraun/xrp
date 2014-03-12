#include <cartwheel/RobotInfo.h>
#include <cartwheel/CWConfig.h>
#include <cartwheel/MathLib.h>

/*
Given a unit quaternion q and a vector v, compute two unit quaterions a and b such that:
    q = a b
    vec(a) || v
    vec(b) _'_ v
    
The vector v is assumed to be normalized (i.e. ||v|| = 1). The function returns only a;
b can be recovered via b = a* q.
*/
Eigen::Quaterniond decompRot(const Eigen::Quaterniond& q, const Eigen::Vector3d& v)
{
    /* As vec(a) || w and a normalized, we must have, for some -1 <= alpha <= 1,
           a = (sqrt(1 - alpha^2), alpha v)
       and
           b = a* q = (sqrt(1 - alpha^2), -alpha v) q    (1)
       
       vec(b) _'_ v implies that vec(b) . v = 0. By putting in (1), this implies:
           sqrt(1 - alpha^2) (vec(q) . v) = alpha s(q)
       where s(q) is the scalar part of q (q.w() in Eigen).
       
       From this, is follows that alpha is given by
           alpha = sign(s(q)) (vec(q) . v) / sqrt(s(q)^2 + (vec(q) . v)^2)
       
       For compatibility with the old implementation, we note that q and -q represent the
       same rotation and move sign(s(q)) to the scalar part of the return value.
    */
    
    const double qdotv = q.vec().dot(v);
    double alpha = qdotv / sqrt(pow(q.w(), 2) + pow(qdotv, 2));
    
    if(q.w() < 0.)
        return Eigen::Quaterniond(-sqrt(1. - pow(alpha, 2)), alpha*v.x(), alpha*v.y(), alpha*v.z());
    else
        return Eigen::Quaterniond(sqrt(1. - pow(alpha, 2)), alpha*v.x(), alpha*v.y(), alpha*v.z());
}

Eigen::Quaterniond RobotInfo::characterFrame() const
{
    return decompRot(rootOrient().conjugate(), CWConfig::UP).conjugate();
}

/**
    this method is used to return the current heading of the character, specified as an angle measured in radians
*/
double RobotInfo::headingAngle() const
{
    //first we need to get the current heading of the character. Also, note that q and -q represent the same rotation
    Eigen::Quaterniond q = characterFrame();
    if (q.w() < 0.) {
        q.w() = -q.w();
        q.vec() = -q.vec();
    }
    double currentHeading = 2 * safeACOS(q.w());
    if (q.vec().dot(CWConfig::UP) < 0.)
        currentHeading = -currentHeading;
    
    return currentHeading;
}

