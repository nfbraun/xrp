#include "ChainSegment.h"

Vector3 ChainSegment::p1() const
{
    return CoG() - w()*nor() - c()*r12hat();
}

Vector3 ChainSegment::p2() const
{
    return CoG() - w()*nor() + (l() - c())*r12hat();
}

// Chain segment center in body (unrotated) coordinates
Vector3 ChainSegment::cb() const
{
    return -w()*Vector3::eX + (c() - l()/2.)*Vector3::eZ;
}

void ChainSegment::SetCoG(Vector3 pos, double theta)
{
    fTheta = theta;
    fPos = pos;
}

void ChainSegment::SetP1(Vector3 p1, double theta)
{
    fTheta = theta;
    fPos = p1 + c()*r12hat() + w()*nor();
}

void ChainSegment::SetP2(Vector3 p2, double theta)
{
    fTheta = theta;
    fPos = p2 - (l() - c())*r12hat() + w()*nor();
}

void ChainSegment::SetP1P2(Vector3 p1, Vector3 p2)
{
    Vector3 r12 = p2 - p1;
    fTheta = atan2(-r12.x(), -r12.z());
    fPos = p1 + (c()/l())*r12 + w()*nor();
}
