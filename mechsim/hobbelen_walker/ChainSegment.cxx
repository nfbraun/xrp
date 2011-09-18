#include "ChainSegment.h"

Eigen::Vector3d ChainSegment::p1() const
{
    return CoG() - w()*nor() - c()*r12hat();
}

Eigen::Vector3d ChainSegment::p2() const
{
    return CoG() - w()*nor() + (l() - c())*r12hat();
}

// Chain segment center in body (unrotated) coordinates
Eigen::Vector3d ChainSegment::cb() const
{
    return -w()*Eigen::Vector3d::UnitX() + (c() - l()/2.)*Eigen::Vector3d::UnitZ();
}

void ChainSegment::SetCoG(Eigen::Vector3d pos, double theta)
{
    fTheta = theta;
    fPos = pos;
}

void ChainSegment::SetP1(Eigen::Vector3d p1, double theta)
{
    fTheta = theta;
    fPos = p1 + c()*r12hat() + w()*nor();
}

void ChainSegment::SetP2(Eigen::Vector3d p2, double theta)
{
    fTheta = theta;
    fPos = p2 - (l() - c())*r12hat() + w()*nor();
}

void ChainSegment::SetP1P2(Eigen::Vector3d p1, Eigen::Vector3d p2)
{
    Eigen::Vector3d r12 = p2 - p1;
    fTheta = atan2(-r12.x(), -r12.z());
    fPos = p1 + (c()/l())*r12 + w()*nor();
}
