#include "FootSegment.h"

// Attach point
Eigen::Vector3d FootSegment::p1() const
{
    return CoG() - w()*rbfhat();
}

// Front contact circle center
Eigen::Vector3d FootSegment::pf() const
{
    return CoG() - h()*nor() + (l() - w() - h())*rbfhat();
}

// Back contact circle center
Eigen::Vector3d FootSegment::pb() const
{
    return CoG() - h()*nor() - (w() + h())*rbfhat();
}

// Front contact circle center in body (unrotated) coordinates
Eigen::Vector3d FootSegment::pfb() const
{
    return -h()*Eigen::Vector3d::UnitZ() + (l() - w() - h())*Eigen::Vector3d::UnitX();
}

// Back contact circle center in body (unrotated) coordinates
Eigen::Vector3d FootSegment::pbb() const
{
    return -h()*Eigen::Vector3d::UnitZ() - (w() + h())*Eigen::Vector3d::UnitX();
}

void FootSegment::SetCoG(Eigen::Vector3d pos, double theta)
{
    fTheta = theta;
    fPos = pos;
}

void FootSegment::SetP1(Eigen::Vector3d p1, double theta)
{
    fTheta = theta;
    fPos = p1 + w()*rbfhat();
}

void FootSegment::SetPF(Eigen::Vector3d pf, double theta)
{
    fTheta = theta;
    fPos = pf + (h() + w() - l())*rbfhat() + h()*nor();
}

void FootSegment::SetPB(Eigen::Vector3d pb, double theta)
{
    fTheta = theta;
    fPos = pb + (h() + w())*rbfhat() + h()*nor();
}
