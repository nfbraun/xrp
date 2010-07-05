#include "FootSegment.h"

// Attach point
Vector3 FootSegment::p1() const
{
    return CoG() - w()*rbfhat();
}

// Front contact circle center
Vector3 FootSegment::pf() const
{
    return CoG() - h()*nor() + (l() - h())*rbfhat();
}

// Back contact circle center
Vector3 FootSegment::pb() const
{
    return CoG() - h()*nor() - h()*rbfhat();
}

// Front contact circle center in body (unrotated) coordinates
Vector3 FootSegment::pfb() const
{
    return -h()*Vector3::eZ - h()*Vector3::eX;
}

// Back contact circle center in body (unrotated) coordinates
Vector3 FootSegment::pbb() const
{
    return -h()*Vector3::eZ + (l()-h())*Vector3::eX;
}

void FootSegment::SetCoG(Vector3 pos, double theta)
{
    fTheta = theta;
    fPos = pos;
}

void FootSegment::SetP1(Vector3 p1, double theta)
{
    fTheta = theta;
    fPos = p1 + w()*rbfhat();
}

void FootSegment::SetPF(Vector3 pf, double theta)
{
    fTheta = theta;
    fPos = pf + (h() + w() - l())*rbfhat() + h()*nor();
}

void FootSegment::SetPB(Vector3 pb, double theta)
{
    fTheta = theta;
    fPos = pb + (h() + w())*rbfhat() + h()*nor();
}
