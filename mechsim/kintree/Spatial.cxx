#include "Spatial.h"

double SpMot::dot(const SpForce& f) const
{
    return ang().dot(f.ang()) + lin().dot(f.lin());
}

SpMot SpMot::cross(const SpMot& rhs) const
{
    return SpMot(ang().cross(rhs.ang()), ang().cross(rhs.lin()) + lin().cross(rhs.ang()));
}

SpForce SpMot::cross_star(const SpForce& rhs) const
{
    return SpForce(ang().cross(rhs.ang()) + lin().cross(rhs.lin()), ang().cross(rhs.lin()));
}

