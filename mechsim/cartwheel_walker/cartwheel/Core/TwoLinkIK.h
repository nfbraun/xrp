#pragma once

#include <MathLib/Point3d.h>
#include <MathLib/Vector3d.h>
#include <MathLib/Quaternion.h>

/**
	This class implements an analytical solution for the inverse kinematics of a linkage composed of two links - a parent link and a child link.
	The origin of the parent link is fixed, and the desired position for the end effector of the child is given as input. The position of the origin of
	the child link is computed, and, with this information, the orientation of the parent and child links are also computed. This can be seen, generally speaking,
	as the solution of the intersection of two spheres, which gives an infinte number of solutions. To reduce this, a vector that represents the normal to the plane
	that the three points (parent origin, child origin and child end effector) should lie on is given as input. The child can only rotate relative to the parent 
	around this normal axis, which can be used as a prior. This results in two possible solutions. The direction of the normal is used to select a unique solution.
*/
namespace TwoLinkIK {

/* see TwoLinkIK.cpp for a description of these functions */

Point3d solve(const Point3d& p1, const Point3d& p2, const Vector3d& n, double r1, double r2);

Quaternion getParentOrientation(const Vector3d& vGlobal, const Vector3d& nGlobal, Vector3d vLocal, const Vector3d& nLocal);

double getChildRotationAngle(const Vector3d& vParent, const Vector3d& vChild, const Vector3d& n);

void getIKOrientations(const Point3d& p1, const Point3d& p2, const Vector3d& n, const Vector3d& vParent, const Vector3d& nParent, const Vector3d& vChild, Quaternion* qP, Quaternion* qC);

} // end namespace TwoLinkIK
