#include "TwoLinkIK.h"

namespace TwoLinkIK {

/**
	This method determines the position of the joint between the child and parent links. 
	Input (all quantities need to be measured in the same coordinate frame):
		p1 - location of parent's origin (for example the shoulder for an arm)
		p2 - target location of end effector of child link (for example, the wrist location)
		n - normal to the plane in which the relative motion between parent and child will take place
		r1 - the length of the parent link
		r2 - the length of the child link
	Return:
		p - the position of the joint (the elbow, for instance), measured in the same coordinates as the values passed in here
*/
Point3d solve(const Point3d& p1, const Point3d& p2, const Vector3d& n, double r1, double r2)
{
	//the solution for this comes from computation of the intersection of two circles of radii r1 and r2, located at
	//p1 and p2 respectively. There are, of course, two solutions to this problem. The calling application can differentiate between these
	//by passing in n or -n for the plane normal.
	
	//this is the distance between p1 and p2. If it is > r1+r2, then we have no solutions. To be nice about it,
	//we will set r to r1+r2 - the behaviour will be to reach as much as possible, even though you don't hit the target
	double r = (p2 - p1).norm();
	if (r > (r1+r2) * 0.993)
		r = (r1 + r2) * 0.993;
	//this is the length of the vector starting at p1 and going to the midpoint between p1 and p2
	double a = (r1 * r1 - r2 * r2 + r * r) / (2 * r);
	double tmp = r1*r1 - a*a;
	if (tmp < 0)
		tmp = 0;
	//and this is the distance from the midpoint of p1-p2 to the intersection point
	double h = sqrt(tmp);
	//now we need to get the two directions needed to reconstruct the intersection point
	Vector3d d1 = (p2 - p1).unit();
	Vector3d d2 = d1.cross(n).toUnit();

	//and now get the intersection point
	Point3d p = p1 + d1 * a + d2 * (-h);

	return p;
}

/**
	This method determines the orientation for the parent link, relative to some other coordinate ("global") frame. 
	Two vectors (one that goes from the parent origin to the child origin v, as well as a normal vector n) are known,
	expressed both in the global frame and the parent frame. Using this information, we can figure out the relative orientation
	between the parent frame and the global frame.

	Input:
		vGlobal - parent's v expressed in grandparent coordinates
		nGlobal	- this is the rotation axis that is used for the relative rotation between the child and the parent joint, expressed in 
				  grandparent coordinates
		vLocal  - parent's v expressed in the parent's local coordinates
		nLocal  - this is the rotation axis that is used for the relative rotation between the child and the parent joint, expressed in 
				  parent's local coordinates
	Output:
		q		- the relative orientation between the parent and the grandparent (i.e. transforms vectors from parent coordinates to grandparent coordinates).
*/
Quaternion getParentOrientation(const Vector3d& vGlobal, const Vector3d& nGlobal, Vector3d vLocal, const Vector3d& nLocal)
{
	Quaternion q;

	//first off, compute the quaternion that rotates nLocal into nGlobal
	Vector3d axis = nLocal.cross(nGlobal);
	axis.toUnit();
	double ang = nLocal.angleWith(nGlobal);

	q = Quaternion::getRotationQuaternion(ang, axis);

	//now q rotates nLocal into nGlobal. We now need an aditional orientation that aligns vLocal to vGlobal

	//nLocal is perpendicular to vLocal so q*nLocal is perpendicular to q*vLocal. Also, q*nLocal is equal to nGlobal,
	//so nGlobal is perpendicular to both q*vLocal and vGlobal, which means that this rotation better be about vGlobal!!!
	vLocal = q.rotate(vLocal);
	axis = vLocal.cross(vGlobal);
	axis.toUnit();
	ang = vLocal.angleWith(vGlobal);

	q = Quaternion::getRotationQuaternion(ang, axis) * q;

	return q;
}

/**
	This method determines the rotation angle about the axis n for the child link, relative to the orientation of the parent
	Input (all quantities need to be measured in the same coordinate frame):
	(Note: v is the vector from a link's origin to the end effector or the child link's origin).
		vParent - the v qunatity for the parent
		vChild  - the v quntity for the child (vector between the child origin and the end effector).
		n		- the axis of rotation
	Output:
		q		- the relative orientation between the child and parent frames
*/
double getChildRotationAngle(const Vector3d& vParent, const Vector3d& vChild, const Vector3d& n)
{
	//compute the angle between the vectors (p1, p) and (p, p2), and that's our result
	double angle = vParent.angleWith(vChild);
	if (vParent.cross(vChild).dot(n)<0)
		angle = -angle;

	return angle;
}

/**
	All quantities that are passed in as parameters here need to be expressed in the same "global" coordinate frame, unless otherwise noted:
		- p1: this is the location of the origin of the parent link
		- p2: this is the target location of the end effector on the child link
		- n: this is the default normal to the rotation plane (it will get modified as little as possible to account for the target location)

		- vParent: vector from parent origin to child origin, expressed in parent coordinates
		- nParent: this is the rotation axis, expressed in parent coordinates. The relative orientation between child and parent will be about this axis
		- vChild: vector from child origin, to position of the end effector, expressed in child coordinates

		Output:
			- qP: relative orientation of parent, relative to "global" coordinate frame
			- qC: relative orientation of child, relative to the parent coordinate frame


	NOTE: for now, the vector vChild is pretty much ignored. Only its length is taken into account. The axis that the child is lined up around is the same as the direction
	of the vector from the parent's origin to the child's origin (when there is zero relative orientation between the two, the axis has the same coordinates in both
	child and parent frame).
*/
void getIKOrientations(const Point3d& p1, const Point3d& p2, const Vector3d& n, const Vector3d& vParent, const Vector3d& nParent, const Vector3d& vChild, Quaternion* qP, Quaternion* qC)
{
	//modify n so that it's perpendicular to the vector (p1, p2), and still as close as possible to n
	Vector3d line = p2 - p1;
	Vector3d temp = n.cross(line);
	Vector3d nG = line.cross(temp);
	nG.toUnit();

	//now compute the location of the child origin, in "global" coordinates
	Vector3d solvedJointPosW = solve(p1, p2, nG, vParent.norm(), vChild.norm());
	Vector3d vParentG = solvedJointPosW - p1;
	Vector3d vChildG = p2 - solvedJointPosW;
	

	//now we need to solve for the orientations of the parent and child
	//if the parent has 0 relative orientation to the grandparent (default), then the grandparent's orientation is also the parent's
	*qP = getParentOrientation(vParentG, nG, vParent, nParent);

	double childAngle = getChildRotationAngle(vParentG, vChildG, nG);
	*qC = Quaternion::getRotationQuaternion(childAngle, nParent);
}

} // end namespace TwoLinkIK

