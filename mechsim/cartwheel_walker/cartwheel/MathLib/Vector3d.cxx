#include "Vector3d.h"
/**
	this method returns a vector that is the current vector, rotated by an angle alpha (in radians) around the axis given as parameter.
	IT IS ASSUMED THAT THE VECTOR PASSED IN IS A UNIT VECTOR!!!
*/
Vector3d Vector3d::rotate(double alpha, const Vector3d &axis){
	//ok... this is fairly hard - check the red book for details.
	double xP = axis.x();
	double yP = axis.y();
	double zP = axis.z();
	double cosa = cos(alpha);
	double sina = sin(alpha);

	double s[3][3] = {{0,			-zP,		yP},
					{zP,			0,			-xP},
					{-yP,			xP,			0}};
	double UUT[3][3]	= {	{xP*xP,		xP*yP,		xP*zP},
						{yP*xP,		yP*yP,		yP*zP},
						{zP*xP,		zP*yP,		zP*zP}};
	double I[3][3] = {{1,0,0},{0,1,0},{0,0,1}};
	double R[3][3] = {{0,0,0},{0,0,0},{0,0,0}};

	for (int i=0;i<3;i++)
		for (int j = 0;j<3;j++)
			R[i][j] = UUT[i][j] + cosa*(I[i][j] - UUT[i][j]) + sina*s[i][j];

	//now that we finally have the transformation matrix set up, we can rotate the vector
	Vector3d result(R[0][0]*x() + R[0][1]*y() + R[0][2]*z(),
	                R[1][0]*x() + R[1][1]*y() + R[1][2]*z(),
                    R[2][0]*x() + R[2][1]*y() + R[2][2]*z());

	return result;
}

