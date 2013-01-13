#pragma once

#include <math.h>
#include <MathLib/MathLib.h>
#include <Eigen/Dense>

/*================================================================================================================================================================*
 | This class implements a Vector3d in a three dimensional space. Note that the w-coordinate of a vector expressed in homogenous coordinates is 0.                  |
 *================================================================================================================================================================*/
class Vector3d {
private:
    double fx, fy, fz;

public:
	/**
		A bunch of useful constructors.
	*/
	
    Vector3d(): fx(0.), fy(0.), fz(0.) {}

    Vector3d(double x_, double y_, double z_) : fx(x_), fy(y_), fz(z_) {}

    Vector3d(const Vector3d &other)
        : fx(other.x()), fy(other.y()), fz(other.z()) {}
    
    Vector3d(const Eigen::Vector3d& other)
        : fx(other.x()), fy(other.y()), fz(other.z()) {}
    
    
    double& x() { return fx; }
    double& y() { return fy; }
    double& z() { return fz; }
    
    const double& x() const { return fx; }
    const double& y() const { return fy; }
    const double& z() const { return fz; }

	/**
		a copy operator
	*/
	inline Vector3d& operator = (const Vector3d &other){
		this->x() = other.x();
		this->y() = other.y();
		this->z() = other.z();
		return *this;
	}
	
	inline Vector3d& operator = (const Eigen::Vector3d &other){
		this->x() = other.x();
		this->y() = other.y();
		this->z() = other.z();
		return *this;
	}
	
	static Vector3d Zero() { return Vector3d(0., 0., 0.); }
	
	Eigen::Vector3d toEigen() const { return Eigen::Vector3d(x(), y(), z()); }
	
	/**
		addition of two vectors - results in a new vector.
	*/
	inline Vector3d operator + (const Vector3d &v) const{
		return Vector3d(this->x() + v.x(), this->y() + v.y(), this->z() + v.z());
	}
	
	/**
		subtraction of two vectors - results in a new vector.
	*/
	inline Vector3d operator - (const Vector3d &v) const{
		return Vector3d(this->x() - v.x(), this->y() - v.y(), this->z() - v.z());
	}
	
	/**
		this method is used to compute two vectors that are orhtogonal to the current vector.
		It is assumed that the current vector is a unit vector.
	*/
	inline void getOrthogonalVectors(Vector3d* a, Vector3d* b) {
		//try to choose a vector in the y-z plane, if the z-coordinate is significant enough
		if (z()*z() > 0.5){
			double tmp1 = y()*y() + z()*z();
			double tmp = 1/sqrt(tmp1);
			a->x() = 0;
			a->y() = -z()*tmp;
			a->z() = y()*tmp;
			//and the third vector is the cross product of *this and a - but take advantage of the fact that we know the two vectors
			b->x() = tmp1*tmp;
			b->y() = -x() * a->z();
			b->z() = x() * a->y();
		}else{	//otherwise, the other two components must be significant enough - so choose a vector in the x-y plane
			double tmp1 = x()*x() + y()*y();
			double tmp = 1/sqrt(tmp1);
			a->x() = -y()*tmp;
			a->y() = x()*tmp;
			a->z() = 0;
			//and again, set the last one to the cross product...
			b->x() = -z()*a->y();
			b->y() = z()*a->x();
			b->z() = tmp*tmp1;
		}
	}
	
	/**
		multiplication by a constant - results in a new vector.
	*/
	inline Vector3d operator * (double n) const{
		return Vector3d(n*this->x(), n*this->y(), n*this->z());
	}
	/**
		dividing by a constant - results in a new vector
	*/
	inline Vector3d operator / (double n) const{
		double m = 1.0/n;
		return Vector3d(this->x()*m, this->y()*m, this->z()*m);
	}
	/**
		add a new vector to the current one.
	*/
	inline Vector3d& operator += (const Vector3d &v){
		this->x() += v.x();
		this->y() += v.y();
		this->z() += v.z();
		return (*this);
	}
	/**
		subtract a new vector to the current one.
	*/
	inline Vector3d& operator -= (const Vector3d &v){
		this->x() -= v.x();
		this->y() -= v.y();
		this->z() -= v.z();
		return (*this);
	}
	/**
		multiply the current vector by a constant
	*/
	inline Vector3d& operator *= (double n){
		this->x() *= n;
		this->y() *= n;
		this->z() *= n;
		
		return (*this);
	}
	/**
		divide the current vector by a constant.
	*/
	inline Vector3d& operator /= (double n){
		double m = 1.0/n;
		this->x() *= m;
		this->y() *= m;
		this->z() *= m;
		return (*this);
	}

	/**
		Returns a vector that has all its components multiplied by -1.
	*/
	inline Vector3d operator - (){
		return Vector3d(-this->x(), -this->y(), -this->z());
	}

	/**
		computes the dot product of this vector with v.
	*/
	inline double dot(const Vector3d &v) const {
		return (this->x() * v.x() + this->y() * v.y() + this->z() * v.z());
	}
	
	/**
		computes the cross product of this vector with v (*this x v).
	*/
	inline Vector3d cross(const Vector3d &v) const {
		/*
			Easiest way to figure it out is to set it up like this:
								___________________
							Ux | Uy   Uz   Ux   Uy | Uz
							Vx | Vy   Vz   Vx   Vy | Vz
								-------------------
			Cross product is given by cross multiplying the items in the box, and subing the other
			diagonal
		*/
		Vector3d result;
		const Vector3d& u = *this;
		result.x() = u.y() * v.z() - u.z() * v.y();
		result.y() = u.z() * v.x() - u.x() * v.z();
		result.z() = u.x() * v.y() - u.y() * v.x();
		
		return result;
	}
	
	/**
		computes the two norm (length) of the current vector
	*/
	double squaredNorm() const {
		return ( x()*x() + y()*y() + z()*z() );
	}
	
	double norm() const {
		return sqrt(squaredNorm());
	}
	
	/**
		computes the projection of the current vector on the vector v.
	*/
	Vector3d projectionOn(const Vector3d &v) const {
	/*  Here's how it goes...
		proj = V/|V| * |U| * cos(angle) = V/|V| * |U| * U.V / |U| / |V|
		after simplifying we get:
				U.V 
		proj = ------- * V
			   |V|*|V|
	*/
		Vector3d u = *this;
		return v*(u.dot(v)/(v.squaredNorm()));
	}


	/**
		This method returns the angle between compute angle between this vector and vector v - this method only returns angles between 0 and PI.
		The angle returned is measured in radians.
	*/
	inline double angleWith(const Vector3d &v) const {
		// U.V = |U|*|V|*cos(angle)
		// therefore angle = inverse cos (U.V/(|U|*|V|))
		double result = this->dot(v) / (this->norm() * v.norm());
		return safeACOS(result);
	}

	/**
		This method changes the current vector to a unit vector.
		NOTE: deliberately not renamed to normalize() because of the behaviour
		for zero vectors.
	*/
	Vector3d& toUnit(){
		double d = this->norm();
		if (IS_ZERO(d))   // FIXME
			return *this;
		*this /= d;
		return (*this);
	}

	/**
		This method returns a unit vector in the direction of the current vector.
	*/
	inline Vector3d unit() const{
		Vector3d v(*this);
		v.toUnit();
		return v;
	}

	/**
		this method returns the cross product matrix - r*
	*/
	//void setCrossProductMatrix(Matrix *m) const;

	/**
		this method returns a vector that is the current vector, rotated by an angle alpha (in radians) around the axis given as parameter.
		IT IS ASSUMED THAT THE VECTOR PASSED IN IS A UNIT VECTOR!!!
	*/
	Vector3d rotate(double alpha, const Vector3d &axis);
};

inline Vector3d operator * (double n, Vector3d v) {
	return v*n;
}
