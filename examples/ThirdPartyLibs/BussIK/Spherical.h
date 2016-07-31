/*
*
* Mathematics Subpackage (VrMath)
*
*
* Author: Samuel R. Buss, sbuss@ucsd.edu.
* Web page: http://math.ucsd.edu/~sbuss/MathCG
*
*
This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose,
including commercial applications, and to alter it and redistribute it freely,
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*
*
*/



//
// Spherical Operations Classes
//
//
// B. SphericalInterpolator
//
// OrientationR3
//
// A. Quaternion		
//
// B. RotationMapR3		// Elsewhere
//
// C. EulerAnglesR3		// TO DO
//
//
// Functions for spherical operations
// A. Many routines for rotation and averaging on a sphere
//

#ifndef SPHERICAL_H
#define SPHERICAL_H

#include "LinearR3.h"
#include "LinearR4.h"
#include "MathMisc.h"

class SphericalInterpolator;		// Spherical linear interpolation of vectors
class SphericalBSpInterpolator;	// Spherical Bspline interpolation of vector
class Quaternion;			// Quaternion (x,y,z,w) values.
class EulerAnglesR3;		// Euler Angles

// *****************************************************
// SphericalInterpolator class                         *
//    - Does linear interpolation (slerp-ing)		   *
// * * * * * * * * * * * * * * * * * * * * * * * * * * *


class SphericalInterpolator {

private:
	VectorR3 startDir, endDir;	// Unit vectors (starting and ending)
	double startLen, endLen;	// Magnitudes of the vectors
	double rotRate;				// Angle between start and end vectors

public:
	SphericalInterpolator( const VectorR3& u, const VectorR3& v );

	VectorR3 InterValue ( double frac ) const;
};


// ***************************************************************
// * Quaternion class - prototypes								 *
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

class Quaternion {

public:
	double x, y, z, w;

public:
	Quaternion() :x(0.0), y(0.0), z(0.0), w(1.0) {};
	Quaternion( double, double, double, double );

	inline Quaternion& Set( double xx, double yy, double zz, double ww );
	inline Quaternion& Set( const VectorR4& );
	Quaternion& Set( const EulerAnglesR3& );
	Quaternion& Set( const RotationMapR3& );
	Quaternion& SetRotate( const VectorR3& );

	Quaternion& SetIdentity();		// Set to the identity map
	Quaternion  Inverse() const;	// Return the Inverse
	Quaternion& Invert();			// Invert this quaternion

	double Angle();					// Angle of rotation
	double Norm();					// Norm of x,y,z component

	Quaternion& operator*=(const Quaternion&);

};

Quaternion operator*(const Quaternion&, const Quaternion&);

inline Quaternion ToQuat( const VectorR4& v) 
{ 
	return Quaternion(v.x,v.y,v.z,v.w); 
}

inline double Quaternion::Norm() 
{
	return sqrt( x*x + y*y + z*z );
}

inline double Quaternion::Angle () 
{
	double halfAngle = asin(Norm());
	return halfAngle+halfAngle;
}


// ****************************************************************
// Solid Geometry Routines										  *
// ****************************************************************

// Compute the angle formed by two geodesics on the unit sphere.
//	Three unit vectors u,v,w specify the geodesics u-v and v-w which
//  meet at vertex uv.  The angle from v-w to v-u is returned.  This
//  is always in the range [0, 2PI).
double SphereAngle( const VectorR3& u, const VectorR3& v, const VectorR3& w );

//  Compute the area of a triangle on the unit sphere.  Three unit vectors
//		specify the corners of the triangle in COUNTERCLOCKWISE order.
inline double SphericalTriangleArea( 
						const VectorR3& u, const VectorR3& v, const VectorR3& w )
{
	double AngleA = SphereAngle( u,v,w );
	double AngleB = SphereAngle( v,w,u );
	double AngleC = SphereAngle( w,u,v );
	return ( AngleA+AngleB+AngleC - PI );
}


// ****************************************************************
// Spherical Mean routines										  *
// ****************************************************************

// Weighted sum of vectors
VectorR3 WeightedSum( long Num, const VectorR3 vv[], const double weights[] );
VectorR4 WeightedSum( long Num, const VectorR4 vv[], const double weights[] );

// Weighted average of vectors on the sphere.  
//		Sum of weights should equal one (but no checking is done)
VectorR3 ComputeMeanSphere( long Num, const VectorR3 vv[], const double weights[],
						  double tolerance = 1.0e-15, double bkuptolerance = 1.0e-13 );
VectorR3 ComputeMeanSphere( long Num, const VectorR3 vv[], const double weights[],
						  const VectorR3& InitialVec,
						  double tolerance = 1.0e-15, double bkuptolerance = 1.0e-13 );
VectorR4 ComputeMeanSphere( long Num, const VectorR4 vv[], const double weights[],
						  double tolerance = 1.0e-15, double bkuptolerance = 1.0e-13 );
Quaternion ComputeMeanQuat( long Num, const Quaternion qq[], const double weights[],
						   double tolerance = 1.0e-15, double bkuptolerance = 1.0e-13 );

// Next functions mostly for internal use.
//		It takes an initial estimate InitialVec (and a flag for
//		indicating quaternions).
VectorR4 ComputeMeanSphere( long Num, const VectorR4 vv[], const double weights[],
						   const VectorR4& InitialVec, int QuatFlag=0,
						   double tolerance = 1.0e-15, double bkuptolerance = 1.0e-13 );
const int SPHERICAL_NOTQUAT=0;
const int SPHERICAL_QUAT=1;

// Slow version, mostly for testing
VectorR3 ComputeMeanSphereSlow( long Num, const VectorR3 vv[], const double weights[],
							double tolerance = 1.0e-16, double bkuptolerance = 5.0e-16 );
VectorR4 ComputeMeanSphereSlow( long Num, const VectorR4 vv[], const double weights[],
							double tolerance = 1.0e-16, double bkuptolerance = 5.0e-16 );
VectorR3 ComputeMeanSphereOld( long Num, const VectorR3 vv[], const double weights[],
						  double tolerance = 1.0e-15, double bkuptolerance = 1.0e-13 );
VectorR4 ComputeMeanSphereOld( long Num, const VectorR4 vv[], const double weights[],
						   const VectorR4& InitialVec, int QuatFlag,
						   double tolerance = 1.0e-15, double bkuptolerance = 1.0e-13 );

// Solves a system of spherical-mean equalities, where the system is
// given as a tridiagonal matrix.
void SolveTriDiagSphere ( int numPoints, 
						   const double* tridiagvalues, const VectorR3* c,
						   VectorR3* p, 
						   double accuracy=1.0e-15, double bkupaccuracy=1.0e-13 );
void SolveTriDiagSphere ( int numPoints, 
						   const double* tridiagvalues, const VectorR4* c,
						   VectorR4* p, 
						   double accuracy=1.0e-15, double bkupaccuracy=1.0e-13 );

//  The "Slow" version uses a simpler but slower iteration with a linear rate of
//		convergence.  The base version uses a Newton iteration with a quadratic
//		rate of convergence.
void SolveTriDiagSphereSlow ( int numPoints, 
						   const double* tridiagvalues, const VectorR3* c,
						   VectorR3* p, 
						   double accuracy=1.0e-15, double bkupaccuracy=5.0e-15 );
void SolveTriDiagSphereSlow ( int numPoints, 
						   const double* tridiagvalues, const VectorR4* c,
						   VectorR4* p, 
						   double accuracy=1.0e-15, double bkupaccuracy=5.0e-15 );

// The "Unstable" version probably shouldn't be used except for very short sequences
//		of knots.  Mostly it's used for testing purposes now.
void SolveTriDiagSphereUnstable ( int numPoints, 
						   const double* tridiagvalues, const VectorR3* c,
						   VectorR3* p, 
						   double accuracy=1.0e-15, double bkupaccuracy=1.0e-13 );
void SolveTriDiagSphereHelperUnstable ( int numPoints, 
								const double* tridiagvalues, const VectorR3 *c,
								const VectorR3& p0value,
								VectorR3 *p, 
								double accuracy=1.0e-15, double bkupaccuracy=1.0e-13 );



// ***************************************************************
// * Quaternion class - inlined member functions				 *
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

inline VectorR4::VectorR4 ( const Quaternion& q )
: x(q.x), y(q.y), z(q.z), w(q.w) 
{}

inline VectorR4& VectorR4::Set ( const Quaternion& q )
{
	x = q.x;  y = q.y;  z = q.z; w = q.w;
	return *this;
}

inline Quaternion::Quaternion( double xx, double yy, double zz, double ww)
: x(xx), y(yy), z(zz), w(ww) 
{}
	
inline Quaternion& Quaternion::Set( double xx, double yy, double zz, double ww )
{
	x = xx;
	y = yy;
	z = zz;
	w = ww;
	return *this;
}

inline Quaternion& Quaternion::Set( const VectorR4& u)
{
	x = u.x;
	y = u.y;
	z = u.z;
	w = u.w;
	return *this;
}

inline Quaternion& Quaternion::SetIdentity()
{
	x = y = z = 0.0;
	w = 1.0;
	return *this;
}

inline Quaternion operator*(const Quaternion& q1, const Quaternion& q2)
{
	Quaternion q(q1);
	q *= q2;
	return q;
}

inline Quaternion& Quaternion::operator*=( const Quaternion& q )
{
	double wnew = w*q.w - (x*q.x + y*q.y + z*q.z);
	double xnew = w*q.x + q.w*x + (y*q.z - z*q.y);
	double ynew = w*q.y + q.w*y + (z*q.x - x*q.z);
	z           = w*q.z + q.w*z + (x*q.y - y*q.x);
	w = wnew;
	x = xnew;
	y = ynew;
	return *this;
}

inline Quaternion Quaternion::Inverse()	const	// Return the Inverse
{
	return ( Quaternion( x, y, z, -w ) );
}

inline Quaternion& Quaternion::Invert()		// Invert this quaternion
{
	w = -w;
	return *this;
}


#endif // SPHERICAL_H
