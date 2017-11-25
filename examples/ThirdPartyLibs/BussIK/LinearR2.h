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
// Linear Algebra Classes over R2
//
//
// A. Vector and Position classes
//
//    A.1. VectorR2: a column vector of length 2
//
//	  A.2. VectorHgR2 - homogenous vector for R2 (a 3-Vector) 
//
// B. Matrix Classes
//
//	  B.1 LinearMapR2 - arbitrary linear map; 2x2 real matrix
//
//	  B.2 RotationMapR2 - orthonormal 2x2 matrix
//

#ifndef LINEAR_R2_H
#define LINEAR_R2_H

#include <math.h>
#include <assert.h>
#include <iostream>
#include "MathMisc.h"
using namespace std;

class VectorR2;				// R2 Vector
class VectorHgR2;
class Matrix2x2;
class LinearMapR2;			// 2x2 real matrix
class AffineMapR3;			// Affine Map (3x4 Matrix)
class RotationMapR2;		// 2x2 rotation map

// **************************************
// VectorR2 class                       *
// * * * * * * * * * * * * * * * * * * **

class VectorR2 {

public:
	double x, y;		// The x & y  coordinates.

public:
	VectorR2( ) : x(0.0), y(0.0) {}
	VectorR2( double xVal, double yVal )
		: x(xVal), y(yVal) {}
	VectorR2( const VectorHgR2& uH );

	VectorR2& SetZero() { x=0.0; y=0.0; return *this;}
	VectorR2& Set( double xx, double yy ) 
			{ x=xx; y=yy; return *this;}
	VectorR2& Load( const double* v );
	VectorR2& Load( const float* v );
	void Dump( double* v ) const;
	void Dump( float* v ) const;

	static const VectorR2 Zero;
	static const VectorR2 UnitX;
	static const VectorR2 UnitY;
	static const VectorR2 NegUnitX;
	static const VectorR2 NegUnitY;

	VectorR2& operator+= ( const VectorR2& v ) 
		{ x+=v.x; y+=v.y; return(*this); } 
	VectorR2& operator-= ( const VectorR2& v ) 
		{ x-=v.x; y-=v.y; return(*this); }
	VectorR2& operator*= ( double m ) 
		{ x*=m; y*=m; return(*this); }
	VectorR2& operator/= ( double m ) 
			{  double mInv = 1.0/m; 
			  x*=mInv; y*=mInv;
			  return(*this); }
	VectorR2 operator- () const { return ( VectorR2(-x, -y) ); }
	VectorR2& ArrayProd(const VectorR2&);		// Component-wise product

	VectorR2& AddScaled( const VectorR2& u, double s );

	double Norm() const { return ( sqrt( x*x + y*y ) ); }
	double L1Norm() const { return (Max(fabs(x),fabs(y))); }
	double Dist( const VectorR2& u ) const;	// Distance from u
	double DistSq( const VectorR2& u ) const;	// Distance from u
	double NormSq() const { return ( x*x + y*y ); }
	double MaxAbs() const;
	VectorR2& Normalize () { *this /= Norm(); return *this;}	// No error checking
	VectorR2& MakeUnit();		// Normalize() with error checking
	VectorR2& ReNormalize();
	bool IsUnit( double tolerance = 1.0e-15 ) const
		{  double norm = Norm();
		  return ( 1.0+tolerance>=norm && norm>=1.0-tolerance ); }
	bool IsZero() const { return ( x==0.0 && y==0.0 ); }
	bool NearZero(double tolerance) const { return( MaxAbs()<=tolerance );}
							// tolerance should be non-negative

	VectorR2& Rotate( double theta );	// rotate through angle theta
	VectorR2& Rotate( double costheta, double sintheta );

};

inline VectorR2 operator+( const VectorR2& u, const VectorR2& v );
inline VectorR2 operator-( const VectorR2& u, const VectorR2& v ); 
inline VectorR2 operator*( const VectorR2& u, double m); 
inline VectorR2 operator*( double m, const VectorR2& u); 
inline VectorR2 operator/( const VectorR2& u, double m); 
inline int operator==( const VectorR2& u, const VectorR2& v ); 

inline double operator^ (const VectorR2& u, const VectorR2& v ); // Dot Product
inline VectorR2 ArrayProd ( const VectorR2& u, const VectorR2& v );

inline double Mag(const VectorR2& u) { return u.Norm(); }
inline double Dist(const VectorR2& u, const VectorR2& v) { return u.Dist(v); }
inline double DistSq(const VectorR2& u, const VectorR2& v) { return u.DistSq(v); }
inline double NormalizeError (const VectorR2&);

// ****************************************
// VectorHgR2 class                       *
// * * * * * * * * * * * * * * * * * * * **

class VectorHgR2 {

public:
	double x, y, w;		// The x & y & w coordinates.

public:
	VectorHgR2( ) : x(0.0), y(0.0), w(1.0) {}
	VectorHgR2( double xVal, double yVal )
		: x(xVal), y(yVal), w(1.0) {}
	VectorHgR2( double xVal, double yVal, double wVal )
		: x(xVal), y(yVal), w(wVal) {}
	VectorHgR2 ( const VectorR2& u ) : x(u.x), y(u.y), w(1.0) {}
};

// ********************************************************************
// Matrix2x2     - base class for 2x2 matrices                        *
// * * * * * * * * * * * * * * * * * * * * * **************************

class Matrix2x2 {

public:
	double m11, m12, m21, m22;	
									
	// Implements a 2x2 matrix: m_i_j - row-i and column-j entry

	static const Matrix2x2 Identity;

public:

	inline Matrix2x2();
	inline Matrix2x2( const VectorR2&, const VectorR2& );	// Sets by columns!
	inline Matrix2x2( double, double, double, double );	// Sets by columns

	inline void SetIdentity ();		// Set to the identity map
	inline void SetZero ();			// Set to the zero map
	inline void Set( const VectorR2&, const VectorR2& );
	inline void Set( double, double, double, double );
	inline void SetByRows( const VectorR2&, const VectorR2& );
	inline void SetByRows( double, double, double, double );
	inline void SetColumn1 ( double, double );
	inline void SetColumn2 ( double, double );
	inline void SetColumn1 ( const VectorR2& );
	inline void SetColumn2 ( const VectorR2& );
	inline VectorR2 Column1() const;
	inline VectorR2 Column2() const;

	inline void SetRow1 ( double, double );
	inline void SetRow2 ( double, double );
	inline void SetRow1 ( const VectorR2& );
	inline void SetRow2 ( const VectorR2& );
	inline VectorR2 Row1() const;
	inline VectorR2 Row2() const;

	inline void SetDiagonal( double, double );
	inline void SetDiagonal( const VectorR2& );
	inline double Diagonal( int );

	inline void MakeTranspose();					// Transposes it.
	inline void operator*= (const Matrix2x2& B);	// Matrix product
	inline Matrix2x2& ReNormalize();

	inline void Transform( VectorR2* ) const;
	inline void Transform( const VectorR2& src, VectorR2* dest) const;

};

inline double NormalizeError( const Matrix2x2& );
inline VectorR2 operator* ( const Matrix2x2&, const VectorR2& );

ostream& operator<< ( ostream& os, const Matrix2x2& A );


// *****************************************
// LinearMapR2 class                       *
// * * * * * * * * * * * * * * * * * * * * *

class LinearMapR2 : public Matrix2x2 {

public:

	LinearMapR2();
	LinearMapR2( const VectorR2&, const VectorR2& );	// Sets by columns!
	LinearMapR2( double, double, double, double );	// Sets by columns
	LinearMapR2 ( const Matrix2x2& );

	inline void Negate();
	inline LinearMapR2& operator+= (const Matrix2x2& );
	inline LinearMapR2& operator-= (const Matrix2x2& );
	inline LinearMapR2& operator*= (double);
	inline LinearMapR2& operator/= (double);
	inline LinearMapR2& operator*= (const Matrix2x2& );	// Matrix product

	inline LinearMapR2 Transpose() const;
	inline double Determinant () const;		// Returns the determinant
	LinearMapR2 Inverse() const;			// Returns inverse
	LinearMapR2& Invert();					// Converts into inverse.
	VectorR2 Solve(const VectorR2&) const;	// Returns solution
	LinearMapR2 PseudoInverse() const;		// Returns pseudo-inverse TO DO
	VectorR2 PseudoSolve(const VectorR2&);	// Finds least squares solution TO DO
};
	
inline LinearMapR2 operator+ ( const LinearMapR2&, const LinearMapR2&);
inline LinearMapR2 operator- ( const Matrix2x2& );
inline LinearMapR2 operator- ( const LinearMapR2&, const LinearMapR2&);
inline LinearMapR2 operator* ( const LinearMapR2&, double);
inline LinearMapR2 operator* ( double, const LinearMapR2& );
inline LinearMapR2 operator/ ( const LinearMapR2&, double );
inline LinearMapR2 operator* ( const Matrix2x2&, const LinearMapR2& ); 
inline LinearMapR2 operator* ( const LinearMapR2&, const Matrix2x2& ); 
inline LinearMapR2 operator* ( const LinearMapR2&, const LinearMapR2& ); 
								// Matrix product (composition)


// *******************************************
// RotationMapR2class                        *
// * * * * * * * * * * * * * * * * * * * * * *

class RotationMapR2 : public Matrix2x2 {

public:

	RotationMapR2();
	RotationMapR2( const VectorR2&, const VectorR2& );	// Sets by columns!
	RotationMapR2( double, double, double, double );	// Sets by columns!

	RotationMapR2& SetZero();	// IT IS AN ERROR TO USE THIS FUNCTION!

	inline RotationMapR2& operator*= (const RotationMapR2& );	// Matrix product

	inline RotationMapR2 Transpose() const;
	inline RotationMapR2 Inverse() const { return Transpose(); }; // Returns the transpose
	inline RotationMapR2& Invert() { MakeTranspose(); return *this; };	// Transposes it.
	inline VectorR2 Invert(const VectorR2&) const;	// Returns solution
};
	
inline RotationMapR2 operator* ( const RotationMapR2&, const RotationMapR2& ); 
										// Matrix product (composition)



// ***************************************************************
// * 2-space vector and matrix utilities (prototypes)			 *
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

// Returns the angle between vectors u and v.
//		Use AngleUnit if both vectors are unit vectors
inline double Angle( const VectorR2& u, const VectorR2& v);
inline double AngleUnit( const VectorR2& u, const VectorR2& v );

// Returns a righthanded orthonormal basis to complement vector  u
//		The vector u must be unit.
inline VectorR2 GetOrtho( const VectorR2& u );

// Projections

inline VectorR2 ProjectToUnit ( const VectorR2& u, const VectorR2& v); 
			// Project u onto v
inline VectorR2 ProjectPerpUnit ( const VectorR2& u, const VectorR2 & v); 
			// Project perp to v
// v must be a unit vector.

// Projection maps (LinearMapR2's)

inline LinearMapR2 VectorProjectMap( const VectorR2& u );

inline LinearMapR2 PerpProjectMap ( const VectorR2& u );
// u - must be unit vector.  

// Rotation Maps

inline RotationMapR2 RotateToMap( const VectorR2& fromVec, const VectorR2& toVec);
// fromVec and toVec should be unit vectors



// ***************************************************************
// * Stream Output Routines	(Prototypes)						 *
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

ostream& operator<< ( ostream& os, const VectorR2& u );


// *****************************************************
// * VectorR2 class - inlined functions				   *
// * * * * * * * * * * * * * * * * * * * * * * * * * * *

inline VectorR2& VectorR2::Load( const double* v ) 
{
	x = *v; 
	y = *(v+1);
	return *this;
}

inline VectorR2& VectorR2::Load( const float* v ) 
{
	x = *v; 
	y = *(v+1);
	return *this;
}

inline 	void VectorR2::Dump( double* v ) const
{
	*v = x; 
	*(v+1) = y;
}

inline 	void VectorR2::Dump( float* v ) const
{
	*v = (float)x; 
	*(v+1) = (float)y;
}

inline VectorR2& VectorR2::ArrayProd (const VectorR2& v)		// Component-wise Product
{
	x *= v.x;
	y *= v.y;
	return ( *this );
}

inline VectorR2& VectorR2::MakeUnit ()	 // Convert to unit vector (or leave zero).
{
	double nSq = NormSq();
	if (nSq != 0.0) {
		*this /= sqrt(nSq);
	}
	return *this;
}

inline VectorR2& VectorR2::ReNormalize()			// Convert near unit back to unit
{
	double nSq = NormSq();
	 double mFact = 1.0-0.5*(nSq-1.0);	// Multiplicative factor
	*this *= mFact;
	return *this;
}

// Rotate through angle theta
inline VectorR2& VectorR2::Rotate( double theta )
{
	double costheta = cos(theta);
	double sintheta = sin(theta);
	double tempx = x*costheta - y*sintheta;
	y = y*costheta + x*sintheta;
	x = tempx;
	return *this;
}

inline VectorR2& VectorR2::Rotate( double costheta, double sintheta )
{
	double tempx = x*costheta + y*sintheta;
	y = y*costheta - x*sintheta;
	x = tempx;
	return *this;
}

inline double VectorR2::MaxAbs() const
{
	 double m;
	m = (x>=0.0) ? x : -x;
	if ( y>m ) m=y;
	else if ( -y >m ) m = -y;
	return m;
}

inline VectorR2 operator+( const VectorR2& u, const VectorR2& v ) 
{ 
	return VectorR2(u.x+v.x, u.y+v.y ); 
}
inline VectorR2 operator-( const VectorR2& u, const VectorR2& v ) 
{ 
	return VectorR2(u.x-v.x, u.y-v.y ); 
}
inline VectorR2 operator*( const VectorR2& u,  double m) 
{ 
	return VectorR2( u.x*m, u.y*m ); 
}
inline VectorR2 operator*(  double m, const VectorR2& u) 
{ 
	return VectorR2( u.x*m, u.y*m ); 
}
inline VectorR2 operator/( const VectorR2& u, double m) 
{ 
	 double mInv = 1.0/m;
	return VectorR2( u.x*mInv, u.y*mInv ); 
}

inline int operator==( const VectorR2& u, const VectorR2& v ) 
{
	return ( u.x==v.x && u.y==v.y );
}

inline double operator^ ( const VectorR2& u, const VectorR2& v ) // Dot Product
{ 
	return ( u.x*v.x + u.y*v.y ); 
}

inline VectorR2 ArrayProd ( const VectorR2& u, const VectorR2& v )
{
	return ( VectorR2( u.x*v.x, u.y*v.y ) );
}

inline VectorR2& VectorR2::AddScaled( const VectorR2& u, double s ) 
{
	x += s*u.x;
	y += s*u.y;
	return(*this);
}

inline VectorR2::VectorR2( const VectorHgR2& uH ) 
: x(uH.x), y(uH.y)
{ 
	*this /= uH.w; 
}

inline double NormalizeError (const VectorR2& u)
{
	 double discrepancy;
	discrepancy = u.x*u.x + u.y*u.y - 1.0;
	if ( discrepancy < 0.0 ) {
		discrepancy = -discrepancy;
	}
	return discrepancy;
}

inline double VectorR2::Dist( const VectorR2& u ) const 	// Distance from u
{
	return sqrt( DistSq(u) );
}

inline double VectorR2::DistSq( const VectorR2& u ) const	// Distance from u
{
	return ( (x-u.x)*(x-u.x) + (y-u.y)*(y-u.y) );
}


// *********************************************************
// * Matrix2x2 class - inlined functions				   *
// * * * * * * * * * * * * * * * * * * * * * * * * * * *****

inline Matrix2x2::Matrix2x2() {}

inline Matrix2x2::Matrix2x2( const VectorR2& u, const VectorR2& v )
{
	m11 = u.x;		// Column 1
	m21 = u.y;
	m12 = v.x;		// Column 2
	m22 = v.y;
}

inline Matrix2x2::Matrix2x2( double a11, double a21, double a12, double a22 )
					// Values specified in column order!!!
{
	m11 = a11;		// Row 1
	m12 = a12;
	m21 = a21;		// Row 2
	m22 = a22;
}

inline void Matrix2x2::SetIdentity ( )
{
	m11 = m22 = 1.0;
	m12 = m21 = 0.0;
}

inline void Matrix2x2::Set( const VectorR2& u, const VectorR2& v )
{
	m11 = u.x;		// Column 1
	m21 = u.y;
	m12 = v.x;		// Column 2
	m22 = v.y;
}

inline void Matrix2x2::Set( double a11, double a21, double a12, double a22 )
					// Values specified in column order!!!
{
	m11 = a11;		// Row 1
	m12 = a12;
	m21 = a21;		// Row 2
	m22 = a22;
}

inline void Matrix2x2::SetZero( ) 
{
	m11 = m12 = m21 = m22 = 0.0;
}

inline void Matrix2x2::SetByRows( const VectorR2& u, const VectorR2& v )
{
	m11 = u.x;		// Row 1
	m12 = u.y;
	m21 = v.x;		// Row 2
	m22 = v.y;
}

inline void Matrix2x2::SetByRows( double a11, double a12, double a21, double a22 )
					// Values specified in row order!!!
{
	m11 = a11;		// Row 1
	m12 = a12;
	m21 = a21;		// Row 2
	m22 = a22;
}

inline void Matrix2x2::SetColumn1 ( double x, double y )
{
	m11 = x; m21 = y;
}

inline void Matrix2x2::SetColumn2 ( double x, double y )
{
	m12 = x; m22 = y;
}

inline void Matrix2x2::SetColumn1 ( const VectorR2& u )
{
	m11 = u.x; m21 = u.y;
}

inline void Matrix2x2::SetColumn2 ( const VectorR2& u )
{
	m12 = u.x; m22 = u.y;
}

VectorR2 Matrix2x2::Column1() const
{
	return ( VectorR2(m11, m21) );
}

VectorR2 Matrix2x2::Column2() const
{
	return ( VectorR2(m12, m22) );
}

inline void Matrix2x2::SetRow1 ( double x, double y )
{
	m11 = x; m12 = y;
}

inline void Matrix2x2::SetRow2 ( double x, double y )
{
	m21 = x; m22 = y;
}

inline void Matrix2x2::SetRow1 ( const VectorR2& u )
{
	m11 = u.x; m12 = u.y;
}

inline void Matrix2x2::SetRow2 ( const VectorR2& u )
{
	m21 = u.x; m22 = u.y;
}

VectorR2 Matrix2x2::Row1() const
{
	return ( VectorR2(m11, m12) );
}

VectorR2 Matrix2x2::Row2() const
{
	return ( VectorR2(m21, m22) );
}

inline void Matrix2x2::SetDiagonal( double x, double y )
{
	m11 = x;
	m22 = y;
}

inline void Matrix2x2::SetDiagonal( const VectorR2& u )
{
	SetDiagonal ( u.x, u.y );
}

inline double Matrix2x2::Diagonal( int i ) 
{
	switch (i) {
	case 0:
		return m11;
	case 1:
		return m22;
	default:
		assert(0);
		return 0.0;
	}
}
inline void Matrix2x2::MakeTranspose()	// Transposes it.
{
	 double temp;
	temp = m12;
	m12 = m21;
	m21=temp;
}

inline void Matrix2x2::operator*= (const Matrix2x2& B)	// Matrix product
{
	double t1;		// temporary value

	t1 =  m11*B.m11 + m12*B.m21;
	m12 = m11*B.m12 + m12*B.m22;
	m11 = t1;

	t1 =  m21*B.m11 + m22*B.m21;
	m22 = m21*B.m12 + m22*B.m22;
	m21 = t1;
}

inline Matrix2x2& Matrix2x2::ReNormalize()	// Re-normalizes nearly orthonormal matrix
{
	 double alpha = m11*m11+m21*m21;	// First column's norm squared
	 double beta  = m12*m12+m22*m22;	// Second column's norm squared
	alpha = 1.0 - 0.5*(alpha-1.0);				// Get mult. factor
	beta  = 1.0 - 0.5*(beta-1.0);
	m11 *= alpha;								// Renormalize first column
	m21 *= alpha;
	m12 *= beta;								// Renormalize second column
	m22 *= beta;
	alpha = m11*m12+m21*m22;					// Columns' inner product
	alpha *= 0.5;								//    times 1/2
	 double temp;
	temp = m11-alpha*m12;						// Subtract alpha times other column
	m12 -= alpha*m11;
	m11 = temp;
	temp = m21-alpha*m22;
	m22 -= alpha*m21;
	m11 = temp;
	return *this;
}

// Gives a measure of how far the matrix is from being normalized.
//		Mostly intended for diagnostic purposes.
inline double NormalizeError( const Matrix2x2& A)
{
	 double discrepancy;
	 double newdisc;
	discrepancy = A.m11*A.m11 + A.m21*A.m21 -1.0;	// First column - inner product - 1
	if (discrepancy<0.0) {
		discrepancy = -discrepancy;
	}
	newdisc = A.m12*A.m12 + A.m22*A.m22 - 1.0;		// Second column inner product - 1
	if ( newdisc<0.0 ) {
		newdisc = -newdisc;
	}
	if ( newdisc>discrepancy ) {
		discrepancy = newdisc;
	}
	newdisc = A.m11*A.m12 + A.m21*A.m22;			// Inner product of two columns
	if ( newdisc<0.0 ) {
		newdisc = -newdisc;
	}
	if ( newdisc>discrepancy ) {
		discrepancy = newdisc;
	}
	return discrepancy;
}

inline VectorR2 operator* ( const Matrix2x2& A, const VectorR2& u)
{
	return(VectorR2 ( A.m11*u.x + A.m12*u.y,
					  A.m21*u.x + A.m22*u.y ) ); 
}
	
inline void Matrix2x2::Transform( VectorR2* u ) const {
	double newX;
	newX = m11*u->x + m12*u->y;
	u->y = m21*u->x + m22*u->y;
	u->x = newX;
}

inline void Matrix2x2::Transform( const VectorR2& src, VectorR2* dest ) const {
	dest->x = m11*src.x + m12*src.y;
	dest->y = m21*src.x + m22*src.y;
}



// ******************************************************
// * LinearMapR2 class - inlined functions				*
// * * * * * * * * * * * * * * * * * * * * * * * * * * **

inline LinearMapR2::LinearMapR2()
{
	SetZero();
	return;
}	

inline LinearMapR2::LinearMapR2( const VectorR2& u, const VectorR2& v )
:Matrix2x2 ( u, v )
{ }

inline LinearMapR2::LinearMapR2(double a11, double a21, double a12, double a22)
					// Values specified in column order!!!
:Matrix2x2 ( a11, a21, a12, a22 )
{ }

inline LinearMapR2::LinearMapR2 ( const Matrix2x2& A )
: Matrix2x2 (A) 
{}

inline void LinearMapR2::Negate ()
{
	m11 = -m11;
	m12 = -m12;
	m21 = -m21;
	m22 = -m22;
}
	
inline LinearMapR2& LinearMapR2::operator+= (const Matrix2x2& B)
{
	m11 += B.m11;
	m12 += B.m12;
	m21 += B.m21;
	m22 += B.m22;
	return ( *this );
}

inline LinearMapR2& LinearMapR2::operator-= (const Matrix2x2& B)
{
	m11 -= B.m11;
	m12 -= B.m12;
	m21 -= B.m21;
	m22 -= B.m22;
	return( *this );
}

inline LinearMapR2 operator+ (const LinearMapR2& A, const LinearMapR2& B)
{
	return( LinearMapR2( A.m11+B.m11, A.m21+B.m21,
						 A.m12+B.m12, A.m22+B.m22 ) );
}

inline LinearMapR2 operator- (const Matrix2x2& A)
{
	return( LinearMapR2( -A.m11, -A.m21, -A.m12, -A.m22 ) );
}

inline LinearMapR2 operator- (const LinearMapR2& A, const LinearMapR2& B)
{
	return( LinearMapR2( A.m11-B.m11, A.m21-B.m21,
						 A.m12-B.m12, A.m22-B.m22 ) );
}

inline LinearMapR2& LinearMapR2::operator*= ( double b)
{
	m11 *= b;
	m12 *= b;
	m21 *= b;
	m22 *= b;
	return ( *this);
}

inline LinearMapR2 operator* ( const LinearMapR2& A,  double b)
{
	return( LinearMapR2( A.m11*b, A.m21*b,
						 A.m12*b, A.m22*b ) );
}

inline LinearMapR2 operator* (  double b, const LinearMapR2& A)
{
	return( LinearMapR2( A.m11*b, A.m21*b,
						 A.m12*b, A.m22*b ) );
}

inline LinearMapR2 operator/ ( const LinearMapR2& A, double b)
{
	 double bInv = 1.0/b;
	return ( A*bInv );
}

inline LinearMapR2& LinearMapR2::operator/= ( double b)
{
	 double bInv = 1.0/b;
	return ( *this *= bInv );
}

inline LinearMapR2 LinearMapR2::Transpose() const	// Returns the transpose
{
	return (LinearMapR2( m11, m12, m21, m22 ) );
}

inline LinearMapR2& LinearMapR2::operator*= (const Matrix2x2& B)	// Matrix product
{
	(*this).Matrix2x2::operator*=(B);

	return( *this );
}

inline LinearMapR2 operator* ( const LinearMapR2& A, const Matrix2x2& B)
{
	LinearMapR2 AA(A);
	AA.Matrix2x2::operator*=(B);
	return AA;
}

inline LinearMapR2 operator* ( const Matrix2x2& A, const LinearMapR2& B)
{
	LinearMapR2 AA(A);
	AA.Matrix2x2::operator*=(B);
	return AA;
}

inline LinearMapR2 operator* ( const LinearMapR2& A, const LinearMapR2& B)
{
	LinearMapR2 AA(A);
	AA.Matrix2x2::operator*=(B);
	return AA;
}

inline double LinearMapR2::Determinant () const		// Returns the determinant
{
	return ( m11*m22 - m12*m21 );
}

// ******************************************************
// * RotationMapR2 class - inlined functions			*
// * * * * * * * * * * * * * * * * * * * * * * * * * * **

inline RotationMapR2::RotationMapR2()
{
	SetIdentity();
	return;
}
	
inline RotationMapR2::RotationMapR2( const VectorR2& u, const VectorR2& v )
:Matrix2x2 ( u, v )
{ }

inline RotationMapR2::RotationMapR2( 
							 double a11, double a21, double a12, double a22 )
					// Values specified in column order!!!
:Matrix2x2 ( a11, a21, a12, a22 )
{ }

inline RotationMapR2 RotationMapR2::Transpose() const	// Returns the transpose
{
	return ( RotationMapR2( m11, m12,
							m21, m22 ) );
}

inline VectorR2 RotationMapR2::Invert(const VectorR2& u) const  // Returns solution
{
	return (VectorR2( m11*u.x + m21*u.y,			// Multiply with Transpose
					  m12*u.x + m22*u.y ) );
}

inline RotationMapR2& RotationMapR2::operator*= (const RotationMapR2& B)  // Matrix product
{
	(*this).Matrix2x2::operator*=(B);

	return( *this );
}

inline RotationMapR2 operator* ( const RotationMapR2& A, const RotationMapR2& B)
{
	RotationMapR2 AA(A);
	AA.Matrix2x2::operator*=(B);
	return AA;
}


// ***************************************************************
// * 2-space vector and matrix utilities (inlined functions)	 *
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

// Returns a righthanded orthonormal basis to complement vector  u
//		The vector u must be unit.
inline VectorR2 GetOrtho( const VectorR2& u )
{
	return VectorR2 ( -u.y, u.x );
}

// Returns the projection of u onto unit v
inline VectorR2 ProjectToUnit ( const VectorR2& u, const VectorR2& v)
{
	return (u^v)*v;
}

// Returns the projection of u onto the plane perpindicular to the unit vector v
inline VectorR2 ProjectPerpUnit ( const VectorR2& u, const VectorR2& v)
{
	return ( u - ((u^v)*v) );
}

// Returns the projection of u onto the plane perpindicular to the unit vector v
//    This one is more stable when u and v are nearly equal.
inline VectorR2 ProjectPerpUnitDiff ( const VectorR2& u, const VectorR2& v)
{
	VectorR2 ans = u;
	ans -= v;
	ans -= ((ans^v)*v);
	return ans;				// ans = (u-v) - ((u-v)^v)*v
}

// Returns the solid angle between vectors u and v.
inline double Angle( const VectorR2& u, const VectorR2& v)
{
	double nSqU = u.NormSq();
	double nSqV = v.NormSq();
	if ( nSqU==0.0 && nSqV==0.0 ) {
		return (0.0);
	}
	else {
		return ( AngleUnit( u/sqrt(nSqU), v/sqrt(nSqV) ) );
	}
}

inline double AngleUnit( const VectorR2& u, const VectorR2& v )
{
	return ( atan2 ( (ProjectPerpUnit(v,u)).Norm(), u^v ) );
}

// Projection maps (LinearMapR2's)

// VectorProjectMap returns map projecting onto a given vector u.
//		u should be a unit vector (otherwise the returned map is
//		scaled according to the magnitude of u.
inline LinearMapR2 VectorProjectMap( const VectorR2& u )
{
	double xy = u.x*u.y;
	return( LinearMapR2( u.x*u.x, xy, xy, u.y*u.y ) ) ;
}

// PlaneProjectMap returns map projecting onto a given plane.
//		The plane is the plane orthognal to u.
//		u must be a unit vector (otherwise the returned map is
//		garbage).
inline LinearMapR2 PerpProjectMap ( const VectorR2& u )
{
	double nxy = -u.x*u.y;
	return ( LinearMapR2 ( 1.0-u.x*u.x, nxy, nxy, 1.0-u.y*u.y ) );
}

// fromVec and toVec should be unit vectors
inline RotationMapR2 RotateToMap( const VectorR2& fromVec, const VectorR2& toVec)
{
	double costheta = fromVec.x*toVec.x + fromVec.y*toVec.y;
	double sintheta = fromVec.x*toVec.y - fromVec.y*toVec.x;
	return( RotationMapR2( costheta, sintheta, -sintheta, costheta ) );
}

#endif	// LINEAR_R2_H
