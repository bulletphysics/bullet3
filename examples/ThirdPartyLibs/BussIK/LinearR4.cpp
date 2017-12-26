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


#include "LinearR4.h"

#include <assert.h>

const VectorR4 VectorR4::Zero(0.0, 0.0, 0.0, 0.0);
const VectorR4 VectorR4::UnitX( 1.0, 0.0, 0.0, 0.0);
const VectorR4 VectorR4::UnitY( 0.0, 1.0, 0.0, 0.0);
const VectorR4 VectorR4::UnitZ( 0.0, 0.0, 1.0, 0.0);
const VectorR4 VectorR4::UnitW( 0.0, 0.0, 0.0, 1.0);
const VectorR4 VectorR4::NegUnitX(-1.0, 0.0, 0.0, 0.0);
const VectorR4 VectorR4::NegUnitY( 0.0,-1.0, 0.0, 0.0);
const VectorR4 VectorR4::NegUnitZ( 0.0, 0.0,-1.0, 0.0);
const VectorR4 VectorR4::NegUnitW( 0.0, 0.0, 0.0,-1.0);

const Matrix4x4 Matrix4x4::Identity(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
									0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0);


// ******************************************************
// * VectorR4 class - math library functions			*
// * * * * * * * * * * * * * * * * * * * * * * * * * * **

double VectorR4::MaxAbs() const
{
	 double m;
	m = (x>0.0) ? x : -x;
	if ( y>m ) m=y;
	else if ( -y >m ) m = -y;
	if ( z>m ) m=z;
	else if ( -z>m ) m = -z;
	if ( w>m ) m=w;
	else if ( -w>m ) m = -w;
	return m;
}

// ******************************************************
// * Matrix4x4 class - math library functions			*
// * * * * * * * * * * * * * * * * * * * * * * * * * * **


void Matrix4x4::operator*= (const Matrix4x4& B)	// Matrix product
{
	double t1, t2, t3;		// temporary values
	t1 =  m11*B.m11 + m12*B.m21 + m13*B.m31 + m14*B.m41;
	t2 =  m11*B.m12 + m12*B.m22 + m13*B.m32 + m14*B.m42;
	t3 =  m11*B.m13 + m12*B.m23 + m13*B.m33 + m14*B.m43;
	m14 = m11*B.m14 + m12*B.m24 + m13*B.m34 + m14*B.m44;
	m11 = t1;
	m12 = t2;
	m13 = t3;

	t1 =  m21*B.m11 + m22*B.m21 + m23*B.m31 + m24*B.m41;
	t2 =  m21*B.m12 + m22*B.m22 + m23*B.m32 + m24*B.m42;
	t3 =  m21*B.m13 + m22*B.m23 + m23*B.m33 + m24*B.m43;
	m24 = m21*B.m14 + m22*B.m24 + m23*B.m34 + m24*B.m44;
	m21 = t1;
	m22 = t2;
	m23 = t3;

	t1 =  m31*B.m11 + m32*B.m21 + m33*B.m31 + m34*B.m41;
	t2 =  m31*B.m12 + m32*B.m22 + m33*B.m32 + m34*B.m42;
	t3 =  m31*B.m13 + m32*B.m23 + m33*B.m33 + m34*B.m43;
	m34 = m31*B.m14 + m32*B.m24 + m33*B.m34 + m34*B.m44;
	m31 = t1;
	m32 = t2;
	m33 = t3;

	t1 =  m41*B.m11 + m42*B.m21 + m43*B.m31 + m44*B.m41;
	t2 =  m41*B.m12 + m42*B.m22 + m43*B.m32 + m44*B.m42;
	t3 =  m41*B.m13 + m42*B.m23 + m43*B.m33 + m44*B.m43;
	m44 = m41*B.m14 + m42*B.m24 + m43*B.m34 + m44*B.m44;
	m41 = t1;
	m42 = t2;
	m43 = t3;
}

inline void ReNormalizeHelper ( double &a, double &b, double &c, double &d )
{
	 double scaleF = a*a+b*b+c*c+d*d;		// Inner product of Vector-R4
	scaleF = 1.0-0.5*(scaleF-1.0);
	a *= scaleF;
	b *= scaleF;
	c *= scaleF;
	d *= scaleF;
}

Matrix4x4& Matrix4x4::ReNormalize() {
	ReNormalizeHelper( m11, m21, m31, m41 );	// Renormalize first column
	ReNormalizeHelper( m12, m22, m32, m42 );	// Renormalize second column
	ReNormalizeHelper( m13, m23, m33, m43 );	// Renormalize third column
	ReNormalizeHelper( m14, m24, m34, m44 );	// Renormalize fourth column
	double alpha = 0.5*(m11*m12 + m21*m22 + m31*m32 + m41*m42);	//1st and 2nd cols
	double beta  = 0.5*(m11*m13 + m21*m23 + m31*m33 + m41*m43);	//1st and 3rd cols
	double gamma = 0.5*(m11*m14 + m21*m24 + m31*m34 + m41*m44);	//1st and 4nd cols
	double delta = 0.5*(m12*m13 + m22*m23 + m32*m33 + m42*m43);	//2nd and 3rd cols
	double eps   = 0.5*(m12*m14 + m22*m24 + m32*m34 + m42*m44);	//2nd and 4nd cols
	double phi   = 0.5*(m13*m14 + m23*m24 + m33*m34 + m43*m44);	//3rd and 4nd cols
	double temp1, temp2, temp3;
	temp1 = m11 - alpha*m12 - beta*m13 - gamma*m14;
	temp2 = m12 - alpha*m11 - delta*m13 - eps*m14;
	temp3 = m13 - beta*m11 - delta*m12 - phi*m14;
	m14 -= (gamma*m11 + eps*m12 + phi*m13);
	m11 = temp1;
	m12 = temp2;
	m13 = temp3;
	temp1 = m21 - alpha*m22 - beta*m23 - gamma*m24;
	temp2 = m22 - alpha*m21 - delta*m23 - eps*m24;
	temp3 = m23 - beta*m21 - delta*m22 - phi*m24;
	m24 -= (gamma*m21 + eps*m22 + phi*m23);
	m21 = temp1;
	m22 = temp2;
	m23 = temp3;
	temp1 = m31 - alpha*m32 - beta*m33 - gamma*m34;
	temp2 = m32 - alpha*m31 - delta*m33 - eps*m34;
	temp3 = m33 - beta*m31 - delta*m32 - phi*m34;
	m34 -= (gamma*m31 + eps*m32 + phi*m33);
	m31 = temp1;
	m32 = temp2;
	m33 = temp3;
	temp1 = m41 - alpha*m42 - beta*m43 - gamma*m44;
	temp2 = m42 - alpha*m41 - delta*m43 - eps*m44;
	temp3 = m43 - beta*m41 - delta*m42 - phi*m44;
	m44 -= (gamma*m41 + eps*m42 + phi*m43);
	m41 = temp1;
	m42 = temp2;
	m43 = temp3;
	return *this;
}

// ******************************************************
// * LinearMapR4 class - math library functions			*
// * * * * * * * * * * * * * * * * * * * * * * * * * * **


double LinearMapR4::Determinant () const		// Returns the determinant
{
	double Tbt34C12 = m31*m42-m32*m41;		// 2x2 subdeterminants
	double Tbt34C13 = m31*m43-m33*m41;
	double Tbt34C14 = m31*m44-m34*m41;
	double Tbt34C23 = m32*m43-m33*m42;
	double Tbt34C24 = m32*m44-m34*m42;
	double Tbt34C34 = m33*m44-m34*m43;

	double sd11 = m22*Tbt34C34 - m23*Tbt34C24 + m24*Tbt34C23;	// 3x3 subdeterminants
	double sd12 = m21*Tbt34C34 - m23*Tbt34C14 + m24*Tbt34C13;
	double sd13 = m21*Tbt34C24 - m22*Tbt34C14 + m24*Tbt34C12;
	double sd14 = m21*Tbt34C23 - m22*Tbt34C13 + m23*Tbt34C12;

	return ( m11*sd11 - m12*sd12 + m13*sd13 - m14*sd14 );
}

LinearMapR4 LinearMapR4::Inverse() const			// Returns inverse
{

	double Tbt34C12 = m31*m42-m32*m41;		// 2x2 subdeterminants
	double Tbt34C13 = m31*m43-m33*m41;
	double Tbt34C14 = m31*m44-m34*m41;
	double Tbt34C23 = m32*m43-m33*m42;
	double Tbt34C24 = m32*m44-m34*m42;
	double Tbt34C34 = m33*m44-m34*m43;
	double Tbt24C12 = m21*m42-m22*m41;		// 2x2 subdeterminants
	double Tbt24C13 = m21*m43-m23*m41;
	double Tbt24C14 = m21*m44-m24*m41;
	double Tbt24C23 = m22*m43-m23*m42;
	double Tbt24C24 = m22*m44-m24*m42;
	double Tbt24C34 = m23*m44-m24*m43;
	double Tbt23C12 = m21*m32-m22*m31;		// 2x2 subdeterminants
	double Tbt23C13 = m21*m33-m23*m31;
	double Tbt23C14 = m21*m34-m24*m31;
	double Tbt23C23 = m22*m33-m23*m32;
	double Tbt23C24 = m22*m34-m24*m32;
	double Tbt23C34 = m23*m34-m24*m33;

	double sd11 = m22*Tbt34C34 - m23*Tbt34C24 + m24*Tbt34C23;	// 3x3 subdeterminants
	double sd12 = m21*Tbt34C34 - m23*Tbt34C14 + m24*Tbt34C13;
	double sd13 = m21*Tbt34C24 - m22*Tbt34C14 + m24*Tbt34C12;
	double sd14 = m21*Tbt34C23 - m22*Tbt34C13 + m23*Tbt34C12;
	double sd21 = m12*Tbt34C34 - m13*Tbt34C24 + m14*Tbt34C23;	// 3x3 subdeterminants
	double sd22 = m11*Tbt34C34 - m13*Tbt34C14 + m14*Tbt34C13;
	double sd23 = m11*Tbt34C24 - m12*Tbt34C14 + m14*Tbt34C12;
	double sd24 = m11*Tbt34C23 - m12*Tbt34C13 + m13*Tbt34C12;
	double sd31 = m12*Tbt24C34 - m13*Tbt24C24 + m14*Tbt24C23;	// 3x3 subdeterminants
	double sd32 = m11*Tbt24C34 - m13*Tbt24C14 + m14*Tbt24C13;
	double sd33 = m11*Tbt24C24 - m12*Tbt24C14 + m14*Tbt24C12;
	double sd34 = m11*Tbt24C23 - m12*Tbt24C13 + m13*Tbt24C12;
	double sd41 = m12*Tbt23C34 - m13*Tbt23C24 + m14*Tbt23C23;	// 3x3 subdeterminants
	double sd42 = m11*Tbt23C34 - m13*Tbt23C14 + m14*Tbt23C13;
	double sd43 = m11*Tbt23C24 - m12*Tbt23C14 + m14*Tbt23C12;
	double sd44 = m11*Tbt23C23 - m12*Tbt23C13 + m13*Tbt23C12;


	 double detInv = 1.0/(m11*sd11 - m12*sd12 + m13*sd13 - m14*sd14);

	return( LinearMapR4( sd11*detInv, -sd12*detInv, sd13*detInv, -sd14*detInv,
						 -sd21*detInv, sd22*detInv, -sd23*detInv, sd24*detInv,
						 sd31*detInv, -sd32*detInv, sd33*detInv, -sd34*detInv,
						 -sd41*detInv, sd42*detInv, -sd43*detInv, sd44*detInv ) );
}

LinearMapR4& LinearMapR4::Invert() 			// Converts into inverse.
{
	double Tbt34C12 = m31*m42-m32*m41;		// 2x2 subdeterminants
	double Tbt34C13 = m31*m43-m33*m41;
	double Tbt34C14 = m31*m44-m34*m41;
	double Tbt34C23 = m32*m43-m33*m42;
	double Tbt34C24 = m32*m44-m34*m42;
	double Tbt34C34 = m33*m44-m34*m43;
	double Tbt24C12 = m21*m42-m22*m41;		// 2x2 subdeterminants
	double Tbt24C13 = m21*m43-m23*m41;
	double Tbt24C14 = m21*m44-m24*m41;
	double Tbt24C23 = m22*m43-m23*m42;
	double Tbt24C24 = m22*m44-m24*m42;
	double Tbt24C34 = m23*m44-m24*m43;
	double Tbt23C12 = m21*m32-m22*m31;		// 2x2 subdeterminants
	double Tbt23C13 = m21*m33-m23*m31;
	double Tbt23C14 = m21*m34-m24*m31;
	double Tbt23C23 = m22*m33-m23*m32;
	double Tbt23C24 = m22*m34-m24*m32;
	double Tbt23C34 = m23*m34-m24*m33;

	double sd11 = m22*Tbt34C34 - m23*Tbt34C24 + m24*Tbt34C23;	// 3x3 subdeterminants
	double sd12 = m21*Tbt34C34 - m23*Tbt34C14 + m24*Tbt34C13;
	double sd13 = m21*Tbt34C24 - m22*Tbt34C14 + m24*Tbt34C12;
	double sd14 = m21*Tbt34C23 - m22*Tbt34C13 + m23*Tbt34C12;
	double sd21 = m12*Tbt34C34 - m13*Tbt34C24 + m14*Tbt34C23;	// 3x3 subdeterminants
	double sd22 = m11*Tbt34C34 - m13*Tbt34C14 + m14*Tbt34C13;
	double sd23 = m11*Tbt34C24 - m12*Tbt34C14 + m14*Tbt34C12;
	double sd24 = m11*Tbt34C23 - m12*Tbt34C13 + m13*Tbt34C12;
	double sd31 = m12*Tbt24C34 - m13*Tbt24C24 + m14*Tbt24C23;	// 3x3 subdeterminants
	double sd32 = m11*Tbt24C34 - m13*Tbt24C14 + m14*Tbt24C13;
	double sd33 = m11*Tbt24C24 - m12*Tbt24C14 + m14*Tbt24C12;
	double sd34 = m11*Tbt24C23 - m12*Tbt24C13 + m13*Tbt24C12;
	double sd41 = m12*Tbt23C34 - m13*Tbt23C24 + m14*Tbt23C23;	// 3x3 subdeterminants
	double sd42 = m11*Tbt23C34 - m13*Tbt23C14 + m14*Tbt23C13;
	double sd43 = m11*Tbt23C24 - m12*Tbt23C14 + m14*Tbt23C12;
	double sd44 = m11*Tbt23C23 - m12*Tbt23C13 + m13*Tbt23C12;

	 double detInv = 1.0/(m11*sd11 - m12*sd12 + m13*sd13 - m14*sd14);

	m11 = sd11*detInv;
	m12 = -sd21*detInv;
	m13 = sd31*detInv;
	m14 = -sd41*detInv;
	m21 = -sd12*detInv;
	m22 = sd22*detInv;
	m23 = -sd32*detInv;
	m24 = sd42*detInv;
	m31 = sd13*detInv;
	m32 = -sd23*detInv;
	m33 = sd33*detInv;
	m34 = -sd43*detInv;
	m41 = -sd14*detInv;
	m42 = sd24*detInv;
	m43 = -sd34*detInv;
	m44 = sd44*detInv;

	return ( *this );
}

VectorR4 LinearMapR4::Solve(const VectorR4& u) const	// Returns solution
{												
	// Just uses Inverse() for now.
	return ( Inverse()*u );
}

// ******************************************************
// * RotationMapR4 class - math library functions		*
// * * * * * * * * * * * * * * * * * * * * * * * * * * **


// ***************************************************************
// * 4-space vector and matrix utilities						 *
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

// Returns u * v^T
LinearMapR4 TimesTranspose( const VectorR4& u, const VectorR4& v)
{
	LinearMapR4 result;
	TimesTranspose( u, v, result );
	return result;
}

// The following routines are use to obtain 
// a righthanded orthonormal basis to complement vectors u,v,w.
//		The vectors u,v,w must be unit and orthonormal.
// The value is returned in "rotmat" with the first column(s) of
//    rotmat equal to u,v,w as appropriate.

void GetOrtho( const VectorR4& u,  RotationMapR4& rotmat )
{
	rotmat.SetColumn1(u);
	GetOrtho( 1, rotmat );
}

void GetOrtho( const VectorR4& u,  const VectorR4& v, RotationMapR4& rotmat )
{
	rotmat.SetColumn1(u);
	rotmat.SetColumn2(v);
	GetOrtho( 2, rotmat );
}

void GetOrtho( const VectorR4& u,  const VectorR4& v, const VectorR4& s,
					RotationMapR4& rotmat )
{
	rotmat.SetColumn1(u);
	rotmat.SetColumn2(v);
	rotmat.SetColumn3(s);
	GetOrtho( 3, rotmat );
}

// This final version of GetOrtho is mainly for internal use.
//    It uses a Gram-Schmidt procedure to extend a partial orthonormal
//    basis to a complete orthonormal basis.
//    j = number of columns of rotmat that have already been set.
void GetOrtho( int j, RotationMapR4& rotmat)
{
	if ( j==0 ) {
		rotmat.SetIdentity();
		return;
	}
	if ( j==1 ) {
		rotmat.SetColumn2( -rotmat.m21, rotmat.m11, -rotmat.m41, rotmat.m31 );
		j = 2;
	}

	assert ( rotmat.Column1().Norm()<1.0001 && 0.9999<rotmat.Column1().Norm()
			 && rotmat.Column1().Norm()<1.0001 && 0.9999<rotmat.Column1().Norm()
			 && (rotmat.Column1()^rotmat.Column2()) < 0.001
			 && (rotmat.Column1()^rotmat.Column2()) > -0.001 );

	// 2x2 subdeterminants in first 2 columns

	double d12 = rotmat.m11*rotmat.m22-rotmat.m12*rotmat.m21;	
	double d13 = rotmat.m11*rotmat.m32-rotmat.m12*rotmat.m31;
	double d14 = rotmat.m11*rotmat.m42-rotmat.m12*rotmat.m41;
	double d23 = rotmat.m21*rotmat.m32-rotmat.m22*rotmat.m31;
	double d24 = rotmat.m21*rotmat.m42-rotmat.m22*rotmat.m41;
	double d34 = rotmat.m31*rotmat.m42-rotmat.m32*rotmat.m41;
	VectorR4 vec3;

	if ( j==2 ) {
		if ( d12>0.4 || d12<-0.4 || d13>0.4 || d13<-0.4 
								 || d23>0.4 || d23<-0.4 ) {
			vec3.Set( d23, -d13, d12, 0.0);
		}
		else if ( d24>0.4 || d24<-0.4 || d14>0.4 || d14<-0.4 ) {
			vec3.Set( d24, -d14, 0.0, d12 );
		}
		else {
			vec3.Set( d34, 0.0, -d14, d13 );
		}
		vec3.Normalize();
		rotmat.SetColumn3(vec3);
	}

	// Do the final column

	rotmat.SetColumn4 (
				-rotmat.m23*d34 + rotmat.m33*d24 - rotmat.m43*d23,
				 rotmat.m13*d34 - rotmat.m33*d14 + rotmat.m43*d13,
				-rotmat.m13*d24 + rotmat.m23*d14 - rotmat.m43*d12,
				 rotmat.m13*d23 - rotmat.m23*d13 + rotmat.m33*d12 );

	assert ( 0.99 < (((LinearMapR4)rotmat)).Determinant()
			 && (((LinearMapR4)rotmat)).Determinant() < 1.01 );

}



// *********************************************************************
// Rotation routines												   *
// *********************************************************************

// Rotate unit vector x in the direction of "dir": length of dir is rotation angle.
//		x must be a unit vector.  dir must be perpindicular to x.
VectorR4& VectorR4::RotateUnitInDirection ( const VectorR4& dir)
{	
	assert ( this->Norm()<1.0001 && this->Norm()>0.9999 &&
				(dir^(*this))<0.0001 && (dir^(*this))>-0.0001 );

	double theta = dir.NormSq();
	if ( theta==0.0 ) {
		return *this;
	}
	else {
		theta = sqrt(theta);
		double costheta = cos(theta);
		double sintheta = sin(theta);
		VectorR4 dirUnit = dir/theta;
		*this = costheta*(*this) + sintheta*dirUnit;
		// this->NormalizeFast();
		return ( *this );
	}
}

// RotateToMap returns a RotationMapR4 that rotates fromVec to toVec,
//		leaving the orthogonal subspace fixed.
// fromVec and toVec should be unit vectors
RotationMapR4 RotateToMap( const VectorR4& fromVec, const VectorR4& toVec)
{
	LinearMapR4 result;		
	result.SetIdentity();
	LinearMapR4 temp;
	VectorR4 vPerp = ProjectPerpUnitDiff( toVec, fromVec );
	double sintheta = vPerp.Norm();		// theta = angle between toVec and fromVec
	VectorR4 vProj = toVec-vPerp;
	double costheta = vProj^fromVec;
	if ( sintheta == 0.0 ) {
		// The vectors either coincide (return identity) or directly oppose
		if ( costheta < 0.0 ) {
			result = -result;		// Vectors directly oppose: return -identity.
		}
	}
	else {
		vPerp /= sintheta;						// Normalize
		VectorProjectMap ( fromVec, temp );		// project in fromVec direction
		temp *= (costheta-1.0);		
		result += temp;
		VectorProjectMap ( vPerp, temp );		// Project in vPerp direction
		temp *= (costheta-1.0);
		result += temp;
		TimesTranspose ( vPerp, fromVec, temp ); // temp = (vPerp)*(fromVec^T)
		temp *= sintheta;
		result += temp;
		temp.MakeTranspose();
		result -= temp;							 // (-sintheta)*(fromVec)*(vPerp^T)
	}
	RotationMapR4 rotationResult;
	rotationResult.Set(result);				 // Make explicitly a RotationMapR4
	return rotationResult;
}


// ***************************************************************
//  Stream Output Routines										 *
// * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *

ostream& operator<< ( ostream& os, const VectorR4& u )
{
	return (os << "<" << u.x << "," << u.y << "," << u.z << "," << u.w << ">");
}


