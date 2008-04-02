/*
 *	ICE / OPCODE - Optimized Collision Detection
 * http://www.codercorner.com/Opcode.htm
 * 
 * Copyright (c) 2001-2008 Pierre Terdiman,  pierre@codercorner.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
/**
 *	Contains code for 3x3 matrices.
 *	\file		IceMatrix3x3.h
 *	\author		Pierre Terdiman
 *	\date		April, 4, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __ICEMATRIX3X3_H__
#define __ICEMATRIX3X3_H__

	// Forward declarations
	class Quat;

	#define	MATRIX3X3_EPSILON		(1.0e-7f)

	class ICEMATHS_API Matrix3x3
	{
		public:
		//! Empty constructor
		inline_					Matrix3x3()									{}
		//! Constructor from 9 values
		inline_					Matrix3x3(float m00, float m01, float m02, float m10, float m11, float m12, float m20, float m21, float m22)
								{
									m[0][0] = m00;	m[0][1] = m01;	m[0][2] = m02;
									m[1][0] = m10;	m[1][1] = m11;	m[1][2] = m12;
									m[2][0] = m20;	m[2][1] = m21;	m[2][2] = m22;
								}
		//! Copy constructor
		inline_					Matrix3x3(const Matrix3x3& mat)				{ CopyMemory(m, &mat.m, 9*sizeof(float));	}
		//! Destructor
		inline_					~Matrix3x3()								{}

		//! Assign values
		inline_	void			Set(float m00, float m01, float m02, float m10, float m11, float m12, float m20, float m21, float m22)
								{
									m[0][0] = m00;	m[0][1] = m01;	m[0][2] = m02;
									m[1][0] = m10;	m[1][1] = m11;	m[1][2] = m12;
									m[2][0] = m20;	m[2][1] = m21;	m[2][2] = m22;
								}

		//! Sets the scale from a Point. The point is put on the diagonal.
		inline_	void			SetScale(const Point& p)					{ m[0][0] = p.x;	m[1][1] = p.y;	m[2][2] = p.z;	}

		//! Sets the scale from floats. Values are put on the diagonal.
		inline_	void			SetScale(float sx, float sy, float sz)		{ m[0][0] = sx;		m[1][1] = sy;	m[2][2] = sz;	}

		//! Scales from a Point. Each row is multiplied by a component.
		inline_	void			Scale(const Point& p)
								{
									m[0][0] *= p.x;	m[0][1] *= p.x;	m[0][2] *= p.x;
									m[1][0] *= p.y;	m[1][1] *= p.y;	m[1][2] *= p.y;
									m[2][0] *= p.z;	m[2][1] *= p.z;	m[2][2] *= p.z;
								}

		//! Scales from floats. Each row is multiplied by a value.
		inline_	void			Scale(float sx, float sy, float sz)
								{
									m[0][0] *= sx;	m[0][1] *= sx;	m[0][2] *= sx;
									m[1][0] *= sy;	m[1][1] *= sy;	m[1][2] *= sy;
									m[2][0] *= sz;	m[2][1] *= sz;	m[2][2] *= sz;
								}

		//! Copy from a Matrix3x3
		inline_	void			Copy(const Matrix3x3& source)				{ CopyMemory(m, source.m, 9*sizeof(float));			}

		// Row-column access
		//! Returns a row.
		inline_	void			GetRow(const udword r, Point& p)	const	{ p.x = m[r][0];	p.y = m[r][1];	p.z = m[r][2];	}
		//! Returns a row.
		inline_	const Point&	GetRow(const udword r)				const	{ return *(const Point*)&m[r][0];	}
		//! Returns a row.
		inline_	Point&			GetRow(const udword r)						{ return *(Point*)&m[r][0];			}
		//! Sets a row.
		inline_	void			SetRow(const udword r, const Point& p)		{ m[r][0] = p.x;	m[r][1] = p.y;	m[r][2] = p.z;	}
		//! Returns a column.
		inline_	void			GetCol(const udword c, Point& p)	const	{ p.x = m[0][c];	p.y = m[1][c];	p.z = m[2][c];	}
		//! Sets a column.
		inline_	void			SetCol(const udword c, const Point& p)		{ m[0][c] = p.x;	m[1][c] = p.y;	m[2][c] = p.z;	}

		//! Computes the trace. The trace is the sum of the 3 diagonal components.
		inline_	float			Trace()					const				{ return m[0][0] + m[1][1] + m[2][2];				}
		//! Clears the matrix.
		inline_	void			Zero()										{ ZeroMemory(&m, sizeof(m));						}
		//! Sets the identity matrix.
		inline_	void			Identity()									{ Zero(); m[0][0] = m[1][1] = m[2][2] = 1.0f; 		}
		//! Checks for identity
		inline_	bool			IsIdentity()			const
								{
									if(IR(m[0][0])!=IEEE_1_0)	return false;
									if(IR(m[0][1])!=0)			return false;
									if(IR(m[0][2])!=0)			return false;

									if(IR(m[1][0])!=0)			return false;
									if(IR(m[1][1])!=IEEE_1_0)	return false;
									if(IR(m[1][2])!=0)			return false;

									if(IR(m[2][0])!=0)			return false;
									if(IR(m[2][1])!=0)			return false;
									if(IR(m[2][2])!=IEEE_1_0)	return false;

									return true;
								}

		//! Checks matrix validity
		inline_	BOOL			IsValid()				const
								{
									for(udword j=0;j<3;j++)
									{
										for(udword i=0;i<3;i++)
										{
											if(!IsValidFloat(m[j][i]))	return FALSE;
										}
									}
									return TRUE;
								}

		//! Makes a skew-symmetric matrix (a.k.a. Star(*) Matrix)
		//!	[  0.0  -a.z   a.y ]
		//!	[  a.z   0.0  -a.x ]
		//!	[ -a.y   a.x   0.0 ]
		//! This is also called a "cross matrix" since for any vectors A and B,
		//! A^B = Skew(A) * B = - B * Skew(A);
		inline_	void			SkewSymmetric(const Point& a)
								{
									m[0][0] = 0.0f;
									m[0][1] = -a.z;
									m[0][2] = a.y;

									m[1][0] = a.z;
									m[1][1] = 0.0f;
									m[1][2] = -a.x;

									m[2][0] = -a.y;
									m[2][1] = a.x;
									m[2][2] = 0.0f;
								}

		//! Negates the matrix
		inline_	void			Neg()
								{
									m[0][0] = -m[0][0];	m[0][1] = -m[0][1];	m[0][2] = -m[0][2];
									m[1][0] = -m[1][0];	m[1][1] = -m[1][1];	m[1][2] = -m[1][2];
									m[2][0] = -m[2][0];	m[2][1] = -m[2][1];	m[2][2] = -m[2][2];
								}

		//! Neg from another matrix
		inline_	void			Neg(const Matrix3x3& mat)
								{
									m[0][0] = -mat.m[0][0];	m[0][1] = -mat.m[0][1];	m[0][2] = -mat.m[0][2];
									m[1][0] = -mat.m[1][0];	m[1][1] = -mat.m[1][1];	m[1][2] = -mat.m[1][2];
									m[2][0] = -mat.m[2][0];	m[2][1] = -mat.m[2][1];	m[2][2] = -mat.m[2][2];
								}

		//! Add another matrix
		inline_	void			Add(const Matrix3x3& mat)
								{
									m[0][0] += mat.m[0][0];	m[0][1] += mat.m[0][1];	m[0][2] += mat.m[0][2];
									m[1][0] += mat.m[1][0];	m[1][1] += mat.m[1][1];	m[1][2] += mat.m[1][2];
									m[2][0] += mat.m[2][0];	m[2][1] += mat.m[2][1];	m[2][2] += mat.m[2][2];
								}

		//! Sub another matrix
		inline_	void			Sub(const Matrix3x3& mat)
								{
									m[0][0] -= mat.m[0][0];	m[0][1]	-= mat.m[0][1];	m[0][2] -= mat.m[0][2];
									m[1][0] -= mat.m[1][0];	m[1][1] -= mat.m[1][1];	m[1][2] -= mat.m[1][2];
									m[2][0] -= mat.m[2][0];	m[2][1] -= mat.m[2][1];	m[2][2] -= mat.m[2][2];
								}
		//! Mac
		inline_	void			Mac(const Matrix3x3& a, const Matrix3x3& b, float s)
								{
									m[0][0] = a.m[0][0] + b.m[0][0] * s;
									m[0][1] = a.m[0][1] + b.m[0][1] * s;
									m[0][2] = a.m[0][2] + b.m[0][2] * s;

									m[1][0] = a.m[1][0] + b.m[1][0] * s;
									m[1][1] = a.m[1][1] + b.m[1][1] * s;
									m[1][2] = a.m[1][2] + b.m[1][2] * s;

									m[2][0] = a.m[2][0] + b.m[2][0] * s;
									m[2][1] = a.m[2][1] + b.m[2][1] * s;
									m[2][2] = a.m[2][2] + b.m[2][2] * s;
								}
		//! Mac
		inline_	void			Mac(const Matrix3x3& a, float s)
								{
									m[0][0] += a.m[0][0] * s;	m[0][1] += a.m[0][1] * s;	m[0][2] += a.m[0][2] * s;
									m[1][0] += a.m[1][0] * s;	m[1][1] += a.m[1][1] * s;	m[1][2] += a.m[1][2] * s;
									m[2][0] += a.m[2][0] * s;	m[2][1] += a.m[2][1] * s;	m[2][2] += a.m[2][2] * s;
								}

		//! this = A * s
		inline_	void			Mult(const Matrix3x3& a, float s)
								{
									m[0][0] = a.m[0][0] * s;	m[0][1] = a.m[0][1] * s;	m[0][2] = a.m[0][2] * s;
									m[1][0] = a.m[1][0] * s;	m[1][1] = a.m[1][1] * s;	m[1][2] = a.m[1][2] * s;
									m[2][0] = a.m[2][0] * s;	m[2][1] = a.m[2][1] * s;	m[2][2] = a.m[2][2] * s;
								}

		inline_	void			Add(const Matrix3x3& a, const Matrix3x3& b)
								{
									m[0][0] = a.m[0][0] + b.m[0][0];	m[0][1] = a.m[0][1] + b.m[0][1];	m[0][2] = a.m[0][2] + b.m[0][2];
									m[1][0] = a.m[1][0] + b.m[1][0];	m[1][1] = a.m[1][1] + b.m[1][1];	m[1][2] = a.m[1][2] + b.m[1][2];
									m[2][0] = a.m[2][0] + b.m[2][0];	m[2][1] = a.m[2][1] + b.m[2][1];	m[2][2] = a.m[2][2] + b.m[2][2];
								}

		inline_	void			Sub(const Matrix3x3& a, const Matrix3x3& b)
								{
									m[0][0] = a.m[0][0] - b.m[0][0];	m[0][1] = a.m[0][1] - b.m[0][1];	m[0][2] = a.m[0][2] - b.m[0][2];
									m[1][0] = a.m[1][0] - b.m[1][0];	m[1][1] = a.m[1][1] - b.m[1][1];	m[1][2] = a.m[1][2] - b.m[1][2];
									m[2][0] = a.m[2][0] - b.m[2][0];	m[2][1] = a.m[2][1] - b.m[2][1];	m[2][2] = a.m[2][2] - b.m[2][2];
								}

		//! this = a * b
		inline_	void			Mult(const Matrix3x3& a, const Matrix3x3& b)
								{
									m[0][0] = a.m[0][0] * b.m[0][0] + a.m[0][1] * b.m[1][0] + a.m[0][2] * b.m[2][0];
									m[0][1] = a.m[0][0] * b.m[0][1] + a.m[0][1] * b.m[1][1] + a.m[0][2] * b.m[2][1];
									m[0][2] = a.m[0][0] * b.m[0][2] + a.m[0][1] * b.m[1][2] + a.m[0][2] * b.m[2][2];
									m[1][0] = a.m[1][0] * b.m[0][0] + a.m[1][1] * b.m[1][0] + a.m[1][2] * b.m[2][0];
									m[1][1] = a.m[1][0] * b.m[0][1] + a.m[1][1] * b.m[1][1] + a.m[1][2] * b.m[2][1];
									m[1][2] = a.m[1][0] * b.m[0][2] + a.m[1][1] * b.m[1][2] + a.m[1][2] * b.m[2][2];
									m[2][0] = a.m[2][0] * b.m[0][0] + a.m[2][1] * b.m[1][0] + a.m[2][2] * b.m[2][0];
									m[2][1] = a.m[2][0] * b.m[0][1] + a.m[2][1] * b.m[1][1] + a.m[2][2] * b.m[2][1];
									m[2][2] = a.m[2][0] * b.m[0][2] + a.m[2][1] * b.m[1][2] + a.m[2][2] * b.m[2][2];
								}

		//! this = transpose(a) * b
		inline_	void			MultAtB(const Matrix3x3& a, const Matrix3x3& b)
								{
									m[0][0] = a.m[0][0] * b.m[0][0] + a.m[1][0] * b.m[1][0] + a.m[2][0] * b.m[2][0];
									m[0][1] = a.m[0][0] * b.m[0][1] + a.m[1][0] * b.m[1][1] + a.m[2][0] * b.m[2][1];
									m[0][2] = a.m[0][0] * b.m[0][2] + a.m[1][0] * b.m[1][2] + a.m[2][0] * b.m[2][2];
									m[1][0] = a.m[0][1] * b.m[0][0] + a.m[1][1] * b.m[1][0] + a.m[2][1] * b.m[2][0];
									m[1][1] = a.m[0][1] * b.m[0][1] + a.m[1][1] * b.m[1][1] + a.m[2][1] * b.m[2][1];
									m[1][2] = a.m[0][1] * b.m[0][2] + a.m[1][1] * b.m[1][2] + a.m[2][1] * b.m[2][2];
									m[2][0] = a.m[0][2] * b.m[0][0] + a.m[1][2] * b.m[1][0] + a.m[2][2] * b.m[2][0];
									m[2][1] = a.m[0][2] * b.m[0][1] + a.m[1][2] * b.m[1][1] + a.m[2][2] * b.m[2][1];
									m[2][2] = a.m[0][2] * b.m[0][2] + a.m[1][2] * b.m[1][2] + a.m[2][2] * b.m[2][2];
								}

		//! this = a * transpose(b)
		inline_	void			MultABt(const Matrix3x3& a, const Matrix3x3& b)
								{
									m[0][0] = a.m[0][0] * b.m[0][0] + a.m[0][1] * b.m[0][1] + a.m[0][2] * b.m[0][2];
									m[0][1] = a.m[0][0] * b.m[1][0] + a.m[0][1] * b.m[1][1] + a.m[0][2] * b.m[1][2];
									m[0][2] = a.m[0][0] * b.m[2][0] + a.m[0][1] * b.m[2][1] + a.m[0][2] * b.m[2][2];
									m[1][0] = a.m[1][0] * b.m[0][0] + a.m[1][1] * b.m[0][1] + a.m[1][2] * b.m[0][2];
									m[1][1] = a.m[1][0] * b.m[1][0] + a.m[1][1] * b.m[1][1] + a.m[1][2] * b.m[1][2];
									m[1][2] = a.m[1][0] * b.m[2][0] + a.m[1][1] * b.m[2][1] + a.m[1][2] * b.m[2][2];
									m[2][0] = a.m[2][0] * b.m[0][0] + a.m[2][1] * b.m[0][1] + a.m[2][2] * b.m[0][2];
									m[2][1] = a.m[2][0] * b.m[1][0] + a.m[2][1] * b.m[1][1] + a.m[2][2] * b.m[1][2];
									m[2][2] = a.m[2][0] * b.m[2][0] + a.m[2][1] * b.m[2][1] + a.m[2][2] * b.m[2][2];
								}

		//! Makes a rotation matrix mapping vector "from" to vector "to".
				Matrix3x3&		FromTo(const Point& from, const Point& to);

		//! Set a rotation matrix around the X axis.
		//!		 1		0		0
		//!	RX = 0		cx		sx
		//!		 0		-sx		cx
				void			RotX(float angle);
		//! Set a rotation matrix around the Y axis.
		//!		 cy		0		-sy
		//!	RY = 0		1		0
		//!		 sy		0		cy
				void			RotY(float angle);
		//! Set a rotation matrix around the Z axis.
		//!		 cz		sz		0
		//!	RZ = -sz	cz		0
		//!		 0		0		1
				void			RotZ(float angle);
		//!			cy		sx.sy		-sy.cx
		//!	RY.RX	0		cx			sx
		//!			sy		-sx.cy		cx.cy
				void			RotYX(float y, float x);

		//! Make a rotation matrix about an arbitrary axis
				Matrix3x3&		Rot(float angle, const Point& axis);

		//! Transpose the matrix.
				void			Transpose()
								{
									IR(m[1][0]) ^= IR(m[0][1]);	IR(m[0][1]) ^= IR(m[1][0]);	IR(m[1][0]) ^= IR(m[0][1]);
									IR(m[2][0]) ^= IR(m[0][2]);	IR(m[0][2]) ^= IR(m[2][0]);	IR(m[2][0]) ^= IR(m[0][2]);
									IR(m[2][1]) ^= IR(m[1][2]);	IR(m[1][2]) ^= IR(m[2][1]);	IR(m[2][1]) ^= IR(m[1][2]);
								}

		//! this = Transpose(a)
				void			Transpose(const Matrix3x3& a)
								{
									m[0][0] = a.m[0][0];	m[0][1] = a.m[1][0];	m[0][2] = a.m[2][0];
									m[1][0] = a.m[0][1];	m[1][1] = a.m[1][1];	m[1][2] = a.m[2][1];
									m[2][0] = a.m[0][2];	m[2][1] = a.m[1][2];	m[2][2] = a.m[2][2];
								}

		//! Compute the determinant of the matrix. We use the rule of Sarrus.
				float			Determinant()					const
								{
									return (m[0][0]*m[1][1]*m[2][2] + m[0][1]*m[1][2]*m[2][0] + m[0][2]*m[1][0]*m[2][1])
										-  (m[2][0]*m[1][1]*m[0][2] + m[2][1]*m[1][2]*m[0][0] + m[2][2]*m[1][0]*m[0][1]);
								}
/*
		//! Compute a cofactor. Used for matrix inversion.
				float			CoFactor(ubyte row, ubyte column)	const
				{
					static sdword gIndex[3+2] = { 0, 1, 2, 0, 1 };
					return	(m[gIndex[row+1]][gIndex[column+1]]*m[gIndex[row+2]][gIndex[column+2]] - m[gIndex[row+2]][gIndex[column+1]]*m[gIndex[row+1]][gIndex[column+2]]);
				}
*/
		//! Invert the matrix. Determinant must be different from zero, else matrix can't be inverted.
				Matrix3x3&		Invert()
								{
									float Det = Determinant();	// Must be !=0
									float OneOverDet = 1.0f / Det;

									Matrix3x3 Temp;
									Temp.m[0][0] = +(m[1][1] * m[2][2] - m[2][1] * m[1][2]) * OneOverDet;
									Temp.m[1][0] = -(m[1][0] * m[2][2] - m[2][0] * m[1][2]) * OneOverDet;
									Temp.m[2][0] = +(m[1][0] * m[2][1] - m[2][0] * m[1][1]) * OneOverDet;
									Temp.m[0][1] = -(m[0][1] * m[2][2] - m[2][1] * m[0][2]) * OneOverDet;
									Temp.m[1][1] = +(m[0][0] * m[2][2] - m[2][0] * m[0][2]) * OneOverDet;
									Temp.m[2][1] = -(m[0][0] * m[2][1] - m[2][0] * m[0][1]) * OneOverDet;
									Temp.m[0][2] = +(m[0][1] * m[1][2] - m[1][1] * m[0][2]) * OneOverDet;
									Temp.m[1][2] = -(m[0][0] * m[1][2] - m[1][0] * m[0][2]) * OneOverDet;
									Temp.m[2][2] = +(m[0][0] * m[1][1] - m[1][0] * m[0][1]) * OneOverDet;

									*this = Temp;

									return	*this;
								}

				Matrix3x3&		Normalize();

		//! this = exp(a)
				Matrix3x3&		Exp(const Matrix3x3& a);

void FromQuat(const Quat &q);
void FromQuatL2(const Quat &q, float l2);

		// Arithmetic operators
		//! Operator for Matrix3x3 Plus = Matrix3x3 + Matrix3x3;
		inline_	Matrix3x3		operator+(const Matrix3x3& mat)	const
								{
									return Matrix3x3(
									m[0][0] + mat.m[0][0],	m[0][1] + mat.m[0][1],	m[0][2] + mat.m[0][2],
									m[1][0] + mat.m[1][0],	m[1][1] + mat.m[1][1],	m[1][2] + mat.m[1][2],
									m[2][0] + mat.m[2][0],	m[2][1] + mat.m[2][1],	m[2][2] + mat.m[2][2]);
								}

		//! Operator for Matrix3x3 Minus = Matrix3x3 - Matrix3x3;
		inline_	Matrix3x3		operator-(const Matrix3x3& mat)	const
								{
									return Matrix3x3(
									m[0][0] - mat.m[0][0],	m[0][1] - mat.m[0][1],	m[0][2] - mat.m[0][2],
									m[1][0] - mat.m[1][0],	m[1][1] - mat.m[1][1],	m[1][2] - mat.m[1][2],
									m[2][0] - mat.m[2][0],	m[2][1] - mat.m[2][1],	m[2][2] - mat.m[2][2]);
								}

		//! Operator for Matrix3x3 Mul = Matrix3x3 * Matrix3x3;
		inline_	Matrix3x3		operator*(const Matrix3x3& mat)	const
								{
									return Matrix3x3(
									m[0][0]*mat.m[0][0] + m[0][1]*mat.m[1][0] + m[0][2]*mat.m[2][0],
									m[0][0]*mat.m[0][1] + m[0][1]*mat.m[1][1] + m[0][2]*mat.m[2][1],
									m[0][0]*mat.m[0][2] + m[0][1]*mat.m[1][2] + m[0][2]*mat.m[2][2],

									m[1][0]*mat.m[0][0] + m[1][1]*mat.m[1][0] + m[1][2]*mat.m[2][0],
									m[1][0]*mat.m[0][1] + m[1][1]*mat.m[1][1] + m[1][2]*mat.m[2][1],
									m[1][0]*mat.m[0][2] + m[1][1]*mat.m[1][2] + m[1][2]*mat.m[2][2],

									m[2][0]*mat.m[0][0] + m[2][1]*mat.m[1][0] + m[2][2]*mat.m[2][0],
									m[2][0]*mat.m[0][1] + m[2][1]*mat.m[1][1] + m[2][2]*mat.m[2][1],
									m[2][0]*mat.m[0][2] + m[2][1]*mat.m[1][2] + m[2][2]*mat.m[2][2]);
								}

		//! Operator for Point Mul = Matrix3x3 * Point;
		inline_	Point			operator*(const Point& v)		const		{ return Point(GetRow(0)|v, GetRow(1)|v, GetRow(2)|v); }

		//! Operator for Matrix3x3 Mul = Matrix3x3 * float;
		inline_	Matrix3x3		operator*(float s)				const
								{
									return Matrix3x3(
									m[0][0]*s,	m[0][1]*s,	m[0][2]*s,
									m[1][0]*s,	m[1][1]*s,	m[1][2]*s,
									m[2][0]*s,	m[2][1]*s,	m[2][2]*s);
								}

		//! Operator for Matrix3x3 Mul = float * Matrix3x3;
		inline_	friend Matrix3x3 operator*(float s, const Matrix3x3& mat)
								{
									return Matrix3x3(
									s*mat.m[0][0],	s*mat.m[0][1],	s*mat.m[0][2],
									s*mat.m[1][0],	s*mat.m[1][1],	s*mat.m[1][2],
									s*mat.m[2][0],	s*mat.m[2][1],	s*mat.m[2][2]);
								}

		//! Operator for Matrix3x3 Div = Matrix3x3 / float;
		inline_	Matrix3x3		operator/(float s)				const
								{
									if (s)	s = 1.0f / s;
									return Matrix3x3(
									m[0][0]*s,	m[0][1]*s,	m[0][2]*s,
									m[1][0]*s,	m[1][1]*s,	m[1][2]*s,
									m[2][0]*s,	m[2][1]*s,	m[2][2]*s);
								}

		//! Operator for Matrix3x3 Div = float / Matrix3x3;
		inline_	friend Matrix3x3 operator/(float s, const Matrix3x3& mat)
								{
									return Matrix3x3(
									s/mat.m[0][0],	s/mat.m[0][1],	s/mat.m[0][2],
									s/mat.m[1][0],	s/mat.m[1][1],	s/mat.m[1][2],
									s/mat.m[2][0],	s/mat.m[2][1],	s/mat.m[2][2]);
								}

		//! Operator for Matrix3x3 += Matrix3x3
		inline_	Matrix3x3&		operator+=(const Matrix3x3& mat)
								{
									m[0][0] += mat.m[0][0];		m[0][1] += mat.m[0][1];		m[0][2] += mat.m[0][2];
									m[1][0] += mat.m[1][0];		m[1][1] += mat.m[1][1];		m[1][2] += mat.m[1][2];
									m[2][0] += mat.m[2][0];		m[2][1] += mat.m[2][1];		m[2][2] += mat.m[2][2];
									return	*this;
								}

		//! Operator for Matrix3x3 -= Matrix3x3
		inline_	Matrix3x3&		operator-=(const Matrix3x3& mat)
								{
									m[0][0] -= mat.m[0][0];		m[0][1] -= mat.m[0][1];		m[0][2] -= mat.m[0][2];
									m[1][0] -= mat.m[1][0];		m[1][1] -= mat.m[1][1];		m[1][2] -= mat.m[1][2];
									m[2][0] -= mat.m[2][0];		m[2][1] -= mat.m[2][1];		m[2][2] -= mat.m[2][2];
									return	*this;
								}

		//! Operator for Matrix3x3 *= Matrix3x3
		inline_	Matrix3x3&		operator*=(const Matrix3x3& mat)
								{
									Point TempRow;

									GetRow(0, TempRow);
									m[0][0] = TempRow.x*mat.m[0][0] + TempRow.y*mat.m[1][0] + TempRow.z*mat.m[2][0];
									m[0][1] = TempRow.x*mat.m[0][1] + TempRow.y*mat.m[1][1] + TempRow.z*mat.m[2][1];
									m[0][2] = TempRow.x*mat.m[0][2] + TempRow.y*mat.m[1][2] + TempRow.z*mat.m[2][2];

									GetRow(1, TempRow);
									m[1][0] = TempRow.x*mat.m[0][0] + TempRow.y*mat.m[1][0] + TempRow.z*mat.m[2][0];
									m[1][1] = TempRow.x*mat.m[0][1] + TempRow.y*mat.m[1][1] + TempRow.z*mat.m[2][1];
									m[1][2] = TempRow.x*mat.m[0][2] + TempRow.y*mat.m[1][2] + TempRow.z*mat.m[2][2];

									GetRow(2, TempRow);
									m[2][0] = TempRow.x*mat.m[0][0] + TempRow.y*mat.m[1][0] + TempRow.z*mat.m[2][0];
									m[2][1] = TempRow.x*mat.m[0][1] + TempRow.y*mat.m[1][1] + TempRow.z*mat.m[2][1];
									m[2][2] = TempRow.x*mat.m[0][2] + TempRow.y*mat.m[1][2] + TempRow.z*mat.m[2][2];
									return	*this;
								}

		//! Operator for Matrix3x3 *= float
		inline_	Matrix3x3&		operator*=(float s)
								{
									m[0][0] *= s;	m[0][1] *= s;	m[0][2] *= s;
									m[1][0] *= s;	m[1][1] *= s;	m[1][2] *= s;
									m[2][0] *= s;	m[2][1] *= s;	m[2][2] *= s;
									return	*this;
								}

		//! Operator for Matrix3x3 /= float
		inline_	Matrix3x3&		operator/=(float s)
								{
									if (s)	s = 1.0f / s;
									m[0][0] *= s;	m[0][1] *= s;	m[0][2] *= s;
									m[1][0] *= s;	m[1][1] *= s;	m[1][2] *= s;
									m[2][0] *= s;	m[2][1] *= s;	m[2][2] *= s;
									return	*this;
								}

		// Cast operators
		//! Cast a Matrix3x3 to a Matrix4x4.
								operator Matrix4x4()	const;
		//! Cast a Matrix3x3 to a Quat.
								operator Quat()			const;

		inline_	const Point&	operator[](int row)		const	{ return *(const Point*)&m[row][0];	}
		inline_	Point&			operator[](int row)				{ return *(Point*)&m[row][0];		}

		public:

				float			m[3][3];
	};

#endif // __ICEMATRIX3X3_H__

