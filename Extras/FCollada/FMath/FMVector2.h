/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FMVector2.h
	The file containing the class and global functions for 2 dimensional 
	vectors.
*/

#ifndef _FM_VECTOR2_H_
#define _FM_VECTOR2_H_

/**
	A 2 dimensional vector.
	Not used within FCollada.
	
	@ingroup FMath
*/
class FCOLLADA_EXPORT FMVector2
{
public:
	float u;	/**< The first coordinate. */
	float v;	/**< The second coordinate. */

public:
	/**
	 * Creates an empty FMVector2.
	 */
	#ifndef _DEBUG
	FMVector2() {}
	#else
	FMVector2() { u = 123456789.0f; v = 123456789.0f; }
	#endif 

	/**
	 * Creates the FMVector2 with the coordinates given.
	 *
	 * @param _u The first coordinate.
	 * @param _v The second coordinate.
	 */
	FMVector2(float _u, float _v) { u = _u; v = _v; }

	/**
	 * Get this FMVector2 as an array of \c floats.
	 *
	 * @return The \c float array.
	 */
	inline operator float*() { return &u; }

	/**
	 * Adds two FMVector2.
	 *
	 * Adds to this FMVector2's coordinates the individual components of the
	 * given FMVector2 and returns this FMVector2.
	 *
	 * @param a The FMVector2 to add with this one.
	 * @return This FMVector2.
	 */
	inline FMVector2& operator +=(const FMVector2& a) { u += a.u; v += a.v; return *this; }

	/**
	 * Multiplies this FMVector2 by a scaler.
	 *
	 * Multiplies each of this FMVector2's coordinates with the scaler and
	 * returns this FMVector2.
	 *
	 * @param a The scalar to multiply with.
	 * @return This FMVector2.
	 */
	inline FMVector2& operator *=(float a) { u *= a; v *= a; return *this; }
	
	/**
	 * Assign this FMVector2 to the given float array.
	 *
	 * Assigns each coordinate of this FMVector2 to the elements in the \c
	 * float array. The first element to the first coordinate and the second to
	 * the second. It returns this FMVector2.
	 *
	 * @param f The \c float array to assign with.
	 * @return This FMVector2.
	 */
	inline FMVector2& operator =(const float* f) { u = *f; v = *(f + 1); return *this; }
};

/**
 * Vector addition with two FMVector2.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return The FMVector2 representation of the resulting vector.
 */
inline FMVector2 operator + (const FMVector2& a, const FMVector2& b) { return FMVector2(a.u + b.u, a.v + b.v); }

/**
 * Vector subtraction with two FMVector2.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return The FMVector2 representation of the resulting vector.
 */
inline FMVector2 operator -(const FMVector2& a, const FMVector2& b) { return FMVector2(a.u - b.u, a.v - b.v); }

/**
 * Dot product of two FMVector2.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return The result of the dot product.
 */
inline float operator *(const FMVector2& a, const FMVector2& b) { return a.u * b.u + a.v * b.v; }

/**
 * Scalar multiplication with a FMVector2.
 *
 * @param a The vector.
 * @param b The scalar.
 * @return The FMVector2 representing the resulting the vector.
 */
inline FMVector2 operator *(const FMVector2& a, float b) { return FMVector2(a.u * b, a.v * b); }

/**
 * Scalar multiplication with a FMVector2.
 *
 * @param a The scalar.
 * @param b The vector.
 * @return The FMVector2 representing the resulting the vector.
 */
inline FMVector2 operator *(float a, const FMVector2& b) { return FMVector2(a * b.u, a * b.v); }

#endif // _FM_VECTOR2_H_
