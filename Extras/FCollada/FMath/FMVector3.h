/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FMVector3.h The file containing the class and global functions for 3 dimensional vectors.
*/

#ifndef _FM_VECTOR3_H_
#define _FM_VECTOR3_H_

/**
	A 3 dimensional vector.

	Simple, non-optimized vector class: * is the dot-product, ^ is the 
	cross-product.
	
	@ingroup FMath
*/
class FCOLLADA_EXPORT FMVector3
{
public:
	float x;	/**< The first coordinate. */
	float y;	/**< The second coordinate. */
	float z;	/**< The third coordinate. */

public:
	/**
	 * Creates an empty FMVector3.
	 */
	#ifndef _DEBUG
	FMVector3() {}
	#else
	FMVector3() { x = 123456789.0f; y = 123456789.0f; z = 123456789.0f; }
	#endif 

	/**
	 * Creates the FMVector3 with the coordinates given.
	 *
	 * @param _x The first coordinate.
	 * @param _y The second coordinate.
	 * @param _z The third coordinate.
	 */
	FMVector3(float _x, float _y, float _z) { x = _x; y = _y; z = _z; }

	/**
	 * Creates the FMVector3 from a list of \c floats.
	 *
	 * It takes the first 3 \c floats starting from and including \a startIndex
	 * (0 indexing) in the array as the 3 coordinates. The first as the first 
	 * coordinate, the second as the second, and the third as the third.
	 *
	 * @param source The \c float array.
	 * @param startIndex The index of the first element.
	 */
	FMVector3(const float* source, uint32 startIndex = 0);

	/**
	 * Get the squared length.
	 *
	 * @return The squared length of this FMVector3.
	 */
	inline float LengthSquared() const { return x * x + y * y + z * z; }

	/**
	 * Retrieves the length of the vector.
	 *
	 * @return The length of this FMVector3.
	 */
	inline float Length() const { return sqrtf(x * x + y * y + z * z); }

	/**
	 * Normalize this FMVector3.
	 */
	inline void NormalizeIt() { float l = Length(); if (!IsEquivalent(l, 0.0f)) { x /= l; y /= l; z /= l; }}

	/**
	 * Get a normalized FMVector3 with the same direction as this FMVector3.
	 *
	 * @return A FMVector3 with length 1 and same direction as this FMVector3.
	 */
	inline FMVector3 Normalize() const { float l = Length(); return FMVector3(x / l, y / l, z / l); }

	/**
	 * Project this FMVector3 onto another FMVector3.
	 * 
	 * @param unto The FMVector3 to project onto.
	 */
	inline void Project(const FMVector3& unto) { (*this) = Projected(unto); }

	/**
	 * Get the projection of this FMVector3 onto another FMVector3.
	 *
	 * @param unto The FMVector3 to project onto.
	 * @return The projected FMVector3.
	 */
	inline FMVector3 Projected(const FMVector3& unto);

	/**
	 * Get this FMVector3 as an array of \c floats.
	 *
	 * @return The \c float array.
	 */
	inline operator float*() { return &x; }

    /**
	 * Get this FMVector3 as an array of \c floats.
	 *
	 * @return The \c float array.
	 */
	inline operator const float*() const { return &x; }

	/**
	 * Assign this FMVector3 to the given float array.
	 *
	 * Assigns each coordinate of this FMVector3 to the elements in the \c
	 * float array. The first element to the first coordinate, the second to
	 * the second, and the third to the third. It returns this FMVector3.
	 *
	 * @param v The \c float array to assign with.
	 * @return This FMVector3.
	 */
	inline FMVector3& operator =(const float* v) { x = *v; y = *(v + 1); z = *(v + 2); return *this; }

	/**
	 * Update each component of this FMVector to the minimum of two FMVector3s.
	 *
	 * Updates each of the three components to be the minimum of the current
	 * value and that of the corresponding value of the given FMVector3.
	 *
	 * @param min The FMVector to take values from.
	 */
	inline void ComponentMinimum(const FMVector3& min) { if (x < min.x) x = min.x; if (y < min.y) y = min.y; if (z < min.z) z = min.z; }

	/**
	 * Update each component of this FMVector to the maximum of two FMVector3s.
	 *
	 * Updates each of the three components to be the maximum of the current
	 * value and that of the corresponding value of the given FMVector3.
	 *
	 * @param max The FMVector to take values from.
	 */
	inline void ComponentMaximum(const FMVector3& max) { if (x > max.x) x = max.x; if (y > max.y) y = max.y; if (z > max.z) z = max.z; }

	/**
	 * Clamp each component of this FMVector by the corresponding components
	 * in the specified min and max FMVector3.
	 *
	 * Clamp refers to setting a value within a given range. If the value is
	 * lower than the minimum of the range, it is set to the minimum; same for
	 * the maximum.
	 *
	 * @param min The FMVector to take the minimum values from.
	 * @param max The FMVector to take the maximum values from.
	 */
	inline void ComponentClamp(const FMVector3& min, const FMVector3& max) { ComponentMinimum(min); ComponentMaximum(max); }

public:
	static const FMVector3 XAxis; /**< The FMVector3 representing the x axis */
	static const FMVector3 YAxis; /**< The FMVector3 representing the y axis */
	static const FMVector3 ZAxis; /**< The FMVector3 representing the z axis */
	static const FMVector3 Zero;  /**< The FMVector3 representing zero */
	static const FMVector3 Origin;/**< The FMVector3 representing the origin */
};

/**
 * Vector addition with two FMVector3.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return The FMVector3 representation of the resulting vector.
 */
inline FMVector3 operator +(const FMVector3& a, const FMVector3& b) { return FMVector3(a.x + b.x, a.y + b.y, a.z + b.z); }

/**
 * Vector subtraction with two FMVector3.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return The FMVector3 representation of the resulting vector.
 */
inline FMVector3 operator -(const FMVector3& a, const FMVector3& b) { return FMVector3(a.x - b.x, a.y - b.y, a.z - b.z); }

/**
 * Positive operator of the given FMVector3.
 *
 * It applies the positive operator to each of the components of the FMVector3.
 *
 * @param a The vector to apply the positive operator to.
 * @return The FMVector3 representation of the resulting vector.
 */
inline FMVector3 operator +(const FMVector3& a) { return FMVector3(+a.x, +a.y, +a.z); }

/**
 * Negates the given FMVector3.
 *
 * It negates each of the components of the FMVector3.
 *
 * @param a The vector to negate.
 * @return The FMVector3 representation of the resulting vector.
 */
inline FMVector3 operator -(const FMVector3& a) { return FMVector3(-a.x, -a.y, -a.z); }

/**
 * Dot product of two FMVector3.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return The result of the dot product.
 */
inline float operator *(const FMVector3& a, const FMVector3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }

/**
 * Scalar multiplication with a FMVector3.
 *
 * @param a The vector.
 * @param b The scalar.
 * @return The FMVector3 representing the resulting vector.
 */
inline FMVector3 operator *(const FMVector3& a, float b) { return FMVector3(a.x * b, a.y * b, a.z * b); }

/**
 * Scalar multiplication with a FMVector3.
 *
 * @param a The scalar.
 * @param b The vector.
 * @return The FMVector3 representing the resulting vector.
 */
inline FMVector3 operator *(float a, const FMVector3& b) { return FMVector3(a * b.x, a * b.y, a * b.z); }

/**
 * Cross product of two FMVector3.
 *
 * @param a The first vector.
 * @param b The second vector.
 * @return The result of the dot product.
 */
inline FMVector3 operator ^(const FMVector3& a, const FMVector3& b) { return FMVector3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x); }

/**
 * Assignment of the addition of two FMVector3.
 *
 * @param b The first vector, which will also be assigned to the result.
 * @param a The second vector.
 * @return The first vector, after it has been assigned new values.
 */
inline FMVector3& operator +=(FMVector3& b, const FMVector3& a) { b.x += a.x; b.y += a.y; b.z += a.z; return b; }

/**
 * Assignment of the subtraction of two FMVector3.
 *
 * @param b The first vector, which will also be assigned to the result.
 * @param a The second vector.
 * @return The first vector, after it has been assigned new values.
 */
inline FMVector3& operator -=(FMVector3& b, const FMVector3& a) { b.x -= a.x; b.y -= a.y; b.z -= a.z; return b; }

/**
 * Assignment of the scalar multiplication of a FMVector3.
 *
 * @param b The vector, which will also be assigned to the result.
 * @param a The scalar.
 * @return The vector, after it has been assigned new values.
 */
inline FMVector3& operator *=(FMVector3& b, float a) { b.x *= a; b.y *= a; b.z *= a; return b; }

/**
 * Assignment of the scalar division of a FMVector3.
 *
 * @param b The vector, which will also be assigned to the result.
 * @param a The scalar.
 * @return The vector, after it has been assigned new values.
 */
inline FMVector3& operator /=(FMVector3& b, float a) { b.x /= a; b.y /= a; b.z /= a; return b; }

/**
	Returns whether two 3D vectors or points are equivalent.
	@param p A first vector.
	@param q A second vector.
	@return Whether the vectors are equivalent.
*/
inline bool IsEquivalent(const FMVector3& p, const FMVector3& q) { return IsEquivalent(p.x, q.x) && IsEquivalent(p.y, q.y) && IsEquivalent(p.z, q.z); }

/**
	Check if two FMVector3 are equivalent (they have relatively the same component values).
	@param a The first vector.
	@param b The second vector.
	@return Whether the vectors are equivalent.
*/
inline bool operator == (const FMVector3& a, const FMVector3& b) { return IsEquivalent(a, b); }

// Already documented above.
inline FMVector3 FMVector3::Projected(const FMVector3& unto) { return ((*this) * unto) / unto.LengthSquared() * unto; }

/** A dynamically-sized array of 3D vectors or points. */
typedef vector<FMVector3> FMVector3List;

#endif // _FM_VECTOR3_H_
