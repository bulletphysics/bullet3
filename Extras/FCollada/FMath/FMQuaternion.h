/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FMQuaternion.h
	The file containing the class for quaternions.
*/

#ifndef _FM_QUATERNION_H_
#define _FM_QUATERNION_H_

/**
	A quaternion.
	Not used within FCollada
	
	[Glaforte 03-08-2006] VERY EXPERIMENTAL CODE: DON'T USE.

	@ingroup FMath
*/
class FCOLLADA_EXPORT FMQuaternion
{
public:
	float x;	/**< The i component. */
	float y;	/**< The j component. */
	float z;	/**< The k component. */
	float w;	/**< The scalar component. */

	#ifndef _DEBUG
	/**
	 * Creates an empty FMQuaternion.
	 *
	 * The default values are non deterministic.
	 */
	FMQuaternion() {}
	#else
	FMQuaternion() { x = 123456789.0f; y = 123456789.0f; z = 123456789.0f; w = 123456789.0f; }
	#endif 

	/**
	 * Creates the FMQuaternion with the given component values.
	 *
	 * @param _x The i component.
	 * @param _y The j component.
	 * @param _z The k component.
	 * @param _w The scalar component.
	 */
	FMQuaternion(float _x, float _y, float _z, float _w) { x = _x; y = _y; z = _z; w = _w; }

	/**
	 * Creates the FMQuaternion from a given axis and angle of rotation.
	 *
	 * @param axis The axis of rotation.
	 * @param angle The angle of rotation in radians.
	 */
	FMQuaternion(const FMVector3& axis, float angle);

	/**
	 * Get this FMQuaternion as an array of \c floats.
	 *
	 * @return The \c float array.
	 */
	inline operator float*() { return &x; }

	/**
	 * Get this FMQuaternion as an array of \c floats.
	 *
	 * @return The \c float array.
	 */
	inline operator const float*() const { return &x; }

	/**
	 * Assign this FMQuaternion to the given \c float array.
	 *
	 * Assigns each coordinate of this FMQuaternion to the elements in the 
	 * \c float array. The first element to the i component, the second to the
	 * j, the third to the k, and the forth to the scalar. It returns this 
	 * FMQuaternion.
	 *
	 * @param v The \c float array to assign with.
	 * @return This FMQuaternion.
	 */
	inline FMQuaternion& operator =(const float* v) { x = *v; y = *(v + 1); z = *(v + 2); w = *(v + 3); return *this; }

	/**
	 * Applys quaternion multiplication of the given FMQuaternion with this
	 * FMQuaternion and returns the value.
	 *
	 * @param q The FMQuaternion multiply with.
	 * @return The resulting FMQuaternion.
	 */
	FMQuaternion operator*(const FMQuaternion& q) const;

	/**
	 * Get the FMQuaternion representation of the Euler rotation angles.
	 *
	 * @param _x The rotation about the x-axis (roll), in radians.
	 * @param _y The rotation about the y-axis (pitch), in radians.
	 * @param _z The rotation about the z-axis (yaw), in radians.
	 */
	static FMQuaternion EulerRotationQuaternion(float _x, float _y, float _z);
};

#endif // _FM_QUATERNION_H_
