/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FMVector4.h
	This file contains the class for 4 dimensional vectors.
*/

#ifndef _FM_VECTOR4_H_
#define _FM_VECTOR4_H_

/**
	A 4 dimensional vector.
	Not used within FCollada.
	
	@ingroup FMath
*/
class FCOLLADA_EXPORT FMVector4
{
public:
	float x;	/**< The first coordinate. */
	float y;	/**< The second coordinate. */
	float z;	/**< The third coordinate. */
	float w;	/**< The forth coordinate. */

	/**
	 * Creates an empty FMVector4.
	 */
	#ifndef _DEBUG
	FMVector4() {}
	#else
	FMVector4() { x = 123456789.0f; y = 123456789.0f; z = 123456789.0f; w = 123456789.0f; }
	#endif 

	/**
	 * Creates the FMVector4 with the coordinates given.
	 *
	 * The first three coordinates are taken from the FMVector3, where the
	 * first one is the x value, the second is that y, and the third is the z.
	 * The forth value is the \c float specified.
	 * 
	 *
	 * @param v The FMVector3 representing the first three coordinates.
	 * @param _w The final coordinate.
	 */
	FMVector4(FMVector3 v, float _w) { x = v.x; y = v.y; z = v.z; w = _w; }

	/**
	 * Creates the FMVector4 with the coordinates given.
	 *
	 * @param _x The first coordinate.
	 * @param _y The second coordinate.
	 * @param _z The third coordinate.
	 * @param _w The forth coordinate.
	 */
	FMVector4(float _x, float _y, float _z, float _w) { x = _x; y = _y; z = _z; w = _w; }

public:
	/**
	 * Get this FMVector4 as an array of \c floats.
	 *
	 * @return The \c float array.
	 */
	inline operator float*() { return &x; }

	/**
	 * Get this FMVector4 as an array of \c floats.
	 *
	 * @return The \c float array.
	 */
	inline operator const float*() const { return &x; }

public:
	static const FMVector4 Zero;	/**< The FMVector4 representing zero */
};

#endif // _FM_VECTOR4_H_
