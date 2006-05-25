/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FMath.h
	The file containing functions and constants for math.

	@defgroup FMath Mathematics Classes.
*/

#ifndef _F_MATH_H_
#define _F_MATH_H_

#include <math.h>
#include <algorithm>

// Includes the improved vector class.
#include "FMath/FMArray.h"

// Includes the common integer and floating-point definitions and functions.
#include "FMath/FMFloat.h"
#include "FMath/FMInteger.h"

/**
	A namespace for common math functions.

	@ingroup FMath
*/
namespace FMath
{
	const double Pi = 3.14159;	/**< Mathematical value of pi to 5 decimals. */

	/**
	 * Convert radians to degrees.
	 * 
	 * @param val The value in radians.
	 * @return The value in degrees.
	 */
	inline double RadToDeg(double val) { return (val * 180.0/Pi); }

	/**
	 * Convert radians to degrees.
	 * 
	 * @param val The value in radians.
	 * @return The value in degrees.
	 */
	inline float RadToDeg(float val) { return (val * 180.0f/(float)Pi); }

	/**
	 * Convert degrees to radians.
	 * 
	 * @param val The value in degrees.
	 * @return The value in radians.
	 */
	inline double DegToRad(double val) { return (val * Pi/180.0); }

	/**
	 * Convert degrees to radians.
	 * 
	 * @param val The value in degrees.
	 * @return The value in radians.
	 */
	inline float DegToRad(float val) { return (val * (float)Pi/180.0f); }

/** 
 * Determines if given float is encoding for not a number (NAN).
 * 
 * @param f The float to check.
 * @return 0 if it is a number, something else if is NAN.
 */
#ifdef WIN32
	inline int IsNotANumber(float f) { return _isnan(f); }
#else // Linux and Mac
	inline int IsNotANumber(float f) { return !finite(f); }
#endif

	/**
	 * Determine the sign of a number.
	 * 
	 * @param val The number to check.
	 * @return 1.0 if positive, -1.0 if negative.
	 */
	inline double Sign(double val) { return (val >= 0.0) ? 1.0 : -1.0; }

	/**
	 * Determine the sign of a number.
	 * 
	 * @param val The number to check.
	 * @return 1.0f if positive, -1.0f if negative.
	 */
	inline float Sign(float val) { return (val >= 0.0f) ? 1.0f : -1.0f; }

	/**
	 * Determine the sign of a number.
	 * 
	 * @param val The number to check.
	 * @return 1 if positive, -1 if negative.
	 */
	inline int32 Sign(int32 val) { return (val >= 0) ? 1 : -1; }

	/**
	 * Clamp the specified object within a range specified by two other objects
	 * of the same class.
	 *
	 * Clamp refers to setting a value within a given range. If the value is
	 * lower than the minimum of the range, it is set to the minimum; same for
	 * the maximum.
	 *
	 * @param val The object to clamp.
	 * @param mx The highest object of the range.
	 * @param mn The lowest object of the range.
	 * @return The clamped value.
	 */
	template <class T>
	inline T Clamp(T val, T mn, T mx) { return std::max(std::min(val, mx), mn); }
};

// Include commonly used mathematical classes
#include "FMath/FMVector2.h"
#include "FMath/FMVector3.h"
#include "FMath/FMVector4.h"
#include "FMath/FMColor.h"
#include "FMath/FMMatrix33.h"
#include "FMath/FMMatrix44.h"

#endif // _F_MATH_H_
