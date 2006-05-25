/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FMFloat.h
	The file containing functions and constants for floating point values.
 */

#ifdef WIN32
#include <float.h>
#endif

/** The default tolerance for double-sized floating-point comparison functions. */
#define DBL_TOLERANCE 0.0001
/** The default tolerance for single-sized floating-point comparison functions. */
#define FLT_TOLERANCE 0.0001f

/** A dynamically-sized array of double-sized floating-point values. */
typedef vector<double> DoubleList;

/** A dynamically-sized array of floating-point values. */
typedef vector<float> FloatList;

/** Returns whether two floating-point values are equivalent within a given tolerance.
	@param f1 A first floating-point value.
	@param f2 A second floating-point value. */
inline bool IsEquivalent(float f1, float f2) { return f1 - f2 < FLT_TOLERANCE && f2 - f1 < FLT_TOLERANCE; }

/** Returns whether two floating-point values are equivalent within a given tolerance.
	@param f1 A first floating-point value.
	@param f2 A second floating-point value.
	@param tolerance The tolerance in which to accept the two floating-point values as equivalent. */
inline bool IsEquivalent(float f1, float f2, float tolerance) { return f1 - f2 < tolerance && f2 - f1 < tolerance; }

/** Returns whether two double-sized floating-point values are equivalent.
	@param f1 A first double-sized floating-point value.
	@param f2 A second double-sized floating-point value. */
inline bool IsEquivalent(double f1, double f2) { return f1 - f2 < DBL_TOLERANCE && f2 - f1 < DBL_TOLERANCE; }

/** Returns whether two double-sized floating-point values are equivalent within a given tolerance.
	@param f1 A first double-sized floating-point value.
	@param f2 A second double-sized floating-point value.
	@param tolerance The tolerance in which to accept the two double-sized floating-point values as equivalent. */
inline bool IsEquivalent(double f1, double f2, double tolerance) { return f1 - f2 < tolerance && f2 - f1 < tolerance; }
