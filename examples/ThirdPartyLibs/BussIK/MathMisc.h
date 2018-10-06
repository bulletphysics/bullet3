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

#ifndef MATH_MISC_H
#define MATH_MISC_H

#include <math.h>

//
// Commonly used constants
//

const double PI = 3.1415926535897932384626433832795028841972;
const double PI2 = 2.0 * PI;
const double PI4 = 4.0 * PI;
const double PISq = PI * PI;
const double PIhalves = 0.5 * PI;
const double PIthirds = PI / 3.0;
const double PItwothirds = PI2 / 3.0;
const double PIfourths = 0.25 * PI;
const double PIsixths = PI / 6.0;
const double PIsixthsSq = PIsixths * PIsixths;
const double PItwelfths = PI / 12.0;
const double PItwelfthsSq = PItwelfths * PItwelfths;
const double PIinv = 1.0 / PI;
const double PI2inv = 0.5 / PI;
const double PIhalfinv = 2.0 / PI;

const double RadiansToDegrees = 180.0 / PI;
const double DegreesToRadians = PI / 180;

const double OneThird = 1.0 / 3.0;
const double TwoThirds = 2.0 / 3.0;
const double OneSixth = 1.0 / 6.0;
const double OneEighth = 1.0 / 8.0;
const double OneTwelfth = 1.0 / 12.0;

const double Root2 = sqrt(2.0);
const double Root3 = sqrt(3.0);
const double Root2Inv = 1.0 / Root2;  // sqrt(2)/2
const double HalfRoot3 = sqrtf(3) / 2.0;

const double LnTwo = log(2.0);
const double LnTwoInv = 1.0 / log(2.0);

// Special purpose constants
const double OnePlusEpsilon15 = 1.0 + 1.0e-15;
const double OneMinusEpsilon15 = 1.0 - 1.0e-15;

inline double ZeroValue(const double& x)
{
	return 0.0;
}

//
// Comparisons
//

template <class T>
inline T Min(T x, T y)
{
	return (x < y ? x : y);
}

template <class T>
inline T Max(T x, T y)
{
	return (y < x ? x : y);
}

template <class T>
inline T ClampRange(T x, T min, T max)
{
	if (x < min)
	{
		return min;
	}
	if (x > max)
	{
		return max;
	}
	return x;
}

template <class T>
inline bool ClampRange(T* x, T min, T max)
{
	if ((*x) < min)
	{
		(*x) = min;
		return false;
	}
	else if ((*x) > max)
	{
		(*x) = max;
		return false;
	}
	else
	{
		return true;
	}
}

template <class T>
inline bool ClampMin(T* x, T min)
{
	if ((*x) < min)
	{
		(*x) = min;
		return false;
	}
	return true;
}

template <class T>
inline bool ClampMax(T* x, T max)
{
	if ((*x) > max)
	{
		(*x) = max;
		return false;
	}
	return true;
}

template <class T>
inline T& UpdateMin(const T& x, T& y)
{
	if (x < y)
	{
		y = x;
	}
	return y;
}

template <class T>
inline T& UpdateMax(const T& x, T& y)
{
	if (x > y)
	{
		y = x;
	}
	return y;
}

template <class T>
inline bool SameSignNonzero(T x, T y)
{
	if (x < 0)
	{
		return (y < 0);
	}
	else if (0 < x)
	{
		return (0 < y);
	}
	else
	{
		return false;
	}
}

inline double Mag(double x)
{
	return fabs(x);
}

inline double Dist(double x, double y)
{
	return fabs(x - y);
}

template <class T>
inline bool NearEqual(T a, T b, double tolerance)
{
	a -= b;
	return (Mag(a) <= tolerance);
}

inline bool EqualZeroFuzzy(double x)
{
	return (fabs(x) <= 1.0e-14);
}

inline bool NearZero(double x, double tolerance)
{
	return (fabs(x) <= tolerance);
}

inline bool LessOrEqualFuzzy(double x, double y)
{
	if (x <= y)
	{
		return true;
	}

	if (y > 0.0)
	{
		if (x > 0.0)
		{
			return (x * OneMinusEpsilon15 < y * OnePlusEpsilon15);
		}
		else
		{
			return (y < 1.0e-15);  // x==0 in this case
		}
	}
	else if (y < 0.0)
	{
		if (x < 0.0)
		{
			return (x * OnePlusEpsilon15 < y * OneMinusEpsilon15);
		}
		else
		{
			return (y > -1.0e-15);  // x==0 in this case
		}
	}
	else
	{
		return (-1.0e-15 < x && x < 1.0e-15);
	}
}

inline bool GreaterOrEqualFuzzy(double x, double y)
{
	return LessOrEqualFuzzy(y, x);
}

inline bool UpdateMaxAbs(double* maxabs, double updateval)
{
	if (updateval > *maxabs)
	{
		*maxabs = updateval;
		return true;
	}
	else if (-updateval > *maxabs)
	{
		*maxabs = -updateval;
		return true;
	}
	else
	{
		return false;
	}
}

// **********************************************************
// Combinations and averages.								*
// **********************************************************

template <class T>
void averageOf(const T& a, const T& b, T& c)
{
	c = a;
	c += b;
	c *= 0.5;
}

template <class T>
void Lerp(const T& a, const T& b, double alpha, T& c)
{
	double beta = 1.0 - alpha;
	if (beta > alpha)
	{
		c = b;
		c *= alpha / beta;
		c += a;
		c *= beta;
	}
	else
	{
		c = a;
		c *= beta / alpha;
		c += b;
		c *= alpha;
	}
}

template <class T>
T Lerp(const T& a, const T& b, double alpha)
{
	T ret;
	Lerp(a, b, alpha, ret);
	return ret;
}

// **********************************************************
// Trigonometry												*
// **********************************************************

// TimesCot(x) returns x*cot(x)
inline double TimesCot(double x)
{
	if (-1.0e-5 < x && x < 1.0e-5)
	{
		return 1.0 + x * OneThird;
	}
	else
	{
		return (x * cos(x) / sin(x));
	}
}

// SineOver(x) returns sin(x)/x.
inline double SineOver(double x)
{
	if (-1.0e-5 < x && x < 1.0e-5)
	{
		return 1.0 - x * x * OneSixth;
	}
	else
	{
		return sin(x) / x;
	}
}
// OverSine(x) returns x/sin(x).
inline double OverSine(double x)
{
	if (-1.0e-5 < x && x < 1.0e-5)
	{
		return 1.0 + x * x * OneSixth;
	}
	else
	{
		return x / sin(x);
	}
}

inline double SafeAsin(double x)
{
	if (x <= -1.0)
	{
		return -PIhalves;
	}
	else if (x >= 1.0)
	{
		return PIhalves;
	}
	else
	{
		return asin(x);
	}
}

inline double SafeAcos(double x)
{
	if (x <= -1.0)
	{
		return PI;
	}
	else if (x >= 1.0)
	{
		return 0.0;
	}
	else
	{
		return acos(x);
	}
}

// **********************************************************************
// Roots and powers														*
// **********************************************************************

// Square(x) returns x*x,  of course!

template <class T>
inline T Square(T x)
{
	return (x * x);
}

// Cube(x) returns x*x*x,  of course!

template <class T>
inline T Cube(T x)
{
	return (x * x * x);
}

// SafeSqrt(x) = returns sqrt(max(x, 0.0));

inline double SafeSqrt(double x)
{
	if (x <= 0.0)
	{
		return 0.0;
	}
	else
	{
		return sqrt(x);
	}
}

// SignedSqrt(a, s) returns (sign(s)*sqrt(a)).
inline double SignedSqrt(double a, double sgn)
{
	if (sgn == 0.0)
	{
		return 0.0;
	}
	else
	{
		return (sgn > 0.0 ? sqrt(a) : -sqrt(a));
	}
}

// Template version of Sign function

template <class T>
inline int Sign(T x)
{
	if (x < 0)
	{
		return -1;
	}
	else if (x == 0)
	{
		return 0;
	}
	else
	{
		return 1;
	}
}

#endif  // #ifndef MATH_MISC_H
