// Bullet Continuous Collision Detection and Physics Library
// Copyright (c) 2003-2006 Erwin Coumans  http://continuousphysics.com/Bullet/
//
// Maths.h
//
// Copyright (c) 2006 Simon Hobbs
//
// This software is provided 'as-is', without any express or implied warranty. In no event will the authors be held liable for any damages arising from the use of this software.
//
// Permission is granted to anyone to use this software for any purpose, including commercial applications, and to alter it and redistribute it freely, subject to the following restrictions:
//
// 1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
//
// 2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
//
// 3. This notice may not be removed or altered from any source distribution.
#ifndef BULLET_MATH_H
#define BULLET_MATH_H

#ifdef WIN32

#include <math.h>
#include <stdlib.h>
#include <float.h>

// intrinsics headers
#include <mmintrin.h>
#include <xmmintrin.h>

// vector maths classes require aligned alloc
#include "Memory2.h"

// constants
#define PI				3.141592654f

#define Angle5			0.087266462f
#define Angle10			0.174532925f
#define Angle15			0.261799388f
#define Angle30			0.523598776f
#define Angle45			0.785398163f
#define Angle60			0.523598776f
#define Angle90			1.570796327f
#define Angle180		PI
#define Angle270		4.71238898f
#define Angle360		6.283185307f

#define Deg2RadF		0.01745329251994329547f
#define Rad2DegF		57.29577951308232286465f

#define MinValueF		-3.402823466e+38f
#define MaxValueF		3.402823466e+38F

#define DefaultEpsilon	0.001f


// float functions

inline float Sin(const float f)
{
	return sinf(f);
}

inline float Cos(const float f)
{
	return cosf(f);
}

inline float Tan(const float f)
{
	return tanf(f);
}

inline float Asin(const float f)
{
	return asinf(f);
}

inline float Acos(const float f)
{
	return acosf(f);
}

inline float Atan(const float f)
{
	return atanf(f);
}

inline float Atan2(const float y, const float x)
{
	return atan2f(y, x);
}

inline float Pow(const float x, const float y)
{
	return powf(x, y);
}

inline float Abs(const float x)
{
	return fabsf(x);
}

inline float Min(const float x, const float y)
{
	return (x < y) ? x : y;
}

inline float Max(const float x, const float y)
{
	return (x > y) ? x : y;
}

inline float Clamp(const float x, const float min, const float max)
{
	return (x >= max) ? max : Max(x, min);
}

inline float Sgn(const float f)
{
	// TODO: replace this with something that doesn't branch
	if (f < 0.0f)
		return -1.0f;
	if (f > 0.0f)
		return 1.0f;
	return 0.0f;
}

inline float Floor(const float f)
{
	return floorf(f);
}

inline float Ceil(const float f)
{
	return ceilf(f);
}

inline float Mod(const float x, const float y)
{
	float n = Floor(x / y);
	return x - n * y;
}

inline float Sqrt(const float f)
{
	return sqrtf(f);
}

inline bool Equal(const float x, const float y, const float epsilon = DefaultEpsilon)
{
	return Abs(x-y) <= epsilon;
}

inline float Lerp(const float x, const float y, const float t)
{
	return x + (y - x) * t;
}

inline int Min(const int x, const int y)
{
	return (x < y) ? x : y;
}

inline int Max(const int x, const int y)
{
	return (x > y) ? x : y;
}

#include "Vector.h"
#include "Matrix.h"
#include "Quat.h"
#include "Geometry.h"

#endif //WIN32
#endif //BULLET_MATH_H
