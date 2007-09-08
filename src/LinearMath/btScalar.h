/*
Copyright (c) 2003-2006 Gino van den Bergen / Erwin Coumans  http://continuousphysics.com/Bullet/

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/



#ifndef SIMD___SCALAR_H
#define SIMD___SCALAR_H

#include <math.h>

#include <cstdlib>
#include <cfloat>
#include <float.h>


#ifdef WIN32

//added __cdecl, thanks Jack

// default new and delete overrides that guarantee 16 byte alignment and zero allocated memory
void* __cdecl operator new(size_t sz) throw();
void* __cdecl operator new[](size_t sz) throw();
void __cdecl operator delete(void* m) throw();
void __cdecl operator delete[](void* m) throw();

#include <malloc.h>
#include <stdio.h>
#define BULLET_ALIGNED_NEW_AND_DELETE \
\
inline void* operator new(size_t sz) throw()	\
{												\
	printf("new %d\n",sz);						\
	void* mem = _aligned_malloc(sz + 64, 16);	\
	return mem;									\
}												\
												\
inline void* operator new[](size_t sz) throw()	\
{												\
	printf("new[] %d\n",sz);					\
	void* mem = _aligned_malloc(sz + 64, 16);	\
	return mem;									\
}												\
												\
inline void operator delete(void* m) throw()	\
{					\
printf("delete %x\n",m);						\
	if (m == 0)									\
		return;									\
	_aligned_free(m);							\
}												\
												\
inline void operator delete[](void* m) throw()	\
{												\
	printf("delete[] %x\n",m); \
_aligned_free(m);							\
}												\

#endif

#ifdef WIN32

		#if defined(__MINGW32__) || defined(__CYGWIN__) || (defined (_MSC_VER) && _MSC_VER < 1300)
			#define SIMD_FORCE_INLINE inline
			#define ATTRIBUTE_ALIGNED16(a) a
		#else
			#define BT_HAS_ALIGNED_ALOCATOR
			#pragma warning(disable:4530)
			#pragma warning(disable:4996)
			#pragma warning(disable:4786)
			#define SIMD_FORCE_INLINE __forceinline
			#define ATTRIBUTE_ALIGNED16(a) __declspec(align(16)) a
		#ifdef _XBOX
			#define BT_USE_VMX128

			#include <ppcintrinsics.h>
 			#define BT_HAVE_NATIVE_FSEL
 			#define btFsel(a,b,c) __fsel((a),(b),(c))
		#else
			#define BT_USE_SSE
		#endif
		#endif //__MINGW32__

		#include <assert.h>
		#define btAssert assert
		//btFullAssert is optional, slows down a lot
		#define btFullAssert(x)
#else
	
#if defined	(__CELLOS_LV2__)
		#define SIMD_FORCE_INLINE inline
		#define ATTRIBUTE_ALIGNED16(a) a __attribute__ ((aligned (16)))
		#ifndef assert
		#include <assert.h>
		#endif
		#define btAssert assert
		//btFullAssert is optional, slows down a lot
		#define btFullAssert(x)
#else

	//non-windows systems

		#define SIMD_FORCE_INLINE inline
		#define ATTRIBUTE_ALIGNED16(a) a
		#ifndef assert
		#include <assert.h>
		#endif
		#define btAssert assert
		//btFullAssert is optional, slows down a lot
		#define btFullAssert(x)
#endif	//__CELLOS_LV2__
#endif

/// older compilers (gcc 3.x) and Sun needs double version of sqrt etc.
/// exclude Apple Intel (i's assumed to be a Macbook or new Intel Dual Core Processor)
#if defined (__sun) || defined (__sun__) || defined (__sparc) || (defined (__APPLE__) && ! defined (__i386__))
//use slow double float precision operation on those platforms
#ifndef BT_USE_DOUBLE_PRECISION
#define BT_FORCE_DOUBLE_FUNCTIONS
#endif
#endif

#if defined(BT_USE_DOUBLE_PRECISION)
typedef double btScalar;
#else
typedef float btScalar;
#endif


#if defined(BT_USE_DOUBLE_PRECISION) || defined(BT_FORCE_DOUBLE_FUNCTIONS)
		
SIMD_FORCE_INLINE btScalar btSqrt(btScalar x) { return sqrt(x); }
SIMD_FORCE_INLINE btScalar btFabs(btScalar x) { return fabs(x); }
SIMD_FORCE_INLINE btScalar btCos(btScalar x) { return cos(x); }
SIMD_FORCE_INLINE btScalar btSin(btScalar x) { return sin(x); }
SIMD_FORCE_INLINE btScalar btTan(btScalar x) { return tan(x); }
SIMD_FORCE_INLINE btScalar btAcos(btScalar x) { return acos(x); }
SIMD_FORCE_INLINE btScalar btAsin(btScalar x) { return asin(x); }
SIMD_FORCE_INLINE btScalar btAtan(btScalar x) { return atan(x); }
SIMD_FORCE_INLINE btScalar btAtan2(btScalar x, btScalar y) { return atan2(x, y); }
SIMD_FORCE_INLINE btScalar btExp(btScalar x) { return exp(x); }
SIMD_FORCE_INLINE btScalar btLog(btScalar x) { return log(x); }
SIMD_FORCE_INLINE btScalar btPow(btScalar x,btScalar y) { return pow(x,y); }

#else
		
SIMD_FORCE_INLINE btScalar btSqrt(btScalar x) { return sqrtf(x); }
SIMD_FORCE_INLINE btScalar btFabs(btScalar x) { return fabsf(x); }
SIMD_FORCE_INLINE btScalar btCos(btScalar x) { return cosf(x); }
SIMD_FORCE_INLINE btScalar btSin(btScalar x) { return sinf(x); }
SIMD_FORCE_INLINE btScalar btTan(btScalar x) { return tanf(x); }
SIMD_FORCE_INLINE btScalar btAcos(btScalar x) { return acosf(x); }
SIMD_FORCE_INLINE btScalar btAsin(btScalar x) { return asinf(x); }
SIMD_FORCE_INLINE btScalar btAtan(btScalar x) { return atanf(x); }
SIMD_FORCE_INLINE btScalar btAtan2(btScalar x, btScalar y) { return atan2f(x, y); }
SIMD_FORCE_INLINE btScalar btExp(btScalar x) { return expf(x); }
SIMD_FORCE_INLINE btScalar btLog(btScalar x) { return logf(x); }
SIMD_FORCE_INLINE btScalar btPow(btScalar x,btScalar y) { return powf(x,y); }
	
#endif

#define SIMD_2_PI         btScalar(6.283185307179586232)
#define SIMD_PI           (SIMD_2_PI * btScalar(0.5))
#define SIMD_HALF_PI      (SIMD_2_PI * btScalar(0.25))
#define SIMD_RADS_PER_DEG (SIMD_2_PI / btScalar(360.0))
#define SIMD_DEGS_PER_RAD  (btScalar(360.0) / SIMD_2_PI)

#ifdef BT_USE_DOUBLE_PRECISION
#define SIMD_EPSILON      DBL_EPSILON
#define SIMD_INFINITY     DBL_MAX
#else
#define SIMD_EPSILON      FLT_EPSILON
#define SIMD_INFINITY     FLT_MAX
#endif

SIMD_FORCE_INLINE btScalar btAtan2Fast(btScalar y, btScalar x) 
{
	btScalar coeff_1 = SIMD_PI / 4.0f;
	btScalar coeff_2 = 3.0f * coeff_1;
	btScalar abs_y = btFabs(y);
	btScalar angle;
	if (x >= 0.0f) {
		btScalar r = (x - abs_y) / (x + abs_y);
		angle = coeff_1 - coeff_1 * r;
	} else {
		btScalar r = (x + abs_y) / (abs_y - x);
		angle = coeff_2 - coeff_1 * r;
	}
	return (y < 0.0f) ? -angle : angle;
}

SIMD_FORCE_INLINE bool      btFuzzyZero(btScalar x) { return btFabs(x) < SIMD_EPSILON; }

SIMD_FORCE_INLINE bool	btEqual(btScalar a, btScalar eps) {
	return (((a) <= eps) && !((a) < -eps));
}
SIMD_FORCE_INLINE bool	btGreaterEqual (btScalar a, btScalar eps) {
	return (!((a) <= eps));
}


SIMD_FORCE_INLINE int       btIsNegative(btScalar x) {
    return x < btScalar(0.0) ? 1 : 0;
}

SIMD_FORCE_INLINE btScalar btRadians(btScalar x) { return x * SIMD_RADS_PER_DEG; }
SIMD_FORCE_INLINE btScalar btDegrees(btScalar x) { return x * SIMD_DEGS_PER_RAD; }

#define BT_DECLARE_HANDLE(name) typedef struct name##__ { int unused; } *name

#ifndef btFsel
SIMD_FORCE_INLINE btScalar btFsel(btScalar a, btScalar b, btScalar c)
{
	return a >= 0 ? b : c;
}
#endif
#define btFsels(a,b,c) (btScalar)btFsel(a,b,c)


///btSelect avoids branches, which makes performance much better for consoles like Playstation 3 and XBox 360
///Thanks Phil Knight. See also http://www.cellperformance.com/articles/2006/04/more_techniques_for_eliminatin_1.html
SIMD_FORCE_INLINE unsigned btSelect(unsigned condition, unsigned valueIfConditionNonZero, unsigned valueIfConditionZero) 
{
    // Set testNz to 0xFFFFFFFF if condition is nonzero, 0x00000000 if condition is zero
    // Rely on positive value or'ed with its negative having sign bit on
    // and zero value or'ed with its negative (which is still zero) having sign bit off 
    // Use arithmetic shift right, shifting the sign bit through all 32 bits
    unsigned testNz = (unsigned)(((int)condition | -(int)condition) >> 31);
    unsigned testEqz = ~testNz;
    return ((valueIfConditionNonZero & testNz) | (valueIfConditionZero & testEqz)); 
}
SIMD_FORCE_INLINE int btSelect(unsigned condition, int valueIfConditionNonZero, int valueIfConditionZero)
{
    unsigned testNz = (unsigned)(((int)condition | -(int)condition) >> 31);
    unsigned testEqz = ~testNz; 
    return ((valueIfConditionNonZero & testNz) | (valueIfConditionZero & testEqz));
}
SIMD_FORCE_INLINE float btSelect(unsigned condition, float valueIfConditionNonZero, float valueIfConditionZero)
{
#ifdef BT_HAVE_NATIVE_FSEL
    return (float)btFsel((btScalar)condition - btScalar(1.0f), valueIfConditionNonZero, valueIfConditionZero);
#else
    return (condition != 0) ? valueIfConditionNonZero : valueIfConditionZero; 
#endif
}


#endif //SIMD___SCALAR_H
