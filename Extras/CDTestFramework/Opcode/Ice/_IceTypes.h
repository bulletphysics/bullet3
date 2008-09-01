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
 *	Contains custom types.
 *	\file		IceTypes.h
 *	\author		Pierre Terdiman
 *	\date		April, 4, 2000
 */
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Include Guard
#ifndef __ICETYPES_H__
#define __ICETYPES_H__

	#define USE_HANDLE_MANAGER

	// Constants
	#define	PI					3.1415926535897932384626433832795028841971693993751f	//!< PI
	#define	HALFPI				1.57079632679489661923f									//!< 0.5 * PI
	#define	TWOPI				6.28318530717958647692f									//!< 2.0 * PI
	#define	INVPI				0.31830988618379067154f									//!< 1.0 / PI

	#define	RADTODEG			57.2957795130823208768f									//!< 180.0 / PI, convert radians to degrees
	#define	DEGTORAD			0.01745329251994329577f									//!< PI / 180.0, convert degrees to radians

	#define	EXP					2.71828182845904523536f									//!< e
	#define	INVLOG2				3.32192809488736234787f									//!< 1.0 / log10(2)
	#define	LN2					0.693147180559945f										//!< ln(2)
	#define	INVLN2				1.44269504089f											//!< 1.0f / ln(2)

	#define	INV3				0.33333333333333333333f									//!< 1/3
	#define	INV6				0.16666666666666666666f									//!< 1/6
	#define	INV7				0.14285714285714285714f									//!< 1/7
	#define	INV9				0.11111111111111111111f									//!< 1/9
	#define	INV255				0.00392156862745098039f									//!< 1/255

	#define	SQRT2				1.41421356237f											//!< sqrt(2)
	#define	INVSQRT2			0.707106781188f											//!< 1 / sqrt(2)

	#define	SQRT3				1.73205080757f											//!< sqrt(3)
	#define	INVSQRT3			0.577350269189f											//!< 1 / sqrt(3)

	#define null				0														//!< our own NULL pointer

	// Custom types used in ICE
	typedef signed char			sbyte;		//!< sizeof(sbyte)	must be 1
	typedef unsigned char		ubyte;		//!< sizeof(ubyte)	must be 1
	typedef signed short		sword;		//!< sizeof(sword)	must be 2
	typedef unsigned short		uword;		//!< sizeof(uword)	must be 2
	typedef signed int			sdword;		//!< sizeof(sdword)	must be 4
	typedef unsigned int		udword;		//!< sizeof(udword)	must be 4
	typedef signed __int64		sqword;		//!< sizeof(sqword)	must be 8
	typedef unsigned __int64	uqword;		//!< sizeof(uqword)	must be 8
	typedef float				float32;	//!< sizeof(float32)	must be 4
	typedef double				float64;	//!< sizeof(float64)	must be 4

	ICE_COMPILE_TIME_ASSERT(sizeof(bool)==1);	// ...otherwise things might fail with VC++ 4.2 !
	ICE_COMPILE_TIME_ASSERT(sizeof(ubyte)==1);
	ICE_COMPILE_TIME_ASSERT(sizeof(sbyte)==1);
	ICE_COMPILE_TIME_ASSERT(sizeof(sword)==2);
	ICE_COMPILE_TIME_ASSERT(sizeof(uword)==2);
	ICE_COMPILE_TIME_ASSERT(sizeof(udword)==4);
	ICE_COMPILE_TIME_ASSERT(sizeof(sdword)==4);
	ICE_COMPILE_TIME_ASSERT(sizeof(uqword)==8);
	ICE_COMPILE_TIME_ASSERT(sizeof(sqword)==8);

	//! TO BE DOCUMENTED
	#define DECLARE_ICE_HANDLE(name)	struct name##__ { int unused; }; typedef struct name##__ *name

	typedef udword				DynID;		//!< Dynamic identifier
#ifdef USE_HANDLE_MANAGER
	typedef udword				KID;		//!< Kernel ID
//	DECLARE_ICE_HANDLE(KID);
#else
	typedef uword				KID;		//!< Kernel ID
#endif
	typedef udword				RTYPE;		//!< Relationship-type (!) between owners and references
	#define	INVALID_ID			0xffffffff	//!< Invalid dword ID (counterpart of null pointers)
#ifdef USE_HANDLE_MANAGER
	#define	INVALID_KID			0xffffffff	//!< Invalid Kernel ID
#else
	#define	INVALID_KID			0xffff		//!< Invalid Kernel ID
#endif
	#define	INVALID_NUMBER		0xDEADBEEF	//!< Standard junk value

	// Define BOOL if needed
	#ifndef BOOL
	typedef int	BOOL;						//!< Another boolean type.
	#endif

	//! Union of a float and a sdword
	typedef union {
		float	f;							//!< The float
		sdword	d;							//!< The integer
	}scell;

	//! Union of a float and a udword
	typedef union {
		float	f;							//!< The float
		udword	d;							//!< The integer
	}ucell;

	// Type ranges
	#define	MAX_SBYTE				0x7f						//!< max possible sbyte value
	#define	MIN_SBYTE				0x80						//!< min possible sbyte value
	#define	MAX_UBYTE				0xff						//!< max possible ubyte value
	#define	MIN_UBYTE				0x00						//!< min possible ubyte value
	#define	MAX_SWORD				0x7fff						//!< max possible sword value
	#define	MIN_SWORD				0x8000						//!< min possible sword value
	#define	MAX_UWORD				0xffff						//!< max possible uword value
	#define	MIN_UWORD				0x0000						//!< min possible uword value
	#define	MAX_SDWORD				0x7fffffff					//!< max possible sdword value
	#define	MIN_SDWORD				0x80000000					//!< min possible sdword value
	#define	MAX_UDWORD				0xffffffff					//!< max possible udword value
	#define	MIN_UDWORD				0x00000000					//!< min possible udword value
	#define	MAX_FLOAT				FLT_MAX						//!< max possible float value
	#define	MIN_FLOAT				(-FLT_MAX)					//!< min possible loat value
	#define IEEE_1_0				0x3f800000					//!< integer representation of 1.0
	#define IEEE_255_0				0x437f0000					//!< integer representation of 255.0
	#define IEEE_MAX_FLOAT			0x7f7fffff					//!< integer representation of MAX_FLOAT
	#define IEEE_MIN_FLOAT			0xff7fffff					//!< integer representation of MIN_FLOAT
	#define IEEE_UNDERFLOW_LIMIT	0x1a000000

	#define ONE_OVER_RAND_MAX		(1.0f / float(RAND_MAX))	//!< Inverse of the max possible value returned by rand()

	typedef int					(__stdcall* PROC)();			//!< A standard procedure call.
	typedef bool				(*ENUMERATION)(udword value, udword param, udword context);	//!< ICE standard enumeration call
	typedef	void**				VTABLE;							//!< A V-Table.

	#undef		MIN
	#undef		MAX
	#define		MIN(a, b)       ((a) < (b) ? (a) : (b))			//!< Returns the min value between a and b
	#define		MAX(a, b)       ((a) > (b) ? (a) : (b))			//!< Returns the max value between a and b
	#define		MAXMAX(a,b,c)   ((a) > (b) ? MAX (a,c) : MAX (b,c))	//!<	Returns the max value between a, b and c

	template<class T>	inline_ const T&	TMin	(const T& a, const T& b)	{ return b < a ? b : a;	}
	template<class T>	inline_ const T&	TMax	(const T& a, const T& b)	{ return a < b ? b : a;	}
	template<class T>	inline_ void		TSetMin	(T& a, const T& b)			{ if(a>b)	a = b;		}
	template<class T>	inline_ void		TSetMax	(T& a, const T& b)			{ if(a<b)	a = b;		}

	#define		SQR(x)			((x)*(x))						//!< Returns x square
	#define		CUBE(x)			((x)*(x)*(x))					//!< Returns x cube

	#define		AND		&										//!< ...
	#define		OR		|										//!< ...
	#define		XOR		^										//!< ...

	#define		QUADRAT(x)		((x)*(x))						//!< Returns x square

#ifdef _WIN32
#   define srand48(x) srand((unsigned int) (x))
#	define srandom(x) srand((unsigned int) (x))
#	define random()   ((double) rand())
#   define drand48()  ((double) (((double) rand()) / ((double) RAND_MAX)))
#endif

#endif // __ICETYPES_H__
