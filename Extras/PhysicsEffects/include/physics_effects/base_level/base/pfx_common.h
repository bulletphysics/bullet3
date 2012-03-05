/*
Physics Effects Copyright(C) 2010 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/

#ifndef _SCE_PFX_COMMON_H
#define _SCE_PFX_COMMON_H

// Include common headers
#ifdef _WIN32
	#include <windows.h>
	#include <stdio.h>
	#include <tchar.h>
#else
	#include <stdio.h>
	#include <stdint.h>
#endif

#include <string.h>
#include <stdlib.h>
#include <math.h>

#if defined(_WIN32)
	#include "pfx_vectormath_include.win32.h"
#else
	#include "pfx_vectormath_include.h"
#endif

namespace sce {
namespace PhysicsEffects {
// Basic Types
#if defined(_WIN32)
	typedef char               PfxInt8;
	typedef unsigned char      PfxUInt8;
	typedef short              PfxInt16;
	typedef unsigned short     PfxUInt16;
	typedef int                PfxInt32;
	typedef unsigned int       PfxUInt32;
	typedef long long          PfxInt64;
	typedef unsigned long long PfxUInt64;
#else
	typedef int8_t             PfxInt8;
	typedef uint8_t            PfxUInt8;
	typedef int16_t            PfxInt16;
	typedef uint16_t           PfxUInt16;
	typedef int32_t            PfxInt32;
	typedef uint32_t           PfxUInt32;
	typedef int64_t            PfxInt64;
	typedef uint64_t           PfxUInt64;
#endif

typedef bool                        PfxBool;
typedef float                       PfxFloat;
} //namespace PhysicsEffects
} //namespace sce

// Debug Print
#ifdef _WIN32
static void pfxOutputDebugString(const char *str, ...)
{
    char strDebug[1024]={0};
    va_list argList;
    va_start(argList, str);
    vsprintf_s(strDebug,str,argList);
	OutputDebugStringA(strDebug);
    va_end(argList);
}
#endif

#if defined(_DEBUG)
	#if defined(_WIN32)
		#define SCE_PFX_DPRINT pfxOutputDebugString
	#else
		#define SCE_PFX_DPRINT(...) printf(__VA_ARGS__)
	#endif
#else
	#ifdef _WIN32
		#define SCE_PFX_DPRINT
	#else
		#define SCE_PFX_DPRINT(...)
	#endif
#endif

// Printf
#if defined(_WIN32)
	#define SCE_PFX_PRINTF pfxOutputDebugString
#else
// ARA begin insert new code
	#ifdef __ANDROID__
		#include <android/log.h>
		#define SCE_PFX_PRINTF(...) __android_log_print(ANDROID_LOG_VERBOSE, "SCE_PFX_PRINTF", __VA_ARGS__)
	#else
		#define SCE_PFX_PRINTF(...) printf(__VA_ARGS__)
	#endif
// ARA end, old baseline code block was:
//	#define SCE_PFX_PRINTF(...) printf(__VA_ARGS__)
//
#endif

#define SCE_PFX_UNLIKELY(a)		(a)
#define SCE_PFX_LIKELY(a)		(a)

// Inline
#if defined(_MSC_VER)
	#define SCE_PFX_FORCE_INLINE __forceinline
#elif defined(__SNC__) || defined(__GNUC__)
	#define SCE_PFX_FORCE_INLINE inline __attribute__((always_inline))
#endif

// Assert
#define SCE_PFX_HALT() abort()

#ifdef _DEBUG
	#define SCE_PFX_ASSERT(test) {if(!(test)){SCE_PFX_PRINTF("Assert "__FILE__ ":%u ("#test")\n", __LINE__);SCE_PFX_HALT();}}
	#define SCE_PFX_ASSERT_MSG(test,msg) {if(!(test)){SCE_PFX_PRINTF("Assert " msg " " __FILE__ ":%u ("#test")\n",__LINE__);SCE_PFX_HALT();}}
#else
	#define SCE_PFX_ASSERT(test)
	#define SCE_PFX_ASSERT_MSG(test,msg)
#endif

#define SCE_PFX_ALWAYS_ASSERT(test) {if(!(test)){SCE_PFX_PRINTF("Assert "__FILE__ ":%u ("#test")\n", __LINE__);SCE_PFX_HALT();}}
#define SCE_PFX_ALWAYS_ASSERT_MSG(test,msg) {if(!(test)){SCE_PFX_PRINTF("Assert:" msg " " __FILE__ ":%u ("#test")\n",__LINE__);SCE_PFX_HALT();}}

// Aligned 
#if defined(_MSC_VER)
	#define SCE_PFX_ALIGNED(alignment)   __declspec(align(alignment))
#elif defined(__SNC__) || defined(__GNUC__)
	#define SCE_PFX_ALIGNED(alignment)   __attribute__((__aligned__((alignment))))
#endif

// Etc
#define SCE_PFX_MIN(a,b) (((a)<(b))?(a):(b))
#define SCE_PFX_MAX(a,b) (((a)>(b))?(a):(b))
#define SCE_PFX_CLAMP(v,a,b) SCE_PFX_MAX(a,SCE_PFX_MIN(v,b))
#define SCE_PFX_SWAP(type, x, y) do {type t; t=x; x=y; y=t; } while (0)
#define SCE_PFX_SQR(a) ((a)*(a))

#define SCE_PFX_ALIGN16(count,size)   ((((((count) * (size)) + 15) & (~15)) + (size)-1) / (size))
#define SCE_PFX_ALIGN128(count,size)  ((((((count) * (size)) + 127) & (~127)) + (size)-1) / (size))

#define SCE_PFX_AVAILABLE_BYTES_ALIGN16(ptr,bytes) (bytes-((uintptr_t)(ptr)&0x0f))
#define SCE_PFX_AVAILABLE_BYTES_ALIGN128(ptr,bytes) (bytes-((uintptr_t)(ptr)&0x7f))

#define SCE_PFX_BYTES_ALIGN16(bytes) (((bytes)+15)&(~15))
#define SCE_PFX_BYTES_ALIGN128(bytes) (((bytes)+127)&(~127))

#define SCE_PFX_PTR_ALIGN16(ptr) (((uintptr_t)(ptr)+15)&(~15))
#define SCE_PFX_PTR_ALIGN128(ptr) (((uintptr_t)(ptr)+127)&(~127))

#define SCE_PFX_PTR_IS_ALIGNED16(ptr) (((uintptr_t)(ptr)&0x0f)==0)
#define SCE_PFX_PTR_IS_ALIGNED128(ptr) (((uintptr_t)(ptr)&0x7f)==0)

#define SCE_PFX_GET_POINTER(offset,stride,id) ((uintptr_t)(offset)+(stride)*(id))

#define SCE_PFX_FLT_MAX 1e+38f
#define SCE_PFX_PI 3.14159265358979f

#define SCE_PFX_RANGE_CHECK(val,minVal,maxVal) (((val)>=(minVal))&&((val)<=(maxVal)))

#define SCE_PFX_IS_RUNNING_ON_64BIT_ENV() ( ( sizeof(void*)==8 )? true : false )

#if defined(__SNC__) || defined(__GNUC__)
	#define SCE_PFX_PADDING(count,bytes) PfxUInt8 padding##count[bytes];
#else
	#define SCE_PFX_PADDING(count,bytes)
#endif

#include "pfx_error_code.h"

#endif // _SCE_PFX_COMMON_H
