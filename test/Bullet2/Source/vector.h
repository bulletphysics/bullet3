//
//  vector.h
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc.
//

#ifndef BulletTest_vector_h
#define BulletTest_vector_h

#ifdef __SSE__
    typedef float float4            __attribute__ ((__vector_size__(16)));
    #include <xmmintrin.h>
#endif

#ifdef __SSE2__
    typedef double double2          __attribute__ ((__vector_size__(16)));
    typedef char char16             __attribute__ ((__vector_size__(16)));
    typedef unsigned char uchar16   __attribute__ ((__vector_size__(16)));
    typedef short short8            __attribute__ ((__vector_size__(16)));
    typedef unsigned short ushort8  __attribute__ ((__vector_size__(16)));
    typedef int int4                __attribute__ ((__vector_size__(16)));
   // typedef unsigned int uint4      __attribute__ ((__vector_size__(16)));
    #ifdef __LP64__ 
        typedef long long2              __attribute__ ((__vector_size__(16)));
        typedef unsigned long ulong2    __attribute__ ((__vector_size__(16)));
    #else
        typedef long long long2         __attribute__ ((__vector_size__(16)));
        typedef unsigned long long ulong2 __attribute__ ((__vector_size__(16)));
    #endif
    #include <emmintrin.h> 
#endif

#ifdef __SSE3__
    #include <pmmintrin.h>
#endif

#ifdef __SSSE3__
    #include <tmmintrin.h>
#endif

#ifdef __SSE4_1__
    #include <smmintrin.h>
#endif

#ifdef __arm__
    #include <arm/arch.h>
    #ifdef _ARM_ARCH_7
        #define ARM_NEON_GCC_COMPATIBILITY  1
        #include <arm_neon.h>
        typedef float float4            __attribute__ ((__vector_size__(16)));
        typedef double double2          __attribute__ ((__vector_size__(16)));
        typedef char char16             __attribute__ ((__vector_size__(16)));
        typedef unsigned char uchar16   __attribute__ ((__vector_size__(16)));
        typedef short short8            __attribute__ ((__vector_size__(16)));
        typedef unsigned short ushort8  __attribute__ ((__vector_size__(16)));
        typedef int int4                __attribute__ ((__vector_size__(16)));
        typedef unsigned int uint4      __attribute__ ((__vector_size__(16)));
        #ifdef __LP64__ 
            typedef long long2              __attribute__ ((__vector_size__(16)));
            typedef unsigned long ulong2    __attribute__ ((__vector_size__(16)));
        #else
            typedef long long long2         __attribute__ ((__vector_size__(16)));
            typedef unsigned long long ulong2 __attribute__ ((__vector_size__(16)));
        #endif
    #endif
#endif


#endif
