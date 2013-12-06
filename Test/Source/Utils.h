//
//  Utils.h
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc.
//

#ifndef BulletTest_Utils_h
#define BulletTest_Utils_h

#include "btIntDefines.h"



#include <stddef.h>
#include <stdio.h>

#ifdef _WIN32
#define LARGE_FLOAT17 (1.f * powf(2,17))
#define RANDF_16   (random_number32() * powf(2,-16))
#define RANDF_01   ( random_number32() * powf(2,-32) )
#define RANDF      ( random_number32() * powf(2,-8) )
#define RANDF_m1p1 (2.0f*( random_number32() * powf(2,-32)-1.0f))
#else
#define LARGE_FLOAT17 (0x1.0p17f)
#define RANDF_16   (random_number32() * 0x1.0p-16f)
#define RANDF_01   ( random_number32() * 0x1.0p-32f )
#define RANDF      ( random_number32() * 0x1.0p-8f )
#define RANDF_m1p1 (2.0f*( random_number32() * 0x1.0p-32f )-1.0f)
#endif//_WIN32


#ifdef __cplusplus
extern "C" { 
#endif
    
    /*********************
     *      Timing       *
     *********************/
    extern int gReportNanoseconds;

    uint64_t ReadTicks( void );
    double  TicksToCycles( uint64_t delta );        // Performance data should be reported in cycles most of the time.
    double  TicksToSeconds( uint64_t delta );

    
    /*********************
     *    Guard Heap     *
     *********************/
    // return buffer containing count objects of size size, with guard pages in betweeen.
    // The stride between one object and the next is given by objectStride. 
    // objectStride may be NULL. Objects so created are freed with GuardFree
    void *GuardCalloc( size_t count, size_t size, size_t *objectStride );
    void GuardFree( void * );
    // mark the contents of a guard buffer read-only or write-only. Return 0 on success.
    int GuardMarkReadOnly( void *);
    int GuardMarkWriteOnly( void *);
    int GuardMarkReadWrite( void *);
    
    /*********************
     *    Printing       *
     *********************/
    #define vlog( ... )        printf( __VA_ARGS__  )
    uint32_t random_number32(void);    
    uint64_t random_number64(void);
    
#ifdef __cplusplus
    }
#endif


#endif
