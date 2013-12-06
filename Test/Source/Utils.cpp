//
//  File.c
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc.
//

#include <stdio.h>
#ifdef __APPLE__
#include <mach/mach_time.h>
#include <sys/sysctl.h>
#include <sys/mman.h>
#include <errno.h>
#else
#include "LinearMath/btAlignedAllocator.h"

#endif //__APPLE__

#include <stdlib.h>

#include "Utils.h"

#pragma mark Timing

int gReportNanoseconds = 0;

#ifdef _WIN32
#include <intrin.h>
uint64_t ReadTicks( void )
{
	 return __rdtsc();
}
double  TicksToCycles( uint64_t delta )
{
	return double(delta);
}

double  TicksToSeconds( uint64_t delta )
{
	return double(delta);
}

void *GuardCalloc( size_t count, size_t size, size_t *objectStride )
{
	if (objectStride)
		*objectStride = size;
	return (void*) btAlignedAlloc(count * size,16);
}
void GuardFree( void *buf )
{
	btAlignedFree(buf);
}

#endif


#ifdef __APPLE__

uint64_t ReadTicks( void )
{
    return mach_absolute_time();
}

double  TicksToCycles( uint64_t delta )
{
    static long double conversion = 0.0L;
    if( 0.0L == conversion )
    {
        // attempt to get conversion to nanoseconds
        mach_timebase_info_data_t info;
        int err = mach_timebase_info( &info );
        if( err )
            return __builtin_nanf("");
        conversion = (long double) info.numer / info.denom;
        
        // attempt to get conversion to cycles
        if( 0 == gReportNanoseconds )
        {
            uint64_t frequency = 0;
            size_t freq_size = sizeof( frequency );
            err = sysctlbyname( "hw.cpufrequency_max", &frequency, &freq_size, NULL, 0 );
            if( err || 0 == frequency )
                vlog( "Failed to get max cpu frequency. Reporting times as nanoseconds.\n" );
            else
            {
                conversion *= 1e-9L /* sec / ns */  * frequency /* cycles / sec */;
                vlog( "Reporting times as cycles. (%2.2f MHz)\n", 1e-6 * frequency );
            }
        }
        else
            vlog( "Reporting times as nanoseconds.\n" );
    }
    
    return (double) (delta * conversion);
}

double  TicksToSeconds( uint64_t delta )
{
    static long double conversion = 0.0L;
    if( 0.0L == conversion )
    {
        // attempt to get conversion to nanoseconds
        mach_timebase_info_data_t info;
        int err = mach_timebase_info( &info );
        if( err )
            return __builtin_nanf("");
        conversion = info.numer / (1e9L * info.denom);
    }
    
    return (double) (delta * conversion);
}



#pragma mark -
#pragma mark GuardCalloc

#define kPageSize 4096


typedef struct BufInfo
{
    void    *head;
    size_t  count;
    size_t  stride;
    size_t  totalSize;
}BufInfo;

static int GuardMarkBuffer( void *buffer, int flag );

void *GuardCalloc( size_t count, size_t size, size_t *objectStride )
{
    if( objectStride )
        *objectStride = 0;
    
    // Round size up to a multiple of a page size
    size_t stride = (size + kPageSize - 1) & -kPageSize;
    
    //Calculate total size of the allocation
    size_t totalSize = count * (stride + kPageSize) + kPageSize;

    // Allocate
    char *buf = (char*)mmap( NULL, 
                     totalSize, 
                     PROT_READ | PROT_WRITE, 
                     MAP_ANON | MAP_SHARED,
                     0, 0 );
    if( MAP_FAILED == buf )
    {
        vlog( "mmap failed: %d\n", errno );
        return NULL;
    }

    // Find the first byte of user data
    char *result = buf + kPageSize;

    // Record what we did for posterity
    BufInfo *bptr = (BufInfo*) result - 1;
    bptr->head = buf;
    bptr->count = count;
    bptr->stride = stride;
    bptr->totalSize = totalSize;
    
    // Place the first guard page. Masks our record above.
    if( mprotect(buf, kPageSize, PROT_NONE) )
    {
        munmap( buf, totalSize);
        vlog( "mprotect -1 failed: %d\n", errno );
        return NULL;
    }
    
    // Place the rest of the guard pages
    size_t i;
    char *p = result;
    for( i = 0; i < count; i++ )
    {
        p += stride;
        if( mprotect(p, kPageSize, PROT_NONE) )
        {
            munmap( buf, totalSize);
            vlog( "mprotect %lu failed: %d\n", i, errno );
            return NULL;
        }
        p += kPageSize;
    }
    
    // record the stride from object to object
    if( objectStride )
        *objectStride = stride + kPageSize;
    
    // return pointer to first object
    return result;
}


void GuardFree( void *buf )
{
    if( mprotect((char*)buf - kPageSize, kPageSize, PROT_READ) )
    {
        vlog( "Unable to read buf info. GuardFree failed! %p  (%d)\n", buf, errno );
        return;
    }
    
    BufInfo *bptr = (BufInfo*) buf - 1;
    
    if( munmap( bptr->head, bptr->totalSize ) )
        vlog( "Unable to unmap data. GuardFree failed! %p (%d)\n", buf, errno );
}

int GuardMarkReadOnly( void *buf )
{
    return GuardMarkBuffer(buf, PROT_READ);
}

int GuardMarkReadWrite( void *buf)
{
    return GuardMarkBuffer(buf, PROT_READ | PROT_WRITE);
}

int GuardMarkWriteOnly( void *buf)
{
    return GuardMarkBuffer(buf, PROT_WRITE);
}

static int GuardMarkBuffer( void *buf, int flag )
{
    if( mprotect((char*)buf - kPageSize, kPageSize, PROT_READ) )
    {
        vlog( "Unable to read buf info. GuardMarkBuffer %d failed! %p  (%d)\n", flag, buf, errno );
        return errno;
    }
    
    BufInfo *bptr = (BufInfo*) buf - 1;
    
    size_t count = bptr->count;
    size_t stride = bptr->stride;
    
    size_t i;
    for( i = 0; i < count; i++ )
    {
        if( mprotect(buf, stride, flag) )
        {
            vlog( "Unable to protect segment %ld. GuardMarkBuffer %d failed! %p  (%d)\n", i, flag, buf, errno );
            return errno;
        }
        bptr += stride + kPageSize;
    }
        
    if( mprotect((char*)buf - kPageSize, kPageSize, PROT_NONE) )
    {
        vlog( "Unable to protect leading guard page. GuardMarkBuffer %d failed! %p  (%d)\n", flag, buf, errno );
        return errno;
    }
    
    return 0;
}
#endif

uint32_t random_number32(void)
{
    return ((uint32_t) rand() << 16) ^ rand();
}


uint64_t random_number64(void)
{
    return ((uint64_t) rand() << 48) ^
            ((uint64_t) rand() << 32) ^
            ((uint64_t) rand() << 16) ^
            rand();
}

