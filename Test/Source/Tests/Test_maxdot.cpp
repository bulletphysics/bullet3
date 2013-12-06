//
//  Test_maxdot.cpp
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc.
//



#include "LinearMath/btScalar.h"
#if defined (BT_USE_SSE_IN_API) || defined (BT_USE_NEON)


#include "Test_maxdot.h"
#include "vector.h"
#include "Utils.h"
#include "main.h"
#include <math.h>
#include <string.h>

#include <LinearMath/btVector3.h>


// reference code for testing purposes
static long maxdot_ref(    const btSimdFloat4 *vertices, 
                float *vec,
                size_t count, 
                float *dotResult );





#ifdef __arm__
    #define MAX_LOG2_SIZE   9
#else
    #define MAX_LOG2_SIZE   10
#endif
#define MAX_SIZE        (1U << MAX_LOG2_SIZE)
#define LOOPCOUNT 10

int Test_maxdot(void)
{
    // Init an array flanked by guard pages
    btSimdFloat4 *data = (btSimdFloat4*) GuardCalloc( 1, MAX_SIZE * sizeof(btSimdFloat4), NULL );
    float *fp = (float*) data;
    long correct, test;
    btVector3 localScaling( 0.1f, 0.2f, 0.3f);
    size_t size;
    
    // Init the data
    size_t i;
    for( i = 0; i < MAX_SIZE; i++ )
    {
        fp[4*i] = (int32_t) RANDF_16;
        fp[4*i+1] = (int32_t) RANDF_16;
        fp[4*i+2] = (int32_t) RANDF_16;
        fp[4*i+3] = BT_NAN;     // w channel NaN
    }
    
    float correctDot, testDot;
    fp = (float*) localScaling;
	float maxRelativeError = 0.f;
	
    for( size = 1; size <= MAX_SIZE; size++ )
    {
        float *in = (float*)(data + MAX_SIZE - size);
        size_t position;
        
        for( position = 0; position < size; position++ )
        {
            float *biggest = in + position * 4;
            float old[4] = { biggest[0], biggest[1], biggest[2], biggest[3] };
            biggest[0] += LARGE_FLOAT17;
            biggest[1] += LARGE_FLOAT17;
            biggest[2] += LARGE_FLOAT17;
            biggest[3] += LARGE_FLOAT17;
            
            correctDot = BT_NAN;
            testDot = BT_NAN;
            correct = maxdot_ref( (btSimdFloat4*) in, (float*) &localScaling, size, &correctDot);
            test = localScaling.maxDot( (btVector3*) in, size, testDot);
            if( test < 0 || test >= size )
            {
                vlog( "Error @ %ld: index out of bounds! *%ld vs %ld \n", size, correct, test);
                continue;
            }
            if( correct != test )
			{
                vlog( "Error @ %ld: index misreported! *%ld vs %ld  (*%f, %f)\n", size, correct, test, 
                       fp[0] * in[4*correct] + fp[1] * in[4*correct+1]  + fp[2] * in[4*correct+2], 
                       fp[0] * in[4*test] + fp[1] * in[4*test+1]  + fp[2] * in[4*test+2] );
				return 1;
			}
            if( test != position )
			{
                vlog( "Biggest not found where it is supposed to be: *%ld vs %ld (*%f, %f)\n", position, test, 
                       fp[0] * in[4*test] + fp[1] * in[4*test+1]  + fp[2] * in[4*test+2],
                       fp[0] * in[4*position] + fp[1] * in[4*position+1]  + fp[2] * in[4*position+2]  );
				return 1;
			}

            if( correctDot != testDot )
			{
				float relativeError = btFabs((testDot - correctDot) / correctDot);
				if (relativeError>1e-6)
				{
                vlog( "Error @ %ld: dotpr misreported! *%f vs %f    (*%f, %f)\n", size, correctDot, testDot, 
                       fp[0] * in[4*correct] + fp[1] * in[4*correct+1]  + fp[2] * in[4*correct+2], 
                       fp[0] * in[4*test] + fp[1] * in[4*test+1]  + fp[2] * in[4*test+2]  );
				return 1;
				} else
				{
					if (maxRelativeError < relativeError)
					{
						maxRelativeError = relativeError;
#ifdef VERBOSE_WARNING
						sprintf(errStr,"Warning @ %ld: dotpr misreported! *%f vs %f    (*%f, %f)\n", size, correctDot, testDot, 
						   fp[0] * in[4*correct] + fp[1] * in[4*correct+1]  + fp[2] * in[4*correct+2], 
						   fp[0] * in[4*test] + fp[1] * in[4*test+1]  + fp[2] * in[4*test+2]);
#endif //VERBOSE_WARNING
					}
				}
			}
            
            memcpy( biggest, old, 16 );
        }
    }
    
	
	if (maxRelativeError)
	{
		printf("Warning: relative error = %e\n", maxRelativeError);
#ifdef VERBOSE_WARNING
		vlog(errStr);
#endif
	}

    uint64_t scalarTimes[33 + (MAX_LOG2_SIZE-5)];
    uint64_t vectorTimes[33 + (MAX_LOG2_SIZE-5)];
    size_t j, k;
    float *in = (float*) data;
    for( size = 1; size <= 32; size++ )
    {
        uint64_t startTime, bestTime, currentTime;
        
        bestTime = -1LL;
        scalarTimes[size] = 0;
        for (j = 0; j < 100; j++) {
            startTime = ReadTicks();
            for( k = 0; k < LOOPCOUNT; k++ )
                correct += maxdot_ref( (btSimdFloat4*) in, (float*) &localScaling, size, &correctDot);
            currentTime = ReadTicks() - startTime;
            scalarTimes[size] += currentTime;
            if( currentTime < bestTime )
                bestTime = currentTime;
        }
        if( 0 == gReportAverageTimes )
            scalarTimes[size] = bestTime;        
        else
            scalarTimes[size] /= 100;
    }
    
    uint64_t *timep = &scalarTimes[33];
    for( size = 64; size <= MAX_SIZE; size *= 2 )
    {
        uint64_t startTime, bestTime, currentTime;
        
        bestTime = -1LL;
        timep[0] =0;
        for (j = 0; j < 100; j++) {
            startTime = ReadTicks();
            for( k = 0; k < LOOPCOUNT; k++ )
                correct += maxdot_ref( (btSimdFloat4*) in, (float*) &localScaling, size, &correctDot);
            currentTime = ReadTicks() - startTime;
            timep[0] += currentTime;
            if( currentTime < bestTime )
                bestTime = currentTime;
        }
        if( 0 == gReportAverageTimes )
            timep[0] = bestTime;        
        else
            timep[0] /= 100;

        timep++;
    }

    for( size = 1; size <= 32; size++ )
    {
        uint64_t startTime, bestTime, currentTime;
        
        bestTime = -1LL;
        vectorTimes[size] = 0;
        for (j = 0; j < 100; j++) {
            startTime = ReadTicks();
            for( k = 0; k < LOOPCOUNT; k++ )
                test += localScaling.maxDot( (btVector3*) in, size, testDot);
            currentTime = ReadTicks() - startTime;
            vectorTimes[size] += currentTime;
            if( currentTime < bestTime )
                bestTime = currentTime;
        }
        if( 0 == gReportAverageTimes )
            vectorTimes[size] = bestTime;        
        else
            vectorTimes[size] /= 100;
    }
    
    timep = &vectorTimes[33];
    for( size = 64; size <= MAX_SIZE; size *= 2 )
    {
        uint64_t startTime, bestTime, currentTime;
        
        bestTime = -1LL;
        timep[0] =0;
        for (j = 0; j < 100; j++) {
            startTime = ReadTicks();
            for( k = 0; k < LOOPCOUNT; k++ )
                test += localScaling.maxDot( (btVector3*) in, size, testDot);
            currentTime = ReadTicks() - startTime;
            timep[0] += currentTime;
            if( currentTime < bestTime )
                bestTime = currentTime;
        }
        if( 0 == gReportAverageTimes )
            timep[0] = bestTime;        
        else
            timep[0] /= 100;
        
        timep++;
    }
    
    vlog( "Timing:\n" );
    vlog( " size\t    scalar\t    vector\n" );
    for( size = 1; size <= 32; size++ )
        vlog( "%5lu\t%10.2f\t%10.2f\n", size, TicksToCycles( scalarTimes[size] ) / LOOPCOUNT, TicksToCycles( vectorTimes[size] ) / LOOPCOUNT );
    size_t index = 33;
    for( size = 64; size <= MAX_SIZE; size *= 2 )
    {
        vlog( "%5lu\t%10.2f\t%10.2f\n", size, TicksToCycles( scalarTimes[index] ) / LOOPCOUNT, TicksToCycles( vectorTimes[index] ) / LOOPCOUNT );
        index++;
    }
    
    // Useless check to make sure that the timing loops are not optimized away
    if( test != correct )
        vlog( "Error: Test != correct: *%ld vs. %ld\n", correct, test);
    
    GuardFree(data);
    
    return 0;
}


static long maxdot_ref(    const btSimdFloat4 *vertices, 
                float *vec,
                size_t count, 
                float *dotResult )
{
    
    const float *dp = (const float*) vertices;
    float  maxDot = -BT_INFINITY;
    long i = 0;
    long ptIndex = -1;
    
    for( i = 0; i < count; i++ )
    {
        float dot = vec[0] * dp[0] + vec[1] * dp[1] + vec[2] * dp[2];   dp += 4;
        
        if( dot > maxDot )
        {
            maxDot = dot;
            ptIndex = i;
        }
    }
    
    *dotResult = maxDot;
    
    return ptIndex;
}

#endif //BT_USE_SSE
