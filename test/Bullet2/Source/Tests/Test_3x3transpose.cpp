//
//  Test_3x3transpose.cpp
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc.
//


#include "LinearMath/btScalar.h"
#if defined (BT_USE_SSE_IN_API) || defined (BT_USE_NEON)


#include "Test_3x3transpose.h"
#include "vector.h"
#include "Utils.h"
#include "main.h"
#include <math.h>
#include <string.h>

#include <LinearMath/btMatrix3x3.h>

#define LOOPCOUNT 1000
#define ARRAY_SIZE 1024 

static inline btSimdFloat4 rand_f4(void)
{
    return btAssign128( RANDF, RANDF, RANDF, BT_NAN );      // w channel NaN
}

static btMatrix3x3 Transpose( btMatrix3x3 &in )
{
    btVector3 row0 = in.getRow(0);
    btVector3 row1 = in.getRow(1);
    btVector3 row2 = in.getRow(2);
	btVector3 col0 = btAssign128(row0.x(), row1.x(), row2.x(), 0 );
	btVector3 col1 = btAssign128(row0.y(), row1.y(), row2.y(), 0 );
	btVector3 col2 = btAssign128(row0.z(), row1.z(), row2.z(), 0);
	return btMatrix3x3( col0, col1, col2);
}

static int operator!= ( const btMatrix3x3 &a, const btMatrix3x3 &b )
{
    if( a.getRow(0) != b.getRow(0) )
        return 1;
    if( a.getRow(1) != b.getRow(1) )
        return 1;
    if( a.getRow(2) != b.getRow(2) )
        return 1;
    return 0;
}

int Test_3x3transpose(void)
{
    // Init an array flanked by guard pages
    btMatrix3x3 in[ARRAY_SIZE];
    btMatrix3x3 out[ARRAY_SIZE];
    btMatrix3x3 out2[ARRAY_SIZE];
    
    // Init the data
    size_t i, j;
    for( i = 0; i < ARRAY_SIZE; i++ )
    {
        in[i] = btMatrix3x3(rand_f4(), rand_f4(), rand_f4() );   
     
        out[i] = Transpose(in[i]);
        out2[i] = in[i].transpose();
        
        if( out[i] != out2[i] )
        {
            printf( "failure @ %ld\n", i);
            return -1;
        }
    }

    uint64_t scalarTime, vectorTime;
    uint64_t startTime, bestTime, currentTime;
    bestTime = -1LL;
    scalarTime = 0;
    for (j = 0; j < LOOPCOUNT; j++) {
        startTime = ReadTicks();
        for( i = 0; i < ARRAY_SIZE; i++ )
            out[i] = Transpose(in[i]);
        currentTime = ReadTicks() - startTime;
        scalarTime += currentTime;
        if( currentTime < bestTime )
            bestTime = currentTime;
    }
    if( 0 == gReportAverageTimes )
        scalarTime = bestTime;        
    else
        scalarTime /= LOOPCOUNT;

    bestTime = -1LL;
    vectorTime = 0;
    for (j = 0; j < LOOPCOUNT; j++) {
        startTime = ReadTicks();
        for( i = 0; i < ARRAY_SIZE; i++ )
            out[i] = in[i].transpose();
        currentTime = ReadTicks() - startTime;
        vectorTime += currentTime;
        if( currentTime < bestTime )
            bestTime = currentTime;
    }
    if( 0 == gReportAverageTimes )
        vectorTime = bestTime;        
    else
        vectorTime /= LOOPCOUNT;
        
    vlog( "Timing:\n" );
    vlog( "\t    scalar\t    vector\n" );
    vlog( "\t%10.2f\t%10.2f\n", TicksToCycles( scalarTime ) / ARRAY_SIZE, TicksToCycles( vectorTime ) / ARRAY_SIZE );
        
    return 0;
}
#endif //BT_USE_SSE

