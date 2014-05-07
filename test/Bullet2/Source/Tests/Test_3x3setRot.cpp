//
//  Test_3x3setRot.cpp
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc.
//



#include "LinearMath/btScalar.h"
#if defined (BT_USE_SSE_IN_API) || defined (BT_USE_NEON)

#include "Test_3x3setRot.h"
#include "vector.h"
#include "Utils.h"
#include "main.h"
#include <math.h>
#include <string.h>

#include <LinearMath/btMatrix3x3.h>

#define LOOPCOUNT 1000
#define ARRAY_SIZE 128

static inline btSimdFloat4 rand_f4(void)
{
    return btAssign128( RANDF_01, RANDF_01, RANDF_01, BT_NAN );      // w channel NaN
}

static inline btSimdFloat4 qtrand_f4(void)
{
    return btAssign128( RANDF_01, RANDF_01, RANDF_01, RANDF_01 );
}

static btMatrix3x3 M3x3setRot_ref( btMatrix3x3 &m, const btQuaternion &q )
{
    btScalar d = q.length2();
    btScalar s = btScalar(2.0) / d;

    btScalar xs = q.x() * s,   ys = q.y() * s,   zs = q.z() * s;
    
    btScalar wx = q.w() * xs,  wy = q.w() * ys,  wz = q.w() * zs;
    btScalar xx = q.x() * xs,  xy = q.x() * ys,  xz = q.x() * zs;
    btScalar yy = q.y() * ys,  yz = q.y() * zs,  zz = q.z() * zs;
    m.setValue(
        btScalar(1.0) - (yy + zz), xy - wz, xz + wy,
        xy + wz, btScalar(1.0) - (xx + zz), yz - wx,
        xz - wy, yz + wx, btScalar(1.0) - (xx + yy));

    return m;
}


static int operator!= ( const btMatrix3x3 &a, const btMatrix3x3 &b )
{
    int i; 
    btVector3 av3, bv3;

    for(i=0; i<3; i++)
    {
        av3 = a.getRow(i);
        bv3 = b.getRow(i);
        
        if( fabs(av3.m_floats[0] - bv3.m_floats[0]) + 
            fabs(av3.m_floats[1] - bv3.m_floats[1]) +
            fabs(av3.m_floats[2] - bv3.m_floats[2]) > FLT_EPSILON * 4)
            return 1;
    }
    
    return 0;
}

int Test_3x3setRot(void)
{
    // Init an array flanked by guard pages
    btMatrix3x3     in1[ARRAY_SIZE];
    btQuaternion    in2[ARRAY_SIZE];
    btMatrix3x3     in3[ARRAY_SIZE];
    btMatrix3x3     out[ARRAY_SIZE];
    btMatrix3x3     out2[ARRAY_SIZE];
    
    // Init the data
    size_t i, j;
    for( i = 0; i < ARRAY_SIZE; i++ )
    {
        in1[i] = btMatrix3x3(rand_f4(), rand_f4(), rand_f4() );   
        in2[i] = btQuaternion(qtrand_f4());   
        in3[i] = in1[i];
        
        out[i] = M3x3setRot_ref(in1[i], in2[i]);
        in3[i].setRotation(in2[i]);
        out2[i] = in3[i];

        if( out[i] != out2[i] )
        {
            vlog( "Error - M3x3setRot result error! ");
            vlog( "failure @ %ld\n", i);
            btVector3 m0, m1, m2;
            m0 = out[i].getRow(0);
            m1 = out[i].getRow(1);
            m2 = out[i].getRow(2);
            
            vlog(   "\ncorrect = (%10.7f, %10.7f, %10.7f, %10.7f) "
					"\n          (%10.7f, %10.7f, %10.7f, %10.7f) "
                    "\n          (%10.7f, %10.7f, %10.7f, %10.7f) \n",
                    m0.m_floats[0], m0.m_floats[1], m0.m_floats[2], m0.m_floats[3], 
                    m1.m_floats[0], m1.m_floats[1], m1.m_floats[2], m1.m_floats[3],
                    m2.m_floats[0], m2.m_floats[1], m2.m_floats[2], m2.m_floats[3]); 

            m0 = out2[i].getRow(0);
            m1 = out2[i].getRow(1);
            m2 = out2[i].getRow(2);
					
            vlog(   "\ntested  = (%10.7f, %10.7f, %10.7f, %10.7f) "
					"\n          (%10.7f, %10.7f, %10.7f, %10.7f) " 
					"\n          (%10.7f, %10.7f, %10.7f, %10.7f) \n", 
					m0.m_floats[0], m0.m_floats[1], m0.m_floats[2], m0.m_floats[3], 
                    m1.m_floats[0], m1.m_floats[1], m1.m_floats[2], m1.m_floats[3],
                    m2.m_floats[0], m2.m_floats[1], m2.m_floats[2], m2.m_floats[3]); 

            return -1;
        }
    }
    
    uint64_t scalarTime, vectorTime;
    uint64_t startTime, bestTime, currentTime;
    bestTime = -1LL;
    scalarTime = 0;
    for (j = 0; j < LOOPCOUNT; j++) 
    {
        startTime = ReadTicks();
        for( i = 0; i < ARRAY_SIZE; i++ )
            out[i] = M3x3setRot_ref(in1[i], in2[i]);
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
    for (j = 0; j < LOOPCOUNT; j++) 
    {
        startTime = ReadTicks();
        for( i = 0; i < ARRAY_SIZE; i++ )
        {
            in3[i].setRotation(in2[i]);
            out2[i] = in3[i];
        }
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

