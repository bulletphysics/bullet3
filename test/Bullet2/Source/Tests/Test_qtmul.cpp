//
//  Test_qtmul.cpp
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc.
//



#include "LinearMath/btScalar.h"
#if defined (BT_USE_SSE_IN_API) || defined (BT_USE_NEON)


#include "Test_qtmul.h"
#include "vector.h"
#include "Utils.h"
#include "main.h"
#include <math.h>
#include <string.h>

#include <LinearMath/btQuaternion.h>

#define BT_OP(a, b)     ((a) *= (b))
// reference code for testing purposes
static inline btQuaternion& qtmul_ref(btQuaternion& q1, btQuaternion& q2);

static inline btQuaternion& qtmul_ref(btQuaternion& q1, btQuaternion& q2)
{
    float x,y,z,w;
    x = q1.w() * q2.x() + q1.x() * q2.w() + q1.y() * q2.z() - q1.z() * q2.y(),
    y = q1.w() * q2.y() + q1.y() * q2.w() + q1.z() * q2.x() - q1.x() * q2.z(),
    z = q1.w() * q2.z() + q1.z() * q2.w() + q1.x() * q2.y() - q1.y() * q2.x(),
    w = q1.w() * q2.w() - q1.x() * q2.x() - q1.y() * q2.y() - q1.z() * q2.z();

    q1.setValue(x, y, z, w);
	return q1;
}

#define LOOPCOUNT 1024
#define NUM_CYCLES 1000

int Test_qtmul(void)
{
    btQuaternion q1, q2, q3;
	
    float x, y, z, w, vNaN;
    
    // Init the data
    x = RANDF_01;
    y = RANDF_01;
    z = RANDF_01;
    w = RANDF_01;
    vNaN = BT_NAN;     // w channel NaN
    q1.setValue(x,y,z,w);
	
    x = RANDF_01;
    y = RANDF_01;
    z = RANDF_01;
    w = RANDF_01;
    q2.setValue(x,y,z,w);

	q3 = q1;
		
    btQuaternion correct_res, test_res;
	 
    {
		float vNaN = BT_NAN;
		correct_res.setValue(vNaN, vNaN, vNaN, vNaN); 
		test_res.setValue(vNaN, vNaN, vNaN, vNaN);
		correct_res = qtmul_ref(q1, q2);
		test_res = BT_OP(q3,q2);
	   
		if( fabsf(correct_res.x() - test_res.x()) + 
			fabsf(correct_res.y() - test_res.y()) +
			fabsf(correct_res.z() - test_res.z()) +
			fabsf(correct_res.w() - test_res.w()) > FLT_EPSILON*10 )
		{	
			vlog( "Error - qtmul result error! "
					"\ncorrect = (%10.4f, %10.4f, %10.4f, %10.4f) "
					"\ntested  = (%10.4f, %10.4f, %10.4f, %10.4f) \n", 
					correct_res.x(), correct_res.y(), 
                    correct_res.z(), correct_res.w(),
					test_res.x(), test_res.y(), 
                    test_res.z(), test_res.w());
		
			return 1;
		}
	}
    
#define DATA_SIZE LOOPCOUNT

	btQuaternion qt_arr1[DATA_SIZE];
	btQuaternion qt_arr2[DATA_SIZE];

    uint64_t scalarTime;
    uint64_t vectorTime;
    size_t j, k;

	{
        uint64_t startTime, bestTime, currentTime;
        
        bestTime = -1LL;
        scalarTime = 0;
        for (j = 0; j < NUM_CYCLES; j++) 
		{
			for( k = 0; k < DATA_SIZE; k++ )
			{
                x = RANDF_01;
                y = RANDF_01;
                z = RANDF_01;
				w = RANDF_01;
				qt_arr1[k].setValue(x,y,z,w);

                x = RANDF_01;
                y = RANDF_01;
                z = RANDF_01;
				w = RANDF_01;
				qt_arr2[k].setValue(x,y,z,w);
			}

            startTime = ReadTicks();
            for( k = 0; k < LOOPCOUNT; k++ )
			{
	            qt_arr1[k] = qtmul_ref(qt_arr1[k], qt_arr2[k]);
			}
			currentTime = ReadTicks() - startTime;
            scalarTime += currentTime;
            if( currentTime < bestTime )
                bestTime = currentTime;
        }
        if( 0 == gReportAverageTimes )
            scalarTime = bestTime;        
        else
            scalarTime /= NUM_CYCLES;
    }
    
    {
        uint64_t startTime, bestTime, currentTime;
        
        bestTime = -1LL;
        vectorTime = 0;
        for (j = 0; j < NUM_CYCLES; j++) 
		{
			for( k = 0; k < DATA_SIZE; k++ )
			{
                x = RANDF_01;
                y = RANDF_01;
                z = RANDF_01;
				w = RANDF_01;
				qt_arr1[k].setValue(x,y,z,w);

                x = RANDF_01;
                y = RANDF_01;
                z = RANDF_01;
				w = RANDF_01;
				qt_arr2[k].setValue(x,y,z,w);
			}

            startTime = ReadTicks();
            for( k = 0; k < LOOPCOUNT; k++ )
			{
				qt_arr1[k] = BT_OP(qt_arr1[k], qt_arr2[k]);
			}
			currentTime = ReadTicks() - startTime;
            vectorTime += currentTime;
            if( currentTime < bestTime )
                bestTime = currentTime;
        }
        if( 0 == gReportAverageTimes )
            vectorTime = bestTime;        
        else
            vectorTime /= NUM_CYCLES;
    }

    vlog( "Timing:\n" );
    vlog( "     \t    scalar\t    vector\n" );
    vlog( "    \t%10.4f\t%10.4f\n", TicksToCycles( scalarTime ) / LOOPCOUNT, 
									TicksToCycles( vectorTime ) / LOOPCOUNT );

    return 0;
}

#endif //BT_USE_SSE
