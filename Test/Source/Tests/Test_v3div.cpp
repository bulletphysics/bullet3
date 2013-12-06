//
//  Test_v3div.cpp
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc.
//



#include "LinearMath/btScalar.h"
#if defined (BT_USE_SSE_IN_API) || defined (BT_USE_NEON)


#include "Test_v3div.h"
#include "vector.h"
#include "Utils.h"
#include "main.h"
#include <math.h>
#include <string.h>

#include <LinearMath/btVector3.h>

#define BT_OP(a, b)     ((a) / (b))
// reference code for testing purposes
static inline btVector3& v3div_ref(btVector3& v1, btVector3& v2);

static btVector3& v3div_ref(btVector3& v0, btVector3& v1, btVector3& v2)
{
	v0.m_floats[0] = BT_OP(v1.m_floats[0] , v2.m_floats[0]), 
	v0.m_floats[1] = BT_OP(v1.m_floats[1] , v2.m_floats[1]),
	v0.m_floats[2] = BT_OP(v1.m_floats[2] , v2.m_floats[2]);
	
	return v0;
}

#define LOOPCOUNT 1024
#define NUM_CYCLES 1000

int Test_v3div(void)
{
    btVector3 v1, v2, v3;
	
    float x,y,z,w;
    
    // Init the data
    x = RANDF_01;
    y = RANDF_01;
    z = RANDF_01;
    w = BT_NAN;     // w channel NaN
    v1.setValue(x,y,z);
	v1.setW(w);

    x = RANDF_01;
    y = RANDF_01;
    z = RANDF_01;
    v2.setValue(x,y,z);
	v2.setW(w);

	v3 = v1;
		
    btVector3 correct_res, test_res;
	 
    {
		float vNaN = BT_NAN;
		correct_res.setValue(vNaN, vNaN, vNaN); 
		test_res.setValue(vNaN, vNaN, vNaN);
		correct_res = v3div_ref(correct_res, v1, v2);
		test_res = BT_OP(v3,v2);
	   
		if( fabsf(correct_res.m_floats[0] - test_res.m_floats[0]) + 
			fabsf(correct_res.m_floats[1] - test_res.m_floats[1]) +
			fabsf(correct_res.m_floats[2] - test_res.m_floats[2]) > FLT_EPSILON*10 )
		{	
			vlog( "Error - v3div result error! "
					"\ncorrect = (%10.4f, %10.4f, %10.4f) "
					"\ntested  = (%10.4f, %10.4f, %10.4f) \n", 
					correct_res.m_floats[0], correct_res.m_floats[1], correct_res.m_floats[2], 
					test_res.m_floats[0], test_res.m_floats[1], test_res.m_floats[2]);
		
			return 1;
		}
	}
    
#define DATA_SIZE LOOPCOUNT

	btVector3 vec3_arr0[DATA_SIZE];
	btVector3 vec3_arr1[DATA_SIZE];
	btVector3 vec3_arr2[DATA_SIZE];

    uint64_t scalarTime;
    uint64_t vectorTime;
    size_t j, k;

	{
        uint64_t startTime, bestTime, currentTime;
		w = BT_NAN;     // w channel NaN
        
        bestTime = -1LL;
        scalarTime = 0;
        for (j = 0; j < NUM_CYCLES; j++) 
		{
			for( k = 0; k < DATA_SIZE; k++ )
			{
                x = RANDF_01;
                y = RANDF_01;
                z = RANDF_01;
				vec3_arr1[k].setValue(x,y,z);
				vec3_arr1[k].setW(w);

                x = RANDF_01;
                y = RANDF_01;
                z = RANDF_01;
				vec3_arr2[k].setValue(x,y,z);
				vec3_arr2[k].setW(w);
			}

            startTime = ReadTicks();
            for( k = 0; k < LOOPCOUNT; k++ )
			{
	            vec3_arr0[k] = v3div_ref(vec3_arr0[k], vec3_arr1[k], vec3_arr2[k]);
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
				vec3_arr1[k].setValue(x,y,z);
				vec3_arr1[k].setW(w);

                x = RANDF_01;
                y = RANDF_01;
                z = RANDF_01;
				vec3_arr2[k].setValue(x,y,z);
				vec3_arr2[k].setW(w);
			}

            startTime = ReadTicks();
            for( k = 0; k < LOOPCOUNT; k++ )
			{
				vec3_arr0[k] = BT_OP(vec3_arr1[k] , vec3_arr2[k]);
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
