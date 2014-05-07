//
//  Test_v3sdiv.cpp
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc.
//



#include "LinearMath/btScalar.h"
#if defined (BT_USE_SSE_IN_API) || defined (BT_USE_NEON)


#include "Test_v3sdiv.h"
#include "vector.h"
#include "Utils.h"
#include "main.h"
#include <math.h>
#include <string.h>

#include <LinearMath/btVector3.h>

// reference code for testing purposes
static inline 
btVector3& v3sdiv_ref(
    btVector3& v, 
	const btScalar& s);

#define LOOPCOUNT 2048
#define NUM_CYCLES 1000

int Test_v3sdiv(void)
{
    btVector3 v1, v2;
	btScalar s;
   
    float x,y,z,w;
    
    // Init the data
    x = RANDF_01;
    y = RANDF_01;
    z = RANDF_01;
    w = BT_NAN;     // w channel NaN
    v1.setValue(x,y,z);
	v1.setW(w);

    v2.setValue(x,y,z);
	v2.setW(w);

    s = (float) RANDF_16;
    
    btVector3 correct_res, test_res;
	 
    {
		float vNaN = BT_NAN;
		correct_res.setValue(vNaN, vNaN, vNaN); 
		test_res.setValue(vNaN, vNaN, vNaN);
		correct_res = v3sdiv_ref(v1, s);
		test_res = (v2 /= s);
	   
		if( fabs(correct_res.m_floats[0] - test_res.m_floats[0]) +  
			fabs(correct_res.m_floats[1] - test_res.m_floats[1]) + 
			fabs(correct_res.m_floats[2] - test_res.m_floats[2]) > FLT_EPSILON * 4)
		{	
			vlog( "Error - v3sdiv result error! "
					"\ncorrect = (%10.4f, %10.4f, %10.4f) "
					"\ntested  = (%10.4f, %10.4f, %10.4f) \n", 
					correct_res.m_floats[0], correct_res.m_floats[1], correct_res.m_floats[2], 
					test_res.m_floats[0], test_res.m_floats[1], test_res.m_floats[2]);
		
			return 1;
		}
	}
    
#define DATA_SIZE LOOPCOUNT

	btVector3 vec3_arr[DATA_SIZE];
    btScalar  s_arr[DATA_SIZE];

    uint64_t scalarTime;
    uint64_t vectorTime;
    size_t j, k;

	{
        uint64_t startTime, bestTime, currentTime;
        
        bestTime = uint64_t(-1LL);
        scalarTime = 0;
        for (j = 0; j < NUM_CYCLES; j++) 
		{
			for( k = 0; k < DATA_SIZE; k++ )
			{
                x = RANDF_01;
                y = RANDF_01;
                z = RANDF_01;
				vec3_arr[k].setValue(x,y,z);
				vec3_arr[k].setW(w);
				
				s_arr[k] = RANDF_01;
			}

            startTime = ReadTicks();
            for( k = 0; k+4 <= LOOPCOUNT; k+=4 )
			{
                v3sdiv_ref( vec3_arr[k], s_arr[k]);
                v3sdiv_ref( vec3_arr[k+1], s_arr[k+1]);
                v3sdiv_ref( vec3_arr[k+2], s_arr[k+2]);
                v3sdiv_ref( vec3_arr[k+3], s_arr[k+3]);
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
				vec3_arr[k].setValue(x,y,z);
				vec3_arr[k].setW(w);
				
				s_arr[k] = RANDF_01;
			}

            startTime = ReadTicks();
            for( k = 0; k+4 <= LOOPCOUNT; k+=4 )
			{
				vec3_arr[k] /= s_arr[k];
				vec3_arr[k+1] /= s_arr[k+1];
				vec3_arr[k+2] /= s_arr[k+2];
				vec3_arr[k+3] /= s_arr[k+3];
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

static inline 
btVector3& 
v3sdiv_ref(
    btVector3& v, 
    const btScalar& s)
{
	btScalar recip = btScalar(1.0) / s;
	
	v.m_floats[0] *= recip; 
	v.m_floats[1] *= recip;
	v.m_floats[2] *= recip;

	return v;
}

#endif //BT_USE_SSE
