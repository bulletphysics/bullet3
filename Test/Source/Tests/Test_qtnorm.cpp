//
//  Test_qtnorm.cpp
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc.
//



#include "LinearMath/btScalar.h"
#if defined (BT_USE_SSE_IN_API) || defined (BT_USE_NEON)


#include "Test_qtnorm.h"
#include "vector.h"
#include "Utils.h"
#include "main.h"
#include <math.h>
#include <string.h>

#include <LinearMath/btQuaternion.h>

#define BT_OP(a)     (a.normalize())
// reference code for testing purposes
static inline btQuaternion& qtnorm_ref(btQuaternion& q1);

static inline btQuaternion& qtnorm_ref(btQuaternion& q1)
{
    float dot =
        q1.x() * q1.x() + 
        q1.y() * q1.y() + 
        q1.z() * q1.z() + 
        q1.w() * q1.w();

	dot = 1.0f / sqrtf(dot);

    q1.setValue(q1.x()*dot, q1.y()*dot, q1.z()*dot, q1.w()*dot);

    return q1;
}

#define LOOPCOUNT 1024
#define NUM_CYCLES 1000

int Test_qtnorm(void)
{
    int i;
    btQuaternion q1, q2;
	float x, y, z, w, vNaN;
    vNaN = BT_NAN;     // w channel NaN
    
	btQuaternion correct_res, test_res;
	
    for (i=0; i<LOOPCOUNT; i++)
    {
        // Init the data
        x = RANDF_01;
        y = RANDF_01;
        z = RANDF_01;
        w = RANDF_01;
        q1.setValue(x,y,z,w);
        
        q2 = q1;

		correct_res.setValue(vNaN, vNaN, vNaN, vNaN); 
		test_res.setValue(vNaN, vNaN, vNaN, vNaN);
		correct_res = qtnorm_ref(q1);
		test_res = BT_OP(q2);
	   
		if( fabsf(correct_res.x() - test_res.x()) + 
			fabsf(correct_res.y() - test_res.y()) +
			fabsf(correct_res.z() - test_res.z()) +
			fabsf(correct_res.w() - test_res.w()) > FLT_EPSILON*10 )
        {	
			vlog( "Error - qtnorm result error! "
					"\ncorrect = (%10.7f, %10.7f, %10.7f, %10.7f) "
					"\ntested  = (%10.7f, %10.7f, %10.7f, %10.7f) \n", 
					correct_res.x(), correct_res.y(), 
                    correct_res.z(), correct_res.w(),
					test_res.x(), test_res.y(), 
                    test_res.z(), test_res.w());
		
			return 1;
		}
	}
    
#define DATA_SIZE LOOPCOUNT

	btQuaternion qt_arr0[DATA_SIZE];
	btQuaternion qt_arr1[DATA_SIZE];

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
            }

            startTime = ReadTicks();
            for( k = 0; k+4 <= LOOPCOUNT; k+=4 )
			{
				size_t km = (k & (DATA_SIZE-1)); 
                qt_arr0[km] = qtnorm_ref(qt_arr1[km]);km++;
                qt_arr0[km] = qtnorm_ref(qt_arr1[km]);km++;
                qt_arr0[km] = qtnorm_ref(qt_arr1[km]);km++;
                qt_arr0[km] = qtnorm_ref(qt_arr1[km]);
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
            }
        
            startTime = ReadTicks();
            for( k = 0; k+4 <= LOOPCOUNT; k+=4 )
			{
				size_t km = (k & (DATA_SIZE-1)); 
            	qt_arr0[km] = BT_OP(qt_arr1[km]);km++;
            	qt_arr0[km] = BT_OP(qt_arr1[km]);km++;
            	qt_arr0[km] = BT_OP(qt_arr1[km]);km++;
            	qt_arr0[km] = BT_OP(qt_arr1[km]);km++;
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
