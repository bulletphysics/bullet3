//
//  Test_v3dot.cpp
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc.
//

#include "LinearMath/btScalar.h"
#if defined(BT_USE_SSE_IN_API) || defined(BT_USE_NEON)

#include "Test_dot3.h"
#include "vector.h"
#include "Utils.h"
#include "main.h"
#include <math.h>
#include <string.h>

#include <LinearMath/btVector3.h>

// reference code for testing purposes
static btVector3 dot3_ref(const btVector3 &, const btVector3 &, const btVector3 &, const btVector3 &);
static btVector3 dot3_ref(const btVector3 &v, const btVector3 &v1, const btVector3 &v2, const btVector3 &v3)
{
	return btVector3(v.dot(v1), v.dot(v2), v.dot(v3));
}

/*
SIMD_FORCE_INLINE int operator!=(const btVector3 &s, const btVector3 &v)
{
#ifdef __SSE__
    __m128 test = _mm_cmpneq_ps( s.mVec128, v.mVec128 );
    return (_mm_movemask_ps( test ) & 7) != 0;
#elif defined __ARM_NEON_H
    uint32x4_t test = vandq_u32( vceqq_f32( s.mVec128, v.mVec128 ), (uint32x4_t){-1,-1,-1,0});
    uint32x2_t t = vpadd_u32( vget_low_u32(test), vget_high_u32(test));
    t = vpadd_u32(t, t);
    return -3 != (int32_t) vget_lane_u32(t, 0);
#else
    return  s.m_floats[0] != v.m_floats[0] ||
    s.m_floats[1] != v.m_floats[1] ||
    s.m_floats[2] != v.m_floats[2];
#endif
}
*/

#define LOOPCOUNT 1000
#define NUM_CYCLES 10000

int Test_dot3(void)
{
	btVector3 v, v1, v2, v3;

#define DATA_SIZE 1024

	btVector3 vec3_arr[DATA_SIZE];
	btVector3 vec3_arr1[DATA_SIZE];
	btVector3 vec3_arr2[DATA_SIZE];
	btVector3 vec3_arr3[DATA_SIZE];
	btVector3 res_arr[DATA_SIZE];

	uint64_t scalarTime;
	uint64_t vectorTime;
	size_t j, k;
	btVector3 correct, test;

	for (k = 0; k < DATA_SIZE; k++)
	{
		vec3_arr[k] = btVector3(btAssign128(RANDF, RANDF, RANDF, BT_NAN));
		vec3_arr1[k] = btVector3(btAssign128(RANDF, RANDF, RANDF, BT_NAN));
		vec3_arr2[k] = btVector3(btAssign128(RANDF, RANDF, RANDF, BT_NAN));
		vec3_arr3[k] = btVector3(btAssign128(RANDF, RANDF, RANDF, BT_NAN));

		correct = dot3_ref(vec3_arr[k], vec3_arr1[k], vec3_arr2[k], vec3_arr3[k]);
		test = vec3_arr[k].dot3(vec3_arr1[k], vec3_arr2[k], vec3_arr3[k]);

		if (correct != test)
		{
			vlog("Error (%ld) - dot3 result error! *{%a, %a, %a, %a} != {%a, %a, %a, %a} \n", k,
				 correct.x(), correct.y(), correct.z(), correct.w(),
				 test.x(), test.y(), test.z(), test.w());

			return 1;
		}
	}

	{
		uint64_t startTime, bestTime, currentTime;

		bestTime = -1LL;
		scalarTime = 0;
		for (j = 0; j < NUM_CYCLES; j++)
		{
			startTime = ReadTicks();
			for (k = 0; k + 4 <= LOOPCOUNT; k += 4)
			{
				size_t k32 = (k & (DATA_SIZE - 1));
				res_arr[k32] = dot3_ref(vec3_arr[k32], vec3_arr1[k32], vec3_arr2[k32], vec3_arr3[k32]);
				k32++;
				res_arr[k32] = dot3_ref(vec3_arr[k32], vec3_arr1[k32], vec3_arr2[k32], vec3_arr3[k32]);
				k32++;
				res_arr[k32] = dot3_ref(vec3_arr[k32], vec3_arr1[k32], vec3_arr2[k32], vec3_arr3[k32]);
				k32++;
				res_arr[k32] = dot3_ref(vec3_arr[k32], vec3_arr1[k32], vec3_arr2[k32], vec3_arr3[k32]);
			}
			currentTime = ReadTicks() - startTime;
			scalarTime += currentTime;
			if (currentTime < bestTime)
				bestTime = currentTime;
		}
		if (0 == gReportAverageTimes)
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
			startTime = ReadTicks();
			for (k = 0; k + 4 <= LOOPCOUNT; k += 4)
			{
				size_t k32 = (k & (DATA_SIZE - 1));
				res_arr[k32] = vec3_arr[k32].dot3(vec3_arr1[k32], vec3_arr2[k32], vec3_arr3[k32]);
				k32++;
				res_arr[k32] = vec3_arr[k32].dot3(vec3_arr1[k32], vec3_arr2[k32], vec3_arr3[k32]);
				k32++;
				res_arr[k32] = vec3_arr[k32].dot3(vec3_arr1[k32], vec3_arr2[k32], vec3_arr3[k32]);
				k32++;
				res_arr[k32] = vec3_arr[k32].dot3(vec3_arr1[k32], vec3_arr2[k32], vec3_arr3[k32]);
			}
			currentTime = ReadTicks() - startTime;
			vectorTime += currentTime;
			if (currentTime < bestTime)
				bestTime = currentTime;
		}
		if (0 == gReportAverageTimes)
			vectorTime = bestTime;
		else
			vectorTime /= NUM_CYCLES;
	}

	vlog("Timing:\n");
	vlog("     \t    scalar\t    vector\n");
	vlog("    \t%10.4f\t%10.4f\n", TicksToCycles(scalarTime) / LOOPCOUNT, TicksToCycles(vectorTime) / LOOPCOUNT);

	return 0;
}

#endif  //BT_USE_SSE
