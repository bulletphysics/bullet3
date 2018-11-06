//
//  Test_v3triple.cpp
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc.
//

#include "LinearMath/btScalar.h"
#if defined(BT_USE_SSE_IN_API) || defined(BT_USE_NEON)

#include "Test_v3triple.h"
#include "vector.h"
#include "Utils.h"
#include "main.h"
#include <math.h>
#include <string.h>

#include <LinearMath/btVector3.h>

// reference code for testing purposes
static btScalar
v3triple_ref(
	const btVector3& v,
	const btVector3& v1,
	const btVector3& v2);

#define LOOPCOUNT 1024
#define NUM_CYCLES 10000

int Test_v3triple(void)
{
	btVector3 v1, v2, v3;

	float x, y, z, w;

	// Init the data
	x = RANDF_01;
	y = RANDF_01;
	z = RANDF_01;
	w = BT_NAN;  // w channel NaN
	v1.setValue(x, y, z);
	v1.setW(w);

	x = RANDF_01;
	y = RANDF_01;
	z = RANDF_01;
	v2.setValue(x, y, z);
	v2.setW(w);

	x = RANDF_01;
	y = RANDF_01;
	z = RANDF_01;
	v3.setValue(x, y, z);
	v3.setW(w);

	float correctTriple0, testTriple0;

	{
		correctTriple0 = w;
		testTriple0 = w;
		testTriple0 = v3triple_ref(v1, v2, v3);
		correctTriple0 = v1.triple(v2, v3);

		if (fabsf(correctTriple0 - testTriple0) > FLT_EPSILON * 4)
		{
			vlog("Error - v3triple result error! %f != %f \n", correctTriple0, testTriple0);

			return 1;
		}
	}

#define DATA_SIZE 1024

	btVector3 v3_arr1[DATA_SIZE];
	btVector3 v3_arr2[DATA_SIZE];
	btVector3 v3_arr3[DATA_SIZE];
	btScalar res_arr[DATA_SIZE];

	uint64_t scalarTime;
	uint64_t vectorTime;
	size_t j, k;

	for (k = 0; k < DATA_SIZE; k++)
	{
		x = RANDF_01;
		y = RANDF_01;
		z = RANDF_01;
		v3_arr1[k].setValue(x, y, z);
		v3_arr1[k].setW(w);

		x = RANDF_01;
		y = RANDF_01;
		z = RANDF_01;
		v3_arr2[k].setValue(x, y, z);
		v3_arr2[k].setW(w);

		x = RANDF_01;
		y = RANDF_01;
		z = RANDF_01;
		v3_arr3[k].setValue(x, y, z);
		v3_arr3[k].setW(w);
	}

	{
		uint64_t startTime, bestTime, currentTime;

		bestTime = uint64_t(-1LL);
		scalarTime = 0;
		for (j = 0; j < NUM_CYCLES; j++)
		{
			startTime = ReadTicks();
			for (k = 0; k + 4 <= LOOPCOUNT; k += 4)
			{
				size_t k32 = (k & (DATA_SIZE - 1));
				res_arr[k32] = v3triple_ref(v3_arr1[k32], v3_arr2[k32], v3_arr3[k32]);
				k32++;
				res_arr[k32] = v3triple_ref(v3_arr1[k32], v3_arr2[k32], v3_arr3[k32]);
				k32++;
				res_arr[k32] = v3triple_ref(v3_arr1[k32], v3_arr2[k32], v3_arr3[k32]);
				k32++;
				res_arr[k32] = v3triple_ref(v3_arr1[k32], v3_arr2[k32], v3_arr3[k32]);
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

		bestTime = uint64_t(-1LL);
		vectorTime = 0;
		for (j = 0; j < NUM_CYCLES; j++)
		{
			startTime = ReadTicks();
			for (k = 0; k + 4 <= LOOPCOUNT; k += 4)
			{
				size_t k32 = k & (DATA_SIZE - 1);
				res_arr[k32] = v3_arr1[k32].triple(v3_arr2[k32], v3_arr3[k32]);
				k32++;
				res_arr[k32] = v3_arr1[k32].triple(v3_arr2[k32], v3_arr3[k32]);
				k32++;
				res_arr[k32] = v3_arr1[k32].triple(v3_arr2[k32], v3_arr3[k32]);
				k32++;
				res_arr[k32] = v3_arr1[k32].triple(v3_arr2[k32], v3_arr3[k32]);
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

static btScalar
v3triple_ref(
	const btVector3& v,
	const btVector3& v1,
	const btVector3& v2)
{
	return v.m_floats[0] * (v1.m_floats[1] * v2.m_floats[2] - v1.m_floats[2] * v2.m_floats[1]) +
		   v.m_floats[1] * (v1.m_floats[2] * v2.m_floats[0] - v1.m_floats[0] * v2.m_floats[2]) +
		   v.m_floats[2] * (v1.m_floats[0] * v2.m_floats[1] - v1.m_floats[1] * v2.m_floats[0]);
}

#endif  //BT_USE_SSE
