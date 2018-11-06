//
//  Test_v3lerp.cpp
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc.
//

#include "LinearMath/btScalar.h"
#if defined(BT_USE_SSE_IN_API) || defined(BT_USE_NEON)

#include "Test_v3lerp.h"
#include "vector.h"
#include "Utils.h"
#include "main.h"
#include <math.h>
#include <string.h>

#include <LinearMath/btVector3.h>

// reference code for testing purposes
static inline btVector3&
v3lerp_ref(
	btVector3& vr,
	btVector3& v0,
	btVector3& v1,
	btScalar& rt);

#define LOOPCOUNT 1024
#define NUM_CYCLES 1000

int Test_v3lerp(void)
{
	btVector3 v1, v2;
	btScalar rt;

	float x, y, z, w;

	float vNaN = BT_NAN;
	w = BT_NAN;  // w channel NaN

	btVector3 correct_res, test_res;

	for (rt = 0.0f; rt <= 1.0f; rt += 0.1f)
	{
		correct_res.setValue(vNaN, vNaN, vNaN);
		test_res.setValue(vNaN, vNaN, vNaN);

		// Init the data
		x = RANDF_01;
		y = RANDF_01;
		z = RANDF_01;
		v1.setValue(x, y, z);
		v1.setW(w);

		x = RANDF_01;
		y = RANDF_01;
		z = RANDF_01;
		v2.setValue(x, y, z);
		v2.setW(w);

		correct_res = v3lerp_ref(correct_res, v1, v2, rt);
		test_res = v1.lerp(v2, rt);

		if (fabs(correct_res.m_floats[0] - test_res.m_floats[0]) +
				fabs(correct_res.m_floats[1] - test_res.m_floats[1]) +
				fabs(correct_res.m_floats[2] - test_res.m_floats[2]) >
			FLT_EPSILON * 4)
		{
			vlog(
				"Error - v3lerp result error! "
				"\ncorrect = (%10.4f, %10.4f, %10.4f) "
				"\ntested  = (%10.4f, %10.4f, %10.4f) \n"
				"\n rt=%10.4f",
				correct_res.m_floats[0], correct_res.m_floats[1], correct_res.m_floats[2],
				test_res.m_floats[0], test_res.m_floats[1], test_res.m_floats[2], rt);

			return 1;
		}
	}

#define DATA_SIZE LOOPCOUNT

	btVector3 vec3_arr1[DATA_SIZE];
	btVector3 vec3_arr2[DATA_SIZE];
	btScalar rt_arr[DATA_SIZE];

	uint64_t scalarTime;
	uint64_t vectorTime;
	size_t j, k;

	{
		uint64_t startTime, bestTime, currentTime;
		w = BT_NAN;  // w channel NaN

		bestTime = -1LL;
		scalarTime = 0;
		for (j = 0; j < NUM_CYCLES; j++)
		{
			for (k = 0; k < DATA_SIZE; k++)
			{
				x = RANDF_01;
				y = RANDF_01;
				z = RANDF_01;
				vec3_arr1[k].setValue(x, y, z);
				vec3_arr1[k].setW(w);

				x = RANDF_01;
				y = RANDF_01;
				z = RANDF_01;
				vec3_arr2[k].setValue(x, y, z);
				vec3_arr2[k].setW(w);

				rt_arr[k] = RANDF_01;
			}

			startTime = ReadTicks();
			for (k = 0; k < LOOPCOUNT; k++)
			{
				v3lerp_ref(vec3_arr1[k], vec3_arr1[k], vec3_arr2[k], rt_arr[k]);
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
			for (k = 0; k < DATA_SIZE; k++)
			{
				x = RANDF_01;
				y = RANDF_01;
				z = RANDF_01;
				vec3_arr1[k].setValue(x, y, z);
				vec3_arr1[k].setW(w);

				x = RANDF_01;
				y = RANDF_01;
				z = RANDF_01;
				vec3_arr2[k].setValue(x, y, z);
				vec3_arr2[k].setW(w);

				rt_arr[k] = RANDF_01;
			}

			startTime = ReadTicks();
			for (k = 0; k < LOOPCOUNT; k++)
			{
				vec3_arr1[k] = vec3_arr1[k].lerp(vec3_arr2[k], rt_arr[k]);
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
	vlog("    \t%10.4f\t%10.4f\n", TicksToCycles(scalarTime) / LOOPCOUNT,
		 TicksToCycles(vectorTime) / LOOPCOUNT);

	return 0;
}

static btVector3&
v3lerp_ref(
	btVector3& vr,
	btVector3& v0,
	btVector3& v1,
	btScalar& rt)
{
	vr.m_floats[0] = v0.m_floats[0] + rt * (v1.m_floats[0] - v0.m_floats[0]);
	vr.m_floats[1] = v0.m_floats[1] + rt * (v1.m_floats[1] - v0.m_floats[1]);
	vr.m_floats[2] = v0.m_floats[2] + rt * (v1.m_floats[2] - v0.m_floats[2]);

	return vr;
}

#endif  //BT_USE_SSE
