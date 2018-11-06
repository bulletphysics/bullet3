//
//  Test_v3rotate.cpp
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc.
//

#include "LinearMath/btScalar.h"
#if defined(BT_USE_SSE_IN_API) || defined(BT_USE_NEON)

#include "Test_v3rotate.h"
#include "vector.h"
#include "Utils.h"
#include "main.h"
#include <math.h>
#include <string.h>

#include <LinearMath/btVector3.h>

// reference code for testing purposes
static inline btVector3& v3rotate_ref(
	btVector3& v0,
	btVector3& v1,
	const btScalar& s);

#define LOOPCOUNT 2048
#define NUM_CYCLES 1000

int Test_v3rotate(void)
{
	btVector3 v1, v2;
	float s;

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

	s = RANDF_01 * (float)SIMD_PI;

	btVector3 correct_res, test_res;

	{
		float vNaN = BT_NAN;
		correct_res.setValue(vNaN, vNaN, vNaN);
		test_res.setValue(vNaN, vNaN, vNaN);
		test_res = v1.rotate(v2, s);
		correct_res = v3rotate_ref(v1, v2, s);

		if (fabs(correct_res.m_floats[0] - test_res.m_floats[0]) +
				fabs(correct_res.m_floats[1] - test_res.m_floats[1]) +
				fabs(correct_res.m_floats[2] - test_res.m_floats[2]) >
			FLT_EPSILON * 4)
		{
			vlog(
				"Error - v3rotate result error! "
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
	btScalar s_arr[DATA_SIZE];

	uint64_t scalarTime;
	uint64_t vectorTime;
	size_t j, k;

	{
		uint64_t startTime, bestTime, currentTime;

		bestTime = -1LL;
		scalarTime = 0;
		for (j = 0; j < NUM_CYCLES; j++)
		{
			for (k = 0; k < DATA_SIZE; k++)
			{
				x = RANDF_01;
				y = RANDF_01;
				z = RANDF_01;
				vec3_arr0[k].setValue(x, y, z);
				vec3_arr0[k].setW(w);

				x = RANDF_01;
				y = RANDF_01;
				z = RANDF_01;
				vec3_arr1[k].setValue(x, y, z);
				vec3_arr1[k].setW(w);

				s_arr[k] = RANDF_01 * (float)SIMD_PI;
			}

			startTime = ReadTicks();
			for (k = 0; k < LOOPCOUNT; k++)
			{
				vec3_arr0[k] = v3rotate_ref(vec3_arr0[k], vec3_arr1[k], s_arr[k]);
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
				vec3_arr0[k].setValue(x, y, z);
				vec3_arr0[k].setW(w);

				x = RANDF_01;
				y = RANDF_01;
				z = RANDF_01;
				vec3_arr1[k].setValue(x, y, z);
				vec3_arr1[k].setW(w);

				s_arr[k] = RANDF_01 * (float)SIMD_PI;
			}

			startTime = ReadTicks();
			for (k = 0; k < LOOPCOUNT; k++)
			{
				vec3_arr0[k] = vec3_arr0[k].rotate(vec3_arr1[k], s_arr[k]);
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

static inline btVector3&
v3rotate_ref(
	btVector3& v0,
	btVector3& wAxis,
	const btScalar& _angle)
{
	btVector3 o = wAxis * wAxis.dot(v0);
	btVector3 _x = v0 - o;
	btVector3 _y;

	_y = wAxis.cross(v0);

	v0 = o + _x * cosf(_angle) + _y * sinf(_angle);

	return v0;
}

#endif  //BT_USE_SSE
