//
//  Test_qtmulQV3.cpp
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc.
//

#include "LinearMath/btScalar.h"
#if defined(BT_USE_SSE_IN_API) || defined(BT_USE_NEON)

#include "Test_qtmulQV3.h"
#include "vector.h"
#include "Utils.h"
#include "main.h"
#include <math.h>
#include <string.h>

#include <LinearMath/btQuaternion.h>

#define BT_OP(a, b) ((a) * (b))
// reference code for testing purposes
static inline btQuaternion qtmulQV3_ref(const btQuaternion& q, const btVector3& w);

static inline btQuaternion qtmulQV3_ref(const btQuaternion& q, const btVector3& w)
{
	return btQuaternion(
		q.w() * w.x() + q.y() * w.z() - q.z() * w.y(),
		q.w() * w.y() + q.z() * w.x() - q.x() * w.z(),
		q.w() * w.z() + q.x() * w.y() - q.y() * w.x(),
		-q.x() * w.x() - q.y() * w.y() - q.z() * w.z());
}

#define LOOPCOUNT 1024
#define NUM_CYCLES 1000

static inline btSimdFloat4 rand_f4(void)
{
	return btAssign128(RANDF_m1p1, RANDF_m1p1, RANDF_m1p1, BT_NAN);  // w channel NaN
}

static inline btSimdFloat4 qtrand_f4(void)
{
	return btAssign128(RANDF_m1p1, RANDF_m1p1, RANDF_m1p1, RANDF_m1p1);
}

static inline btSimdFloat4 qtNAN_f4(void)
{
	return btAssign128(BT_NAN, BT_NAN, BT_NAN, BT_NAN);
}

int Test_qtmulQV3(void)
{
	btQuaternion q;
	btVector3 v3;

	// Init the data
	q = btQuaternion(qtrand_f4());
	v3 = btVector3(rand_f4());

	btQuaternion correct_res, test_res;
	correct_res = btQuaternion(qtNAN_f4());
	test_res = btQuaternion(qtNAN_f4());

	{
		correct_res = qtmulQV3_ref(q, v3);
		test_res = BT_OP(q, v3);

		if (fabsf(correct_res.x() - test_res.x()) +
				fabsf(correct_res.y() - test_res.y()) +
				fabsf(correct_res.z() - test_res.z()) +
				fabsf(correct_res.w() - test_res.w()) >
			FLT_EPSILON * 8)
		{
			vlog(
				"Error - qtmulQV3 result error! "
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

	btQuaternion qt_arrR[DATA_SIZE];
	btQuaternion qt_arr[DATA_SIZE];
	btVector3 v3_arr[DATA_SIZE];

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
				qt_arr[k] = btQuaternion(qtrand_f4());
				v3_arr[k] = btVector3(rand_f4());
			}

			startTime = ReadTicks();
			for (k = 0; k < LOOPCOUNT; k++)
			{
				qt_arrR[k] = qtmulQV3_ref(qt_arr[k], v3_arr[k]);
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
				qt_arr[k] = btQuaternion(qtrand_f4());
				v3_arr[k] = btVector3(rand_f4());
			}

			startTime = ReadTicks();
			for (k = 0; k < LOOPCOUNT; k++)
			{
				qt_arrR[k] = BT_OP(qt_arr[k], v3_arr[k]);
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
#endif  //BT_USE_SSE
