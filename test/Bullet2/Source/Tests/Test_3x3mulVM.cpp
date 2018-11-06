//
//  Test_3x3mulVM.cpp
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc.
//

#include "LinearMath/btScalar.h"
#if defined(BT_USE_SSE_IN_API) || defined(BT_USE_NEON)

#include "Test_3x3mulVM.h"
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
	return btAssign128(RANDF_01, RANDF_01, RANDF_01, BT_NAN);  // w channel NaN
}

static btVector3 M3x3mulVM_ref(const btVector3 &v, const btMatrix3x3 &m)
{
	return btVector3(m.tdotx(v), m.tdoty(v), m.tdotz(v));
}

int Test_3x3mulVM(void)
{
	// Init an array flanked by guard pages
	btVector3 in1[ARRAY_SIZE];
	btMatrix3x3 in2[ARRAY_SIZE];
	btVector3 out[ARRAY_SIZE];
	btVector3 out2[ARRAY_SIZE];

	// Init the data
	size_t i, j;
	for (i = 0; i < ARRAY_SIZE; i++)
	{
		in1[i] = btVector3(rand_f4());
		in2[i] = btMatrix3x3(rand_f4(), rand_f4(), rand_f4());

		out[i] = M3x3mulVM_ref(in1[i], in2[i]);
		out2[i] = (in1[i] * in2[i]);

		if (fabsf(out[i].m_floats[0] - out2[i].m_floats[0]) +
				fabsf(out[i].m_floats[1] - out2[i].m_floats[1]) +
				fabsf(out[i].m_floats[2] - out2[i].m_floats[2]) +
				fabsf(out[i].m_floats[3] - out2[i].m_floats[3]) >
			FLT_EPSILON * 4)
		{
			vlog("Error - M3x3mulVM result error! ");
			vlog("failure @ %ld\n", i);
			vlog(
				"\ncorrect = (%10.4f, %10.4f, %10.4f, %10.4f) "
				"\ntested  = (%10.4f, %10.4f, %10.4f, %10.4f) \n",
				out[i].m_floats[0], out[i].m_floats[1], out[i].m_floats[2], out[i].m_floats[3],
				out2[i].m_floats[0], out2[i].m_floats[1], out2[i].m_floats[2], out2[i].m_floats[3]);

			return 1;
		}
	}

	uint64_t scalarTime, vectorTime;
	uint64_t startTime, bestTime, currentTime;
	bestTime = -1LL;
	scalarTime = 0;
	for (j = 0; j < LOOPCOUNT; j++)
	{
		startTime = ReadTicks();
		for (i = 0; i < ARRAY_SIZE; i++)
			out[i] = M3x3mulVM_ref(in1[i], in2[i]);
		currentTime = ReadTicks() - startTime;
		scalarTime += currentTime;
		if (currentTime < bestTime)
			bestTime = currentTime;
	}
	if (0 == gReportAverageTimes)
		scalarTime = bestTime;
	else
		scalarTime /= LOOPCOUNT;

	bestTime = -1LL;
	vectorTime = 0;
	for (j = 0; j < LOOPCOUNT; j++)
	{
		startTime = ReadTicks();
		for (i = 0; i < ARRAY_SIZE; i++)
			out2[i] = (in1[i] * in2[i]);
		currentTime = ReadTicks() - startTime;
		vectorTime += currentTime;
		if (currentTime < bestTime)
			bestTime = currentTime;
	}
	if (0 == gReportAverageTimes)
		vectorTime = bestTime;
	else
		vectorTime /= LOOPCOUNT;

	vlog("Timing:\n");
	vlog("\t    scalar\t    vector\n");
	vlog("\t%10.2f\t%10.2f\n", TicksToCycles(scalarTime) / ARRAY_SIZE, TicksToCycles(vectorTime) / ARRAY_SIZE);

	return 0;
}
#endif  //BT_USE_SSE
