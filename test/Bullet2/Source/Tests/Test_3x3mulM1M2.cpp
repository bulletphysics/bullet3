//
//  Test_3x3mulM1M2.cpp
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc.
//

#include "LinearMath/btScalar.h"
#if defined(BT_USE_SSE_IN_API) || defined(BT_USE_NEON)

#include "Test_3x3mulM1M2.h"
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

static btMatrix3x3 M3x3mulM1M2_ref(const btMatrix3x3 &m1, const btMatrix3x3 &m2)
{
	return btMatrix3x3(
		m2.tdotx(m1[0]), m2.tdoty(m1[0]), m2.tdotz(m1[0]),
		m2.tdotx(m1[1]), m2.tdoty(m1[1]), m2.tdotz(m1[1]),
		m2.tdotx(m1[2]), m2.tdoty(m1[2]), m2.tdotz(m1[2]));
}

static bool fuzzyEqualSlow(const btVector3 &ref, const btVector3 &other)
{
	const btScalar epsilon = SIMD_EPSILON;
	return ((btFabs(ref.m_floats[3] - other.m_floats[3]) <= epsilon) &&
			(btFabs(ref.m_floats[2] - other.m_floats[2]) <= epsilon) &&
			(btFabs(ref.m_floats[1] - other.m_floats[1]) <= epsilon) &&
			(btFabs(ref.m_floats[0] - other.m_floats[0]) <= epsilon));
}

static int operator!=(const btMatrix3x3 &a, const btMatrix3x3 &b)
{
	if (a.getRow(0) != b.getRow(0))
	{
		if (!fuzzyEqualSlow(a.getRow(0), b.getRow(0)))
		{
			return 1;
		}
	}
	if (a.getRow(1) != b.getRow(1))
	{
		if (!fuzzyEqualSlow(a.getRow(1), b.getRow(1)))
			return 1;
	}
	if (a.getRow(2) != b.getRow(2))
	{
		if (!fuzzyEqualSlow(a.getRow(2), b.getRow(2)))
		{
			return 1;
		}
	}
	return 0;
}

int Test_3x3mulM1M2(void)
{
	// Init an array flanked by guard pages
	btMatrix3x3 in1[ARRAY_SIZE];
	btMatrix3x3 in2[ARRAY_SIZE];
	btMatrix3x3 out[ARRAY_SIZE];
	btMatrix3x3 out2[ARRAY_SIZE];

	// Init the data
	size_t i, j;
	for (i = 0; i < ARRAY_SIZE; i++)
	{
		in1[i] = btMatrix3x3(rand_f4(), rand_f4(), rand_f4());
		in2[i] = btMatrix3x3(rand_f4(), rand_f4(), rand_f4());

		out[i] = M3x3mulM1M2_ref(in1[i], in2[i]);
		out2[i] = (in1[i] * in2[i]);

		if (out[i] != out2[i])
		{
			vlog("Error - M3x3mulM1M2 result error! ");
			vlog("failure @ %ld\n", i);
			btVector3 m0, m1, m2;
			m0 = out[i].getRow(0);
			m1 = out[i].getRow(1);
			m2 = out[i].getRow(2);

			vlog(
				"\ncorrect = (%10.4f, %10.4f, %10.4f, %10.4f) "
				"\n          (%10.4f, %10.4f, %10.4f, %10.4f) "
				"\n          (%10.4f, %10.4f, %10.4f, %10.4f) \n",
				m0.m_floats[0], m0.m_floats[1], m0.m_floats[2], m0.m_floats[3],
				m1.m_floats[0], m1.m_floats[1], m1.m_floats[2], m1.m_floats[3],
				m2.m_floats[0], m2.m_floats[1], m2.m_floats[2], m2.m_floats[3]);

			m0 = out2[i].getRow(0);
			m1 = out2[i].getRow(1);
			m2 = out2[i].getRow(2);

			vlog(
				"\ntested  = (%10.4f, %10.4f, %10.4f, %10.4f) "
				"\n          (%10.4f, %10.4f, %10.4f, %10.4f) "
				"\n          (%10.4f, %10.4f, %10.4f, %10.4f) \n",
				m0.m_floats[0], m0.m_floats[1], m0.m_floats[2], m0.m_floats[3],
				m1.m_floats[0], m1.m_floats[1], m1.m_floats[2], m1.m_floats[3],
				m2.m_floats[0], m2.m_floats[1], m2.m_floats[2], m2.m_floats[3]);

			return -1;
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
			out[i] = M3x3mulM1M2_ref(in1[i], in2[i]);
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
