//
//  Test_3x3transposeTimes.cpp
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc.
//

#include "LinearMath/btScalar.h"
#if defined(BT_USE_SSE_IN_API) || defined(BT_USE_NEON)

#include "Test_3x3transposeTimes.h"
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

static btMatrix3x3 TransposeTimesReference(const btMatrix3x3 &in, const btMatrix3x3 &m)
{
	btVector3 m_el[3] = {in[0], in[1], in[2]};
	btSimdFloat4 r0 = btAssign128(m_el[0].x() * m[0].x() + m_el[1].x() * m[1].x() + m_el[2].x() * m[2].x(),
								  m_el[0].x() * m[0].y() + m_el[1].x() * m[1].y() + m_el[2].x() * m[2].y(),
								  m_el[0].x() * m[0].z() + m_el[1].x() * m[1].z() + m_el[2].x() * m[2].z(),
								  0.0f);
	btSimdFloat4 r1 = btAssign128(m_el[0].y() * m[0].x() + m_el[1].y() * m[1].x() + m_el[2].y() * m[2].x(),
								  m_el[0].y() * m[0].y() + m_el[1].y() * m[1].y() + m_el[2].y() * m[2].y(),
								  m_el[0].y() * m[0].z() + m_el[1].y() * m[1].z() + m_el[2].y() * m[2].z(),
								  0.0f);
	btSimdFloat4 r2 = btAssign128(m_el[0].z() * m[0].x() + m_el[1].z() * m[1].x() + m_el[2].z() * m[2].x(),
								  m_el[0].z() * m[0].y() + m_el[1].z() * m[1].y() + m_el[2].z() * m[2].y(),
								  m_el[0].z() * m[0].z() + m_el[1].z() * m[1].z() + m_el[2].z() * m[2].z(),
								  0.0f);
	return btMatrix3x3(r0, r1, r2);
}

static int operator!=(const btMatrix3x3 &a, const btMatrix3x3 &b)
{
	if (a.getRow(0) != b.getRow(0))
		return 1;
	if (a.getRow(1) != b.getRow(1))
		return 1;
	if (a.getRow(2) != b.getRow(2))
		return 1;
	return 0;
}

int Test_3x3transposeTimes(void)
{
	// Init an array flanked by guard pages
	btMatrix3x3 in1[ARRAY_SIZE];
	btMatrix3x3 in2[ARRAY_SIZE];
	btMatrix3x3 out[ARRAY_SIZE];
	btMatrix3x3 out2[ARRAY_SIZE];

	float maxRelativeError = 0.f;
	// Init the data
	size_t i, j;
	for (i = 0; i < ARRAY_SIZE; i++)
	{
		in1[i] = btMatrix3x3(rand_f4(), rand_f4(), rand_f4());
		in2[i] = btMatrix3x3(rand_f4(), rand_f4(), rand_f4());

		out[i] = TransposeTimesReference(in1[i], in2[i]);
		out2[i] = in1[i].transposeTimes(in2[i]);

		if (out[i] != out2[i])
		{
			float relativeError = 0.f;

			for (int column = 0; column < 3; column++)
				for (int row = 0; row < 3; row++)
					relativeError = btMax(relativeError, btFabs(out2[i][row][column] - out[i][row][column]) / out[i][row][column]);

			if (relativeError > 1e-6)
			{
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
			else
			{
				if (relativeError > maxRelativeError)
					maxRelativeError = relativeError;
			}
		}
	}

	if (maxRelativeError)
	{
		printf("Warning: maxRelativeError = %e\n", maxRelativeError);
	}
	uint64_t scalarTime, vectorTime;
	uint64_t startTime, bestTime, currentTime;
	bestTime = -1LL;
	scalarTime = 0;
	for (j = 0; j < LOOPCOUNT; j++)
	{
		startTime = ReadTicks();
		for (i = 0; i < ARRAY_SIZE; i++)
			out[i] = TransposeTimesReference(in1[i], in2[i]);
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
			out[i] = in1[i].transposeTimes(in2[i]);
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
