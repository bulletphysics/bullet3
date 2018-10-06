//
//  Test_3x3getRot.cpp
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc.
//

#include "LinearMath/btScalar.h"
#if defined(BT_USE_SSE_IN_API) || defined(BT_USE_NEON)

#include "Test_3x3getRot.h"
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
	return btAssign128(RANDF_m1p1, RANDF_m1p1, RANDF_m1p1, BT_NAN);  // w channel NaN
}

static inline btSimdFloat4 qtNAN_f4(void)
{
	return btAssign128(BT_NAN, BT_NAN, BT_NAN, BT_NAN);
}

static void M3x3getRot_ref(const btMatrix3x3 &m, btQuaternion &q)
{
	btVector3 m_el[3] = {m[0], m[1], m[2]};

	btScalar trace = m_el[0].x() + m_el[1].y() + m_el[2].z();

	btScalar temp[4];

	if (trace > btScalar(0.0))
	{
		btScalar s = btSqrt(trace + btScalar(1.0));
		temp[3] = (s * btScalar(0.5));
		s = btScalar(0.5) / s;

		temp[0] = ((m_el[2].y() - m_el[1].z()) * s);
		temp[1] = ((m_el[0].z() - m_el[2].x()) * s);
		temp[2] = ((m_el[1].x() - m_el[0].y()) * s);
	}
	else
	{
		int i = m_el[0].x() < m_el[1].y() ? (m_el[1].y() < m_el[2].z() ? 2 : 1) : (m_el[0].x() < m_el[2].z() ? 2 : 0);
		int j = (i + 1) % 3;
		int k = (i + 2) % 3;

		btScalar s = btSqrt(m_el[i][i] - m_el[j][j] - m_el[k][k] + btScalar(1.0));
		temp[i] = s * btScalar(0.5);
		s = btScalar(0.5) / s;

		temp[3] = (m_el[k][j] - m_el[j][k]) * s;
		temp[j] = (m_el[j][i] + m_el[i][j]) * s;
		temp[k] = (m_el[k][i] + m_el[i][k]) * s;
	}
	q.setValue(temp[0], temp[1], temp[2], temp[3]);
}

static int operator!=(const btQuaternion &a, const btQuaternion &b)
{
	if (fabs(a.x() - b.x()) +
			fabs(a.y() - b.y()) +
			fabs(a.z() - b.z()) +
			fabs(a.w() - b.w()) >
		FLT_EPSILON * 4)
		return 1;

	return 0;
}

int Test_3x3getRot(void)
{
	// Init an array flanked by guard pages
	btMatrix3x3 in1[ARRAY_SIZE];
	btQuaternion out[ARRAY_SIZE];
	btQuaternion out2[ARRAY_SIZE];

	// Init the data
	size_t i, j;
	for (i = 0; i < ARRAY_SIZE; i++)
	{
		in1[i] = btMatrix3x3(rand_f4(), rand_f4(), rand_f4());
		out[i] = btQuaternion(qtNAN_f4());
		out2[i] = btQuaternion(qtNAN_f4());

		M3x3getRot_ref(in1[i], out[i]);
		in1[i].getRotation(out2[i]);

		if (out[i] != out2[i])
		{
			vlog("Error - M3x3getRot result error! ");
			vlog("failure @ %ld\n", i);
			vlog(
				"\ncorrect = (%10.7f, %10.7f, %10.7f, %10.7f) "
				"\ntested  = (%10.7f, %10.7f, %10.7f, %10.7f) \n",
				out[i].x(), out[i].y(), out[i].z(), out[i].w(),
				out2[i].x(), out2[i].y(), out2[i].z(), out2[i].w());

			return -1;
		}
	}

	uint64_t scalarTime, vectorTime;
	uint64_t startTime, bestTime, currentTime;
	bestTime = ~(bestTime & 0);  //-1ULL;
	scalarTime = 0;
	for (j = 0; j < LOOPCOUNT; j++)
	{
		startTime = ReadTicks();
		for (i = 0; i < ARRAY_SIZE; i++)
			M3x3getRot_ref(in1[i], out[i]);
		currentTime = ReadTicks() - startTime;
		scalarTime += currentTime;
		if (currentTime < bestTime)
			bestTime = currentTime;
	}
	if (0 == gReportAverageTimes)
		scalarTime = bestTime;
	else
		scalarTime /= LOOPCOUNT;

	bestTime = ~(bestTime & 0);  //-1ULL;
	vectorTime = 0;
	for (j = 0; j < LOOPCOUNT; j++)
	{
		startTime = ReadTicks();
		for (i = 0; i < ARRAY_SIZE; i++)
		{
			in1[i].getRotation(out2[i]);
		}
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
