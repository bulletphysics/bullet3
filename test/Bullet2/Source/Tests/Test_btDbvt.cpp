//
//  Test_btDbvt.cpp
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc., Inc.
//

#include "LinearMath/btScalar.h"
#if defined(BT_USE_SSE_IN_API) || defined(BT_USE_NEON)

#include "Test_btDbvt.h"
#include "vector.h"
#include "Utils.h"
#include "main.h"
#include <math.h>
#include <string.h>

#include <BulletCollision/BroadphaseCollision/btDbvt.h>

// reference code for testing purposes
SIMD_FORCE_INLINE bool Intersect_ref(btDbvtAabbMm& a, btDbvtAabbMm& b)
{
	return ((a.tMins().x() <= b.tMaxs().x()) &&
			(a.tMaxs().x() >= b.tMins().x()) &&
			(a.tMins().y() <= b.tMaxs().y()) &&
			(a.tMaxs().y() >= b.tMins().y()) &&
			(a.tMins().z() <= b.tMaxs().z()) &&
			(a.tMaxs().z() >= b.tMins().z()));
}

SIMD_FORCE_INLINE btScalar Proximity_ref(btDbvtAabbMm& a,
										 btDbvtAabbMm& b)
{
	const btVector3 d = (a.tMins() + a.tMaxs()) - (b.tMins() + b.tMaxs());
	return (btFabs(d.x()) + btFabs(d.y()) + btFabs(d.z()));
}

SIMD_FORCE_INLINE int Select_ref(btDbvtAabbMm& o,
								 btDbvtAabbMm& a,
								 btDbvtAabbMm& b)
{
	return (Proximity_ref(o, a) < Proximity_ref(o, b) ? 0 : 1);
}

SIMD_FORCE_INLINE void Merge_ref(btDbvtAabbMm& a,
								 btDbvtAabbMm& b,
								 btDbvtAabbMm& r)
{
	//
	//Changing '3' into '4' to compare with the vector code which changes all 4 floats.
	//Erwin: don't do this because the 4th component is ignore and not computed on non-vector code (there is no NEON version and scalar is just 3 components)
	//
	for (int i = 0; i < 3; ++i)
	{
		if (a.tMins().m_floats[i] < b.tMins().m_floats[i])
			r.tMins().m_floats[i] = a.tMins().m_floats[i];
		else
			r.tMins().m_floats[i] = b.tMins().m_floats[i];

		if (a.tMaxs().m_floats[i] > b.tMaxs().m_floats[i])
			r.tMaxs().m_floats[i] = a.tMaxs().m_floats[i];
		else
			r.tMaxs().m_floats[i] = b.tMaxs().m_floats[i];
	}
}
/*
[0]	float32_t	0.0318338
[1]	float32_t	0.0309355
[2]	float32_t	0.93264
[3]	float32_t	0.88788

[0]	float32_t	0.59133
[1]	float32_t	0.478779
[2]	float32_t	0.833354
[3]	float32_t	0.186335

[0]	float32_t	0.242578
[1]	float32_t	0.0134696
[2]	float32_t	0.383139
[3]	float32_t	0.414653

[0]	float32_t	0.067769
[1]	float32_t	0.993127
[2]	float32_t	0.484308
[3]	float32_t	0.765338
*/

#define LOOPCOUNT 1000
#define NUM_CYCLES 10000
#define DATA_SIZE 1024

int Test_btDbvt(void)
{
	btDbvtAabbMm a[DATA_SIZE], b[DATA_SIZE], c[DATA_SIZE];
	btDbvtAabbMm a_ref[DATA_SIZE], b_ref[DATA_SIZE], c_ref[DATA_SIZE];

	int i;

	bool Intersect_Test_Res[DATA_SIZE], Intersect_Ref_Res[DATA_SIZE];
	int Select_Test_Res[DATA_SIZE], Select_Ref_Res[DATA_SIZE];

	for (i = 0; i < DATA_SIZE; i++)
	{
		a[i].tMins().m_floats[0] = (float)rand() / (float)RAND_MAX;
		a[i].tMins().m_floats[1] = (float)rand() / (float)RAND_MAX;
		a[i].tMins().m_floats[2] = (float)rand() / (float)RAND_MAX;
		a[i].tMins().m_floats[3] = (float)rand() / (float)RAND_MAX;

		a[i].tMaxs().m_floats[0] = (float)rand() / (float)RAND_MAX;
		a[i].tMaxs().m_floats[1] = (float)rand() / (float)RAND_MAX;
		a[i].tMaxs().m_floats[2] = (float)rand() / (float)RAND_MAX;
		a[i].tMaxs().m_floats[3] = (float)rand() / (float)RAND_MAX;

		b[i].tMins().m_floats[0] = (float)rand() / (float)RAND_MAX;
		b[i].tMins().m_floats[1] = (float)rand() / (float)RAND_MAX;
		b[i].tMins().m_floats[2] = (float)rand() / (float)RAND_MAX;
		b[i].tMins().m_floats[3] = (float)rand() / (float)RAND_MAX;

		b[i].tMaxs().m_floats[0] = (float)rand() / (float)RAND_MAX;
		b[i].tMaxs().m_floats[1] = (float)rand() / (float)RAND_MAX;
		b[i].tMaxs().m_floats[2] = (float)rand() / (float)RAND_MAX;
		b[i].tMaxs().m_floats[3] = (float)rand() / (float)RAND_MAX;

		c[i].tMins().m_floats[0] = (float)rand() / (float)RAND_MAX;
		c[i].tMins().m_floats[1] = (float)rand() / (float)RAND_MAX;
		c[i].tMins().m_floats[2] = (float)rand() / (float)RAND_MAX;
		c[i].tMins().m_floats[3] = (float)rand() / (float)RAND_MAX;

		c[i].tMaxs().m_floats[0] = (float)rand() / (float)RAND_MAX;
		c[i].tMaxs().m_floats[1] = (float)rand() / (float)RAND_MAX;
		c[i].tMaxs().m_floats[2] = (float)rand() / (float)RAND_MAX;
		c[i].tMaxs().m_floats[3] = (float)rand() / (float)RAND_MAX;

		a_ref[i].tMins().m_floats[0] = a[i].tMins().m_floats[0];
		a_ref[i].tMins().m_floats[1] = a[i].tMins().m_floats[1];
		a_ref[i].tMins().m_floats[2] = a[i].tMins().m_floats[2];
		a_ref[i].tMins().m_floats[3] = a[i].tMins().m_floats[3];

		a_ref[i].tMaxs().m_floats[0] = a[i].tMaxs().m_floats[0];
		a_ref[i].tMaxs().m_floats[1] = a[i].tMaxs().m_floats[1];
		a_ref[i].tMaxs().m_floats[2] = a[i].tMaxs().m_floats[2];
		a_ref[i].tMaxs().m_floats[3] = a[i].tMaxs().m_floats[3];

		b_ref[i].tMins().m_floats[0] = b[i].tMins().m_floats[0];
		b_ref[i].tMins().m_floats[1] = b[i].tMins().m_floats[1];
		b_ref[i].tMins().m_floats[2] = b[i].tMins().m_floats[2];
		b_ref[i].tMins().m_floats[3] = b[i].tMins().m_floats[3];

		b_ref[i].tMaxs().m_floats[0] = b[i].tMaxs().m_floats[0];
		b_ref[i].tMaxs().m_floats[1] = b[i].tMaxs().m_floats[1];
		b_ref[i].tMaxs().m_floats[2] = b[i].tMaxs().m_floats[2];
		b_ref[i].tMaxs().m_floats[3] = b[i].tMaxs().m_floats[3];

		c_ref[i].tMins().m_floats[0] = c[i].tMins().m_floats[0];
		c_ref[i].tMins().m_floats[1] = c[i].tMins().m_floats[1];
		c_ref[i].tMins().m_floats[2] = c[i].tMins().m_floats[2];
		c_ref[i].tMins().m_floats[3] = c[i].tMins().m_floats[3];

		c_ref[i].tMaxs().m_floats[0] = c[i].tMaxs().m_floats[0];
		c_ref[i].tMaxs().m_floats[1] = c[i].tMaxs().m_floats[1];
		c_ref[i].tMaxs().m_floats[2] = c[i].tMaxs().m_floats[2];
		c_ref[i].tMaxs().m_floats[3] = c[i].tMaxs().m_floats[3];
	}

#if 1
	for (i = 0; i < DATA_SIZE; i++)
	{
		Intersect_Test_Res[i] = Intersect(a[i], b[i]);
		Intersect_Ref_Res[i] = Intersect_ref(a_ref[i], b_ref[i]);

		if (Intersect_Test_Res[i] != Intersect_Ref_Res[i])
		{
			printf("Diff on %d\n", i);

			printf("a_mx_f[0] = %.3f, a_mx_f[1] = %.3f, a_mx_f[2] = %.3f, a_mx_f[3] = %.3f\n", a[i].tMaxs().m_floats[0], a[i].tMaxs().m_floats[1], a[i].tMaxs().m_floats[2], a[i].tMaxs().m_floats[3]);
			printf("a_mi_f[0] = %.3f, a_mi_f[1] = %.3f, a_mi_f[2] = %.3f, a_mi_f[3] = %.3f\n", a[i].tMins().m_floats[0], a[i].tMins().m_floats[1], a[i].tMins().m_floats[2], a[i].tMins().m_floats[3]);
			printf("b_mx_f[0] = %.3f, b_mx_f[1] = %.3f, b_mx_f[2] = %.3f, b_mx_f[3] = %.3f\n", b[i].tMaxs().m_floats[0], b[i].tMaxs().m_floats[1], b[i].tMaxs().m_floats[2], b[i].tMaxs().m_floats[3]);
			printf("b_mi_f[0] = %.3f, b_mi_f[1] = %.3f, b_mi_f[2] = %.3f, b_mi_f[3] = %.3f\n", b[i].tMins().m_floats[0], b[i].tMins().m_floats[1], b[i].tMins().m_floats[2], b[i].tMins().m_floats[3]);

			printf("a_mx_f_ref[0] = %.3f, a_mx_f_ref[1] = %.3f, a_mx_f_ref[2] = %.3f, a_mx_f_ref[3] = %.3f\n", a_ref[i].tMaxs().m_floats[0], a_ref[i].tMaxs().m_floats[1], a_ref[i].tMaxs().m_floats[2], a_ref[i].tMaxs().m_floats[3]);
			printf("a_mi_f_ref[0] = %.3f, a_mi_f_ref[1] = %.3f, a_mi_f_ref[2] = %.3f, a_mi_f_ref[3] = %.3f\n", a_ref[i].tMins().m_floats[0], a_ref[i].tMins().m_floats[1], a_ref[i].tMins().m_floats[2], a_ref[i].tMins().m_floats[3]);
			printf("b_mx_f_ref[0] = %.3f, b_mx_f_ref[1] = %.3f, b_mx_f_ref[2] = %.3f, b_mx_f_ref[3] = %.3f\n", b_ref[i].tMaxs().m_floats[0], b_ref[i].tMaxs().m_floats[1], b_ref[i].tMaxs().m_floats[2], b_ref[i].tMaxs().m_floats[3]);
			printf("b_mi_f_ref[0] = %.3f, b_mi_f_ref[1] = %.3f, b_mi_f_ref[2] = %.3f, b_mi_f_ref[3] = %.3f\n", b_ref[i].tMins().m_floats[0], b_ref[i].tMins().m_floats[1], b_ref[i].tMins().m_floats[2], b_ref[i].tMins().m_floats[3]);
		}
	}
#endif

	uint64_t scalarTime;
	uint64_t vectorTime;
	size_t j;

	////////////////////////////////////
	//
	// Time and Test Intersect
	//
	////////////////////////////////////
	{
		uint64_t startTime, bestTime, currentTime;

		bestTime = -1LL;
		scalarTime = 0;
		for (j = 0; j < NUM_CYCLES; j++)
		{
			startTime = ReadTicks();

			for (i = 0; i < DATA_SIZE; i++)
			{
				Intersect_Ref_Res[i] = Intersect_ref(a_ref[i], b_ref[i]);
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

			for (i = 0; i < DATA_SIZE; i++)
			{
				Intersect_Test_Res[i] = Intersect(a[i], b[i]);
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

	vlog("Intersect Timing:\n");
	vlog("     \t    scalar\t    vector\n");
	vlog("    \t%10.4f\t%10.4f\n", TicksToCycles(scalarTime) / LOOPCOUNT, TicksToCycles(vectorTime) / LOOPCOUNT);

	//printf("scalar = %llu, vector = %llu\n", scalarTime, vectorTime);

	for (i = 0; i < DATA_SIZE; i++)
	{
		if (Intersect_Test_Res[i] != Intersect_Ref_Res[i])
		{
			printf("Intersect fail at %d\n", i);
			return 1;
		}
	}

	////////////////////////////////////
	//
	// Time and Test Merge
	//
	////////////////////////////////////
	{
		uint64_t startTime, bestTime, currentTime;

		bestTime = -1LL;
		scalarTime = 0;
		for (j = 0; j < NUM_CYCLES; j++)
		{
			startTime = ReadTicks();

			for (i = 0; i < DATA_SIZE; i++)
			{
				Merge_ref(a_ref[i], b_ref[i], c_ref[i]);
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

			for (i = 0; i < DATA_SIZE; i++)
			{
				Merge(a[i], b[i], c[i]);
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

	vlog("Merge Timing:\n");
	vlog("     \t    scalar\t    vector\n");
	vlog("    \t%10.4f\t%10.4f\n", TicksToCycles(scalarTime) / LOOPCOUNT, TicksToCycles(vectorTime) / LOOPCOUNT);

	//printf("scalar = %llu, vector = %llu\n", scalarTime, vectorTime);
	/*
 c  [0]	float32_t	0.00455523
    [1]	float32_t	0.559712
    [2]	float32_t	0.0795838
    [3]	float32_t	0.10182
    
c_ref
    [0]	float32_t	0.00455523
    [1]	float32_t	0.559712
    [2]	float32_t	0.0795838
    [3]	float32_t	0.552081
    
    
c   [0]	float32_t	0.829904
    [1]	float32_t	0.692891
    [2]	float32_t	0.961654
    [3]	float32_t	0.666956
    
 c_ref
    [0]	float32_t	0.829904
    [1]	float32_t	0.692891
    [2]	float32_t	0.961654
    [3]	float32_t	0.522878
    */
	for (i = 0; i < DATA_SIZE; i++)
	{
		//ignore 4th component because it is not computed in all code-paths
		if ((fabs(c[i].tMaxs().m_floats[0] - c_ref[i].tMaxs().m_floats[0]) > 0.001) ||
			(fabs(c[i].tMaxs().m_floats[1] - c_ref[i].tMaxs().m_floats[1]) > 0.001) ||
			(fabs(c[i].tMaxs().m_floats[2] - c_ref[i].tMaxs().m_floats[2]) > 0.001) ||
			// (fabs(c[i].tMaxs().m_floats[3] - c_ref[i].tMaxs().m_floats[3]) > 0.001) ||
			(fabs(c[i].tMins().m_floats[0] - c_ref[i].tMins().m_floats[0]) > 0.001) ||
			(fabs(c[i].tMins().m_floats[1] - c_ref[i].tMins().m_floats[1]) > 0.001) ||
			(fabs(c[i].tMins().m_floats[2] - c_ref[i].tMins().m_floats[2]) > 0.001)
			//|| (fabs(c[i].tMins().m_floats[3] - c_ref[i].tMins().m_floats[3]) > 0.001)
		)

		//if((c[i].tMaxs().m_floats[0] != c_ref[i].tMaxs().m_floats[0]) || (c[i].tMaxs().m_floats[1] != c_ref[i].tMaxs().m_floats[1]) || (c[i].tMaxs().m_floats[2] != c_ref[i].tMaxs().m_floats[2]) || (c[i].tMaxs().m_floats[3] != c_ref[i].tMaxs().m_floats[3]) || (c[i].tMins().m_floats[0] != c_ref[i].tMins().m_floats[0]) || (c[i].tMins().m_floats[1] != c_ref[i].tMins().m_floats[1]) || (c[i].tMins().m_floats[2] != c_ref[i].tMins().m_floats[2]) || (c[i].tMins().m_floats[3] != c_ref[i].tMins().m_floats[3]))
		{
			printf("Merge fail at %d with test = %d, ref = %d\n", i, Select_Test_Res[i], Select_Ref_Res[i]);

			printf("a_mx_f[0] = %.3f, a_mx_f[1] = %.3f, a_mx_f[2] = %.3f, a_mx_f[3] = %.3f\n", a[i].tMaxs().m_floats[0], a[i].tMaxs().m_floats[1], a[i].tMaxs().m_floats[2], a[i].tMaxs().m_floats[3]);
			printf("a_mi_f[0] = %.3f, a_mi_f[1] = %.3f, a_mi_f[2] = %.3f, a_mi_f[3] = %.3f\n", a[i].tMins().m_floats[0], a[i].tMins().m_floats[1], a[i].tMins().m_floats[2], a[i].tMins().m_floats[3]);
			printf("b_mx_f[0] = %.3f, b_mx_f[1] = %.3f, b_mx_f[2] = %.3f, b_mx_f[3] = %.3f\n", b[i].tMaxs().m_floats[0], b[i].tMaxs().m_floats[1], b[i].tMaxs().m_floats[2], b[i].tMaxs().m_floats[3]);
			printf("b_mi_f[0] = %.3f, b_mi_f[1] = %.3f, b_mi_f[2] = %.3f, b_mi_f[3] = %.3f\n", b[i].tMins().m_floats[0], b[i].tMins().m_floats[1], b[i].tMins().m_floats[2], b[i].tMins().m_floats[3]);
			printf("c_mx_f[0] = %.3f, c_mx_f[1] = %.3f, c_mx_f[2] = %.3f, c_mx_f[3] = %.3f\n", c[i].tMaxs().m_floats[0], c[i].tMaxs().m_floats[1], c[i].tMaxs().m_floats[2], c[i].tMaxs().m_floats[3]);
			printf("c_mi_f[0] = %.3f, c_mi_f[1] = %.3f, c_mi_f[2] = %.3f, c_mi_f[3] = %.3f\n", c[i].tMins().m_floats[0], c[i].tMins().m_floats[1], c[i].tMins().m_floats[2], c[i].tMins().m_floats[3]);

			printf("a_mx_f_ref[0] = %.3f, a_mx_f_ref[1] = %.3f, a_mx_f_ref[2] = %.3f, a_mx_f_ref[3] = %.3f\n", a_ref[i].tMaxs().m_floats[0], a_ref[i].tMaxs().m_floats[1], a_ref[i].tMaxs().m_floats[2], a_ref[i].tMaxs().m_floats[3]);
			printf("a_mi_f_ref[0] = %.3f, a_mi_f_ref[1] = %.3f, a_mi_f_ref[2] = %.3f, a_mi_f_ref[3] = %.3f\n", a_ref[i].tMins().m_floats[0], a_ref[i].tMins().m_floats[1], a_ref[i].tMins().m_floats[2], a_ref[i].tMins().m_floats[3]);
			printf("b_mx_f_ref[0] = %.3f, b_mx_f_ref[1] = %.3f, b_mx_f_ref[2] = %.3f, b_mx_f_ref[3] = %.3f\n", b_ref[i].tMaxs().m_floats[0], b_ref[i].tMaxs().m_floats[1], b_ref[i].tMaxs().m_floats[2], b_ref[i].tMaxs().m_floats[3]);
			printf("b_mi_f_ref[0] = %.3f, b_mi_f_ref[1] = %.3f, b_mi_f_ref[2] = %.3f, b_mi_f_ref[3] = %.3f\n", b_ref[i].tMins().m_floats[0], b_ref[i].tMins().m_floats[1], b_ref[i].tMins().m_floats[2], b_ref[i].tMins().m_floats[3]);
			printf("c_mx_f_ref[0] = %.3f, c_mx_f_ref[1] = %.3f, c_mx_f_ref[2] = %.3f, c_mx_f_ref[3] = %.3f\n", c_ref[i].tMaxs().m_floats[0], c_ref[i].tMaxs().m_floats[1], c_ref[i].tMaxs().m_floats[2], c_ref[i].tMaxs().m_floats[3]);
			printf("c_mi_f_ref[0] = %.3f, c_mi_f_ref[1] = %.3f, c_mi_f_ref[2] = %.3f, c_mi_f_ref[3] = %.3f\n", c_ref[i].tMins().m_floats[0], c_ref[i].tMins().m_floats[1], c_ref[i].tMins().m_floats[2], c_ref[i].tMins().m_floats[3]);
			return 1;
		}
	}

	////////////////////////////////////
	//
	// Time and Test Select
	//
	////////////////////////////////////
	{
		uint64_t startTime, bestTime, currentTime;

		bestTime = -1LL;
		scalarTime = 0;
		for (j = 0; j < NUM_CYCLES; j++)
		{
			startTime = ReadTicks();

			for (i = 0; i < DATA_SIZE; i++)
			{
				Select_Ref_Res[i] = Select_ref(a_ref[i], b_ref[i], c_ref[i]);
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

			for (i = 0; i < DATA_SIZE; i++)
			{
				Select_Test_Res[i] = Select(a[i], b[i], c[i]);
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

	vlog("Select Timing:\n");
	vlog("     \t    scalar\t    vector\n");
	vlog("    \t%10.4f\t%10.4f\n", TicksToCycles(scalarTime) / LOOPCOUNT, TicksToCycles(vectorTime) / LOOPCOUNT);

	//printf("scalar = %llu, vector = %llu\n", scalarTime, vectorTime);

	for (i = 0; i < DATA_SIZE; i++)
	{
		Select_Ref_Res[i] = Select_ref(a_ref[i], b_ref[i], c_ref[i]);
		Select_Test_Res[i] = Select(a[i], b[i], c[i]);

		if (Select_Test_Res[i] != Select_Ref_Res[i])
		{
			printf("Select fail at %d with test = %d, ref = %d\n", i, Select_Test_Res[i], Select_Ref_Res[i]);

			printf("a_mx_f[0] = %.3f, a_mx_f[1] = %.3f, a_mx_f[2] = %.3f, a_mx_f[3] = %.3f\n", a[i].tMaxs().m_floats[0], a[i].tMaxs().m_floats[1], a[i].tMaxs().m_floats[2], a[i].tMaxs().m_floats[3]);
			printf("a_mi_f[0] = %.3f, a_mi_f[1] = %.3f, a_mi_f[2] = %.3f, a_mi_f[3] = %.3f\n", a[i].tMins().m_floats[0], a[i].tMins().m_floats[1], a[i].tMins().m_floats[2], a[i].tMins().m_floats[3]);
			printf("b_mx_f[0] = %.3f, b_mx_f[1] = %.3f, b_mx_f[2] = %.3f, b_mx_f[3] = %.3f\n", b[i].tMaxs().m_floats[0], b[i].tMaxs().m_floats[1], b[i].tMaxs().m_floats[2], b[i].tMaxs().m_floats[3]);
			printf("b_mi_f[0] = %.3f, b_mi_f[1] = %.3f, b_mi_f[2] = %.3f, b_mi_f[3] = %.3f\n", b[i].tMins().m_floats[0], b[i].tMins().m_floats[1], b[i].tMins().m_floats[2], b[i].tMins().m_floats[3]);
			printf("c_mx_f[0] = %.3f, c_mx_f[1] = %.3f, c_mx_f[2] = %.3f, c_mx_f[3] = %.3f\n", c[i].tMaxs().m_floats[0], c[i].tMaxs().m_floats[1], c[i].tMaxs().m_floats[2], c[i].tMaxs().m_floats[3]);
			printf("c_mi_f[0] = %.3f, c_mi_f[1] = %.3f, c_mi_f[2] = %.3f, c_mi_f[3] = %.3f\n", c[i].tMins().m_floats[0], c[i].tMins().m_floats[1], c[i].tMins().m_floats[2], c[i].tMins().m_floats[3]);

			printf("a_mx_f_ref[0] = %.3f, a_mx_f_ref[1] = %.3f, a_mx_f_ref[2] = %.3f, a_mx_f_ref[3] = %.3f\n", a_ref[i].tMaxs().m_floats[0], a_ref[i].tMaxs().m_floats[1], a_ref[i].tMaxs().m_floats[2], a_ref[i].tMaxs().m_floats[3]);
			printf("a_mi_f_ref[0] = %.3f, a_mi_f_ref[1] = %.3f, a_mi_f_ref[2] = %.3f, a_mi_f_ref[3] = %.3f\n", a_ref[i].tMins().m_floats[0], a_ref[i].tMins().m_floats[1], a_ref[i].tMins().m_floats[2], a_ref[i].tMins().m_floats[3]);
			printf("b_mx_f_ref[0] = %.3f, b_mx_f_ref[1] = %.3f, b_mx_f_ref[2] = %.3f, b_mx_f_ref[3] = %.3f\n", b_ref[i].tMaxs().m_floats[0], b_ref[i].tMaxs().m_floats[1], b_ref[i].tMaxs().m_floats[2], b_ref[i].tMaxs().m_floats[3]);
			printf("b_mi_f_ref[0] = %.3f, b_mi_f_ref[1] = %.3f, b_mi_f_ref[2] = %.3f, b_mi_f_ref[3] = %.3f\n", b_ref[i].tMins().m_floats[0], b_ref[i].tMins().m_floats[1], b_ref[i].tMins().m_floats[2], b_ref[i].tMins().m_floats[3]);
			printf("c_mx_f_ref[0] = %.3f, c_mx_f_ref[1] = %.3f, c_mx_f_ref[2] = %.3f, c_mx_f_ref[3] = %.3f\n", c_ref[i].tMaxs().m_floats[0], c_ref[i].tMaxs().m_floats[1], c_ref[i].tMaxs().m_floats[2], c_ref[i].tMaxs().m_floats[3]);
			printf("c_mi_f_ref[0] = %.3f, c_mi_f_ref[1] = %.3f, c_mi_f_ref[2] = %.3f, c_mi_f_ref[3] = %.3f\n", c_ref[i].tMins().m_floats[0], c_ref[i].tMins().m_floats[1], c_ref[i].tMins().m_floats[2], c_ref[i].tMins().m_floats[3]);
			return 1;
		}
	}

	return 0;
}
#endif
