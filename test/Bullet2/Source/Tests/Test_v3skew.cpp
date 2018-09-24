//
//  Test_v3skew.cpp
//  BulletTest
//
//  Copyright (c) 2011 Apple Inc.
//

#include "LinearMath/btScalar.h"
#if defined(BT_USE_SSE_IN_API) || defined(BT_USE_NEON)

#include "Test_v3skew.h"
#include "vector.h"
#include "Utils.h"
#include "main.h"
#include <math.h>
#include <string.h>

#include <LinearMath/btVector3.h>

// reference code for testing purposes
static void
v3skew_ref(
	const btVector3* v,
	btVector3* v1,
	btVector3* v2,
	btVector3* v3);

#define LOOPCOUNT 2048
#define NUM_CYCLES 10000

int Test_v3skew(void)
{
	btVector3 v, v1, v2, v3, vt1, vt2, vt3;

	float x, y, z, w;

	// Init the data
	x = RANDF_01;
	y = RANDF_01;
	z = RANDF_01;
	w = BT_NAN;  // w channel NaN
	v.setValue(x, y, z);
	v.setW(w);

	v1.setValue(w, w, w);
	v1.setW(w);

	vt3 = vt2 = vt1 = v3 = v2 = v1;

	{
		v3skew_ref(&v, &v1, &v2, &v3);
		v.getSkewSymmetricMatrix(&vt1, &vt2, &vt3);
		/*
		if( v1.m_floats[0] != vt1.m_floats[0] || 
			v1.m_floats[1] != vt1.m_floats[1] ||
			v1.m_floats[2] != vt1.m_floats[2] )
		*/
		if (!(v1 == vt1))
		{
			vlog(
				"Error - v3skew result error! "
				"\ncorrect v1 = (%10.4f, %10.4f, %10.4f) "
				"\ntested  v1 = (%10.4f, %10.4f, %10.4f) \n",
				v1.m_floats[0], v1.m_floats[1], v1.m_floats[2],
				vt1.m_floats[0], vt1.m_floats[1], vt1.m_floats[2]);

			return 1;
		}

		/*
        if( v2.m_floats[0] != vt2.m_floats[0] || 
			v2.m_floats[1] != vt2.m_floats[1] ||
			v2.m_floats[2] != vt2.m_floats[2] )
		*/
		if (!(v2 == vt2))
		{
			vlog(
				"Error - v3skew result error! "
				"\ncorrect v2 = (%10.4f, %10.4f, %10.4f) "
				"\ntested  v2 = (%10.4f, %10.4f, %10.4f) \n",
				v2.m_floats[0], v2.m_floats[1], v2.m_floats[2],
				vt2.m_floats[0], vt2.m_floats[1], vt2.m_floats[2]);

			return 1;
		}

		/*
        if( v3.m_floats[0] != vt3.m_floats[0] || 
			v3.m_floats[1] != vt3.m_floats[1] ||
			v3.m_floats[2] != vt3.m_floats[2] )
		*/
		if (!(v3 == vt3))
		{
			vlog(
				"Error - v3skew result error! "
				"\ncorrect v3 = (%10.4f, %10.4f, %10.4f) "
				"\ntested  v3 = (%10.4f, %10.4f, %10.4f) \n",
				v3.m_floats[0], v3.m_floats[1], v3.m_floats[2],
				vt3.m_floats[0], vt3.m_floats[1], vt3.m_floats[2]);

			return 1;
		}
	}

#define DATA_SIZE 256

	btVector3 v3_arr0[DATA_SIZE];
	btVector3 v3_arr1[DATA_SIZE];
	btVector3 v3_arr2[DATA_SIZE];
	btVector3 v3_arr3[DATA_SIZE];

	uint64_t scalarTime;
	uint64_t vectorTime;
	size_t j, k;

	for (k = 0; k < DATA_SIZE; k++)
	{
		x = RANDF_01;
		y = RANDF_01;
		z = RANDF_01;
		v3_arr0[k].setValue(x, y, z);
		v3_arr0[k].setW(w);

		v3_arr1[k].setValue(w, w, w);
		v3_arr1[k].setW(w);

		v3_arr3[k] = v3_arr2[k] = v3_arr1[k];
	}

	{
		uint64_t startTime, bestTime, currentTime;

		bestTime = -1LL;
		scalarTime = 0;
		for (j = 0; j < NUM_CYCLES; j++)
		{
			startTime = ReadTicks();
			for (k = 0; k < LOOPCOUNT; k++)
			{
				size_t k32 = (k & (DATA_SIZE - 1));
				v3skew_ref(&v3_arr0[k32], &v3_arr1[k32], &v3_arr2[k32], &v3_arr3[k32]);
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
			for (k = 0; k < LOOPCOUNT; k++)
			{
				size_t k32 = (k & (DATA_SIZE - 1));
				v3_arr0[k32].getSkewSymmetricMatrix(&v3_arr1[k32], &v3_arr2[k32], &v3_arr3[k32]);
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
	vlog("    \t    scalar\t    vector\n");
	vlog("    \t%10.4f\t%10.4f\n", TicksToCycles(scalarTime) / LOOPCOUNT, TicksToCycles(vectorTime) / LOOPCOUNT);

	return 0;
}

static void
v3skew_ref(
	const btVector3* v,
	btVector3* v1,
	btVector3* v2,
	btVector3* v3)
{
	v1->setValue(0., -v->z(), v->y());
	v2->setValue(v->z(), 0., -v->x());
	v3->setValue(-v->y(), v->x(), 0.);
}

#endif  //BT_USE_SSE
