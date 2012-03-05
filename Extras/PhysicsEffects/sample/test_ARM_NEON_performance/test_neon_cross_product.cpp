/*
 Applied Research Associates Inc. (c)2011

 Redistribution and use in source and binary forms,
   with or without modification, are permitted provided that the
   following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Applied Research Associates Inc nor the names
      of its contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
*/

#include "test_neon.h"
#define SCE_PFX_USE_PERFCOUNTER
#include "physics_effects.h"
#include <arm_neon.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <android/log.h>

// This works with gcc
#define SET_ALIGNMENT(alignment)   __attribute__((__aligned__((alignment))))

//#define PRINT_COMPUTED_VECTOR_RESULTS

// assembly implementations
extern "C"
{
	void CrossProductNeonResultInMemoryAssembly(float *a, float *b, float *pfResult);
	void CrossProductNeonResultInMemoryAssembly2(float *a, float *b, float *pfResult);
	void CrossProductNeonResultInMemoryAssembly3(float *a, float *b, float *pfResult);
}

//----------------------------------------------------------------------------
//  CrossProductNeonResultInMemory
//
/// Performs a Vector3 style cross product using NEON intrinsics, storing the
/// result directly into system memory.
///
/// @param  a         Input vector 1. Must point to 4 float values
/// @param  b         Input vector 2. Must point to 4 float values
/// @param  pfResult  [in] must point to an array of at least *4*
///                   float values. [out] The result of the cross
///                   product is contained in the first 3 lanes.
///
/// NOTE: The parameter types here are floats, not float32_t's. gcc
/// sometimes doesn't interpret float32_t's correctly. In particular,
/// if the type of pfResult is set to float32_t*, gcc will throw an
/// internal compiler error (ICE) for this code. In memory, float32_t
/// and float are equivalent, so can cast between them explicitly.
//----------------------------------------------------------------------------
void CrossProductNeonResultInMemory(float *a, float *b, float *pfResult)
{
	float32x4_t v1 = {a[1],a[2],a[0], 0.0f};
	float32x4_t v2 = {b[2],b[0],b[1], 0.0f};
	float32x4_t v3 = {a[2],a[0],a[1], 0.0f};
	float32x4_t v4 = {b[1],b[2],b[0], 0.0f};
	v1 = vmulq_f32(v1, v2);
	v1 = vmlsq_f32(v1, v3, v4);
	vst1q_f32((float32_t*)pfResult, v1);
}

void CrossProductNeonResultInMemoryCPPAssembly(float *a, float *b, float *result) {
    asm volatile(
		"vld1.32 {d18[1]}, [r1]!	 \n\t"
		"vld1.32 {d19[0]}, [r1]!	 \n\t"
		"vld1.32 {d18[0]}, [r1]!	 \n\t"
		"vld1.32 {d19[1]}, [r1]		 \n\t"
		"vld1.32 {d17[0]}, [r0]!	 \n\t"
		"vld1.32 {d16}, [r0]!	 	 \n\t"
		"vld1.32 {d17[1]}, [r0]		 \n\t"
		"vmul.f32 q10, q8, q9		 \n\t"
		"vtrn.32 d18,d19			 \n\t"
		"vrev64.32 d16,d16			 \n\t"
		"vrev64.32 d18,d18			 \n\t"
		"vtrn.32 d16,d17			 \n\t"
		"vmls.f32 q10, q8, q9		 \n\t"
		"vst1.32	{q10}, [r2]	 	 \n\t"
    );
}

//----------------------------------------------------------------------------
//  CrossProductScalarResultInMemory
//
/// Performs a Vector3 style cross product using scalar math, storing the
/// result directly into system memory.
///
/// @param  a         Input vector 1. Must point to 4 float values
/// @param  b         Input vector 2. Must point to 4 float values
/// @param  pfResult  [in] pointer to a float. [out] Contains the
///                   result, dotproduct(a,b)
//----------------------------------------------------------------------------
void CrossProductScalarResultInMemory(float *a, float *b, float *pfResult)
{
	pfResult[0] = a[1]*b[2] - a[2]*b[1];
	pfResult[1] = a[2]*b[0] - a[0]*b[2];
	pfResult[2] = a[0]*b[1] - a[1]*b[0];
}

//----------------------------------------------------------------------------
//  TestFastNeonCrossProduct
//
/// Run timing study of the cross product functions above, writing the
/// results to the Android verbose log.
//----------------------------------------------------------------------------
void TestNeonCrossProduct()
{
	float SET_ALIGNMENT(64) data[] = {float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),0.0f,
										float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),0.0f};

	float *a = &data[0];
	float *b = &data[4];

	char szMsg[256];

	sprintf(szMsg, "");
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg,"---------------------------------------");
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg, "Cross product test inputs A=<%f,%f,%f>, B=<%f,%f,%f>",
					a[0], a[1], a[2], b[0], b[1], b[2]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);

	float SET_ALIGNMENT(64) fResult[4];
    
	double dTimeSpan, dRefTimeSpan;; 
	unsigned int uiNumTries = 10000000;
	unsigned int i;

	sce::PhysicsEffects::PfxPerfCounter pc;

// profile scalar cross product with direct memory return
	fResult[0] = 0.0f;
	pc.countBegin("");
	for(i = 0; i < uiNumTries; i++)
	{
		CrossProductScalarResultInMemory(a, b, fResult);
	}
	pc.countEnd();
	dTimeSpan = pc.getCountTime(0);
	pc.resetCount();
	dRefTimeSpan = dTimeSpan;
	sprintf(szMsg, "Time to do %i calls for CrossProductScalarResultInMemory: %f secs, speedup: %5.2f, result value=<%f,%f,%f>",
					uiNumTries, dTimeSpan, dRefTimeSpan/dTimeSpan, fResult[0], fResult[1], fResult[2]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);



// profile NEON assembly volatile cross product with direct memory return
	fResult[0] = 0.0f;
	pc.countBegin("");
	for(i = 0; i < uiNumTries; i++)
	{
		CrossProductNeonResultInMemoryCPPAssembly(a, b, fResult);
	}
	pc.countEnd();
	dTimeSpan = pc.getCountTime(0);
	pc.resetCount();
	sprintf(szMsg, "Time to do %i calls for CrossProductNeonResultInMemoryFast: %f secs, speedup: %5.2f, result value=<%f,%f,%f>",
					uiNumTries, dTimeSpan, dRefTimeSpan/dTimeSpan, fResult[0], fResult[1], fResult[2]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);



// profile NEON cross product with direct memory return, assembly version
	fResult[0] = 0.0f;
	pc.countBegin("");
	for(i = 0; i < uiNumTries; i++)
	{
		CrossProductNeonResultInMemoryAssembly(a, b, fResult);
	}
	pc.countEnd();
	dTimeSpan = pc.getCountTime(0);
	pc.resetCount();
	sprintf(szMsg, "Time to do %i calls for CrossProductNeonResultInMemoryAssembly: %f secs, speedup: %5.2f, result value=<%f,%f,%f>",
					uiNumTries, dTimeSpan, dRefTimeSpan/dTimeSpan, fResult[0], fResult[1], fResult[2]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);



// profile NEON cross product with direct memory return, assembly version 2
	fResult[0] = 0.0f;
	pc.countBegin("");
	for(i = 0; i < uiNumTries; i++)
	{
		CrossProductNeonResultInMemoryAssembly2(a, b, fResult);
	}
	pc.countEnd();
	dTimeSpan = pc.getCountTime(0);
	pc.resetCount();

	sprintf(szMsg, "Time to do %i calls for CrossProductNeonResultInMemoryAssembly2: %f secs, speedup: %5.2f, result value=<%f,%f,%f>",
					uiNumTries, dTimeSpan, dRefTimeSpan/dTimeSpan, fResult[0], fResult[1], fResult[2]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);



// profile NEON cross product with direct memory return, assembly version 3
	fResult[0] = 0.0f;
	pc.countBegin("");
	for(i = 0; i < uiNumTries; i++)
	{
		CrossProductNeonResultInMemoryAssembly3(a, b, fResult);
	}
	pc.countEnd();
	dTimeSpan = pc.getCountTime(0);
	pc.resetCount();

	sprintf(szMsg, "Time to do %i calls for CrossProductNeonResultInMemoryAssembly3: %f secs, speedup: %5.2f, result value=<%f,%f,%f>",
					uiNumTries, dTimeSpan, dRefTimeSpan/dTimeSpan, fResult[0], fResult[1], fResult[2]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
}
