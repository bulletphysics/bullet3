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
	float32_t DotProductNeonResultInRegisterAssembly(float *a, float *b);
	void DotProductNeonResultInMemoryAssembly(float *a, float *b, float *pfResult);
	void DotProductNeonResultInMemoryAssembly2(float *data, float *pfResult);
	void DotProductNeonResultInMemoryAssembly2b(float *data, float *pfResult);
}

//----------------------------------------------------------------------------
//  DotProductNeonResultInRegister
//
/// Performs a Vector4 style dot product using NEON intrinsics, returning
/// the result in an ARM register.
///
/// @param  a     Input vector 1. Must point to 4 float values
/// @param  b     Input vector 2. Must point to 4 float values
///
/// @return dotproduct(a,b)
//----------------------------------------------------------------------------
float32_t DotProductNeonResultInRegister(float *a, float *b)
{
	float32_t result[2];
	float32x2_t v1a = vld1_f32((float32_t*)a);
	float32x2_t v1b = vld1_f32((float32_t*)(a + 2));
	float32x2_t v2a = vld1_f32((float32_t*)b);
	float32x2_t v2b = vld1_f32((float32_t*)(b + 2));
	v1a = vmul_f32(v1a, v2a);
	v1a = vmla_f32(v1a, v1b, v2b);
	v1a = vpadd_f32(v1a, v1a);
	vst1_f32(result, v1a);
	return(result[0]);
}

// pfResult must point to memory large enough to store *two*
// float32_t values
//----------------------------------------------------------------------------
//  DotProductNeonResultInMemory
//
/// Performs a Vector4 style dot product using NEON intrinsics, storing the
/// result directly into system memory.
///
/// @param  a         Input vector 1. Must point to 4 float values
/// @param  b         Input vector 2. Must point to 4 float values
/// @param  pfResult  [in] must point to an array of at least *2*
///                   float values. [out] The result of the dot
///                   product is contained in both slots of the array.
///                   Recommended to use the first one though they
///                   have the same value
///
/// NOTE: The parameter types here are floats, not float32_t's. gcc
/// sometimes doesn't interpret float32_t's correctly. In particular,
/// if the type of pfResult is set to float32_t*, gcc will throw an
/// internal compiler error (ICE) for this code. In memory, float32_t
/// and float are equivalent, so can cast between them explicitly.
//----------------------------------------------------------------------------
void DotProductNeonResultInMemory(float *a, float *b, float *pfResult)
{
	float32x2_t v1a = vld1_f32((float32_t*)a);
	float32x2_t v1b = vld1_f32((float32_t*)(a + 2));
	float32x2_t v2a = vld1_f32((float32_t*)b);
	float32x2_t v2b = vld1_f32((float32_t*)(b + 2));
	v1a = vmul_f32(v1a, v2a);
	v1a = vmla_f32(v1a, v1b, v2b);
	v1a = vpadd_f32(v1a, v1a);
	vst1_f32((float32_t*)pfResult, v1a);
}

//----------------------------------------------------------------------------
//  DotProductScalarResultInRegister
//
/// Performs a Vector4 style dot product using scalar math, and returning
/// the result in an ARM register.
///
/// @param  a     Input vector 1. Must point to 4 float values
/// @param  b     Input vector 2. Must point to 4 float values
///
/// @return dotproduct(a,b)
//----------------------------------------------------------------------------
float DotProductScalarResultInRegister(float *a, float *b)
{
	return(a[0]*b[0]+a[1]*b[1]+a[2]*b[2]+a[3]*b[3]);
}

//----------------------------------------------------------------------------
//  DotProductScalarResultInMemory
//
/// Performs a Vector4 style dot product using scalar math, storing the
/// result directly into system memory.
///
/// @param  a         Input vector 1. Must point to 4 float values
/// @param  b         Input vector 2. Must point to 4 float values
/// @param  pfResult  [in] pointer to a float. [out] Contains the
///                   result, dotproduct(a,b)
//----------------------------------------------------------------------------
void DotProductScalarResultInMemory(float *a, float *b, float *pfResult)
{
	*pfResult = a[0]*b[0]+a[1]*b[1]+a[2]*b[2]+a[3]*b[3];
}

//----------------------------------------------------------------------------
//  TestFastNeonDotProduct
//
/// Run timing study of the four dot product functions above, writing the
/// results to the Android verbose log.
//----------------------------------------------------------------------------
void TestNeonDotProduct()
{
	float SET_ALIGNMENT(64) data[] = {float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),
										float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX)};

	float *a = &data[0];
	float *b = &data[4];

	char szMsg[256];
	
	sprintf(szMsg, "");
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg,"---------------------------------------");
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);

#ifdef PRINT_COMPUTED_VECTOR_RESULTS
	sprintf(szMsg, "Dot product test inputs A=<%f,%f,%f>, B=<%f,%f,%f>",
					a[0], a[1], a[2], b[0], b[1], b[2]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
#endif

	float SET_ALIGNMENT(64) fResult[2];
    
	sce::PhysicsEffects::PfxPerfCounter pc;
	double dTimeSpan, dRefTimeSpan;
	unsigned int uiNumTries = 10000000;
	unsigned int i;

	// profile scalar dot product with register return
	fResult[0] = 0.0f;
	pc.countBegin("");
	for(i = 0; i < uiNumTries; i++)
	{
		fResult[0] = DotProductScalarResultInRegister(a, b); // C++
	}
	pc.countEnd();
	dTimeSpan = pc.getCountTime(0);
	pc.resetCount();
	dRefTimeSpan = dTimeSpan;
#ifdef PRINT_COMPUTED_VECTOR_RESULTS
	sprintf(szMsg, "Time to do %i calls to scalar dot product return in register: %f, result value=%f",
					uiNumTries, dTimeSpan, fResult[0]);
#else
	sprintf(szMsg, "Sclr C++ Dot Product Register Return: %10.7f secs, REF TIME", dTimeSpan);
#endif
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);

	// profile NEON dot product with register return
	fResult[0] = 0.0f;
	pc.countBegin("");
	for(i = 0; i < uiNumTries; i++)
	{
		fResult[0] = DotProductNeonResultInRegister(a, b); // C++
	}
	pc.countEnd();
	dTimeSpan = pc.getCountTime(0);
	pc.resetCount();
#ifdef PRINT_COMPUTED_VECTOR_RESULTS
	sprintf(szMsg, "Time to do %i calls to NEON dot product return in register: %f, result value=%f",
					uiNumTries, dTimeSpan, fResult[0]);
#else
	sprintf(szMsg, "Neon C++ Dot Product Register Return: %10.7f secs, speedup: %5.2f", dTimeSpan, dRefTimeSpan/dTimeSpan);
#endif
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);

	// profile NEON dot product with register return, assembly version
	fResult[0] = 0.0f;
	pc.countBegin("");
	for(i = 0; i < uiNumTries; i++)
	{
		fResult[0] = DotProductNeonResultInRegisterAssembly(a, b);
	}
	pc.countEnd();
	dTimeSpan = pc.getCountTime(0);
	pc.resetCount();

#ifdef PRINT_COMPUTED_VECTOR_RESULTS
	sprintf(szMsg, "Time to do %i calls to NEON dot product return in register (assembly version): %f, result value=%f",
					uiNumTries, dTimeSpan, fResult[0]);
#else
	sprintf(szMsg, "Neon Asm Dot Product Register Return: %10.7f secs, speedup: %5.2f", dTimeSpan, dRefTimeSpan/dTimeSpan);
#endif
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);

	// profile scalar dot product with direct memory return
	fResult[0] = 0.0f;
	pc.countBegin("");
	for(i = 0; i < uiNumTries; i++)
	{
		DotProductScalarResultInMemory(a, b, fResult);
	}
	pc.countEnd();
	dTimeSpan = pc.getCountTime(0);
	pc.resetCount();

	dRefTimeSpan = dTimeSpan;
#ifdef PRINT_COMPUTED_VECTOR_RESULTS
	sprintf(szMsg, "Time to do %i calls to scalar dot product return in memory: %f, result value=%f",
					uiNumTries, dTimeSpan, fResult[0]);
#else
	sprintf(szMsg, "  Sclr C++ Dot Product Memory Return: %10.7f secs, REF TIME", dTimeSpan);
#endif
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);

	// profile NEON dot product with direct memory return
	fResult[0] = 0.0f;
	pc.countBegin("");
	for(i = 0; i < uiNumTries; i++)
	{
		DotProductNeonResultInMemory(a, b, fResult);
	}
	pc.countEnd();
	dTimeSpan = pc.getCountTime(0);
	pc.resetCount();

#ifdef PRINT_COMPUTED_VECTOR_RESULTS
	sprintf(szMsg, "Time to do %i calls to NEON dot product return in memory: %f, result value=%f",
					uiNumTries, dTimeSpan, fResult[0]);
#else
	sprintf(szMsg, "  Neon C++ Dot Product Memory Return: %10.7f secs, speedup: %5.2f", dTimeSpan, dRefTimeSpan/dTimeSpan);
#endif
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);

	// profile NEON dot product with direct memory return, assembly version
	fResult[0] = 0.0f;
	pc.countBegin("");
	for(i = 0; i < uiNumTries; i++)
	{
		DotProductNeonResultInMemoryAssembly(a, b, fResult);
	}
	pc.countEnd();
	dTimeSpan = pc.getCountTime(0);
	pc.resetCount();

#ifdef PRINT_COMPUTED_VECTOR_RESULTS
	sprintf(szMsg, "Time to do %i calls to NEON dot product return in memory (assembly version): %f, result value=%f",
					uiNumTries, dTimeSpan, fResult[0]);
#else
	sprintf(szMsg, "  Neon Asm Dot Product Memory Return: %10.7f secs, speedup: %5.2f", dTimeSpan, dRefTimeSpan/dTimeSpan);
#endif
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);

	// profile NEON dot product with direct memory return, assembly version with both
	// inputs in contiguous memory block (array of structures)
	fResult[0] = 0.0f;
	pc.countBegin("");
	for(i = 0; i < uiNumTries; i++)
	{
		DotProductNeonResultInMemoryAssembly2(data, fResult);
	}
	pc.countEnd();
	dTimeSpan = pc.getCountTime(0);
	pc.resetCount();

#ifdef PRINT_COMPUTED_VECTOR_RESULTS
	sprintf(szMsg, "Time to do %i calls to NEON dot product return in memory (assembly version - array of structures): %f, result value=%f",
					uiNumTries, dTimeSpan, fResult[0]);
#else
	sprintf(szMsg, " Neon Asm2 Dot Product Memory Return: %10.7f secs, speedup: %5.2f", dTimeSpan, dRefTimeSpan/dTimeSpan);
#endif
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);

	// profile NEON dot product with direct memory return, assembly version with both
	// inputs in contiguous memory block (array of structures), alternate methodology
	fResult[0] = 0.0f;
	pc.countBegin("");
	for(i = 0; i < uiNumTries; i++)
	{
		DotProductNeonResultInMemoryAssembly2b(data, fResult);
	}
	pc.countEnd();
	dTimeSpan = pc.getCountTime(0);
	pc.resetCount();

#ifdef PRINT_COMPUTED_VECTOR_RESULTS
	sprintf(szMsg, "Time to do %i calls to NEON dot product return in memory (assembly version - alternate methodology): %f, result value=%f",
					uiNumTries, dTimeSpan, fResult[0]);
#else
	sprintf(szMsg, " Neon Asm3 Dot Product Memory Return: %10.7f secs, speedup: %5.2f", dTimeSpan, dRefTimeSpan/dTimeSpan);
#endif
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
}
