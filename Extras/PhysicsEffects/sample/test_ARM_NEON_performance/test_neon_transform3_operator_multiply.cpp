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

// assembly implementations
extern "C"
{
	void Transform3OperatorMultiplyNeon(float *trfm, float *vec3, float *pfResult);
}

//----------------------------------------------------------------------------
//  Transform3OperatorMultiplyScalar
//
/// Performs a multiply operation on a tranform3 using scalar math, storing the
/// result directly into system memory.
///
/// @param  trfm      Input transform. Must point to 4x 4 float values
/// @param  vec3      Input vector3. Must point to 4 float values
/// @param  pfResult  [in] pointer to a float(vector3). [out] Contains the result
//----------------------------------------------------------------------------
//inline const Vector3 Transform3::operator *( const Vector3 & vec ) const
//{
//    return Vector3(
//        ( ( ( mCol0.getX() * vec.getX() ) + ( mCol1.getX() * vec.getY() ) ) + ( mCol2.getX() * vec.getZ() ) ),
//        ( ( ( mCol0.getY() * vec.getX() ) + ( mCol1.getY() * vec.getY() ) ) + ( mCol2.getY() * vec.getZ() ) ),
//        ( ( ( mCol0.getZ() * vec.getX() ) + ( mCol1.getZ() * vec.getY() ) ) + ( mCol2.getZ() * vec.getZ() ) )
//    );
//}
void Transform3OperatorMultiplyScalar(float *trfm, float *vec3, float *pfResult)
{
	pfResult[0] = ( ( ( trfm[0] * vec3[0] ) + ( trfm[4] * vec3[1] ) ) + ( trfm[8] * vec3[2] ) );
	pfResult[1] = ( ( ( trfm[1] * vec3[0] ) + ( trfm[5] * vec3[1] ) ) + ( trfm[9] * vec3[2] ) );
	pfResult[2] = ( ( ( trfm[2] * vec3[0] ) + ( trfm[6] * vec3[1] ) ) + ( trfm[10] * vec3[2] ) );
	pfResult[3] = 0.0f;
}

//----------------------------------------------------------------------------
//  TestNeonTransform3OperatorMultiply
//
/// Run timing study of the Tranform3 multiply operator from above, writing the
/// results to the Android verbose log.
//----------------------------------------------------------------------------
void TestNeonTransform3OperatorMultiply()
{
	float SET_ALIGNMENT(64) data1[] = {float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),0.0f,
									   float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),0.0f,
									   float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),0.0f,
									   float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),0.0f};
	float SET_ALIGNMENT(64) data2[] = {float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),0.0f};

	float *trfm = &data1[0];
	float *vec3 = &data2[0];


	char szMsg[256];

	sprintf(szMsg, "");
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg,"---------------------------------------");
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg,"TestNeonTransform3OperatorMultiply Start");
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg, "Test input vec3: <%f,%f,%f,%f>",
					vec3[0], vec3[1], vec3[2], vec3[3]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg, "Test input trfm: <%f,%f,%f,%f> <%f,%f,%f,%f> <%f,%f,%f,%f> <%f,%f,%f,%f>>",
					trfm[0], trfm[1], trfm[2], trfm[3],
					trfm[4], trfm[5], trfm[6], trfm[7],
					trfm[8], trfm[9], trfm[10], trfm[11],
					trfm[12], trfm[13], trfm[14], trfm[15]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);

	float SET_ALIGNMENT(64) fResult[4];
    

	sce::PhysicsEffects::PfxPerfCounter pc;
	double dTimeSpan, dRefTimeSpan;; 
	unsigned int uiNumTries = 10000000;
	unsigned int i;

// profile scalar Transform3OperatorMultiplyScalar with direct memory return, c++ version
	fResult[0] = 0.0f;
	pc.countBegin("");
	for(i = 0; i < uiNumTries; i++)
	{
		Transform3OperatorMultiplyScalar(trfm, vec3, fResult);
	}
	pc.countEnd();
	dTimeSpan = pc.getCountTime(0);
	pc.resetCount();
	dRefTimeSpan = dTimeSpan;
	sprintf(szMsg, "Time to do %i calls for Transform3OperatorMultiplyScalar: %f secs, speedup: %5.2f, result value=<%f,%f,%f,%f>",
					uiNumTries, dTimeSpan, dRefTimeSpan/dTimeSpan, fResult[0], fResult[1], fResult[2], fResult[3]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);


// profile NEON Transform3OperatorMultiplyNeon with direct memory return, assembly version
	fResult[0] = 0.0f;
	pc.countBegin("");
	for(i = 0; i < uiNumTries; i++)
	{
		Transform3OperatorMultiplyNeon(trfm, vec3, fResult);
	}
	pc.countEnd();
	dTimeSpan = pc.getCountTime(0);
	pc.resetCount();
	sprintf(szMsg, "Time to do %i calls for Transform3OperatorMultiplyNeon: %f secs, speedup: %5.2f, result value=<%f,%f,%f,%f>",
					uiNumTries, dTimeSpan, dRefTimeSpan/dTimeSpan, fResult[0], fResult[1], fResult[2], fResult[3]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);


	sprintf(szMsg,"TestNeonTransform3OperatorMultiply End");
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
}
