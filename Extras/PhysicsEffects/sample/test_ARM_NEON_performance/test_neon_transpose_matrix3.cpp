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
	void TransposeMatrix3Neon(float *mCol, float *pfResult);
}

//----------------------------------------------------------------------------
//  TransposeMatrix3Scalar
//
/// Performs a transpose on a matrix3 using scalar math, storing the
/// result directly into system memory.
///
/// @param  mCol      Input matrix. Must point to 3x 4 float values
/// @param  pfResult  [in] pointer to a float(matrix3). [out] Contains the result
//----------------------------------------------------------------------------
//inline const Matrix3 transpose( const Matrix3 & mat )
//{
//    return Matrix3(
//        Vector3( mat.getCol0().getX(), mat.getCol1().getX(), mat.getCol2().getX() ),
//        Vector3( mat.getCol0().getY(), mat.getCol1().getY(), mat.getCol2().getY() ),
//        Vector3( mat.getCol0().getZ(), mat.getCol1().getZ(), mat.getCol2().getZ() )
//    );
//}
void TransposeMatrix3Scalar(float *mCol, float *pfResult)
{
	pfResult[0] = mCol[0];
	pfResult[1] = mCol[4];
	pfResult[2] = mCol[8];
	pfResult[3] = 0.0f;

	pfResult[4] = mCol[1];
	pfResult[5] = mCol[5];
	pfResult[6] = mCol[9];
	pfResult[7] = 0.0f;

	pfResult[8] = mCol[2];
	pfResult[9] = mCol[6];
	pfResult[10] = mCol[10];
	pfResult[11] = 0.0f;
}

//----------------------------------------------------------------------------
//  TestNeonTransposeMatrix3
//
/// Run timing study of the matrix3 transpose functions above, writing the
/// results to the Android verbose log.
//----------------------------------------------------------------------------
void TestNeonTransposeMatrix3()
{
	float SET_ALIGNMENT(64) data[] = {float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),0.0f,
									  float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),0.0f,
									  float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),0.0f};
	float *mCol = &data[0];

	char szMsg[256];

	sprintf(szMsg, "");
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg,"---------------------------------------");
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg,"TestNeonTransposeMatrix3 Start");
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg, "Test input mCol:  <%f,%f,%f,%f> <%f,%f,%f,%f> <%f,%f,%f,%f>",
			mCol[0], mCol[1], mCol[2], mCol[3],
			mCol[4], mCol[5], mCol[6], mCol[7],
			mCol[8], mCol[9], mCol[10], mCol[11]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);

	float SET_ALIGNMENT(64) fResult[12];
    

	sce::PhysicsEffects::PfxPerfCounter pc;
	double dTimeSpan, dRefTimeSpan;; 
	unsigned int uiNumTries = 10000000;
	unsigned int i;

// profile scalar TransposeMatrix3Scalar with direct memory return, c++ version
	fResult[0] = 0.0f;
	pc.countBegin("");
	for(i = 0; i < uiNumTries; i++)
	{
		TransposeMatrix3Scalar(mCol, fResult);
	}
	pc.countEnd();
	dTimeSpan = pc.getCountTime(0);
	pc.resetCount();
	dRefTimeSpan = dTimeSpan;
	sprintf(szMsg, "Time to do %i calls for TransposeMatrix3Scalar: %f secs, speedup: %5.2f, result value=<%f,%f,%f,%f> <%f,%f,%f,%f> <%f,%f,%f,%f>",
					uiNumTries, dTimeSpan, dRefTimeSpan/dTimeSpan, fResult[0], fResult[1], fResult[2], fResult[3],
																   fResult[4], fResult[5], fResult[6], fResult[7],
																   fResult[8], fResult[9], fResult[10], fResult[11]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);


// profile NEON TransposeMatrix3Neon with direct memory return, assembly version
	fResult[0] = 0.0f;
	pc.countBegin("");
	for(i = 0; i < uiNumTries; i++)
	{
		TransposeMatrix3Neon(mCol, fResult);
	}
	pc.countEnd();
	dTimeSpan = pc.getCountTime(0);
	pc.resetCount();
	sprintf(szMsg, "Time to do %i calls for TransposeMatrix3Neon: %f secs, speedup: %5.2f, result value=<%f,%f,%f,%f> <%f,%f,%f,%f> <%f,%f,%f,%f>",
					uiNumTries, dTimeSpan, dRefTimeSpan/dTimeSpan, fResult[0], fResult[1], fResult[2], fResult[3],
																   fResult[4], fResult[5], fResult[6], fResult[7],
																   fResult[8], fResult[9], fResult[10], fResult[11]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);


	sprintf(szMsg,"TestNeonTransposeMatrix3 End");
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
}
