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
	void OrthoInverseTransform3Neon(float *trfm, float *pfResult);
}

//----------------------------------------------------------------------------
//  OrthoInverseTransform3Scalar
//
/// Performs a ortho inverse of a transform3 using scalar math, storing the
/// result directly into system memory.
///
/// @param  trfm      Input transform3. Must point to 4x 4 float values
/// @param  pfResult  [in] pointer to a float(tranform3). [out] Contains the result
//----------------------------------------------------------------------------
//inline const Transform3 orthoInverse( const Transform3 & tfrm )
//{
//    Vector3 inv0, inv1, inv2;
//    inv0 = Vector3( tfrm.getCol0().getX(), tfrm.getCol1().getX(), tfrm.getCol2().getX() );
//    inv1 = Vector3( tfrm.getCol0().getY(), tfrm.getCol1().getY(), tfrm.getCol2().getY() );
//    inv2 = Vector3( tfrm.getCol0().getZ(), tfrm.getCol1().getZ(), tfrm.getCol2().getZ() );
//    return Transform3(
//        inv0,
//        inv1,
//        inv2,
//        Vector3( ( -( ( inv0 * tfrm.getCol3().getX() ) + ( ( inv1 * tfrm.getCol3().getY() ) + ( inv2 * tfrm.getCol3().getZ() ) ) ) ) )
//    );
//}
void OrthoInverseTransform3Scalar(float *trfm, float *pfResult)
{
	pfResult[0] = trfm[0];
	pfResult[1] = trfm[4];
	pfResult[2] = trfm[8];
	pfResult[3] = 0.0f;

	pfResult[4] = trfm[1];
	pfResult[5] = trfm[5];
	pfResult[6] = trfm[9];
	pfResult[7] = 0.0f;

	pfResult[8] = trfm[2];
	pfResult[9] = trfm[6];
	pfResult[10] = trfm[10];
	pfResult[11] = 0.0f;

	pfResult[12] = ( -( ( trfm[0] * trfm[8]) + ( ( trfm[1] * trfm[9]) + ( trfm[2] * trfm[10]) ) ) );
	pfResult[13] = ( -( ( trfm[4] * trfm[8]) + ( ( trfm[5] * trfm[9]) + ( trfm[6] * trfm[10]) ) ) );
	pfResult[14] = ( -( ( trfm[8] * trfm[8]) + ( ( trfm[9] * trfm[9]) + ( trfm[10] * trfm[10]) ) ) );
	pfResult[15] = 0.0f;
}

//----------------------------------------------------------------------------
//  TestNeonOrthoInverseTransform3
//
/// Run timing study of the orthoinverse on transform3 functions above, writing the
/// results to the Android verbose log.
//----------------------------------------------------------------------------
void TestNeonOrthoInverseTransform3()
{
	float SET_ALIGNMENT(64) data1[] = {float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),0.0f,
									   float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),0.0f,
									   float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),0.0f,
									   float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),0.0f};
	float *trfm = &data1[0];

	char szMsg[256];

	sprintf(szMsg, "");
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg,"---------------------------------------");
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg,"TestNeonOrthoInverseTransform3 Start");
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg, "Test input trfm: <%f,%f,%f,%f> <%f,%f,%f,%f> <%f,%f,%f,%f> <%f,%f,%f,%f>>",
					trfm[0], trfm[1], trfm[2], trfm[3],
					trfm[4], trfm[5], trfm[6], trfm[7],
					trfm[8], trfm[9], trfm[10], trfm[11],
					trfm[12], trfm[13], trfm[14], trfm[15]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);

	float SET_ALIGNMENT(64) fResult[16];
    

	sce::PhysicsEffects::PfxPerfCounter pc;
	double dTimeSpan, dRefTimeSpan;; 
	unsigned int uiNumTries = 10000000;
	unsigned int i;

// profile scalar OrthoInverseTransform3Scalar with direct memory return, c++ version
	fResult[0] = 0.0f;
	pc.countBegin("");
	for(i = 0; i < uiNumTries; i++)
	{
		OrthoInverseTransform3Scalar(trfm, fResult);
	}
	pc.countEnd();
	dTimeSpan = pc.getCountTime(0);
	pc.resetCount();
	dRefTimeSpan = dTimeSpan;
	sprintf(szMsg, "Time to do %i calls for OrthoInverseTransform3Scalar: %f secs, speedup: %5.2f",
					uiNumTries, dTimeSpan, dRefTimeSpan/dTimeSpan);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg, "result value=<%f,%f,%f,%f> <%f,%f,%f,%f> <%f,%f,%f,%f> <%f,%f,%f,%f>",
				   fResult[0], fResult[1], fResult[2], fResult[3],
				   fResult[4], fResult[5], fResult[6], fResult[7],
				   fResult[8], fResult[9], fResult[10], fResult[11],
				   fResult[12], fResult[13], fResult[14], fResult[15]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);


// profile NEON OrthoInverseTransform3Neon with direct memory return, assembly version
	fResult[0] = 0.0f;
	pc.countBegin("");
	for(i = 0; i < uiNumTries; i++)
	{
		OrthoInverseTransform3Neon(trfm, fResult);
	}
	pc.countEnd();
	dTimeSpan = pc.getCountTime(0);
	pc.resetCount();
	sprintf(szMsg, "Time to do %i calls for OrthoInverseTransform3Neon: %f secs, speedup: %5.2f",
					uiNumTries, dTimeSpan, dRefTimeSpan/dTimeSpan);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg, "result value=<%f,%f,%f,%f> <%f,%f,%f,%f> <%f,%f,%f,%f> <%f,%f,%f,%f>",
					fResult[0], fResult[1], fResult[2], fResult[3],
				    fResult[4], fResult[5], fResult[6], fResult[7],
				    fResult[8], fResult[9], fResult[10], fResult[11],
				    fResult[12], fResult[13], fResult[14], fResult[15]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);

	sprintf(szMsg,"TestNeonOrthoInverseTransform3 End");
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
}
