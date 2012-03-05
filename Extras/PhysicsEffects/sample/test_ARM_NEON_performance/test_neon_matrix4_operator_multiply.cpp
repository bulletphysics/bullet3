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
	void Matrix4OperatorMultiplyNeon(float *mCol, float *mat, float *pfResult);
}

//----------------------------------------------------------------------------
//  Matrix4OperatorMultiplyScalar
//
/// Performs a Matrix4 vector3 multiply using scalar math, storing the
/// result directly into system memory.
///
/// @param  mCol      Input matrix 1. Must point to 4x 4 float values
/// @param  mat       Input matrix 2. Must point to 4x 4 float values
/// @param  pfResult  [in] pointer to a float(matrix). [out] Contains the result
//----------------------------------------------------------------------------
//inline const Vector4 Matrix4::operator *( const Vector4 & vec ) const
//{
//    return Vector4(
//        ( ( ( ( mCol0.getX() * vec.getX() ) + ( mCol1.getX() * vec.getY() ) ) + ( mCol2.getX() * vec.getZ() ) ) + ( mCol3.getX() * vec.getW() ) ),
//        ( ( ( ( mCol0.getY() * vec.getX() ) + ( mCol1.getY() * vec.getY() ) ) + ( mCol2.getY() * vec.getZ() ) ) + ( mCol3.getY() * vec.getW() ) ),
//        ( ( ( ( mCol0.getZ() * vec.getX() ) + ( mCol1.getZ() * vec.getY() ) ) + ( mCol2.getZ() * vec.getZ() ) ) + ( mCol3.getZ() * vec.getW() ) ),
//        ( ( ( ( mCol0.getW() * vec.getX() ) + ( mCol1.getW() * vec.getY() ) ) + ( mCol2.getW() * vec.getZ() ) ) + ( mCol3.getW() * vec.getW() ) )
//    );
//}
//inline const Matrix4 Matrix4::operator *( const Matrix4 & mat ) const
//{
//    return Matrix4(
//        ( *this * mat.mCol0 ),
//        ( *this * mat.mCol1 ),
//        ( *this * mat.mCol2 ),
//        ( *this * mat.mCol3 )
//    );
//}
void Matrix4OperatorMultiplyScalar(float *mCol, float *mat, float *pfResult)
{
	pfResult[0] = ( ( ( mCol[0] * mat[0] ) + ( mCol[4] * mat[1] ) ) + ( mCol[8] * mat[2] )  + ( mCol[12] * mat[3] ) );
	pfResult[1] = ( ( ( mCol[1] * mat[0] ) + ( mCol[5] * mat[1] ) ) + ( mCol[9] * mat[2] )  + ( mCol[13] * mat[3] ) );
	pfResult[2] = ( ( ( mCol[2] * mat[0] ) + ( mCol[6] * mat[1] ) ) + ( mCol[10] * mat[2] )  + ( mCol[14] * mat[3] ) );
	pfResult[3] = ( ( ( mCol[3] * mat[0] ) + ( mCol[7] * mat[1] ) ) + ( mCol[11] * mat[2] )  + ( mCol[15] * mat[3] ) );

	pfResult[4] = ( ( ( mCol[0] * mat[4] ) + ( mCol[4] * mat[5] ) ) + ( mCol[8] * mat[6] )  + ( mCol[12] * mat[7] ) );
	pfResult[5] = ( ( ( mCol[1] * mat[4] ) + ( mCol[5] * mat[5] ) ) + ( mCol[9] * mat[6] )  + ( mCol[13] * mat[7] ) );
	pfResult[6] = ( ( ( mCol[2] * mat[4] ) + ( mCol[6] * mat[5] ) ) + ( mCol[10] * mat[6] )  + ( mCol[14] * mat[7] ) );
	pfResult[7] = ( ( ( mCol[3] * mat[4] ) + ( mCol[7] * mat[5] ) ) + ( mCol[11] * mat[6] )  + ( mCol[15] * mat[7] ) );

	pfResult[8] = ( ( ( mCol[0] * mat[8] ) + ( mCol[4] * mat[9] ) ) + ( mCol[8] * mat[10] )  + ( mCol[12] * mat[11] ) );
	pfResult[9] = ( ( ( mCol[1] * mat[8] ) + ( mCol[5] * mat[9] ) ) + ( mCol[9] * mat[10] )  + ( mCol[13] * mat[11] ) );
	pfResult[10] = ( ( ( mCol[2] * mat[8] ) + ( mCol[6] * mat[9] ) ) + ( mCol[10] * mat[10] )  + ( mCol[14] * mat[11] ) );
	pfResult[11] = ( ( ( mCol[3] * mat[8] ) + ( mCol[7] * mat[9] ) ) + ( mCol[11] * mat[10] )  + ( mCol[15] * mat[11] ) );

	pfResult[12] = ( ( ( mCol[0] * mat[12] ) + ( mCol[4] * mat[13] ) ) + ( mCol[8] * mat[14] )  + ( mCol[12] * mat[15] ) );
	pfResult[13] = ( ( ( mCol[1] * mat[12] ) + ( mCol[5] * mat[13] ) ) + ( mCol[9] * mat[14] )  + ( mCol[13] * mat[15] ) );
	pfResult[14] = ( ( ( mCol[2] * mat[12] ) + ( mCol[6] * mat[13] ) ) + ( mCol[10] * mat[14] )  + ( mCol[14] * mat[15] ) );
	pfResult[15] = ( ( ( mCol[3] * mat[12] ) + ( mCol[7] * mat[13] ) ) + ( mCol[11] * mat[14] )  + ( mCol[15] * mat[15] ) );
}

//----------------------------------------------------------------------------
//  TestNeonMatrix4OperatorMultiply
//
/// Run timing study of the matrix4 vector3 multiply operator from above, writing the
/// results to the Android verbose log.
//----------------------------------------------------------------------------
void TestNeonMatrix4OperatorMultiply()
{
	float SET_ALIGNMENT(64) data1[] = {float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),
									   float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),
									   float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),
									   float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX)};
	float SET_ALIGNMENT(64) data2[] = {float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),
									   float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),
									   float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),
									   float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX),float(rand())/float(RAND_MAX)};
	float *mCol = &data1[0];
	float *mat = &data2[0];

	char szMsg[256];

	sprintf(szMsg, "");
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg,"---------------------------------------");
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg,"TestNeonMatrix4OperatorMultiply Start");
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg, "Test input mCol:  <%f,%f,%f,%f> <%f,%f,%f,%f> <%f,%f,%f,%f> <%f,%f,%f,%f>",
			mCol[0], mCol[1], mCol[2], mCol[3],
			mCol[4], mCol[5], mCol[6], mCol[7],
			mCol[8], mCol[9], mCol[10], mCol[11],
			mCol[12], mCol[13], mCol[14], mCol[15]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg, "Test input mat: <%f,%f,%f,%f> <%f,%f,%f,%f> <%f,%f,%f,%f> <%f,%f,%f,%f>",
			mat[0], mat[1], mat[2], mat[3],
			mat[4], mat[5], mat[6], mat[7],
			mat[8], mat[9], mat[10], mat[11],
			mCol[12], mCol[13], mCol[14], mCol[15]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);

	float SET_ALIGNMENT(64) fResult[16];
    

	sce::PhysicsEffects::PfxPerfCounter pc;
	double dTimeSpan, dRefTimeSpan;; 
	unsigned int uiNumTries = 10000000;
	unsigned int i;

// profile scalar Matrix4OperatorMultiplyScalar with direct memory return, c++ version
	fResult[0] = 0.0f;
	pc.countBegin("");
	for(i = 0; i < uiNumTries; i++)
	{
		Matrix4OperatorMultiplyScalar(mCol, mat, fResult);
	}
	pc.countEnd();
	dTimeSpan = pc.getCountTime(0);
	pc.resetCount();
	dRefTimeSpan = dTimeSpan;
	sprintf(szMsg, "Time to do %i calls for Matrix4OperatorMultiplyScalar: %f secs, speedup: %5.2f",
					uiNumTries, dTimeSpan, dRefTimeSpan/dTimeSpan);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg, "result value: <%f,%f,%f,%f> <%f,%f,%f,%f> <%f,%f,%f,%f> <%f,%f,%f,%f>",
			fResult[0], fResult[1], fResult[2], fResult[3],
		    fResult[4], fResult[5], fResult[6], fResult[7],
		    fResult[8], fResult[9], fResult[10], fResult[11],
		    fResult[12], fResult[13], fResult[14], fResult[15]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);


// profile NEON Matrix4OperatorMultiplyNeon with direct memory return, assembly version
	fResult[0] = 0.0f;
	pc.countBegin("");
	for(i = 0; i < uiNumTries; i++)
	{
		Matrix4OperatorMultiplyNeon(mCol, mat, fResult);
	}
	pc.countEnd();
	dTimeSpan = pc.getCountTime(0);
	pc.resetCount();
	sprintf(szMsg, "Time to do %i calls for Matrix4OperatorMultiplyNeon: %f secs, speedup: %5.2f, result value=<%f,%f,%f,%f> <%f,%f,%f,%f> <%f,%f,%f,%f> <%f,%f,%f,%f>",
					uiNumTries, dTimeSpan, dRefTimeSpan/dTimeSpan, fResult[0], fResult[1], fResult[2], fResult[3],
																   fResult[4], fResult[5], fResult[6], fResult[7],
																   fResult[8], fResult[9], fResult[10], fResult[11],
																   fResult[12], fResult[13], fResult[14], fResult[15]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);


	sprintf(szMsg,"TestNeonMatrix4OperatorMultiply End");
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
}
