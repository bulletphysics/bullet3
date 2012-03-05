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

//----------------------------------------------------------------------------
//  pfxSolveLinearConstraintRowScalar
//
/// Performs
///
/// @param           Input
//----------------------------------------------------------------------------
void pfxSolveLinearConstraintRowScalar(float *constraint,				// 8 floats
									   float *deltaLinearVelocityA,		// 4 floats
									   float *deltaAngularVelocityA,	// 4 floats
									   float massInvA,					// 1 floats
									   float *inertiaInvA,				// 12 floats
									   float *rA,						// 4 floats
									   float *deltaLinearVelocityB,		// 4 floats
									   float *deltaAngularVelocityB,	// 4 floats
									   float massInvB,					// 1 floats
									   float *inertiaInvB,				// 12 floats
									   float *rB)						// 4 floats
{
//const PfxVector3 normal(pfxReadVector3(constraint.m_normal)); //fourth element is not zero and it need to be in assembly
	float SET_ALIGNMENT(64) normalRead[] = {constraint[0],constraint[1],constraint[2],0.0f};
	float *normal = &normalRead[0];
//PfxFloat deltaImpulse = constraint.m_rhs;
	float deltaImpulse = constraint[4];
//PfxVector3 dVA = deltaLinearVelocityA + cross(deltaAngularVelocityA,rA);
	float SET_ALIGNMENT(64) dVARead[] = {deltaLinearVelocityA[0] + (deltaAngularVelocityA[1]*rA[2] - deltaAngularVelocityA[2]*rA[1]),
										 deltaLinearVelocityA[1] + (deltaAngularVelocityA[2]*rA[0] - deltaAngularVelocityA[0]*rA[2]),
										 deltaLinearVelocityA[2] + (deltaAngularVelocityA[0]*rA[1] - deltaAngularVelocityA[1]*rA[0]),
										 0.0f};
	float *dVA = &dVARead[0];
//PfxVector3 dVB = deltaLinearVelocityB + cross(deltaAngularVelocityB,rB);
	float SET_ALIGNMENT(64) dVBRead[] = {deltaLinearVelocityB[0] + (deltaAngularVelocityB[1]*rB[2] - deltaAngularVelocityB[2]*rB[1]),
										 deltaLinearVelocityB[1] + (deltaAngularVelocityB[2]*rB[0] - deltaAngularVelocityB[0]*rB[2]),
										 deltaLinearVelocityB[2] + (deltaAngularVelocityB[0]*rB[1] - deltaAngularVelocityB[1]*rB[0]),
										 0.0f};
	float *dVB = &dVBRead[0];
//deltaImpulse -= constraint.m_jacDiagInv * dot(normal,dVA-dVB);
	deltaImpulse -= constraint[4] * (normal[0]*(dVA[0]-dVB[0]) + normal[1]*(dVA[1]-dVB[1]) + normal[2]*(dVA[2]-dVB[2]));
//PfxFloat oldImpulse = constraint.m_accumImpulse;
	float oldImpulse = constraint[7];
//constraint.m_accumImpulse = SCE_PFX_CLAMP(oldImpulse + deltaImpulse,constraint.m_lowerLimit,constraint.m_upperLimit);
	if((oldImpulse + deltaImpulse) > constraint[6]){
		if(constraint[6] > constraint[5]){
			constraint[7] = constraint[6];
		}else {
			constraint[7] = constraint[5];
		}
	} else {
		if(oldImpulse + deltaImpulse > constraint[5]){
			constraint[7] = oldImpulse + deltaImpulse;
		}else {
			constraint[7] = constraint[5];
		}
	}
//deltaImpulse = constraint.m_accumImpulse - oldImpulse;
	deltaImpulse = constraint[7] - oldImpulse;
//deltaLinearVelocityA += deltaImpulse * massInvA * normal;
	deltaLinearVelocityA[0] += deltaImpulse * massInvA * normal[0];
	deltaLinearVelocityA[1] += deltaImpulse * massInvA * normal[1];
	deltaLinearVelocityA[2] += deltaImpulse * massInvA * normal[2];
	deltaLinearVelocityA[3] += deltaImpulse * massInvA * normal[3];

//deltaAngularVelocityA += deltaImpulse * inertiaInvA * cross(rA,normal);
	deltaAngularVelocityA[0] += deltaImpulse * ((inertiaInvA[0] * (rA[1]*normal[2] - rA[2]*normal[1])) +
											    (inertiaInvA[4] * (rA[2]*normal[0] - rA[0]*normal[2])) +//
											    (inertiaInvA[8] * (rA[0]*normal[1] - rA[1]*normal[0])));
	deltaAngularVelocityA[1] += deltaImpulse * ((inertiaInvA[1] * (rA[1]*normal[2] - rA[2]*normal[1])) +
												(inertiaInvA[5] * (rA[2]*normal[0] - rA[0]*normal[2])) +
												(inertiaInvA[9] * (rA[0]*normal[1] - rA[1]*normal[0])));
	deltaAngularVelocityA[2] += deltaImpulse * ((inertiaInvA[2] * (rA[1]*normal[2] - rA[2]*normal[1])) +
												(inertiaInvA[6] * (rA[2]*normal[0] - rA[0]*normal[2])) +
												(inertiaInvA[10] * (rA[0]*normal[1] - rA[1]*normal[0])));
	deltaAngularVelocityA[3] += deltaImpulse * ((inertiaInvA[3] * (rA[1]*normal[2] - rA[2]*normal[1])) +
												(inertiaInvA[7] * (rA[2]*normal[0] - rA[0]*normal[2])) +
												(inertiaInvA[11] * (rA[0]*normal[1] - rA[1]*normal[0])));
//deltaLinearVelocityB -= deltaImpulse * massInvB * normal;
	deltaLinearVelocityB[0] -= deltaImpulse * massInvB * normal[0];
	deltaLinearVelocityB[1] -= deltaImpulse * massInvB * normal[1];
	deltaLinearVelocityB[2] -= deltaImpulse * massInvB * normal[2];
	deltaLinearVelocityB[3] -= deltaImpulse * massInvB * normal[3];

//deltaAngularVelocityB -= deltaImpulse * inertiaInvB * cross(rB,normal);
	deltaAngularVelocityB[0] -= deltaImpulse * ((inertiaInvB[0] * (rB[1]*normal[2] - rB[2]*normal[1])) +
											    (inertiaInvB[4] * (rB[2]*normal[0] - rB[0]*normal[2])) +
											    (inertiaInvB[8] * (rB[0]*normal[1] - rB[1]*normal[0])));
	deltaAngularVelocityB[1] -= deltaImpulse * ((inertiaInvB[1] * (rB[1]*normal[2] - rB[2]*normal[1])) +
												(inertiaInvB[5] * (rB[2]*normal[0] - rB[0]*normal[2])) +
												(inertiaInvB[9] * (rB[0]*normal[1] - rB[1]*normal[0])));
	deltaAngularVelocityB[2] -= deltaImpulse * ((inertiaInvB[2] * (rB[1]*normal[2] - rB[2]*normal[1])) +
												(inertiaInvB[6] * (rB[2]*normal[0] - rB[0]*normal[2])) +
												(inertiaInvB[10] * (rB[0]*normal[1] - rB[1]*normal[0])));
	deltaAngularVelocityB[3] -= deltaImpulse * ((inertiaInvB[3] * (rB[1]*normal[2] - rB[2]*normal[1])) +
												(inertiaInvB[7] * (rB[2]*normal[0] - rB[0]*normal[2])) +
												(inertiaInvB[11] * (rB[0]*normal[1] - rB[1]*normal[0])));
}

/*//////////////////////////////////////Source Code///////////////////////////////////////
void pfxSolveLinearConstraintRow(
	  PfxConstraintRow &constraint,      	// 8 floats
	  PfxVector3 &deltaLinearVelocityA,		// 4 floats
	  PfxVector3 &deltaAngularVelocityA,	// 4 floats
	  PfxFloat massInvA,					// 1 float
const PfxMatrix3 &inertiaInvA,				// 12 floats
const PfxVector3 &rA,						// 4 floats
	  PfxVector3 &deltaLinearVelocityB,		// 4 floats
	  PfxVector3 &deltaAngularVelocityB,	// 4 floats
	  PfxFloat massInvB,					// 1 float
const PfxMatrix3 &inertiaInvB,				// 12 floats
const PfxVector3 &rB)						// 4 floats
{
	//  PfxConstraintRow structure:
	//		PfxFloat m_normal[3];
	//		PfxFloat m_rhs;
	//		PfxFloat m_jacDiagInv;
	//		PfxFloat m_lowerLimit;
	//		PfxFloat m_upperLimit;
	//		PfxFloat m_accumImpulse;
const PfxVector3 normal(pfxReadVector3(constraint.m_normal));
PfxFloat deltaImpulse = constraint.m_rhs;
PfxVector3 dVA = deltaLinearVelocityA + cross(deltaAngularVelocityA,rA);
PfxVector3 dVB = deltaLinearVelocityB + cross(deltaAngularVelocityB,rB);
deltaImpulse -= constraint.m_jacDiagInv * dot(normal,dVA-dVB);
PfxFloat oldImpulse = constraint.m_accumImpulse;
constraint.m_accumImpulse = SCE_PFX_CLAMP(oldImpulse + deltaImpulse,constraint.m_lowerLimit,constraint.m_upperLimit);
deltaImpulse = constraint.m_accumImpulse - oldImpulse;
deltaLinearVelocityA += deltaImpulse * massInvA * normal;
deltaAngularVelocityA += deltaImpulse * inertiaInvA * cross(rA,normal);
deltaLinearVelocityB -= deltaImpulse * massInvB * normal;
deltaAngularVelocityB -= deltaImpulse * inertiaInvB * cross(rB,normal);
}
*/

void pfxSolveLinearConstraintRowNeonInline(float *constraint,		// 8 floats
								  float *deltaLinearVelocityA,		// 4 floats
								  float *deltaAngularVelocityA,		// 4 floats
								  float *massInvA,					// 1 floats
								  float *inertiaInvA,				// 12 floats
								  float *rA,						// 4 floats
								  float *deltaLinearVelocityB,		// 4 floats
								  float *deltaAngularVelocityB,		// 4 floats
								  float *massInvB,					// 1 floats
								  float *inertiaInvB,				// 12 floats
								  float *rB)						// 4 floats
{
//	  "r" (constraint), 			//%0
//	  "r" (deltaLinearVelocityA), 	//%1
//	  "r" (deltaAngularVelocityA), 	//%2
//	  "r" (massInvA), 				//%3
//	  "r" (inertiaInvA), 			//%4
//	  "r" (rA), 					//%5
//	  "r" (deltaLinearVelocityB), 	//%6
//	  "r" (deltaAngularVelocityB),  //%7
//	  "r" (massInvB), 				//%8
//	  "r" (inertiaInvB), 			//%9
//	  "r" (rB) 						//%10
	asm volatile
			(
			//Loads beforehand so the normal vector will be able to have zero in the 4th element
				"vld1.32 	{q0}, [%1]			\n\t" //LOAD => deltaLinearVelocityA ----------------> q0
				"vld1.32	{q1}, [%2]			\n\t" //LOAD => deltaAngularVelocityA ---------------> q1
				"vld1.32	{q2}, [%6]			\n\t" //LOAD => deltaLinearVelocityB ----------------> q2
				"vld1.32	{q3}, [%7]			\n\t" //LOAD => deltaAngularVelocityB ---------------> q3
	//const PfxVector3 normal(pfxReadVector3(constraint.m_normal));
				"vld1.32	{q4}, [%0]!			\n\t" //LOAD => constraint normal--------------------> q4
				"vdup.32	d10, d1[1]			\n\t" //set 4th element on normal vector to zero
				"vtrn.32	d9, d10				\n\t" //LOAD => deltaImpulse ------------------------> q5 (d10[0] only)
	//PfxFloat deltaImpulse = constraint.m_rhs;
				"vld1.32	{d12}, [%0]!		\n\t" //LOAD => constraint variables ----------------> q6
				"vld1.32	{d13[0]}, [%0]!		\n\t" //constraint  variables
				"vld1.32	{d13[1]}, [%0]		\n\t" //constraint load of remaining variables loaded this way keep pointer to m_accumilate in order to store back in later
	//PfxVector3 dVA = deltaLinearVelocityA + cross(deltaAngularVelocityA,rA);
	//PfxVector3 dVB = deltaLinearVelocityB + cross(deltaAngularVelocityB,rB);
				"vld1.32 	{d14[1]}, [%5]		\n\t" //LOAD => rA for cross product use 1A ---------> q7 (save these for use later)
				"vld1.32 	{d18[1]}, [%10]		\n\t" //LOAD => rB for cross product use 1B ---------> q9
				"vld1.32 	{d17[0]}, [%5]!		\n\t" //LOAD => rA for cross product use 2A ---------> q8
				"vld1.32 	{d21[0]}, [%10]!	\n\t" //LOAD => rB for cross product use 2B ---------> q10
				"vld1.32 	{d16}, [%5]			\n\t" //rA 2
				"vld1.32 	{d20}, [%10]		\n\t" //rB 2
				"vld1.32 	{d15[0]}, [%5]!		\n\t" //rA 1
				"vld1.32 	{d19[0]}, [%10]!	\n\t" //rB 1
				"vld1.32 	{d14[0]}, [%5]!		\n\t" //rA 1
				"vld1.32 	{d18[0]}, [%10]!	\n\t" //rB 1
				"vld1.32 	{d15[1]}, [%5]		\n\t" //rA 1
				"vld1.32 	{d19[1]}, [%10]		\n\t" //rB 1
				"vld1.32 	{d17[1]}, [%5]!		\n\t" //rA 2
				"vld1.32 	{d21[1]}, [%10]!	\n\t" //rB 2
				"vdup.32	q12, d1[1]			\n\t" //set deltaAngularVelocityB 2
				"vdup.32	q13, d1[1]			\n\t" //set deltaAngularVelocityB 1
				"vdup.32	q14, d1[1]			\n\t" //set deltaAngularVelocityA 1
				"vdup.32	q15, d1[1]			\n\t" //set deltaAngularVelocityA 2
				"vadd.f32 	q14, q14, q1		\n\t"
				"vadd.f32 	q13, q13, q3		\n\t"
				"vadd.f32 	q15, q15, q1		\n\t"
				"vadd.f32 	q12, q12, q3		\n\t"
				"vrev64.32	d28, d28			\n\t" //set deltaAngularVelocityA 1
				"vrev64.32	d26, d26			\n\t" //set deltaAngularVelocityB 1
				"vtrn.32	d30, d31			\n\t" //set deltaAngularVelocityA 2
				"vtrn.32	d24, d25			\n\t" //set deltaAngularVelocityB 2
				"vtrn.32	d28, d29			\n\t" //set deltaAngularVelocityA 1
				"vtrn.32	d26, d27			\n\t" //set deltaAngularVelocityB 1
				"vrev64.32	d30, d30			\n\t" //set deltaAngularVelocityA 2
				"vrev64.32	d24, d24			\n\t" //set deltaAngularVelocityB 2
				"vmul.f32 	q14, q7, q14		\n\t" //operation for cross product 1A
				"vmul.f32 	q13, q9, q13		\n\t" //operation for cross product 1B
				"vmls.f32 	q14, q8, q15		\n\t" //operation for cross product 2A
				"vmls.f32 	q13, q10, q12		\n\t" //operation for cross product 2B
				"vadd.f32 	q14, q14, q0		\n\t" //operation for adding cross to linearVelocityA
				"vadd.f32 	q13, q13, q2		\n\t" //operation for adding cross to linearVelocityB
													  //LOAD => dVA --------------------------------> q14
													  //LOAD => dVB --------------------------------> q13
													  //FREE q11, q12, q15, d11
	//deltaImpulse -= constraint.m_jacDiagInv * dot(normal,dVA-dVB);
				"vsub.f32 	q11, q14, q13		\n\t" //TEMP q11 => dVA-dVB for dot product
	/*find*/	"vmul.f32 	q11, q11, q4		\n\t" //operation for dot product
	/*fastest*/	"vpadd.f32 	d22, d22, d22		\n\t" //operation for dot product
	/*dot*/		"vpadd.f32 	d23, d23, d23		\n\t" //operation for dot product
				"vadd.f32 	d22, d22, d23		\n\t" //operation for dot product
				"vmul.f32 	d22, d22, d12[0]	\n\t" //m_jacDiagInv times dot product result
				"vsub.f32 	d10, d10, d22		\n\t" //subtract result from deltaImpule
	//PfxFloat oldImpulse = constraint.m_accumImpulse;
				"vdup.32	d11, d13[1]			\n\t" //LOAD => oldImpulse -------------------------> q5 (d11 only)
	//constraint.m_accumImpulse = SCE_PFX_CLAMP(oldImpulse + deltaImpulse,constraint.m_lowerLimit,constraint.m_upperLimit);
				"vdup.32	d23, d13[0]			\n\t" //TEMP q11 => m_upperLimit (d23 only)
				"vdup.32	d24, d12[1]			\n\t" //TEMP q12 => m_lowerLimit (d34 only)
				"vadd.f32   d25, d10, d11		\n\t" //TEMP q12 => deltaImpulse + oldImplues (d25 only)
				"vmin.f32   d25, d25, d23		\n\t" //operation MIN(v,b)
				"vmax.f32	d22, d25, d24		\n\t" //operation MAX(a,MIN(v,b)
				"vst1.32    {d22[0]}, [%0]		\n\t" //store m_accumImpulse (incremented so that it can be reloaded for cross product later)
	//deltaImpulse = constraint.m_accumImpulse - oldImpulse;
				"vsub.f32 	d10, d22, d11		\n\t" //operation to calculate new deltaImpule
													  //FREE q6, q11, q12, q13, q14, q15, d11
	//deltaLinearVelocityB -= deltaImpulse * massInvB * normal;
	//deltaLinearVelocityA += deltaImpulse * massInvA * normal;
				"vld1.32 	{d11[0]}, [%3]		\n\t" //LOAD => massInvA ---------------------------> q5 (d11[0] only)
				"vld1.32 	{d11[1]}, [%8]		\n\t" //LOAD => massInvB ---------------------------> q5 (d11[1] only)
				"vmul.f32	q11, q4, d11[0]		\n\t" //TEMP q11 => operation normal times massInvA A
				"vmul.f32	q12, q4, d11[1]		\n\t" //TEMP q12 => operation normal times massInvB B
				"vmul.f32	q11, q11, d10[0]	\n\t" //TEMP q11 => operation result times DeltaImpulse A
				"vmul.f32	q12, q12, d10[0]	\n\t" //TEMP q12 => operation result times DeltaImpulse B
				"vadd.f32	q0, q0, q11			\n\t" //operation create new deltaLinearVelocityA A
				"vsub.f32	q2, q2, q12			\n\t" //operation create new deltaLinearVelocityB B
				"vst1.32	{q0}, [%1]			\n\t" //store the new deltaLinearVelocityA A
				"vst1.32	{q2}, [%6]			\n\t" //store the new deltaLinearVelocityB B
													  //FREE q0, q2, q6, q11, q12, q13, q14, q15, d11
	//deltaAngularVelocityA += deltaImpulse * inertiaInvA * cross(rA,normal);
	//deltaAngularVelocityB -= deltaImpulse * inertiaInvB * cross(rB,normal);
				"vdup.32	q14, d1[1]			\n\t" //set normal cross load
				"vdup.32	q15, d1[1]			\n\t" //
				"vadd.f32 	q14, q14, q4		\n\t" //
				"vadd.f32 	q15, q15, q4		\n\t" //
				"vrev64.32	d28, d28			\n\t" //
				"vtrn.32	d30, d31			\n\t" //
				"vtrn.32	d28, d29			\n\t" //
				"vrev64.32	d30, d30			\n\t" //
				"vmul.f32 	q0, q8, q15			\n\t" //operation for cross product A
				"vmul.f32 	q2, q10, q15		\n\t" //operation for cross product B
				"vmls.f32 	q0, q14, q7			\n\t" //operation for cross product A
				"vmls.f32 	q2, q14, q9			\n\t" //operation for cross product B
										  	  	  	  //LOAD => cross product result A ------------> q0
													  //LOAD => cross product result B ------------> q2
													  //FREE q6, q7, q8, q9, q10, q11, q12, q13, q14, q15, d11

				"vld1.32    {q13-q14}, [%4]! 	\n\t" //LOAD => inertiaInvA col0, col1 A ----------> q13, q14
				"vld1.32    {q9-q10}, [%9]!	 	\n\t" //LOAD => inertiaInvB col0, col1 B -----------> q9, q10
				"vld1.32    {q15}, [%4]   	    \n\t" //LOAD => inertiaInvA col2 A ----------------> q5
				"vld1.32    {q11}, [%9]   	    \n\t" //LOAD => inertiaInvB col2 B -----------------> q11
				"vmul.f32   q13, q13, d0[0]    	\n\t" //operation inertiaInvA col0 = (col0) * (crossA elem0) A
				"vmul.f32   q9, q9, d4[0]      	\n\t" //operation inertiaInvB col0 = (col0) * (crossB elem0) B
				"vmla.f32   q13, q14, d0[1]    	\n\t" //operation inertiaInvA col1 = (col1) * (crossA elem1) A
				"vmla.f32   q9, q10, d4[1]     	\n\t" //operation inertiaInvB col1 = (col1) * (crossB elem1) B
				"vmla.f32   q13, q15, d1[0]    	\n\t" //operation inertiaInvA col2 = (col2) * (crossA elem2) A
				"vmla.f32   q9, q11, d5[0]     	\n\t" //operation inertiaInvB col2 = (col2) * (crossB elem2) B
				"vmul.f32   q13, q13, d10[0]    \n\t" //operation inertiaInvA times deltaImpulse A
				"vmul.f32   q9, q9, d10[0]      \n\t" //operation inertiaInvB times deltaImpulse B
				"vadd.f32   q1, q1, q13      	\n\t" //operation accumulate the deltaAngularVelocityA A
				"vsub.f32   q3, q3, q9      	\n\t" //operation accumulate the deltaAngularVelocityB B
				"vst1.32    {q1}, [%2]      	\n\t" //store deltaAngularVelocityA A
				"vst1.32    {q3}, [%7]      	\n\t" //store deltaAngularVelocityB B
				:	// NO outputs! It is important to *not* put anything here. (Putting something here forces use of r0, which wreak havok
				: "r" (constraint), "r" (deltaLinearVelocityA), "r" (deltaAngularVelocityA), "r" (massInvA), "r" (inertiaInvA), "r" (rA), "r" (deltaLinearVelocityB), "r" (deltaAngularVelocityB), "r" (massInvB), "r" (inertiaInvB), "r" (rB) //inputs
				: "memory", "q0", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8", "q9", "q10", "q11", "q12", "q13", "q14", "q15" // clobbers
			);
}

//----------------------------------------------------------------------------
//  TestNeonSolveLinearConstraintRow
//
/// Run timing study of the Linear Constraint Row Solver from above, writing the
/// results to the Android verbose log.
//----------------------------------------------------------------------------
void TestNeonSolveLinearConstraintRow()
{
	//Scalar
	float SET_ALIGNMENT(64) constraintDataScalar[] = 				{0.0f,1.0f,2.0f,3.0f,1.0f,2.0f,3.0f,4.0f};
	float SET_ALIGNMENT(64) deltaLinearVelocityADataScalar[] = 		{1.0f,2.0f,3.0f,0.0f};
	float SET_ALIGNMENT(64) deltaAngularVelocityADataScalar[] = 	{2.0f,3.0f,4.0f,0.0f};
	float SET_ALIGNMENT(64) inertiaInvADataScalar[] = 				{4.0f,5.0f,6.0f,0.0f,5.0f,6.0f,7.0f,0.0f,6.0f,7.0f,8.0f,0.0f};
	float SET_ALIGNMENT(64) rADataScalar[] = 						{5.0f,6.0f,7.0f,0.0f};
	float SET_ALIGNMENT(64) deltaLinearVelocityBDataScalar[] = 		{6.0f,7.0f,8.0f,0.0f};
	float SET_ALIGNMENT(64) deltaAngularVelocityBDataScalar[] = 	{9.0f,8.0f,7.0f,0.0f};
	float SET_ALIGNMENT(64) inertiaInvBDataScalar[] = 				{7.0f,6.0f,5.0f,0.0f,9.0f,8.0f,7.0f,0.0f,6.0f,7.0f,8.0f,0.0f};
	float SET_ALIGNMENT(64) rBDataScalar[] = 						{6.0f,5.0f,4.0f,0.0f};

	float *constraintScalar = &constraintDataScalar[0];
	float *deltaLinearVelocityAScalar = &deltaLinearVelocityADataScalar[0];
	float *deltaAngularVelocityAScalar = &deltaAngularVelocityADataScalar[0];
	float massInvAScalar = 3.0f;
	float *inertiaInvAScalar = &inertiaInvADataScalar[0];
	float *rAScalar = &rADataScalar[0];
	float *deltaLinearVelocityBScalar = &deltaLinearVelocityBDataScalar[0];
	float *deltaAngularVelocityBScalar = &deltaAngularVelocityBDataScalar[0];
	float massInvBScalar = 8.0f;
	float *inertiaInvBScalar = &inertiaInvBDataScalar[0];
	float *rBScalar = &rBDataScalar[0];

	//Neon
	float SET_ALIGNMENT(64) constraintDataNeon[] = 				{0.0f,1.0f,2.0f,3.0f,1.0f,2.0f,3.0f,4.0f};
	float SET_ALIGNMENT(64) deltaLinearVelocityADataNeon[] = 	{1.0f,2.0f,3.0f,0.0f};
	float SET_ALIGNMENT(64) deltaAngularVelocityADataNeon[] = 	{2.0f,3.0f,4.0f,0.0f};
	float SET_ALIGNMENT(64) massInvADataNeon[] = 				{3.0f};
	float SET_ALIGNMENT(64) inertiaInvADataNeon[] = 			{4.0f,5.0f,6.0f,0.0f,5.0f,6.0f,7.0f,0.0f,6.0f,7.0f,8.0f,0.0f};
	float SET_ALIGNMENT(64) rADataNeon[] = 						{5.0f,6.0f,7.0f,0.0f};
	float SET_ALIGNMENT(64) deltaLinearVelocityBDataNeon[] = 	{6.0f,7.0f,8.0f,0.0f};
	float SET_ALIGNMENT(64) deltaAngularVelocityBDataNeon[] = 	{9.0f,8.0f,7.0f,0.0f};
	float SET_ALIGNMENT(64) massInvBDataNeon[] = 				{8.0f};
	float SET_ALIGNMENT(64) inertiaInvBDataNeon[] = 			{7.0f,6.0f,5.0f,0.0f,9.0f,8.0f,7.0f,0.0f,6.0f,7.0f,8.0f,0.0f};
	float SET_ALIGNMENT(64) rBDataNeon[] = 						{6.0f,5.0f,4.0f,0.0f};

	float *constraintNeon = &constraintDataNeon[0];
	float *deltaLinearVelocityANeon = &deltaLinearVelocityADataNeon[0];
	float *deltaAngularVelocityANeon = &deltaAngularVelocityADataNeon[0];
	float *massInvANeon = &massInvADataNeon[0];
	float *inertiaInvANeon = &inertiaInvADataNeon[0];
	float *rANeon = &rADataNeon[0];
	float *deltaLinearVelocityBNeon = &deltaLinearVelocityBDataNeon[0];
	float *deltaAngularVelocityBNeon = &deltaAngularVelocityBDataNeon[0];
	float *massInvBNeon = &massInvBDataNeon[0];
	float *inertiaInvBNeon = &inertiaInvBDataNeon[0];
	float *rBNeon = &rBDataNeon[0];

	char szMsg[256];

	sprintf(szMsg, "");
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg,"---------------------------------------");
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg,"TestNeonSolveLinearConstraintRow Start");
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);


	sce::PhysicsEffects::PfxPerfCounter pc;
	double dTimeSpan, dRefTimeSpan;; 
	unsigned int uiNumTries = 10000000;
	unsigned int i;

// profile pfxSolveLinearConstraintRowScalar with direct memory return, c++ version
	pc.countBegin("");
	for(i = 0; i < uiNumTries; i++)
	{
		pfxSolveLinearConstraintRowScalar(
				 	 constraintScalar,
					 deltaLinearVelocityAScalar,
					 deltaAngularVelocityAScalar,
					 massInvAScalar,
					 inertiaInvAScalar,
					 rAScalar,
					 deltaLinearVelocityBScalar,
					 deltaAngularVelocityBScalar,
					 massInvBScalar,
					 inertiaInvBScalar,
					 rBScalar
					 );
	}
	pc.countEnd();
	dTimeSpan = pc.getCountTime(0);
	pc.resetCount();
	dRefTimeSpan = dTimeSpan;
	sprintf(szMsg, "Time to do %i calls for pfxSolveLinearConstraintRowScalar: %f secs, speedup: %5.2f",
					uiNumTries, dTimeSpan, dRefTimeSpan/dTimeSpan);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg, "constraint.m_accumImpulse = <%f>", constraintScalar[7]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg, "deltaLinearVelocityA = <%f,%f,%f,%f>",
					deltaLinearVelocityAScalar[0], deltaLinearVelocityAScalar[1], deltaLinearVelocityAScalar[2], deltaLinearVelocityAScalar[3]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg, "deltaAngularVelocityA = <%f,%f,%f,%f>",
					deltaAngularVelocityAScalar[0], deltaAngularVelocityAScalar[1], deltaAngularVelocityAScalar[2], deltaAngularVelocityAScalar[3]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg, "deltaLinearVelocityB = <%f,%f,%f,%f>",
					deltaLinearVelocityBScalar[0], deltaLinearVelocityBScalar[1], deltaLinearVelocityBScalar[2], deltaLinearVelocityBScalar[3]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg, "deltaAngularVelocityB = <%f,%f,%f,%f>",
					deltaAngularVelocityBScalar[0], deltaAngularVelocityBScalar[1], deltaAngularVelocityBScalar[2], deltaAngularVelocityBScalar[3]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);


// profile NEON pfxSolveLinearConstraintRowNeonInline with direct memory return, inline assembly version
	pc.countBegin("");
	for(i = 0; i < uiNumTries; i++)
	{
		pfxSolveLinearConstraintRowNeonInline(constraintNeon,
											  deltaLinearVelocityANeon,
											  deltaAngularVelocityANeon,
											  massInvANeon,
											  inertiaInvANeon,
											  rANeon,
											  deltaLinearVelocityBNeon,
											  deltaAngularVelocityBNeon,
											  massInvBNeon,
											  inertiaInvBNeon,
											  rBNeon);
	}
	pc.countEnd();
	dTimeSpan = pc.getCountTime(0);
	pc.resetCount();
	sprintf(szMsg, "Time to do %i calls for pfxSolveLinearConstraintRowNeon: %f secs, speedup: %5.2f",
					uiNumTries, dTimeSpan, dRefTimeSpan/dTimeSpan);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg, "constraint.m_accumImpulse = <%f>", constraintNeon[7]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg, "deltaLinearVelocityA = <%f,%f,%f,%f>",
					deltaLinearVelocityANeon[0], deltaLinearVelocityANeon[1], deltaLinearVelocityANeon[2], deltaLinearVelocityANeon[3]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg, "deltaAngularVelocityA = <%f,%f,%f,%f>",
					deltaAngularVelocityANeon[0], deltaAngularVelocityANeon[1], deltaAngularVelocityANeon[2], deltaAngularVelocityANeon[3]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg, "deltaLinearVelocityB = <%f,%f,%f,%f>",
					deltaLinearVelocityBNeon[0], deltaLinearVelocityBNeon[1], deltaLinearVelocityBNeon[2], deltaLinearVelocityBNeon[3]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
	sprintf(szMsg, "deltaAngularVelocityB = <%f,%f,%f,%f>",
					deltaAngularVelocityBNeon[0], deltaAngularVelocityBNeon[1], deltaAngularVelocityBNeon[2], deltaAngularVelocityBNeon[3]);
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);


	sprintf(szMsg,"TestNeonSolveLinearConstraintRow End");
	__android_log_write(ANDROID_LOG_VERBOSE,"PHYSICS TIMING STUDY", szMsg);
}
