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
#ifdef __ANDROID__

#include "../../../include/physics_effects.h"
#include <arm_neon.h>

namespace sce {
namespace PhysicsEffects {

//----------------------------------------------------------------------------
//  pfxSolveLinearConstraintRowNEON
//
/// NEON inline assembly implementation of the pfxSolveLinearConstraintRow
/// function.
///
/// @param  constraint                Constraint row to be solved
/// @param  deltaLinearVelocityA      Change in linear velocity for object A
/// @param  deltaAngularVelocityA     Change in angular velocity for object A
/// @param  massInvA                  Mass inverse for object A
/// @param  inertiaInvA               Inertia tensor inverse for object A
/// @param  rA                        Position of contact point for object A
/// @param  deltaLinearVelocityB      Change in linear velocity for object B
/// @param  deltaAngularVelocityB     Change in angular velocity for object B
/// @param  massInvB                  Mass inverse for object B
/// @param  inertiaInvB               Inertia tensor inverse for object B
/// @param  rB                        Position of contact point for object B
//----------------------------------------------------------------------------
//GSR version
void pfxSolveLinearConstraintRowNEON(PfxConstraintRow &constraint,
	PfxVector3 &deltaLinearVelocityA,PfxVector3 &deltaAngularVelocityA,
	PfxFloat &massInvA,const PfxMatrix3 &inertiaInvA,const PfxVector3 &rA,
	PfxVector3 &deltaLinearVelocityB,PfxVector3 &deltaAngularVelocityB,
	PfxFloat &massInvB,const PfxMatrix3 &inertiaInvB,const PfxVector3 &rB)
{
	asm volatile
	(
		//Loads beforehand so the normal vector will be able to have zero in the 4th element
		"vld1.32 	{q0}, [%1]			\n\t" //LOAD => deltaLinearVelocityA ----------------> q0
		"vld1.32	{q1}, [%2]			\n\t" //LOAD => deltaAngularVelocityA ---------------> q1
		"vld1.32	{q2}, [%6]			\n\t" //LOAD => deltaLinearVelocityB ----------------> q2
		"vld1.32	{q3}, [%7]			\n\t" //LOAD => deltaAngularVelocityB ---------------> q3
		//const PfxVector3 normal(pfxReadVector3(constraint.m_normal));
		"vld1.32	{q4}, [%0]!			\n\t" //LOAD => constraint normal--------------------> q4
		"vdup.32	d10, d1[1]			\n\t" //set 4th element on normal vector to zero. It is zero since q0 holds a vec3...d1[1] is 4th element and on C++ side we set this to zero
		"vtrn.32	d9, d10				\n\t" //LOAD => deltaImpulse ------------------------> q5 (d10[0] only)
		//PfxFloat deltaImpulse = constraint.m_rhs;
		"vld1.32	{d12}, [%0]!		\n\t" //LOAD => constraint variables ----------------> q6
		"vld1.32	{d13[0]}, [%0]!		\n\t" //constraint  variables
		"vld1.32	{d13[1]}, [%0]		\n\t" //constraint load of remaining variables loaded this way keep pointer to m_accumulate in order to store back in later
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
		"vmov.f32 	q14, q1				\n\t"
		"vmov.f32 	q13, q3				\n\t"
		"vmov.f32 	q15, q1				\n\t"
		"vmov.f32 	q12, q3				\n\t"
		"vrev64.32	d28, d28			\n\t" //set deltaAngularVelocityA 1
		"vtrn.32	d28, d29			\n\t" //set deltaAngularVelocityA 1
		"vrev64.32	d26, d26			\n\t" //set deltaAngularVelocityB 1
		"vmul.f32 	q14, q7, q14		\n\t" //operation for cross product 1A
		"vtrn.32	d26, d27			\n\t" //set deltaAngularVelocityB 1
		"vtrn.32	d30, d31			\n\t" //set deltaAngularVelocityA 2
		"vmul.f32 	q13, q9, q13		\n\t" //operation for cross product 1B
		"vtrn.32	d24, d25			\n\t" //set deltaAngularVelocityB 2
		"vrev64.32	d30, d30			\n\t" //set deltaAngularVelocityA 2
		"vrev64.32	d24, d24			\n\t" //set deltaAngularVelocityB 2
		"vmls.f32 	q14, q8, q15		\n\t" //operation for cross product 2A
		"vmls.f32 	q13, q10, q12		\n\t" //operation for cross product 2B
		"vadd.f32 	q14, q14, q0		\n\t" //operation for adding cross to linearVelocityA
		"vadd.f32 	q13, q13, q2		\n\t" //operation for adding cross to linearVelocityB
												//LOAD => dVA --------------------------------> q14
												//LOAD => dVB --------------------------------> q13
												//FREE q11, q12, q15, d11
		//deltaImpulse -= constraint.m_jacDiagInv * dot(normal,dVA-dVB);
		"vsub.f32 	q11, q14, q13		\n\t" //TEMP q11 => dVA-dVB for dot product
		"vmul.f32 	q11, q11, q4		\n\t" //operation for dot product
		"vpadd.f32 	d22, d22, d22		\n\t" //operation for dot product
		"vpadd.f32 	d23, d23, d23		\n\t" //operation for dot product
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
		"vst1.32	{q0}, [%1]			\n\t" //store the new deltaLinearVelocityA A
		"vsub.f32	q2, q2, q12			\n\t" //operation create new deltaLinearVelocityB B
		"vst1.32	{q2}, [%6]			\n\t" //store the new deltaLinearVelocityB B
												//FREE q0, q2, q6, q11, q12, q13, q14, q15, d11
		//deltaAngularVelocityA += deltaImpulse * inertiaInvA * cross(rA,normal);
		//deltaAngularVelocityB -= deltaImpulse * inertiaInvB * cross(rB,normal);
		"vmov.f32 	q14, q4				\n\t" //
		"vmov.f32 	q15, q4				\n\t" //
		"vtrn.32	d30, d31			\n\t" //
		"vrev64.32	d30, d30			\n\t" //
		"vmul.f32 	q0, q8, q15			\n\t" //operation for cross product A
		"vrev64.32	d28, d28			\n\t" //
		"vmul.f32 	q2, q10, q15		\n\t" //operation for cross product B
		"vtrn.32	d28, d29			\n\t" //
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
		"vst1.32    {q1}, [%2]      	\n\t" //store deltaAngularVelocityA A
		"vsub.f32   q3, q3, q9      	\n\t" //operation accumulate the deltaAngularVelocityB B
		"vst1.32    {q3}, [%7]      	\n\t" //store deltaAngularVelocityB B
		: // NO direct outputs! It is important to *not* put anything here. Real output is written directly to memory pointed to by inputs
		: "r" (&constraint), "r" (&deltaLinearVelocityA), "r" (&deltaAngularVelocityA), "r" (&massInvA), "r" (&inertiaInvA), "r" (&rA), "r" (&deltaLinearVelocityB), "r" (&deltaAngularVelocityB), "r" (&massInvB), "r" (&inertiaInvB), "r" (&rB) //inputs
		: "memory", "q0", "q1", "q2", "q3", "q4", "q5", "q6", "q7", "q8", "q9", "q10", "q11", "q12", "q13", "q14", "q15" // clobbers
	);
}

} //namespace PhysicsEffects
} //namespace sce
#endif //__ANDROID__
