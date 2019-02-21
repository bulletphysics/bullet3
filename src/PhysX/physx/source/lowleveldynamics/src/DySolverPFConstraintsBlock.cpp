//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#include "foundation/PxPreprocessor.h"
#include "PsVecMath.h"
#include "PsFPU.h"
#include "CmPhysXCommon.h"
#include "DySolverBody.h"
#include "DySolverContactPF4.h"
#include "DySolverConstraint1D.h"
#include "DySolverConstraintDesc.h"
#include "DyThresholdTable.h"
#include "DySolverContext.h"
#include "PsUtilities.h"
#include "DyConstraint.h"
#include "PsAtomic.h"
#include "DySolverContact.h"

namespace physx
{

namespace Dy
{

static void solveContactCoulomb4_Block(const PxSolverConstraintDesc* PX_RESTRICT desc, SolverContext& /*cache*/)
{
	PxSolverBody& b00 = *desc[0].bodyA;
	PxSolverBody& b01 = *desc[0].bodyB;
	PxSolverBody& b10 = *desc[1].bodyA;
	PxSolverBody& b11 = *desc[1].bodyB;
	PxSolverBody& b20 = *desc[2].bodyA;
	PxSolverBody& b21 = *desc[2].bodyB;
	PxSolverBody& b30 = *desc[3].bodyA;
	PxSolverBody& b31 = *desc[3].bodyB;

	//We'll need this.
	const Vec4V vZero	= V4Zero();
		
	Vec4V linVel00 = V4LoadA(&b00.linearVelocity.x);
	Vec4V linVel01 = V4LoadA(&b01.linearVelocity.x);
	Vec4V angState00 = V4LoadA(&b00.angularState.x);
	Vec4V angState01 = V4LoadA(&b01.angularState.x);

	Vec4V linVel10 = V4LoadA(&b10.linearVelocity.x);
	Vec4V linVel11 = V4LoadA(&b11.linearVelocity.x);
	Vec4V angState10 = V4LoadA(&b10.angularState.x);
	Vec4V angState11 = V4LoadA(&b11.angularState.x);

	Vec4V linVel20 = V4LoadA(&b20.linearVelocity.x);
	Vec4V linVel21 = V4LoadA(&b21.linearVelocity.x);
	Vec4V angState20 = V4LoadA(&b20.angularState.x);
	Vec4V angState21 = V4LoadA(&b21.angularState.x);

	Vec4V linVel30 = V4LoadA(&b30.linearVelocity.x);
	Vec4V linVel31 = V4LoadA(&b31.linearVelocity.x);
	Vec4V angState30 = V4LoadA(&b30.angularState.x);
	Vec4V angState31 = V4LoadA(&b31.angularState.x);


	Vec4V linVel0T0, linVel0T1, linVel0T2, linVel0T3;
	Vec4V linVel1T0, linVel1T1, linVel1T2, linVel1T3;
	Vec4V angState0T0, angState0T1, angState0T2, angState0T3;
	Vec4V angState1T0, angState1T1, angState1T2, angState1T3;


	PX_TRANSPOSE_44(linVel00, linVel10, linVel20, linVel30, linVel0T0, linVel0T1, linVel0T2, linVel0T3);
	PX_TRANSPOSE_44(linVel01, linVel11, linVel21, linVel31, linVel1T0, linVel1T1, linVel1T2, linVel1T3);
	PX_TRANSPOSE_44(angState00, angState10, angState20, angState30, angState0T0, angState0T1, angState0T2, angState0T3);
	PX_TRANSPOSE_44(angState01, angState11, angState21, angState31, angState1T0, angState1T1, angState1T2, angState1T3);


	

	//hopefully pointer aliasing doesn't bite.
	PxU8* PX_RESTRICT currPtr = desc[0].constraint;

	SolverContactCoulombHeader4* PX_RESTRICT firstHeader = reinterpret_cast<SolverContactCoulombHeader4*>(currPtr);

	const PxU8* PX_RESTRICT last = desc[0].constraint + firstHeader->frictionOffset;

	//const PxU8* PX_RESTRICT endPtr = desc[0].constraint + getConstraintLength(desc[0]);


	//TODO - can I avoid this many tests???
	while(currPtr < last)
	{

		SolverContactCoulombHeader4* PX_RESTRICT hdr = reinterpret_cast<SolverContactCoulombHeader4*>(currPtr);

		Vec4V* appliedForceBuffer = reinterpret_cast<Vec4V*>(currPtr + hdr->frictionOffset + sizeof(SolverFrictionHeader4));

		//PX_ASSERT((PxU8*)appliedForceBuffer < endPtr);

		currPtr = reinterpret_cast<PxU8*>(hdr + 1);

		const PxU32 numNormalConstr = hdr->numNormalConstr;

		SolverContact4Dynamic* PX_RESTRICT contacts = reinterpret_cast<SolverContact4Dynamic*>(currPtr);
		//const Vec4V dominance1 = V4Neg(__dominance1);

		currPtr = reinterpret_cast<PxU8*>(contacts + numNormalConstr);

		const Vec4V invMass0D0 = hdr->invMassADom;
		const Vec4V invMass1D1 = hdr->invMassBDom;
		const Vec4V angD0 = hdr->angD0;
		const Vec4V angD1 = hdr->angD1;

		const Vec4V normalT0 = hdr->normalX;
		const Vec4V normalT1 = hdr->normalY;
		const Vec4V normalT2 = hdr->normalZ;

		const Vec4V __normalVel1 = V4Mul(linVel0T0, normalT0);
		const Vec4V __normalVel3 = V4Mul(linVel1T0, normalT0);
		const Vec4V _normalVel1 = V4MulAdd(linVel0T1, normalT1, __normalVel1);
		const Vec4V _normalVel3 = V4MulAdd(linVel1T1, normalT1, __normalVel3);

		Vec4V normalVel1 = V4MulAdd(linVel0T2, normalT2, _normalVel1);
		Vec4V normalVel3 = V4MulAdd(linVel1T2, normalT2, _normalVel3);

		Vec4V accumDeltaF = vZero;

		for(PxU32 i=0;i<numNormalConstr;i++)
		{
			SolverContact4Dynamic& c = contacts[i];
			Ps::prefetchLine((&contacts[i+1]));
			Ps::prefetchLine((&contacts[i+1]), 128);
			Ps::prefetchLine((&contacts[i+1]), 256);
			Ps::prefetchLine((&contacts[i+1]), 384);

			const Vec4V appliedForce = c.appliedForce;
			const Vec4V velMultiplier = c.velMultiplier;
			
			const Vec4V targetVel = c.targetVelocity;
			const Vec4V scaledBias = c.scaledBias;
			const Vec4V maxImpulse = c.maxImpulse;

			const Vec4V raXnT0 = c.raXnX;
			const Vec4V raXnT1 = c.raXnY;
			const Vec4V raXnT2 = c.raXnZ;
			const Vec4V rbXnT0 = c.rbXnX;
			const Vec4V rbXnT1 = c.rbXnY;
			const Vec4V rbXnT2 = c.rbXnZ;

			
			const Vec4V __normalVel2 = V4Mul(raXnT0, angState0T0);
			const Vec4V __normalVel4 = V4Mul(rbXnT0, angState1T0);

			
			const Vec4V _normalVel2 = V4MulAdd(raXnT1, angState0T1, __normalVel2);
			const Vec4V _normalVel4 = V4MulAdd(rbXnT1, angState1T1, __normalVel4);

			
			const Vec4V normalVel2 = V4MulAdd(raXnT2, angState0T2, _normalVel2);
			const Vec4V normalVel4 = V4MulAdd(rbXnT2, angState1T2, _normalVel4);

			const Vec4V biasedErr = V4MulAdd(targetVel, velMultiplier, V4Neg(scaledBias));

			//Linear component - normal * invMass_dom

			const Vec4V _normalVel(V4Add(normalVel1, normalVel2));
			const Vec4V __normalVel(V4Add(normalVel3, normalVel4));
		
			const Vec4V normalVel = V4Sub(_normalVel, __normalVel );

			const Vec4V _deltaF = V4NegMulSub(normalVel, velMultiplier, biasedErr);
			const Vec4V nAppliedForce = V4Neg(appliedForce);
			const Vec4V _deltaF2 = V4Max(_deltaF, nAppliedForce);
			const Vec4V _newAppliedForce(V4Add(appliedForce, _deltaF2));
			const Vec4V newAppliedForce = V4Min(_newAppliedForce, maxImpulse);
			const Vec4V deltaF = V4Sub(newAppliedForce, appliedForce);

			normalVel1 = V4MulAdd(invMass0D0, deltaF, normalVel1);
			normalVel3 = V4NegMulSub(invMass1D1, deltaF, normalVel3);

			accumDeltaF = V4Add(deltaF, accumDeltaF);

			const Vec4V deltaFAng0 = V4Mul(angD0, deltaF);
			const Vec4V deltaFAng1 = V4Mul(angD1, deltaF);

			angState0T0 = V4MulAdd(raXnT0, deltaFAng0, angState0T0);
			angState1T0 = V4NegMulSub(rbXnT0, deltaFAng1, angState1T0);

			angState0T1 = V4MulAdd(raXnT1, deltaFAng0, angState0T1);
			angState1T1 = V4NegMulSub(rbXnT1, deltaFAng1, angState1T1);

			angState0T2 = V4MulAdd(raXnT2, deltaFAng0, angState0T2);
			angState1T2 = V4NegMulSub(rbXnT2, deltaFAng1, angState1T2);

			c.appliedForce = newAppliedForce;
			appliedForceBuffer[i] = newAppliedForce;
		}

		const Vec4V accumDeltaF0 = V4Mul(accumDeltaF, invMass0D0);
		const Vec4V accumDeltaF1 = V4Mul(accumDeltaF, invMass1D1);

		linVel0T0 = V4MulAdd(normalT0, accumDeltaF0, linVel0T0);
		linVel1T0 = V4NegMulSub(normalT0, accumDeltaF1, linVel1T0);
		linVel0T1 = V4MulAdd(normalT1, accumDeltaF0, linVel0T1);
		linVel1T1 = V4NegMulSub(normalT1, accumDeltaF1, linVel1T1);
		linVel0T2 = V4MulAdd(normalT2, accumDeltaF0, linVel0T2);
		linVel1T2 = V4NegMulSub(normalT2, accumDeltaF1, linVel1T2);
	}

	PX_ASSERT(currPtr == last);
	

	//KS - we need to use PX_TRANSPOSE_44 here instead of the 34_43 variants because the W components are being used to 
	//store the bodies' progress counters.

	PX_TRANSPOSE_44(linVel0T0, linVel0T1, linVel0T2, linVel0T3, linVel00, linVel10, linVel20, linVel30);
	PX_TRANSPOSE_44(linVel1T0, linVel1T1, linVel1T2, linVel1T3, linVel01, linVel11, linVel21, linVel31);
	PX_TRANSPOSE_44(angState0T0, angState0T1, angState0T2, angState0T3, angState00, angState10, angState20, angState30);
	PX_TRANSPOSE_44(angState1T0, angState1T1, angState1T2, angState1T3, angState01, angState11, angState21, angState31);


	// Write back
	V4StoreA(linVel00, &b00.linearVelocity.x);
	V4StoreA(linVel10, &b10.linearVelocity.x);
	V4StoreA(linVel20, &b20.linearVelocity.x);
	V4StoreA(linVel30, &b30.linearVelocity.x);

	V4StoreA(linVel01, &b01.linearVelocity.x);
	V4StoreA(linVel11, &b11.linearVelocity.x);
	V4StoreA(linVel21, &b21.linearVelocity.x);
	V4StoreA(linVel31, &b31.linearVelocity.x);

	V4StoreA(angState00, &b00.angularState.x);
	V4StoreA(angState10, &b10.angularState.x);
	V4StoreA(angState20, &b20.angularState.x);
	V4StoreA(angState30, &b30.angularState.x);

	V4StoreA(angState01, &b01.angularState.x);
	V4StoreA(angState11, &b11.angularState.x);
	V4StoreA(angState21, &b21.angularState.x);
	V4StoreA(angState31, &b31.angularState.x);
}


static void solveContactCoulomb4_StaticBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, SolverContext& /*cache*/)
{
	PxSolverBody& b00 = *desc[0].bodyA;
	PxSolverBody& b10 = *desc[1].bodyA;
	PxSolverBody& b20 = *desc[2].bodyA;
	PxSolverBody& b30 = *desc[3].bodyA;

	//We'll need this.
	const Vec4V vZero	= V4Zero();
		
	Vec4V linVel00 = V4LoadA(&b00.linearVelocity.x);
	Vec4V angState00 = V4LoadA(&b00.angularState.x);

	Vec4V linVel10 = V4LoadA(&b10.linearVelocity.x);
	Vec4V angState10 = V4LoadA(&b10.angularState.x);

	Vec4V linVel20 = V4LoadA(&b20.linearVelocity.x);
	Vec4V angState20 = V4LoadA(&b20.angularState.x);

	Vec4V linVel30 = V4LoadA(&b30.linearVelocity.x);
	Vec4V angState30 = V4LoadA(&b30.angularState.x);


	Vec4V linVel0T0, linVel0T1, linVel0T2, linVel0T3;
	Vec4V angState0T0, angState0T1, angState0T2, angState0T3;


	PX_TRANSPOSE_44(linVel00, linVel10, linVel20, linVel30, linVel0T0, linVel0T1, linVel0T2, linVel0T3);
	PX_TRANSPOSE_44(angState00, angState10, angState20, angState30, angState0T0, angState0T1, angState0T2, angState0T3);
	

	//hopefully pointer aliasing doesn't bite.
	PxU8* PX_RESTRICT currPtr = desc[0].constraint;

	SolverContactCoulombHeader4* PX_RESTRICT firstHeader = reinterpret_cast<SolverContactCoulombHeader4*>(currPtr);

	const PxU8* PX_RESTRICT last = desc[0].constraint + firstHeader->frictionOffset;


	//TODO - can I avoid this many tests???
	while(currPtr < last)
	{

		SolverContactCoulombHeader4* PX_RESTRICT hdr = reinterpret_cast<SolverContactCoulombHeader4*>(currPtr);

		Vec4V* appliedForceBuffer = reinterpret_cast<Vec4V*>(currPtr + hdr->frictionOffset + sizeof(SolverFrictionHeader4));

		currPtr = reinterpret_cast<PxU8*>(hdr + 1);

		const PxU32 numNormalConstr = hdr->numNormalConstr;

		SolverContact4Base* PX_RESTRICT contacts = reinterpret_cast<SolverContact4Base*>(currPtr);
	
		currPtr = reinterpret_cast<PxU8*>(contacts + numNormalConstr);

		const Vec4V invMass0D0 = hdr->invMassADom;
		const Vec4V angD0 = hdr->angD0;

		const Vec4V normalT0 = hdr->normalX;
		const Vec4V normalT1 = hdr->normalY;
		const Vec4V normalT2 = hdr->normalZ;

		const Vec4V __normalVel1 = V4Mul(linVel0T0, normalT0);
		const Vec4V _normalVel1 = V4MulAdd(linVel0T1, normalT1, __normalVel1);

		Vec4V normalVel1 = V4MulAdd(linVel0T2, normalT2, _normalVel1);

		Vec4V accumDeltaF = vZero;

		for(PxU32 i=0;i<numNormalConstr;i++)
		{
			SolverContact4Base& c = contacts[i];
			Ps::prefetchLine((&contacts[i+1]));
			Ps::prefetchLine((&contacts[i+1]), 128);
			Ps::prefetchLine((&contacts[i+1]), 256);

			const Vec4V appliedForce = c.appliedForce;
			const Vec4V velMultiplier = c.velMultiplier;
			
			const Vec4V targetVel = c.targetVelocity;
			const Vec4V scaledBias = c.scaledBias;
			const Vec4V maxImpulse = c.maxImpulse;

			const Vec4V raXnT0 = c.raXnX;
			const Vec4V raXnT1 = c.raXnY;
			const Vec4V raXnT2 = c.raXnZ;

			
			const Vec4V __normalVel2 = V4Mul(raXnT0, angState0T0);
			
			const Vec4V _normalVel2 = V4MulAdd(raXnT1, angState0T1, __normalVel2);
			
			const Vec4V normalVel2 = V4MulAdd(raXnT2, angState0T2, _normalVel2);

			const Vec4V biasedErr = V4MulAdd(targetVel, velMultiplier, V4Neg(scaledBias));

			//Linear component - normal * invMass_dom

			const Vec4V normalVel(V4Add(normalVel1, normalVel2));

			const Vec4V _deltaF = V4NegMulSub(normalVel, velMultiplier, biasedErr);
			const Vec4V nAppliedForce = V4Neg(appliedForce);

			const Vec4V _deltaF2 = V4Max(_deltaF, nAppliedForce);

			const Vec4V _newAppliedForce(V4Add(appliedForce, _deltaF2));
			const Vec4V newAppliedForce = V4Min(_newAppliedForce, maxImpulse);
			const Vec4V deltaF = V4Sub(newAppliedForce, appliedForce);
			const Vec4V deltaAngF = V4Mul(deltaF, angD0);

			normalVel1 = V4MulAdd(invMass0D0, deltaF, normalVel1);

			accumDeltaF = V4Add(deltaF, accumDeltaF);

			angState0T0 = V4MulAdd(raXnT0, deltaAngF, angState0T0);
			angState0T1 = V4MulAdd(raXnT1, deltaAngF, angState0T1);
			angState0T2 = V4MulAdd(raXnT2, deltaAngF, angState0T2);

			c.appliedForce = newAppliedForce;
			appliedForceBuffer[i] = newAppliedForce;
		}
		const Vec4V scaledAccumDeltaF = V4Mul(accumDeltaF, invMass0D0);
		linVel0T0 = V4MulAdd(normalT0, scaledAccumDeltaF, linVel0T0);
		linVel0T1 = V4MulAdd(normalT1, scaledAccumDeltaF, linVel0T1);
		linVel0T2 = V4MulAdd(normalT2, scaledAccumDeltaF, linVel0T2);
	}

	PX_ASSERT(currPtr == last);
	
	//KS - we need to use PX_TRANSPOSE_44 here instead of the 34_43 variants because the W components are being used to 
	//store the bodies' progress counters.

	PX_TRANSPOSE_44(linVel0T0, linVel0T1, linVel0T2, linVel0T3, linVel00, linVel10, linVel20, linVel30);
	PX_TRANSPOSE_44(angState0T0, angState0T1, angState0T2, angState0T3, angState00, angState10, angState20, angState30);

	// Write back
	// Write back
	V4StoreA(linVel00, &b00.linearVelocity.x);
	V4StoreA(linVel10, &b10.linearVelocity.x);
	V4StoreA(linVel20, &b20.linearVelocity.x);
	V4StoreA(linVel30, &b30.linearVelocity.x);

	V4StoreA(angState00, &b00.angularState.x);
	V4StoreA(angState10, &b10.angularState.x);
	V4StoreA(angState20, &b20.angularState.x);
	V4StoreA(angState30, &b30.angularState.x);
}

static void solveFriction4_Block(const PxSolverConstraintDesc* PX_RESTRICT desc, SolverContext& /*cache*/)
{
	PxSolverBody& b00 = *desc[0].bodyA;
	PxSolverBody& b01 = *desc[0].bodyB;
	PxSolverBody& b10 = *desc[1].bodyA;
	PxSolverBody& b11 = *desc[1].bodyB;
	PxSolverBody& b20 = *desc[2].bodyA;
	PxSolverBody& b21 = *desc[2].bodyB;
	PxSolverBody& b30 = *desc[3].bodyA;
	PxSolverBody& b31 = *desc[3].bodyB;


	Vec4V linVel00 = V4LoadA(&b00.linearVelocity.x);
	Vec4V linVel01 = V4LoadA(&b01.linearVelocity.x);
	Vec4V angState00 = V4LoadA(&b00.angularState.x);
	Vec4V angState01 = V4LoadA(&b01.angularState.x);

	Vec4V linVel10 = V4LoadA(&b10.linearVelocity.x);
	Vec4V linVel11 = V4LoadA(&b11.linearVelocity.x);
	Vec4V angState10 = V4LoadA(&b10.angularState.x);
	Vec4V angState11 = V4LoadA(&b11.angularState.x);

	Vec4V linVel20 = V4LoadA(&b20.linearVelocity.x);
	Vec4V linVel21 = V4LoadA(&b21.linearVelocity.x);
	Vec4V angState20 = V4LoadA(&b20.angularState.x);
	Vec4V angState21 = V4LoadA(&b21.angularState.x);

	Vec4V linVel30 = V4LoadA(&b30.linearVelocity.x);
	Vec4V linVel31 = V4LoadA(&b31.linearVelocity.x);
	Vec4V angState30 = V4LoadA(&b30.angularState.x);
	Vec4V angState31 = V4LoadA(&b31.angularState.x);


	Vec4V linVel0T0, linVel0T1, linVel0T2, linVel0T3;
	Vec4V linVel1T0, linVel1T1, linVel1T2, linVel1T3;
	Vec4V angState0T0, angState0T1, angState0T2, angState0T3;
	Vec4V angState1T0, angState1T1, angState1T2, angState1T3;


	PX_TRANSPOSE_44(linVel00, linVel10, linVel20, linVel30, linVel0T0, linVel0T1, linVel0T2, linVel0T3);
	PX_TRANSPOSE_44(linVel01, linVel11, linVel21, linVel31, linVel1T0, linVel1T1, linVel1T2, linVel1T3);
	PX_TRANSPOSE_44(angState00, angState10, angState20, angState30, angState0T0, angState0T1, angState0T2, angState0T3);
	PX_TRANSPOSE_44(angState01, angState11, angState21, angState31, angState1T0, angState1T1, angState1T2, angState1T3);

	PxU8* PX_RESTRICT currPtr = desc[0].constraint;
	PxU8* PX_RESTRICT endPtr = desc[0].constraint + getConstraintLength(desc[0]);
	

	while(currPtr < endPtr)
	{
		SolverFrictionHeader4* PX_RESTRICT hdr = reinterpret_cast<SolverFrictionHeader4*>(currPtr);

		currPtr = reinterpret_cast<PxU8*>(hdr + 1);

		Vec4V* appliedImpulses = reinterpret_cast<Vec4V*>(currPtr);

		currPtr += hdr->numNormalConstr * sizeof(Vec4V);

		Ps::prefetchLine(currPtr, 128);
		Ps::prefetchLine(currPtr,256);
		Ps::prefetchLine(currPtr,384);
		
		const PxU32	numFrictionConstr = hdr->numFrictionConstr;

		SolverFriction4Dynamic* PX_RESTRICT frictions = reinterpret_cast<SolverFriction4Dynamic*>(currPtr);

		currPtr = reinterpret_cast<PxU8*>(frictions + hdr->numFrictionConstr);

		const PxU32 maxFrictionConstr = numFrictionConstr;
	
		const Vec4V staticFric = hdr->staticFriction;

		const Vec4V invMass0D0 = hdr->invMassADom;
		const Vec4V invMass1D1 = hdr->invMassBDom;

		const Vec4V angD0 = hdr->angD0;
		const Vec4V angD1 = hdr->angD1;

		for(PxU32 i=0;i<maxFrictionConstr;i++)
		{
			SolverFriction4Dynamic& f = frictions[i];
			Ps::prefetchLine((&f)+1);
			Ps::prefetchLine((&f)+1,128);
			Ps::prefetchLine((&f)+1,256);
			Ps::prefetchLine((&f)+1,384);

			const Vec4V appliedImpulse = appliedImpulses[i>>hdr->frictionPerContact];

			const Vec4V maxFriction =  V4Mul(staticFric, appliedImpulse);

			const Vec4V nMaxFriction = V4Neg(maxFriction); 

			const Vec4V normalX = f.normalX;
			const Vec4V normalY = f.normalY;
			const Vec4V normalZ = f.normalZ;

			const Vec4V raXnX = f.raXnX;
			const Vec4V raXnY = f.raXnY;
			const Vec4V raXnZ = f.raXnZ;

			const Vec4V rbXnX = f.rbXnX;
			const Vec4V rbXnY = f.rbXnY;
			const Vec4V rbXnZ = f.rbXnZ;

			const Vec4V appliedForce(f.appliedForce);
			const Vec4V velMultiplier(f.velMultiplier);
			const Vec4V targetVel(f.targetVelocity);
	
			//4 x 4 Dot3 products encoded as 8 M44 transposes, 4 MulV and 8 MulAdd ops

			const Vec4V __normalVel1 = V4Mul(linVel0T0, normalX);
			const Vec4V __normalVel2 = V4Mul(raXnX, angState0T0);
			const Vec4V __normalVel3 = V4Mul(linVel1T0, normalX);
			const Vec4V __normalVel4 = V4Mul(rbXnX, angState1T0);

			const Vec4V _normalVel1 = V4MulAdd(linVel0T1, normalY, __normalVel1);
			const Vec4V _normalVel2 = V4MulAdd(raXnY, angState0T1, __normalVel2);
			const Vec4V _normalVel3 = V4MulAdd(linVel1T1, normalY, __normalVel3);
			const Vec4V _normalVel4 = V4MulAdd(rbXnY, angState1T1, __normalVel4);

			const Vec4V normalVel1 = V4MulAdd(linVel0T2, normalZ, _normalVel1);
			const Vec4V normalVel2 = V4MulAdd(raXnZ, angState0T2, _normalVel2);
			const Vec4V normalVel3 = V4MulAdd(linVel1T2, normalZ, _normalVel3);
			const Vec4V normalVel4 = V4MulAdd(rbXnZ, angState1T2, _normalVel4);


			const Vec4V _normalVel = V4Add(normalVel1, normalVel2);
			const Vec4V __normalVel = V4Add(normalVel3, normalVel4);

			const Vec4V normalVel = V4Sub(_normalVel, __normalVel );

			const Vec4V tmp = V4NegMulSub(targetVel, velMultiplier, appliedForce);
			Vec4V newAppliedForce = V4MulAdd(normalVel, velMultiplier, tmp);
			newAppliedForce = V4Clamp(newAppliedForce,nMaxFriction,  maxFriction);
			const Vec4V deltaF = V4Sub(newAppliedForce, appliedForce);

			const Vec4V deltaLinF0 = V4Mul(invMass0D0, deltaF);
			const Vec4V deltaLinF1 = V4Mul(invMass1D1, deltaF);

			const Vec4V deltaAngF0 = V4Mul(angD0, deltaF);
			const Vec4V deltaAngF1 = V4Mul(angD1, deltaF);


			linVel0T0 = V4MulAdd(normalX, deltaLinF0, linVel0T0);
			linVel1T0 = V4NegMulSub(normalX, deltaLinF1, linVel1T0);
			angState0T0 = V4MulAdd(raXnX, deltaAngF0, angState0T0);
			angState1T0 = V4NegMulSub(rbXnX, deltaAngF1, angState1T0);

			linVel0T1 = V4MulAdd(normalY, deltaLinF0, linVel0T1);
			linVel1T1 = V4NegMulSub(normalY, deltaLinF1, linVel1T1);
			angState0T1 = V4MulAdd(raXnY, deltaAngF0, angState0T1);
			angState1T1 = V4NegMulSub(rbXnY, deltaAngF1, angState1T1);

			linVel0T2 = V4MulAdd(normalZ, deltaLinF0, linVel0T2);
			linVel1T2 = V4NegMulSub(normalZ, deltaLinF1, linVel1T2);
			angState0T2 = V4MulAdd(raXnZ, deltaAngF0, angState0T2);
			angState1T2 = V4NegMulSub(rbXnZ, deltaAngF1, angState1T2);

			f.appliedForce = newAppliedForce;
		}
	}

	PX_ASSERT(currPtr == endPtr);

	//KS - we need to use PX_TRANSPOSE_44 here instead of the 34_43 variants because the W components are being used to 
	//store the bodies' progress counters.

	PX_TRANSPOSE_44(linVel0T0, linVel0T1, linVel0T2, linVel0T3, linVel00, linVel10, linVel20, linVel30);
	PX_TRANSPOSE_44(linVel1T0, linVel1T1, linVel1T2, linVel1T3, linVel01, linVel11, linVel21, linVel31);
	PX_TRANSPOSE_44(angState0T0, angState0T1, angState0T2, angState0T3, angState00, angState10, angState20, angState30);
	PX_TRANSPOSE_44(angState1T0, angState1T1, angState1T2, angState1T3, angState01, angState11, angState21, angState31);


	// Write back
	// Write back
	V4StoreA(linVel00, &b00.linearVelocity.x);
	V4StoreA(linVel10, &b10.linearVelocity.x);
	V4StoreA(linVel20, &b20.linearVelocity.x);
	V4StoreA(linVel30, &b30.linearVelocity.x);

	V4StoreA(linVel01, &b01.linearVelocity.x);
	V4StoreA(linVel11, &b11.linearVelocity.x);
	V4StoreA(linVel21, &b21.linearVelocity.x);
	V4StoreA(linVel31, &b31.linearVelocity.x);

	V4StoreA(angState00, &b00.angularState.x);
	V4StoreA(angState10, &b10.angularState.x);
	V4StoreA(angState20, &b20.angularState.x);
	V4StoreA(angState30, &b30.angularState.x);

	V4StoreA(angState01, &b01.angularState.x);
	V4StoreA(angState11, &b11.angularState.x);
	V4StoreA(angState21, &b21.angularState.x);
	V4StoreA(angState31, &b31.angularState.x);

}


static void solveFriction4_StaticBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, SolverContext& /*cache*/)
{

	PxSolverBody& b00 = *desc[0].bodyA;
	PxSolverBody& b10 = *desc[1].bodyA;
	PxSolverBody& b20 = *desc[2].bodyA;
	PxSolverBody& b30 = *desc[3].bodyA;


	Vec4V linVel00 = V4LoadA(&b00.linearVelocity.x);
	Vec4V angState00 = V4LoadA(&b00.angularState.x);

	Vec4V linVel10 = V4LoadA(&b10.linearVelocity.x);
	Vec4V angState10 = V4LoadA(&b10.angularState.x);

	Vec4V linVel20 = V4LoadA(&b20.linearVelocity.x);
	Vec4V angState20 = V4LoadA(&b20.angularState.x);

	Vec4V linVel30 = V4LoadA(&b30.linearVelocity.x);
	Vec4V angState30 = V4LoadA(&b30.angularState.x);


	Vec4V linVel0T0, linVel0T1, linVel0T2, linVel0T3;
	Vec4V angState0T0, angState0T1, angState0T2, angState0T3;


	PX_TRANSPOSE_44(linVel00, linVel10, linVel20, linVel30, linVel0T0, linVel0T1, linVel0T2, linVel0T3);
	PX_TRANSPOSE_44(angState00, angState10, angState20, angState30, angState0T0, angState0T1, angState0T2, angState0T3);

	PxU8* PX_RESTRICT currPtr = desc[0].constraint;
	PxU8* PX_RESTRICT endPtr = desc[0].constraint + getConstraintLength(desc[0]);
	

	while(currPtr < endPtr)
	{
		SolverFrictionHeader4* PX_RESTRICT hdr = reinterpret_cast<SolverFrictionHeader4*>(currPtr);

		currPtr = reinterpret_cast<PxU8*>(hdr + 1);

		Vec4V* appliedImpulses = reinterpret_cast<Vec4V*>(currPtr);

		currPtr += hdr->numNormalConstr * sizeof(Vec4V);

		Ps::prefetchLine(currPtr, 128);
		Ps::prefetchLine(currPtr,256);
		Ps::prefetchLine(currPtr,384);
		
		const PxU32	numFrictionConstr = hdr->numFrictionConstr;

		SolverFriction4Base* PX_RESTRICT frictions = reinterpret_cast<SolverFriction4Base*>(currPtr);

		currPtr = reinterpret_cast<PxU8*>(frictions + hdr->numFrictionConstr);

		const PxU32 maxFrictionConstr = numFrictionConstr;
	
		const Vec4V staticFric = hdr->staticFriction;

		const Vec4V invMass0D0 = hdr->invMassADom;
		const Vec4V angD0 = hdr->angD0;

		for(PxU32 i=0;i<maxFrictionConstr;i++)
		{
			SolverFriction4Base& f = frictions[i];
			Ps::prefetchLine((&f)+1);
			Ps::prefetchLine((&f)+1,128);
			Ps::prefetchLine((&f)+1,256);

			const Vec4V appliedImpulse = appliedImpulses[i>>hdr->frictionPerContact];

			const Vec4V maxFriction =  V4Mul(staticFric, appliedImpulse);

			const Vec4V nMaxFriction = V4Neg(maxFriction); 

			const Vec4V normalX = f.normalX;
			const Vec4V normalY = f.normalY;
			const Vec4V normalZ = f.normalZ;

			const Vec4V raXnX = f.raXnX;
			const Vec4V raXnY = f.raXnY;
			const Vec4V raXnZ = f.raXnZ;

			const Vec4V appliedForce(f.appliedForce);
			const Vec4V velMultiplier(f.velMultiplier);
			const Vec4V targetVel(f.targetVelocity);
	
			//4 x 4 Dot3 products encoded as 8 M44 transposes, 4 MulV and 8 MulAdd ops

			const Vec4V __normalVel1 = V4Mul(linVel0T0, normalX);
			const Vec4V __normalVel2 = V4Mul(raXnX, angState0T0);

			const Vec4V _normalVel1 = V4MulAdd(linVel0T1, normalY, __normalVel1);
			const Vec4V _normalVel2 = V4MulAdd(raXnY, angState0T1, __normalVel2);

			const Vec4V normalVel1 = V4MulAdd(linVel0T2, normalZ, _normalVel1);
			const Vec4V normalVel2 = V4MulAdd(raXnZ, angState0T2, _normalVel2);

			const Vec4V delLinVel00 = V4Mul(normalX, invMass0D0);

			const Vec4V delLinVel10 = V4Mul(normalY, invMass0D0);

			const Vec4V normalVel = V4Add(normalVel1, normalVel2);

			const Vec4V delLinVel20 = V4Mul(normalZ, invMass0D0);

			const Vec4V tmp = V4NegMulSub(targetVel, velMultiplier, appliedForce);

			Vec4V newAppliedForce = V4MulAdd(normalVel, velMultiplier, tmp);
			newAppliedForce = V4Clamp(newAppliedForce,nMaxFriction,  maxFriction);
			const Vec4V deltaF = V4Sub(newAppliedForce, appliedForce);

			const Vec4V deltaAngF0 = V4Mul(angD0, deltaF);

			linVel0T0 = V4MulAdd(delLinVel00, deltaF, linVel0T0);
			angState0T0 = V4MulAdd(raXnX, deltaAngF0, angState0T0);

			linVel0T1 = V4MulAdd(delLinVel10, deltaF, linVel0T1);
			angState0T1 = V4MulAdd(raXnY, deltaAngF0, angState0T1);

			linVel0T2 = V4MulAdd(delLinVel20, deltaF, linVel0T2);
			angState0T2 = V4MulAdd(raXnZ, deltaAngF0, angState0T2);

			f.appliedForce = newAppliedForce;
		}
	}

	PX_ASSERT(currPtr == endPtr);

	//KS - we need to use PX_TRANSPOSE_44 here instead of the 34_43 variants because the W components are being used to 
	//store the bodies' progress counters.

	PX_TRANSPOSE_44(linVel0T0, linVel0T1, linVel0T2, linVel0T3, linVel00, linVel10, linVel20, linVel30);
	PX_TRANSPOSE_44(angState0T0, angState0T1, angState0T2, angState0T3, angState00, angState10, angState20, angState30);

	// Write back
	// Write back
	V4StoreA(linVel00, &b00.linearVelocity.x);
	V4StoreA(linVel10, &b10.linearVelocity.x);
	V4StoreA(linVel20, &b20.linearVelocity.x);
	V4StoreA(linVel30, &b30.linearVelocity.x);

	V4StoreA(angState00, &b00.angularState.x);
	V4StoreA(angState10, &b10.angularState.x);
	V4StoreA(angState20, &b20.angularState.x);
	V4StoreA(angState30, &b30.angularState.x);
}

static void concludeContactCoulomb4(const PxSolverConstraintDesc* desc, SolverContext& /*cache*/)
{
	PxU8* PX_RESTRICT cPtr = desc[0].constraint;

	const Vec4V zero = V4Zero();

	const SolverContactCoulombHeader4* PX_RESTRICT firstHeader = reinterpret_cast<const SolverContactCoulombHeader4*>(cPtr);
	PxU8* PX_RESTRICT last = desc[0].constraint + firstHeader->frictionOffset;

	PxU32 pointStride = firstHeader->type == DY_SC_TYPE_BLOCK_RB_CONTACT ? sizeof(SolverContact4Dynamic) : sizeof(SolverContact4Base);

	while(cPtr < last)
	{
		const SolverContactCoulombHeader4* PX_RESTRICT hdr = reinterpret_cast<const SolverContactCoulombHeader4*>(cPtr);
		cPtr += sizeof(SolverContactCoulombHeader4);

		const PxU32 numNormalConstr = hdr->numNormalConstr;
		
		//if(cPtr < last)
		//Ps::prefetchLine(cPtr, 512);
		Ps::prefetchLine(cPtr,128);
		Ps::prefetchLine(cPtr,256);
		Ps::prefetchLine(cPtr,384);

		for(PxU32 i=0;i<numNormalConstr;i++)
		{
			SolverContact4Base *c = reinterpret_cast<SolverContact4Base*>(cPtr);
			cPtr += pointStride;
			c->scaledBias = V4Max(c->scaledBias, zero);
		}
	}
	PX_ASSERT(cPtr == last);
}

void  writeBackContactCoulomb4(const PxSolverConstraintDesc* desc, SolverContext& cache,
					  const PxSolverBodyData** PX_RESTRICT bd0, const PxSolverBodyData** PX_RESTRICT bd1)
{
	Vec4V normalForceV = V4Zero();
	PxU8* PX_RESTRICT cPtr = desc[0].constraint;
	PxReal* PX_RESTRICT vForceWriteback0 = reinterpret_cast<PxReal*>(desc[0].writeBack);
	PxReal* PX_RESTRICT vForceWriteback1 = reinterpret_cast<PxReal*>(desc[1].writeBack);
	PxReal* PX_RESTRICT vForceWriteback2 = reinterpret_cast<PxReal*>(desc[2].writeBack);
	PxReal* PX_RESTRICT vForceWriteback3 = reinterpret_cast<PxReal*>(desc[3].writeBack);

	const SolverContactCoulombHeader4* PX_RESTRICT firstHeader = reinterpret_cast<const SolverContactCoulombHeader4*>(cPtr);
	PxU8* PX_RESTRICT last = desc[0].constraint + firstHeader->frictionOffset;

	const PxU32 pointStride = firstHeader->type == DY_SC_TYPE_BLOCK_RB_CONTACT ? sizeof(SolverContact4Dynamic)
																	   : sizeof(SolverContact4Base);

	bool writeBackThresholds[4] = {false, false, false, false};


	while(cPtr < last)
	{
		const SolverContactCoulombHeader4* PX_RESTRICT hdr = reinterpret_cast<const SolverContactCoulombHeader4*>(cPtr);
		cPtr += sizeof(SolverContactCoulombHeader4);

		writeBackThresholds[0] = hdr->flags[0] & SolverContactHeader::eHAS_FORCE_THRESHOLDS;
		writeBackThresholds[1] = hdr->flags[1] & SolverContactHeader::eHAS_FORCE_THRESHOLDS;
		writeBackThresholds[2] = hdr->flags[2] & SolverContactHeader::eHAS_FORCE_THRESHOLDS;
		writeBackThresholds[3] = hdr->flags[3] & SolverContactHeader::eHAS_FORCE_THRESHOLDS;

		const PxU32 numNormalConstr = hdr->numNormalConstr;

		Ps::prefetchLine(cPtr, 256);
		Ps::prefetchLine(cPtr, 384);

		
		for(PxU32 i=0; i<numNormalConstr; i++)
		{
			SolverContact4Base* c = reinterpret_cast<SolverContact4Base*>(cPtr);
			cPtr += pointStride;

			const Vec4V appliedForce = c->appliedForce;
			if(vForceWriteback0 && i < hdr->numNormalConstr0)
				FStore(V4GetX(appliedForce), vForceWriteback0++);
			if(vForceWriteback1 && i < hdr->numNormalConstr1)
				FStore(V4GetY(appliedForce), vForceWriteback1++);
			if(vForceWriteback2 && i < hdr->numNormalConstr2)
				FStore(V4GetZ(appliedForce), vForceWriteback2++);
			if(vForceWriteback3 && i < hdr->numNormalConstr3)
				FStore(V4GetW(appliedForce), vForceWriteback3++);
			
			normalForceV = V4Add(normalForceV, appliedForce);
		}
	}
	PX_ASSERT(cPtr == last);

	PX_ALIGN(16, PxReal nf[4]);
	V4StoreA(normalForceV, nf);

	//all constraint pointer in descs are the same constraint
	Sc::ShapeInteraction** shapeInteractions = reinterpret_cast<SolverContactCoulombHeader4*>(desc[0].constraint)->shapeInteraction;

	for(PxU32 a = 0; a < 4; ++a)
	{
		if(writeBackThresholds[a] && desc[a].linkIndexA == PxSolverConstraintDesc::NO_LINK && desc[a].linkIndexB == PxSolverConstraintDesc::NO_LINK &&
			nf[a] !=0.f && (bd0[a]->reportThreshold < PX_MAX_REAL  || bd1[a]->reportThreshold < PX_MAX_REAL))
		{
			ThresholdStreamElement elt;
			elt.normalForce = nf[a];
			elt.threshold = PxMin<float>(bd0[a]->reportThreshold, bd1[a]->reportThreshold);
			elt.nodeIndexA = IG::NodeIndex(bd0[a]->nodeIndex);
			elt.nodeIndexB = IG::NodeIndex(bd1[a]->nodeIndex);
			elt.shapeInteraction = shapeInteractions[a];
			Ps::order(elt.nodeIndexA, elt.nodeIndexB);
			PX_ASSERT(elt.nodeIndexA < elt.nodeIndexB);
			PX_ASSERT(cache.mThresholdStreamIndex<cache.mThresholdStreamLength);
			cache.mThresholdStream[cache.mThresholdStreamIndex++] = elt;
		}
	}
}

void solveContactCoulombPreBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 /*constraintCount*/, SolverContext& cache)
{
	solveContactCoulomb4_Block(desc, cache);
}

void solveContactCoulombPreBlock_Static(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32  /*constraintCount*/, SolverContext& cache)
{
	solveContactCoulomb4_StaticBlock(desc, cache);
}

void solveContactCoulombPreBlock_Conclude(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32  /*constraintCount*/, SolverContext& cache)
{
	solveContactCoulomb4_Block(desc, cache);
	concludeContactCoulomb4(desc, cache);
}

void solveContactCoulombPreBlock_ConcludeStatic(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32  /*constraintCount*/, SolverContext& cache)
{
	solveContactCoulomb4_StaticBlock(desc, cache);
	concludeContactCoulomb4(desc, cache);
}

void solveContactCoulombPreBlock_WriteBack(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32  /*constraintCount*/, SolverContext& cache)
{
	solveContactCoulomb4_Block(desc, cache);

	const PxSolverBodyData* bd0[4] = {	&cache.solverBodyArray[desc[0].bodyADataIndex], 
										&cache.solverBodyArray[desc[1].bodyADataIndex],
										&cache.solverBodyArray[desc[2].bodyADataIndex],
										&cache.solverBodyArray[desc[3].bodyADataIndex]};

	const PxSolverBodyData* bd1[4] = {	&cache.solverBodyArray[desc[0].bodyBDataIndex], 
										&cache.solverBodyArray[desc[1].bodyBDataIndex],
										&cache.solverBodyArray[desc[2].bodyBDataIndex],
										&cache.solverBodyArray[desc[3].bodyBDataIndex]};



	writeBackContactCoulomb4(desc, cache, bd0, bd1);

	if(cache.mThresholdStreamIndex > (cache.mThresholdStreamLength - 4))
	{
		//Write back to global buffer
		PxI32 threshIndex = physx::shdfnd::atomicAdd(cache.mSharedOutThresholdPairs, PxI32(cache.mThresholdStreamIndex)) - PxI32(cache.mThresholdStreamIndex);
		for(PxU32 a = 0; a < cache.mThresholdStreamIndex; ++a)
		{
			cache.mSharedThresholdStream[a + threshIndex] = cache.mThresholdStream[a];
		}
		cache.mThresholdStreamIndex = 0;
	}
}

void solveContactCoulombPreBlock_WriteBackStatic(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 /*constraintCount*/, SolverContext& cache)
{
	solveContactCoulomb4_StaticBlock(desc, cache);
	const PxSolverBodyData* bd0[4] = {	&cache.solverBodyArray[desc[0].bodyADataIndex], 
										&cache.solverBodyArray[desc[1].bodyADataIndex],
										&cache.solverBodyArray[desc[2].bodyADataIndex],
										&cache.solverBodyArray[desc[3].bodyADataIndex]};

	const PxSolverBodyData* bd1[4] = {	&cache.solverBodyArray[desc[0].bodyBDataIndex], 
										&cache.solverBodyArray[desc[1].bodyBDataIndex],
										&cache.solverBodyArray[desc[2].bodyBDataIndex],
										&cache.solverBodyArray[desc[3].bodyBDataIndex]};

	writeBackContactCoulomb4(desc, cache, bd0, bd1);

	if(cache.mThresholdStreamIndex > (cache.mThresholdStreamLength - 4))
	{
		//Write back to global buffer
		PxI32 threshIndex = physx::shdfnd::atomicAdd(cache.mSharedOutThresholdPairs, PxI32(cache.mThresholdStreamIndex)) - PxI32(cache.mThresholdStreamIndex);
		for(PxU32 a = 0; a < cache.mThresholdStreamIndex; ++a)
		{
			cache.mSharedThresholdStream[a + threshIndex] = cache.mThresholdStream[a];
		}
		cache.mThresholdStreamIndex = 0;
	}
}

void solveFrictionCoulombPreBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32  /*constraintCount*/, SolverContext& cache)
{
	solveFriction4_Block(desc, cache);
}

void solveFrictionCoulombPreBlock_Static(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32  /*constraintCount*/, SolverContext& cache)
{
	solveFriction4_StaticBlock(desc, cache);
}

void solveFrictionCoulombPreBlock_Conclude(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32  /*constraintCount*/, SolverContext& cache)
{
	solveFriction4_Block(desc, cache);
}

void solveFrictionCoulombPreBlock_ConcludeStatic(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32  /*constraintCount*/, SolverContext& cache)
{
	solveFriction4_StaticBlock(desc, cache);
}

void solveFrictionCoulombPreBlock_WriteBack(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32  /*constraintCount*/, SolverContext& cache)
{
	solveFriction4_Block(desc, cache);
}

void solveFrictionCoulombPreBlock_WriteBackStatic(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32  /*constraintCount*/, SolverContext& cache)
{
	solveFriction4_StaticBlock(desc, cache);
}


}

}

