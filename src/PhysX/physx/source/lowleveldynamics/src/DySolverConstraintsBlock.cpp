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
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  


#include "foundation/PxPreprocessor.h"
#include "PsVecMath.h"
#include "PsFPU.h"

#include "CmPhysXCommon.h"
#include "DySolverBody.h"
#include "DySolverContact.h"
#include "DySolverConstraint1D.h"
#include "DySolverConstraintDesc.h"
#include "DyThresholdTable.h"
#include "DySolverContext.h"
#include "PsUtilities.h"
#include "DyConstraint.h"
#include "PsAtomic.h"
#include "DySolverContact4.h"
#include "DySolverConstraint1D4.h"

namespace physx
{

namespace Dy
{

static void solveContact4_Block(const PxSolverConstraintDesc* PX_RESTRICT desc, SolverContext& cache)
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


	const PxU8* PX_RESTRICT last = desc[0].constraint + getConstraintLength(desc[0]);

	//hopefully pointer aliasing doesn't bite.
	PxU8* PX_RESTRICT currPtr = desc[0].constraint;

	Vec4V vMax = V4Splat(FMax());

	const PxU8* PX_RESTRICT prefetchAddress = currPtr + sizeof(SolverContactHeader4) + sizeof(SolverContactBatchPointDynamic4);

	const SolverContactHeader4* PX_RESTRICT hdr = reinterpret_cast<SolverContactHeader4*>(currPtr);

	const Vec4V invMassA = hdr->invMass0D0;
	const Vec4V invMassB = hdr->invMass1D1;

	const Vec4V sumInvMass = V4Add(invMassA, invMassB);


	while(currPtr < last)
	{

		hdr = reinterpret_cast<const SolverContactHeader4*>(currPtr);

		PX_ASSERT(hdr->type == DY_SC_TYPE_BLOCK_RB_CONTACT);

		currPtr = reinterpret_cast<PxU8*>(const_cast<SolverContactHeader4*>(hdr) + 1);

		const PxU32 numNormalConstr = hdr->numNormalConstr;
		const PxU32	numFrictionConstr = hdr->numFrictionConstr;

		bool hasMaxImpulse = (hdr->flag & SolverContactHeader4::eHAS_MAX_IMPULSE) != 0;

		Vec4V* appliedForces = reinterpret_cast<Vec4V*>(currPtr);
		currPtr += sizeof(Vec4V)*numNormalConstr;

		SolverContactBatchPointDynamic4* PX_RESTRICT contacts = reinterpret_cast<SolverContactBatchPointDynamic4*>(currPtr);

		Vec4V* maxImpulses;
		currPtr = reinterpret_cast<PxU8*>(contacts + numNormalConstr);
		PxU32 maxImpulseMask = 0;
		if(hasMaxImpulse)
		{
			maxImpulseMask = 0xFFFFFFFF;
			maxImpulses = reinterpret_cast<Vec4V*>(currPtr);
			currPtr += sizeof(Vec4V) * numNormalConstr;
		}
		else
		{
			maxImpulses = &vMax;
		}

				
		SolverFrictionSharedData4* PX_RESTRICT fd = reinterpret_cast<SolverFrictionSharedData4*>(currPtr);
		if(numFrictionConstr)
			currPtr += sizeof(SolverFrictionSharedData4);

		Vec4V* frictionAppliedForce = reinterpret_cast<Vec4V*>(currPtr);
		currPtr += sizeof(Vec4V)*numFrictionConstr;

		const SolverContactFrictionDynamic4* PX_RESTRICT frictions = reinterpret_cast<SolverContactFrictionDynamic4*>(currPtr);
		currPtr += numFrictionConstr * sizeof(SolverContactFrictionDynamic4);
		
		Vec4V accumulatedNormalImpulse = vZero;

		const Vec4V angD0 = hdr->angDom0;
		const Vec4V angD1 = hdr->angDom1;

		const Vec4V _normalT0 = hdr->normalX;
		const Vec4V _normalT1 = hdr->normalY;
		const Vec4V _normalT2 = hdr->normalZ;

		Vec4V contactNormalVel1 = V4Mul(linVel0T0, _normalT0);
		Vec4V contactNormalVel3 = V4Mul(linVel1T0, _normalT0);
		contactNormalVel1 = V4MulAdd(linVel0T1, _normalT1, contactNormalVel1);
		contactNormalVel3 = V4MulAdd(linVel1T1, _normalT1, contactNormalVel3);
		contactNormalVel1 = V4MulAdd(linVel0T2, _normalT2, contactNormalVel1);
		contactNormalVel3 = V4MulAdd(linVel1T2, _normalT2, contactNormalVel3);

		Vec4V relVel1 = V4Sub(contactNormalVel1, contactNormalVel3);

		Vec4V accumDeltaF = vZero;

		for(PxU32 i=0;i<numNormalConstr;i++)
		{
			const SolverContactBatchPointDynamic4& c = contacts[i];

			PxU32 offset = 0;
			Ps::prefetchLine(prefetchAddress, offset += 64);
			Ps::prefetchLine(prefetchAddress, offset += 64);
			Ps::prefetchLine(prefetchAddress, offset += 64);
			prefetchAddress += offset;

			const Vec4V appliedForce = appliedForces[i];
			const Vec4V maxImpulse = maxImpulses[i & maxImpulseMask];			
			
			Vec4V contactNormalVel2 = V4Mul(c.raXnX, angState0T0);
			Vec4V contactNormalVel4 = V4Mul(c.rbXnX, angState1T0);

			contactNormalVel2 = V4MulAdd(c.raXnY, angState0T1, contactNormalVel2);
			contactNormalVel4 = V4MulAdd(c.rbXnY, angState1T1, contactNormalVel4);

			contactNormalVel2 = V4MulAdd(c.raXnZ, angState0T2, contactNormalVel2);
			contactNormalVel4 = V4MulAdd(c.rbXnZ, angState1T2, contactNormalVel4);

			const Vec4V normalVel = V4Add(relVel1, V4Sub(contactNormalVel2, contactNormalVel4));

			Vec4V deltaF = V4NegMulSub(normalVel, c.velMultiplier, c.biasedErr);

			deltaF = V4Max(deltaF,  V4Neg(appliedForce));
			const Vec4V newAppliedForce = V4Min(V4Add(appliedForce, deltaF), maxImpulse);
			deltaF = V4Sub(newAppliedForce, appliedForce);

			accumDeltaF = V4Add(accumDeltaF, deltaF);

			const Vec4V angDetaF0 = V4Mul(deltaF, angD0);
			const Vec4V angDetaF1 = V4Mul(deltaF, angD1);

			relVel1 = V4MulAdd(sumInvMass, deltaF, relVel1);
			
			angState0T0 = V4MulAdd(c.raXnX, angDetaF0, angState0T0);
			angState1T0 = V4NegMulSub(c.rbXnX, angDetaF1, angState1T0);
			
			angState0T1 = V4MulAdd(c.raXnY, angDetaF0, angState0T1);
			angState1T1 = V4NegMulSub(c.rbXnY, angDetaF1, angState1T1);

			angState0T2 = V4MulAdd(c.raXnZ, angDetaF0, angState0T2);
			angState1T2 = V4NegMulSub(c.rbXnZ, angDetaF1, angState1T2);

			appliedForces[i] = newAppliedForce;

			accumulatedNormalImpulse = V4Add(accumulatedNormalImpulse, newAppliedForce);
		}

		const Vec4V accumDeltaF_IM0 = V4Mul(accumDeltaF, invMassA);
		const Vec4V accumDeltaF_IM1 = V4Mul(accumDeltaF, invMassB);

		linVel0T0 = V4MulAdd(_normalT0, accumDeltaF_IM0, linVel0T0);
		linVel1T0 = V4NegMulSub(_normalT0, accumDeltaF_IM1, linVel1T0);
		linVel0T1 = V4MulAdd(_normalT1, accumDeltaF_IM0, linVel0T1);
		linVel1T1 = V4NegMulSub(_normalT1, accumDeltaF_IM1, linVel1T1);
		linVel0T2 = V4MulAdd(_normalT2, accumDeltaF_IM0, linVel0T2);
		linVel1T2 = V4NegMulSub(_normalT2, accumDeltaF_IM1, linVel1T2);


		if(cache.doFriction && numFrictionConstr)
		{
			const Vec4V staticFric = hdr->staticFriction;
			const Vec4V dynamicFric = hdr->dynamicFriction;

			const Vec4V maxFrictionImpulse = V4Mul(staticFric, accumulatedNormalImpulse);
			const Vec4V maxDynFrictionImpulse = V4Mul(dynamicFric, accumulatedNormalImpulse);
			const Vec4V negMaxDynFrictionImpulse = V4Neg(maxDynFrictionImpulse);
			//const Vec4V negMaxFrictionImpulse = V4Neg(maxFrictionImpulse);
			BoolV broken = BFFFF();

			if(cache.writeBackIteration)
			{
				Ps::prefetchLine(fd->frictionBrokenWritebackByte[0]);
				Ps::prefetchLine(fd->frictionBrokenWritebackByte[1]);
				Ps::prefetchLine(fd->frictionBrokenWritebackByte[2]);
			}


			for(PxU32 i=0;i<numFrictionConstr;i++)
			{
				const SolverContactFrictionDynamic4& f = frictions[i];

				PxU32 offset = 0;
				Ps::prefetchLine(prefetchAddress, offset += 64);
				Ps::prefetchLine(prefetchAddress, offset += 64);
				Ps::prefetchLine(prefetchAddress, offset += 64);
				Ps::prefetchLine(prefetchAddress, offset += 64);
				prefetchAddress += offset;

				const Vec4V appliedForce = frictionAppliedForce[i];

				const Vec4V normalT0 = fd->normalX[i&1];
				const Vec4V normalT1 = fd->normalY[i&1];
				const Vec4V normalT2 = fd->normalZ[i&1];

				Vec4V normalVel1 = V4Mul(linVel0T0, normalT0);
				Vec4V normalVel2 = V4Mul(f.raXnX, angState0T0);
				Vec4V normalVel3 = V4Mul(linVel1T0, normalT0);
				Vec4V normalVel4 = V4Mul(f.rbXnX, angState1T0);

				normalVel1 = V4MulAdd(linVel0T1, normalT1, normalVel1);
				normalVel2 = V4MulAdd(f.raXnY, angState0T1, normalVel2);
				normalVel3 = V4MulAdd(linVel1T1, normalT1, normalVel3);
				normalVel4 = V4MulAdd(f.rbXnY, angState1T1, normalVel4);

				normalVel1 = V4MulAdd(linVel0T2, normalT2, normalVel1);
				normalVel2 = V4MulAdd(f.raXnZ, angState0T2, normalVel2);
				normalVel3 = V4MulAdd(linVel1T2, normalT2, normalVel3);
				normalVel4 = V4MulAdd(f.rbXnZ, angState1T2, normalVel4);

				const Vec4V _normalVel = V4Add(normalVel1, normalVel2);
				const Vec4V __normalVel = V4Add(normalVel3, normalVel4);

				// appliedForce -bias * velMultiplier - a hoisted part of the total impulse computation
			
				const Vec4V normalVel = V4Sub(_normalVel, __normalVel );

				const Vec4V tmp1 = V4Sub(appliedForce, f.scaledBias); 

				const Vec4V totalImpulse = V4NegMulSub(normalVel, f.velMultiplier, tmp1);
				
				broken = BOr(broken, V4IsGrtr(V4Abs(totalImpulse), maxFrictionImpulse));

				const Vec4V newAppliedForce = V4Sel(broken, V4Min(maxDynFrictionImpulse, V4Max(negMaxDynFrictionImpulse, totalImpulse)), totalImpulse);

				const Vec4V deltaF =V4Sub(newAppliedForce, appliedForce);

				frictionAppliedForce[i] = newAppliedForce;

				const Vec4V deltaFIM0 = V4Mul(deltaF, invMassA);
				const Vec4V deltaFIM1 = V4Mul(deltaF, invMassB);

				const Vec4V angDetaF0 = V4Mul(deltaF, angD0);
				const Vec4V angDetaF1 = V4Mul(deltaF, angD1);

				linVel0T0 = V4MulAdd(normalT0, deltaFIM0, linVel0T0);
				linVel1T0 = V4NegMulSub(normalT0, deltaFIM1, linVel1T0);
				angState0T0 = V4MulAdd(f.raXnX, angDetaF0, angState0T0);
				angState1T0 = V4NegMulSub(f.rbXnX, angDetaF1, angState1T0);

				linVel0T1 = V4MulAdd(normalT1, deltaFIM0, linVel0T1);
				linVel1T1 = V4NegMulSub(normalT1, deltaFIM1, linVel1T1);
				angState0T1 = V4MulAdd(f.raXnY, angDetaF0, angState0T1);
				angState1T1 = V4NegMulSub(f.rbXnY, angDetaF1, angState1T1);

				linVel0T2 = V4MulAdd(normalT2, deltaFIM0, linVel0T2);
				linVel1T2 = V4NegMulSub(normalT2, deltaFIM1, linVel1T2);
				angState0T2 = V4MulAdd(f.raXnZ, angDetaF0, angState0T2);
				angState1T2 = V4NegMulSub(f.rbXnZ, angDetaF1, angState1T2);
			}
			fd->broken = broken;
		}
	}

	PX_TRANSPOSE_44(linVel0T0, linVel0T1, linVel0T2, linVel0T3, linVel00, linVel10, linVel20, linVel30);
	PX_TRANSPOSE_44(linVel1T0, linVel1T1, linVel1T2, linVel1T3, linVel01, linVel11, linVel21, linVel31);
	PX_TRANSPOSE_44(angState0T0, angState0T1, angState0T2, angState0T3, angState00, angState10, angState20, angState30);
	PX_TRANSPOSE_44(angState1T0, angState1T1, angState1T2, angState1T3, angState01, angState11, angState21, angState31);

	PX_ASSERT(b00.linearVelocity.isFinite());
	PX_ASSERT(b00.angularState.isFinite());
	PX_ASSERT(b10.linearVelocity.isFinite());
	PX_ASSERT(b10.angularState.isFinite());
	PX_ASSERT(b20.linearVelocity.isFinite());
	PX_ASSERT(b20.angularState.isFinite());
	PX_ASSERT(b30.linearVelocity.isFinite());
	PX_ASSERT(b30.angularState.isFinite());

	PX_ASSERT(b01.linearVelocity.isFinite());
	PX_ASSERT(b01.angularState.isFinite());
	PX_ASSERT(b11.linearVelocity.isFinite());
	PX_ASSERT(b11.angularState.isFinite());
	PX_ASSERT(b21.linearVelocity.isFinite());
	PX_ASSERT(b21.angularState.isFinite());
	PX_ASSERT(b31.linearVelocity.isFinite());
	PX_ASSERT(b31.angularState.isFinite());

	// Write back
	V4StoreA(linVel00, &b00.linearVelocity.x);
	V4StoreA(angState00, &b00.angularState.x);
	V4StoreA(linVel10, &b10.linearVelocity.x);
	V4StoreA(angState10, &b10.angularState.x);
	V4StoreA(linVel20, &b20.linearVelocity.x);
	V4StoreA(angState20, &b20.angularState.x);
	V4StoreA(linVel30, &b30.linearVelocity.x);
	V4StoreA(angState30, &b30.angularState.x);

	if(desc[0].bodyBDataIndex != 0)
	{
		V4StoreA(linVel01, &b01.linearVelocity.x);
		V4StoreA(angState01, &b01.angularState.x);
	}
	if(desc[1].bodyBDataIndex != 0)
	{
		V4StoreA(linVel11, &b11.linearVelocity.x);
		V4StoreA(angState11, &b11.angularState.x);
	}
	if(desc[2].bodyBDataIndex != 0)
	{
		V4StoreA(linVel21, &b21.linearVelocity.x);
		V4StoreA(angState21, &b21.angularState.x);
	}
	if(desc[3].bodyBDataIndex != 0)
	{
		V4StoreA(linVel31, &b31.linearVelocity.x);
		V4StoreA(angState31, &b31.angularState.x);
	}

	PX_ASSERT(b00.linearVelocity.isFinite());
	PX_ASSERT(b00.angularState.isFinite());
	PX_ASSERT(b10.linearVelocity.isFinite());
	PX_ASSERT(b10.angularState.isFinite());
	PX_ASSERT(b20.linearVelocity.isFinite());
	PX_ASSERT(b20.angularState.isFinite());
	PX_ASSERT(b30.linearVelocity.isFinite());
	PX_ASSERT(b30.angularState.isFinite());

	PX_ASSERT(b01.linearVelocity.isFinite());
	PX_ASSERT(b01.angularState.isFinite());
	PX_ASSERT(b11.linearVelocity.isFinite());
	PX_ASSERT(b11.angularState.isFinite());
	PX_ASSERT(b21.linearVelocity.isFinite());
	PX_ASSERT(b21.angularState.isFinite());
	PX_ASSERT(b31.linearVelocity.isFinite());
	PX_ASSERT(b31.angularState.isFinite());
}

static void solveContact4_StaticBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, SolverContext& cache)
{
	PxSolverBody& b00 = *desc[0].bodyA;
	PxSolverBody& b10 = *desc[1].bodyA;
	PxSolverBody& b20 = *desc[2].bodyA;
	PxSolverBody& b30 = *desc[3].bodyA;

	const PxU8* PX_RESTRICT last = desc[0].constraint + getConstraintLength(desc[0]);

	//hopefully pointer aliasing doesn't bite.
	PxU8* PX_RESTRICT currPtr = desc[0].constraint;


	//We'll need this.
	const Vec4V vZero	= V4Zero();
	Vec4V vMax	= V4Splat(FMax());
	
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

	const PxU8* PX_RESTRICT prefetchAddress = currPtr + sizeof(SolverContactHeader4) + sizeof(SolverContactBatchPointBase4);

	const SolverContactHeader4* PX_RESTRICT hdr = reinterpret_cast<SolverContactHeader4*>(currPtr);

	const Vec4V invMass0 = hdr->invMass0D0;

	while((currPtr < last))
	{
		hdr = reinterpret_cast<const SolverContactHeader4*>(currPtr);

		PX_ASSERT(hdr->type == DY_SC_TYPE_BLOCK_STATIC_RB_CONTACT);
		
		currPtr = const_cast<PxU8*>(reinterpret_cast<const PxU8*>(hdr + 1));

		const PxU32 numNormalConstr = hdr->numNormalConstr;
		const PxU32	numFrictionConstr = hdr->numFrictionConstr;
		bool hasMaxImpulse = (hdr->flag & SolverContactHeader4::eHAS_MAX_IMPULSE) != 0;

		Vec4V* appliedForces = reinterpret_cast<Vec4V*>(currPtr);
		currPtr += sizeof(Vec4V)*numNormalConstr;

		SolverContactBatchPointBase4* PX_RESTRICT contacts = reinterpret_cast<SolverContactBatchPointBase4*>(currPtr);

		currPtr = reinterpret_cast<PxU8*>(contacts + numNormalConstr);

		Vec4V* maxImpulses;
		PxU32 maxImpulseMask;
		if(hasMaxImpulse)
		{
			maxImpulseMask = 0xFFFFFFFF;
			maxImpulses = reinterpret_cast<Vec4V*>(currPtr);
			currPtr += sizeof(Vec4V) * numNormalConstr;
		}
		else
		{
			maxImpulseMask = 0;
			maxImpulses = &vMax;
		}

		SolverFrictionSharedData4* PX_RESTRICT fd = reinterpret_cast<SolverFrictionSharedData4*>(currPtr);
		if(numFrictionConstr)
			currPtr += sizeof(SolverFrictionSharedData4);

		Vec4V* frictionAppliedForces = reinterpret_cast<Vec4V*>(currPtr);
		currPtr += sizeof(Vec4V)*numFrictionConstr;

		const SolverContactFrictionBase4* PX_RESTRICT frictions = reinterpret_cast<SolverContactFrictionBase4*>(currPtr);
		currPtr += numFrictionConstr * sizeof(SolverContactFrictionBase4);

		
		Vec4V accumulatedNormalImpulse = vZero;

		const Vec4V angD0 = hdr->angDom0;
		const Vec4V _normalT0 = hdr->normalX;
		const Vec4V _normalT1 = hdr->normalY;
		const Vec4V _normalT2 = hdr->normalZ;

		Vec4V contactNormalVel1 = V4Mul(linVel0T0, _normalT0);
		contactNormalVel1 = V4MulAdd(linVel0T1, _normalT1, contactNormalVel1);

		contactNormalVel1 = V4MulAdd(linVel0T2, _normalT2, contactNormalVel1);

		Vec4V accumDeltaF = vZero;


		for(PxU32 i=0;i<numNormalConstr;i++)
		{
			const SolverContactBatchPointBase4& c = contacts[i];

			PxU32 offset = 0;
			Ps::prefetchLine(prefetchAddress, offset += 64);
			Ps::prefetchLine(prefetchAddress, offset += 64);
			Ps::prefetchLine(prefetchAddress, offset += 64);
			prefetchAddress += offset;

			const Vec4V appliedForce = appliedForces[i];
			const Vec4V maxImpulse = maxImpulses[i&maxImpulseMask];
			Vec4V contactNormalVel2 = V4MulAdd(c.raXnX, angState0T0, contactNormalVel1);
			contactNormalVel2 = V4MulAdd(c.raXnY, angState0T1, contactNormalVel2);
			const Vec4V normalVel = V4MulAdd(c.raXnZ, angState0T2, contactNormalVel2);

			const Vec4V _deltaF = V4Max(V4NegMulSub(normalVel, c.velMultiplier, c.biasedErr), V4Neg(appliedForce));

			Vec4V newAppliedForce(V4Add(appliedForce, _deltaF));
			newAppliedForce = V4Min(newAppliedForce, maxImpulse);
			const Vec4V deltaF = V4Sub(newAppliedForce, appliedForce);
			const Vec4V angDeltaF = V4Mul(angD0, deltaF);

			accumDeltaF = V4Add(accumDeltaF, deltaF);

			contactNormalVel1 = V4MulAdd(invMass0, deltaF, contactNormalVel1);
			angState0T0 = V4MulAdd(c.raXnX, angDeltaF, angState0T0);
			angState0T1 = V4MulAdd(c.raXnY, angDeltaF, angState0T1);
			angState0T2 = V4MulAdd(c.raXnZ, angDeltaF, angState0T2);
			
#if 1
			appliedForces[i] = newAppliedForce;
#endif
			
			accumulatedNormalImpulse = V4Add(accumulatedNormalImpulse, newAppliedForce);
		}	

		const Vec4V deltaFInvMass0 = V4Mul(accumDeltaF, invMass0);

		linVel0T0 = V4MulAdd(_normalT0, deltaFInvMass0, linVel0T0);
		linVel0T1 = V4MulAdd(_normalT1, deltaFInvMass0, linVel0T1);
		linVel0T2 = V4MulAdd(_normalT2, deltaFInvMass0, linVel0T2);

		if(cache.doFriction && numFrictionConstr)
		{
			const Vec4V staticFric = hdr->staticFriction;

			const Vec4V dynamicFric = hdr->dynamicFriction;

			const Vec4V maxFrictionImpulse = V4Mul(staticFric, accumulatedNormalImpulse);
			const Vec4V maxDynFrictionImpulse = V4Mul(dynamicFric, accumulatedNormalImpulse);
			const Vec4V negMaxDynFrictionImpulse = V4Neg(maxDynFrictionImpulse);

			BoolV broken = BFFFF();

			if(cache.writeBackIteration)
			{
				Ps::prefetchLine(fd->frictionBrokenWritebackByte[0]);
				Ps::prefetchLine(fd->frictionBrokenWritebackByte[1]);
				Ps::prefetchLine(fd->frictionBrokenWritebackByte[2]);
				Ps::prefetchLine(fd->frictionBrokenWritebackByte[3]);
			}

			for(PxU32 i=0;i<numFrictionConstr;i++)
			{
				const SolverContactFrictionBase4& f = frictions[i];

				PxU32 offset = 0;
				Ps::prefetchLine(prefetchAddress, offset += 64);
				Ps::prefetchLine(prefetchAddress, offset += 64);
				Ps::prefetchLine(prefetchAddress, offset += 64);
				prefetchAddress += offset;

				const Vec4V appliedForce = frictionAppliedForces[i];

				const Vec4V normalT0 = fd->normalX[i&1];
				const Vec4V normalT1 = fd->normalY[i&1];
				const Vec4V normalT2 = fd->normalZ[i&1];

				Vec4V normalVel1 = V4Mul(linVel0T0, normalT0);
				Vec4V normalVel2 = V4Mul(f.raXnX, angState0T0);

				normalVel1 = V4MulAdd(linVel0T1, normalT1, normalVel1);
				normalVel2 = V4MulAdd(f.raXnY, angState0T1, normalVel2);

				normalVel1 = V4MulAdd(linVel0T2, normalT2, normalVel1);
				normalVel2 = V4MulAdd(f.raXnZ, angState0T2, normalVel2);

				//relative normal velocity for all 4 constraints
				const Vec4V normalVel = V4Add(normalVel1, normalVel2);

				// appliedForce -bias * velMultiplier - a hoisted part of the total impulse computation
				const Vec4V tmp1 = V4Sub(appliedForce, f.scaledBias); 

				const Vec4V totalImpulse = V4NegMulSub(normalVel, f.velMultiplier, tmp1);

				broken = BOr(broken, V4IsGrtr(V4Abs(totalImpulse), maxFrictionImpulse));

				const Vec4V newAppliedForce = V4Sel(broken, V4Min(maxDynFrictionImpulse, V4Max(negMaxDynFrictionImpulse, totalImpulse)), totalImpulse);

				const Vec4V deltaF =V4Sub(newAppliedForce, appliedForce);

				const Vec4V deltaFInvMass = V4Mul(invMass0, deltaF);
				const Vec4V angDeltaF = V4Mul(angD0, deltaF);

				linVel0T0 = V4MulAdd(normalT0, deltaFInvMass, linVel0T0);
				angState0T0 = V4MulAdd(f.raXnX, angDeltaF, angState0T0);

				linVel0T1 = V4MulAdd(normalT1, deltaFInvMass, linVel0T1);
				angState0T1 = V4MulAdd(f.raXnY, angDeltaF, angState0T1);

				linVel0T2 = V4MulAdd(normalT2, deltaFInvMass, linVel0T2);
				angState0T2 = V4MulAdd(f.raXnZ, angDeltaF, angState0T2);

#if 1
				frictionAppliedForces[i] = newAppliedForce;
#endif

			}

			fd->broken = broken;
		}
	}

	PX_TRANSPOSE_44(linVel0T0, linVel0T1, linVel0T2, linVel0T3, linVel00, linVel10, linVel20, linVel30);
	PX_TRANSPOSE_44(angState0T0, angState0T1, angState0T2, angState0T3, angState00, angState10, angState20, angState30);

	PX_ASSERT(b00.linearVelocity.isFinite());
	PX_ASSERT(b00.angularState.isFinite());
	PX_ASSERT(b10.linearVelocity.isFinite());
	PX_ASSERT(b10.angularState.isFinite());
	PX_ASSERT(b20.linearVelocity.isFinite());
	PX_ASSERT(b20.angularState.isFinite());
	PX_ASSERT(b30.linearVelocity.isFinite());
	PX_ASSERT(b30.angularState.isFinite());

	// Write back
	V4StoreA(linVel00, &b00.linearVelocity.x);
	V4StoreA(linVel10, &b10.linearVelocity.x);
	V4StoreA(linVel20, &b20.linearVelocity.x);
	V4StoreA(linVel30, &b30.linearVelocity.x);

	V4StoreA(angState00, &b00.angularState.x);
	V4StoreA(angState10, &b10.angularState.x);
	V4StoreA(angState20, &b20.angularState.x);
	V4StoreA(angState30, &b30.angularState.x);

	PX_ASSERT(b00.linearVelocity.isFinite());
	PX_ASSERT(b00.angularState.isFinite());
	PX_ASSERT(b10.linearVelocity.isFinite());
	PX_ASSERT(b10.angularState.isFinite());
	PX_ASSERT(b20.linearVelocity.isFinite());
	PX_ASSERT(b20.angularState.isFinite());
	PX_ASSERT(b30.linearVelocity.isFinite());
	PX_ASSERT(b30.angularState.isFinite());
}

static void concludeContact4_Block(const PxSolverConstraintDesc* PX_RESTRICT desc, SolverContext& /*cache*/, PxU32 contactSize, PxU32 frictionSize)
{
	const PxU8* PX_RESTRICT last = desc[0].constraint + getConstraintLength(desc[0]);

	//hopefully pointer aliasing doesn't bite.
	PxU8* PX_RESTRICT currPtr = desc[0].constraint;

	while((currPtr < last))
	{
		const SolverContactHeader4* PX_RESTRICT hdr = reinterpret_cast<SolverContactHeader4*>(currPtr);
		
		currPtr = const_cast<PxU8*>(reinterpret_cast<const PxU8*>(hdr + 1));

		const PxU32 numNormalConstr = hdr->numNormalConstr;
		const PxU32	numFrictionConstr = hdr->numFrictionConstr;

		currPtr += sizeof(Vec4V)*numNormalConstr;

		SolverContactBatchPointBase4* PX_RESTRICT contacts = reinterpret_cast<SolverContactBatchPointBase4*>(currPtr);
		currPtr += (numNormalConstr * contactSize);
		bool hasMaxImpulse = (hdr->flag & SolverContactHeader4::eHAS_MAX_IMPULSE) != 0;

		if(hasMaxImpulse)
			currPtr += sizeof(Vec4V) * numNormalConstr;

		currPtr += sizeof(Vec4V)*numFrictionConstr;

		SolverFrictionSharedData4* PX_RESTRICT fd = reinterpret_cast<SolverFrictionSharedData4*>(currPtr);
		if(numFrictionConstr)
			currPtr += sizeof(SolverFrictionSharedData4);
		PX_UNUSED(fd);

		SolverContactFrictionBase4* PX_RESTRICT frictions = reinterpret_cast<SolverContactFrictionBase4*>(currPtr);
		currPtr += (numFrictionConstr * frictionSize);

		for(PxU32 i=0;i<numNormalConstr;i++)
		{
			SolverContactBatchPointBase4& c = *contacts;
			contacts = reinterpret_cast<SolverContactBatchPointBase4*>((reinterpret_cast<PxU8*>(contacts)) + contactSize);
			c.biasedErr = V4Sub(c.biasedErr, c.scaledBias);
		}	

		for(PxU32 i=0;i<numFrictionConstr;i++)
		{
			SolverContactFrictionBase4& f = *frictions;
			frictions = reinterpret_cast<SolverContactFrictionBase4*>((reinterpret_cast<PxU8*>(frictions)) + frictionSize);
			f.scaledBias = f.targetVelocity;
		}
	}
}

void writeBackContact4_Block(const PxSolverConstraintDesc* PX_RESTRICT desc, SolverContext& cache,
							 const PxSolverBodyData** PX_RESTRICT bd0, const PxSolverBodyData** PX_RESTRICT bd1)
{
	const PxU8* PX_RESTRICT last = desc[0].constraint + getConstraintLength(desc[0]);

	//hopefully pointer aliasing doesn't bite.
	PxU8* PX_RESTRICT currPtr = desc[0].constraint;
	PxReal* PX_RESTRICT vForceWriteback0 = reinterpret_cast<PxReal*>(desc[0].writeBack);
	PxReal* PX_RESTRICT vForceWriteback1 = reinterpret_cast<PxReal*>(desc[1].writeBack);
	PxReal* PX_RESTRICT vForceWriteback2 = reinterpret_cast<PxReal*>(desc[2].writeBack);
	PxReal* PX_RESTRICT vForceWriteback3 = reinterpret_cast<PxReal*>(desc[3].writeBack);

	const PxU8 type = *desc[0].constraint;
	const PxU32 contactSize = type == DY_SC_TYPE_BLOCK_RB_CONTACT ? sizeof(SolverContactBatchPointDynamic4) : sizeof(SolverContactBatchPointBase4);
	const PxU32 frictionSize = type == DY_SC_TYPE_BLOCK_RB_CONTACT ? sizeof(SolverContactFrictionDynamic4) : sizeof(SolverContactFrictionBase4);


	Vec4V normalForce = V4Zero();


	//We'll need this.
	//const Vec4V vZero	= V4Zero();

	bool writeBackThresholds[4] = {false, false, false, false};

	while((currPtr < last))
	{
		SolverContactHeader4* PX_RESTRICT hdr = reinterpret_cast<SolverContactHeader4*>(currPtr);
		
		currPtr = reinterpret_cast<PxU8*>(hdr + 1);		

		const PxU32 numNormalConstr = hdr->numNormalConstr;
		const PxU32	numFrictionConstr = hdr->numFrictionConstr;

		Vec4V* PX_RESTRICT appliedForces = reinterpret_cast<Vec4V*>(currPtr);
		currPtr += sizeof(Vec4V)*numNormalConstr;

		//SolverContactBatchPointBase4* PX_RESTRICT contacts = (SolverContactBatchPointBase4*)currPtr;
		currPtr += (numNormalConstr * contactSize);

		bool hasMaxImpulse = (hdr->flag & SolverContactHeader4::eHAS_MAX_IMPULSE) != 0;

		if(hasMaxImpulse)
			currPtr += sizeof(Vec4V) * numNormalConstr;

		SolverFrictionSharedData4* PX_RESTRICT fd = reinterpret_cast<SolverFrictionSharedData4*>(currPtr);
		if(numFrictionConstr)
			currPtr += sizeof(SolverFrictionSharedData4);

		currPtr += sizeof(Vec4V)*numFrictionConstr;

		//SolverContactFrictionBase4* PX_RESTRICT frictions = (SolverContactFrictionBase4*)currPtr;
		currPtr += (numFrictionConstr * frictionSize);

		writeBackThresholds[0] = hdr->flags[0] & SolverContactHeader::eHAS_FORCE_THRESHOLDS;
		writeBackThresholds[1] = hdr->flags[1] & SolverContactHeader::eHAS_FORCE_THRESHOLDS;
		writeBackThresholds[2] = hdr->flags[2] & SolverContactHeader::eHAS_FORCE_THRESHOLDS;
		writeBackThresholds[3] = hdr->flags[3] & SolverContactHeader::eHAS_FORCE_THRESHOLDS;


		for(PxU32 i=0;i<numNormalConstr;i++)
		{
			//contacts = (SolverContactBatchPointBase4*)(((PxU8*)contacts) + contactSize);
			const FloatV appliedForce0 = V4GetX(appliedForces[i]);
			const FloatV appliedForce1 = V4GetY(appliedForces[i]);
			const FloatV appliedForce2 = V4GetZ(appliedForces[i]);
			const FloatV appliedForce3 = V4GetW(appliedForces[i]);

			normalForce = V4Add(normalForce, appliedForces[i]);

			if(vForceWriteback0 && i < hdr->numNormalConstr0)
				FStore(appliedForce0, vForceWriteback0++);
			if(vForceWriteback1 && i < hdr->numNormalConstr1)
				FStore(appliedForce1, vForceWriteback1++);
			if(vForceWriteback2 && i < hdr->numNormalConstr2)
				FStore(appliedForce2, vForceWriteback2++);
			if(vForceWriteback3 && i < hdr->numNormalConstr3)
				FStore(appliedForce3, vForceWriteback3++);
		}	

		if(numFrictionConstr)
		{
			PX_ALIGN(16, PxU32 broken[4]);
			BStoreA(fd->broken, broken);

			PxU8* frictionCounts = &hdr->numFrictionConstr0;

			for(PxU32 a = 0; a < 4; ++a)
			{
				if(frictionCounts[a] && broken[a])
					*fd->frictionBrokenWritebackByte[a] = 1;	// PT: bad L2 miss here
			}
		}
	}

	PX_ALIGN(16, PxReal nf[4]);
	V4StoreA(normalForce, nf);

	Sc::ShapeInteraction** shapeInteractions = reinterpret_cast<SolverContactHeader4*>(desc[0].constraint)->shapeInteraction;

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

static void solve1D4_Block(const PxSolverConstraintDesc* PX_RESTRICT desc, SolverContext& /*cache*/)
{

	PxSolverBody& b00 = *desc[0].bodyA;
	PxSolverBody& b01 = *desc[0].bodyB;

	PxSolverBody& b10 = *desc[1].bodyA;
	PxSolverBody& b11 = *desc[1].bodyB;

	PxSolverBody& b20 = *desc[2].bodyA;
	PxSolverBody& b21 = *desc[2].bodyB;

	PxSolverBody& b30 = *desc[3].bodyA;
	PxSolverBody& b31 = *desc[3].bodyB;

	PxU8* PX_RESTRICT bPtr = desc[0].constraint;
	//PxU32 length = desc.constraintLength;

	SolverConstraint1DHeader4* PX_RESTRICT  header = reinterpret_cast<SolverConstraint1DHeader4*>(bPtr);
	SolverConstraint1DDynamic4* PX_RESTRICT base = reinterpret_cast<SolverConstraint1DDynamic4*>(header+1);

	//const FloatV fZero = FZero();
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

	const Vec4V	invMass0D0 = header->invMass0D0;
	const Vec4V	invMass1D1 = header->invMass1D1;

	const Vec4V	angD0 = header->angD0;
	const Vec4V	angD1 = header->angD1;

	PxU32 maxConstraints = header->count;

	for(PxU32 a = 0; a < maxConstraints; ++a)
	{
		SolverConstraint1DDynamic4& c = *base;
		base++;

		Ps::prefetchLine(base);
		Ps::prefetchLine(base, 64);
		Ps::prefetchLine(base, 128);
		Ps::prefetchLine(base, 192);
		Ps::prefetchLine(base, 256);
		
		const Vec4V appliedForce = c.appliedForce;

		Vec4V linProj0(V4Mul(c.lin0X, linVel0T0));
		Vec4V linProj1(V4Mul(c.lin1X, linVel1T0));
		Vec4V angProj0(V4Mul(c.ang0X, angState0T0));
		Vec4V angProj1(V4Mul(c.ang1X, angState1T0));

		linProj0 = V4MulAdd(c.lin0Y, linVel0T1, linProj0);
		linProj1 = V4MulAdd(c.lin1Y, linVel1T1, linProj1);
		angProj0 = V4MulAdd(c.ang0Y, angState0T1, angProj0);
		angProj1 = V4MulAdd(c.ang1Y, angState1T1, angProj1);
		
		linProj0 = V4MulAdd(c.lin0Z, linVel0T2, linProj0);
		linProj1 = V4MulAdd(c.lin1Z, linVel1T2, linProj1);
		angProj0 = V4MulAdd(c.ang0Z, angState0T2, angProj0);
		angProj1 = V4MulAdd(c.ang1Z, angState1T2, angProj1);

		const Vec4V projectVel0 = V4Add(linProj0, angProj0);
		const Vec4V projectVel1 = V4Add(linProj1, angProj1);
		
		const Vec4V normalVel = V4Sub(projectVel0, projectVel1);

		const Vec4V unclampedForce = V4MulAdd(appliedForce, c.impulseMultiplier, V4MulAdd(normalVel, c.velMultiplier, c.constant));
		const Vec4V clampedForce = V4Max(c.minImpulse, V4Min(c.maxImpulse, unclampedForce));
		const Vec4V deltaF = V4Sub(clampedForce, appliedForce);
		c.appliedForce = clampedForce;

		const Vec4V deltaFInvMass0 = V4Mul(deltaF, invMass0D0);
		const Vec4V deltaFInvMass1 = V4Mul(deltaF, invMass1D1);

		const Vec4V angDeltaFInvMass0 = V4Mul(deltaF, angD0);
		const Vec4V angDeltaFInvMass1 = V4Mul(deltaF, angD1);

		linVel0T0 = V4MulAdd(c.lin0X, deltaFInvMass0, linVel0T0);
		linVel1T0 = V4NegMulSub(c.lin1X, deltaFInvMass1, linVel1T0);
		angState0T0 = V4MulAdd(c.ang0X, angDeltaFInvMass0, angState0T0);
		angState1T0 = V4NegMulSub(c.ang1X, angDeltaFInvMass1, angState1T0);

		linVel0T1 = V4MulAdd(c.lin0Y, deltaFInvMass0, linVel0T1);
		linVel1T1 = V4NegMulSub(c.lin1Y, deltaFInvMass1, linVel1T1);
		angState0T1 = V4MulAdd(c.ang0Y, angDeltaFInvMass0, angState0T1);
		angState1T1 = V4NegMulSub(c.ang1Y, angDeltaFInvMass1, angState1T1);

		linVel0T2 = V4MulAdd(c.lin0Z, deltaFInvMass0, linVel0T2);
		linVel1T2 = V4NegMulSub(c.lin1Z, deltaFInvMass1, linVel1T2);
		angState0T2 = V4MulAdd(c.ang0Z, angDeltaFInvMass0, angState0T2);
		angState1T2 = V4NegMulSub(c.ang1Z, angDeltaFInvMass1, angState1T2);
	}

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

static void conclude1D4_Block(const PxSolverConstraintDesc* PX_RESTRICT desc, SolverContext& /*cache*/)
{
	SolverConstraint1DHeader4* header = reinterpret_cast<SolverConstraint1DHeader4*>(desc[0].constraint);
	PxU8* base = desc[0].constraint + sizeof(SolverConstraint1DHeader4);
	PxU32 stride = header->type == DY_SC_TYPE_BLOCK_1D ? sizeof(SolverConstraint1DDynamic4) : sizeof(SolverConstraint1DBase4);

	for(PxU32 i=0; i<header->count; i++)
	{
		SolverConstraint1DBase4& c = *reinterpret_cast<SolverConstraint1DBase4*>(base);
		c.constant = c.unbiasedConstant;
		base += stride;
	}
	PX_ASSERT(desc[0].constraint + getConstraintLength(desc[0]) == base);
}

void writeBack1D4(const PxSolverConstraintDesc* PX_RESTRICT desc, SolverContext& /*cache*/,
							 const PxSolverBodyData** PX_RESTRICT /*bd0*/, const PxSolverBodyData** PX_RESTRICT /*bd1*/)
{
	ConstraintWriteback* writeback0 = reinterpret_cast<ConstraintWriteback*>(desc[0].writeBack);
	ConstraintWriteback* writeback1 = reinterpret_cast<ConstraintWriteback*>(desc[1].writeBack);
	ConstraintWriteback* writeback2 = reinterpret_cast<ConstraintWriteback*>(desc[2].writeBack);
	ConstraintWriteback* writeback3 = reinterpret_cast<ConstraintWriteback*>(desc[3].writeBack);

	if(writeback0 || writeback1 || writeback2 || writeback3)
	{
		SolverConstraint1DHeader4* header = reinterpret_cast<SolverConstraint1DHeader4*>(desc[0].constraint);
		PxU8* base = desc[0].constraint + sizeof(SolverConstraint1DHeader4);
		PxU32 stride = header->type == DY_SC_TYPE_BLOCK_1D ? sizeof(SolverConstraint1DDynamic4) : sizeof(SolverConstraint1DBase4);

		const Vec4V zero = V4Zero();
		Vec4V linX(zero), linY(zero), linZ(zero); 
		Vec4V angX(zero), angY(zero), angZ(zero); 

		for(PxU32 i=0; i<header->count; i++)
		{
			const SolverConstraint1DBase4* c = reinterpret_cast<SolverConstraint1DBase4*>(base);

			//Load in flags
			const VecI32V flags = I4LoadU(reinterpret_cast<const PxI32*>(&c->flags[0]));
			//Work out masks
			const VecI32V mask = I4Load(DY_SC_FLAG_OUTPUT_FORCE);

			const VecI32V masked = VecI32V_And(flags, mask);
			const BoolV isEq = VecI32V_IsEq(masked, mask);

			const Vec4V appliedForce = V4Sel(isEq, c->appliedForce, zero);

			linX = V4MulAdd(c->lin0X, appliedForce, linX);
			linY = V4MulAdd(c->lin0Y, appliedForce, linY);
			linZ = V4MulAdd(c->lin0Z, appliedForce, linZ);

			angX = V4MulAdd(c->ang0WritebackX, appliedForce, angX);
			angY = V4MulAdd(c->ang0WritebackY, appliedForce, angY);
			angZ = V4MulAdd(c->ang0WritebackZ, appliedForce, angZ);

			base += stride;
		}

		//We need to do the cross product now

		angX = V4Sub(angX, V4NegMulSub(header->body0WorkOffsetZ, linY, V4Mul(header->body0WorkOffsetY, linZ)));
		angY = V4Sub(angY, V4NegMulSub(header->body0WorkOffsetX, linZ, V4Mul(header->body0WorkOffsetZ, linX)));
		angZ = V4Sub(angZ, V4NegMulSub(header->body0WorkOffsetY, linX, V4Mul(header->body0WorkOffsetX, linY)));

		const Vec4V linLenSq = V4MulAdd(linZ, linZ, V4MulAdd(linY, linY, V4Mul(linX, linX)));
		const Vec4V angLenSq = V4MulAdd(angZ, angZ, V4MulAdd(angY, angY, V4Mul(angX, angX)));

		const Vec4V linLen = V4Sqrt(linLenSq);
		const Vec4V angLen = V4Sqrt(angLenSq);

		const BoolV broken = BOr(V4IsGrtr(linLen, header->linBreakImpulse), V4IsGrtr(angLen, header->angBreakImpulse));

		PX_ALIGN(16, PxU32 iBroken[4]);
		BStoreA(broken, iBroken);

		Vec4V lin0, lin1, lin2, lin3;
		Vec4V ang0, ang1, ang2, ang3;

		PX_TRANSPOSE_34_44(linX, linY, linZ, lin0, lin1, lin2, lin3);
		PX_TRANSPOSE_34_44(angX, angY, angZ, ang0, ang1, ang2, ang3);

		if(writeback0)
		{
			V3StoreU(Vec3V_From_Vec4V_WUndefined(lin0), writeback0->linearImpulse);
			V3StoreU(Vec3V_From_Vec4V_WUndefined(ang0), writeback0->angularImpulse);
			writeback0->broken = header->break0 ? PxU32(iBroken[0] != 0) : 0;
		}
		if(writeback1)
		{
			V3StoreU(Vec3V_From_Vec4V_WUndefined(lin1), writeback1->linearImpulse);
			V3StoreU(Vec3V_From_Vec4V_WUndefined(ang1), writeback1->angularImpulse);
			writeback1->broken = header->break1 ? PxU32(iBroken[1] != 0) : 0;
		}
		if(writeback2)
		{
			V3StoreU(Vec3V_From_Vec4V_WUndefined(lin2), writeback2->linearImpulse);
			V3StoreU(Vec3V_From_Vec4V_WUndefined(ang2), writeback2->angularImpulse);
			writeback2->broken = header->break2 ? PxU32(iBroken[2] != 0) : 0;
		}
		if(writeback3)
		{
			V3StoreU(Vec3V_From_Vec4V_WUndefined(lin3), writeback3->linearImpulse);
			V3StoreU(Vec3V_From_Vec4V_WUndefined(ang3), writeback3->angularImpulse);
			writeback3->broken = header->break3 ? PxU32(iBroken[3] != 0) : 0;
		}

		PX_ASSERT(desc[0].constraint + getConstraintLength(desc[0]) == base);
	}
}


void solveContactPreBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 /*constraintCount*/, SolverContext& cache)
{
	solveContact4_Block(desc, cache);
}

void solveContactPreBlock_Static(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32  /*constraintCount*/, SolverContext& cache)
{
	solveContact4_StaticBlock(desc, cache);
}

void solveContactPreBlock_Conclude(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32  /*constraintCount*/, SolverContext& cache)
{
	solveContact4_Block(desc, cache);
	concludeContact4_Block(desc, cache, sizeof(SolverContactBatchPointDynamic4), sizeof(SolverContactFrictionDynamic4));
}

void solveContactPreBlock_ConcludeStatic(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32  /*constraintCount*/, SolverContext& cache)
{
	solveContact4_StaticBlock(desc, cache);
	concludeContact4_Block(desc, cache, sizeof(SolverContactBatchPointBase4), sizeof(SolverContactFrictionBase4));
}

void solveContactPreBlock_WriteBack(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32  /*constraintCount*/, SolverContext& cache)
{
	solveContact4_Block(desc, cache);

	const PxSolverBodyData* bd0[4] = {	&cache.solverBodyArray[desc[0].bodyADataIndex], 
										&cache.solverBodyArray[desc[1].bodyADataIndex],
										&cache.solverBodyArray[desc[2].bodyADataIndex],
										&cache.solverBodyArray[desc[3].bodyADataIndex]};

	const PxSolverBodyData* bd1[4] = {	&cache.solverBodyArray[desc[0].bodyBDataIndex], 
										&cache.solverBodyArray[desc[1].bodyBDataIndex],
										&cache.solverBodyArray[desc[2].bodyBDataIndex],
										&cache.solverBodyArray[desc[3].bodyBDataIndex]};

	writeBackContact4_Block(desc, cache, bd0, bd1);

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

void solveContactPreBlock_WriteBackStatic(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32  /*constraintCount*/, SolverContext& cache)
{
	solveContact4_StaticBlock(desc, cache);
	const PxSolverBodyData* bd0[4] = {	&cache.solverBodyArray[desc[0].bodyADataIndex], 
										&cache.solverBodyArray[desc[1].bodyADataIndex],
										&cache.solverBodyArray[desc[2].bodyADataIndex],
										&cache.solverBodyArray[desc[3].bodyADataIndex]};

	const PxSolverBodyData* bd1[4] = {	&cache.solverBodyArray[desc[0].bodyBDataIndex], 
										&cache.solverBodyArray[desc[1].bodyBDataIndex],
										&cache.solverBodyArray[desc[2].bodyBDataIndex],
										&cache.solverBodyArray[desc[3].bodyBDataIndex]};

	writeBackContact4_Block(desc, cache, bd0, bd1);

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

void solve1D4_Block(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32  /*constraintCount*/, SolverContext& cache)
{
	solve1D4_Block(desc, cache);
}


void solve1D4Block_Conclude(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32  /*constraintCount*/, SolverContext& cache)
{
	solve1D4_Block(desc, cache);
	conclude1D4_Block(desc, cache);
}


void solve1D4Block_WriteBack(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32  /*constraintCount*/, SolverContext& cache)
{
	solve1D4_Block(desc, cache);

	const PxSolverBodyData* bd0[4] = {	&cache.solverBodyArray[desc[0].bodyADataIndex], 
										&cache.solverBodyArray[desc[1].bodyADataIndex],
										&cache.solverBodyArray[desc[2].bodyADataIndex],
										&cache.solverBodyArray[desc[3].bodyADataIndex]};

	const PxSolverBodyData* bd1[4] = {	&cache.solverBodyArray[desc[0].bodyBDataIndex], 
										&cache.solverBodyArray[desc[1].bodyBDataIndex],
										&cache.solverBodyArray[desc[2].bodyBDataIndex],
										&cache.solverBodyArray[desc[3].bodyBDataIndex]};

	writeBack1D4(desc, cache, bd0, bd1);
}

void writeBack1D4Block(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32  /*constraintCount*/, SolverContext& cache)
{
	const PxSolverBodyData* bd0[4] = {	&cache.solverBodyArray[desc[0].bodyADataIndex], 
										&cache.solverBodyArray[desc[1].bodyADataIndex],
										&cache.solverBodyArray[desc[2].bodyADataIndex],
										&cache.solverBodyArray[desc[3].bodyADataIndex]};

	const PxSolverBodyData* bd1[4] = {	&cache.solverBodyArray[desc[0].bodyBDataIndex], 
										&cache.solverBodyArray[desc[1].bodyBDataIndex],
										&cache.solverBodyArray[desc[2].bodyBDataIndex],
										&cache.solverBodyArray[desc[3].bodyBDataIndex]};

	writeBack1D4(desc, cache, bd0, bd1);
}

}

}

