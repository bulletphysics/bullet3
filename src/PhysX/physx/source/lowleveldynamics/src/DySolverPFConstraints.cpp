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

#include "CmPhysXCommon.h"
#include "DySolverBody.h"
#include "DySolverContact.h"
#include "DySolverContactPF.h"
#include "DySolverConstraint1D.h"
#include "DySolverConstraintDesc.h"
#include "DyThresholdTable.h"
#include "DySolverContext.h"
#include "PsUtilities.h"
#include "DyConstraint.h"
#include "PsAtomic.h"
#include "DyThresholdTable.h"
#include "DySolverConstraintsShared.h"

namespace physx
{

namespace Dy
{

void solveContactCoulomb(const PxSolverConstraintDesc& desc, SolverContext& /*cache*/)
{
	PxSolverBody& b0 = *desc.bodyA;
	PxSolverBody& b1 = *desc.bodyB;

	Vec3V linVel0 = V3LoadA(b0.linearVelocity);
	Vec3V linVel1 = V3LoadA(b1.linearVelocity);
	Vec3V angState0 = V3LoadA(b0.angularState);
	Vec3V angState1 = V3LoadA(b1.angularState);

	SolverContactCoulombHeader* PX_RESTRICT firstHeader = reinterpret_cast<SolverContactCoulombHeader*>(desc.constraint);
	const PxU8* PX_RESTRICT last = desc.constraint + firstHeader->frictionOffset;//getConstraintLength(desc);

	//hopefully pointer aliasing doesn't bite.
	PxU8* PX_RESTRICT currPtr = desc.constraint;

	
	//const FloatV zero = FZero();

	while(currPtr < last)
	{
		SolverContactCoulombHeader* PX_RESTRICT hdr = reinterpret_cast<SolverContactCoulombHeader*>(currPtr);
		currPtr += sizeof(SolverContactCoulombHeader);

		const PxU32 numNormalConstr = hdr->numNormalConstr;

		const Vec3V normal = hdr->getNormal();
		const FloatV invMassDom0 = FLoad(hdr->dominance0);
		const FloatV invMassDom1 = FLoad(hdr->dominance1);
		const FloatV angD0 = FLoad(hdr->angDom0);
		const FloatV angD1 = FLoad(hdr->angDom1);
		


		SolverContactPoint* PX_RESTRICT contacts = reinterpret_cast<SolverContactPoint*>(currPtr);
		currPtr += numNormalConstr * sizeof(SolverContactPoint);

		PxF32* appliedImpulse = reinterpret_cast<PxF32*> ((reinterpret_cast<PxU8*>(hdr)) + hdr->frictionOffset + sizeof(SolverFrictionHeader));
		Ps::prefetchLine(appliedImpulse);

		solveDynamicContacts(contacts, numNormalConstr, normal, invMassDom0, invMassDom1, 
			angD0, angD1, linVel0, angState0, linVel1, angState1, appliedImpulse); 
	}

	// Write back
	V3StoreA(linVel0, b0.linearVelocity);
	V3StoreA(linVel1, b1.linearVelocity);
	V3StoreA(angState0, b0.angularState);
	V3StoreA(angState1, b1.angularState);

	PX_ASSERT(currPtr == last);
}

void solveFriction(const PxSolverConstraintDesc& desc, SolverContext& /*cache*/)
{
	PxSolverBody& b0 = *desc.bodyA;
	PxSolverBody& b1 = *desc.bodyB;

	Vec3V linVel0 = V3LoadA(b0.linearVelocity);
	Vec3V linVel1 = V3LoadA(b1.linearVelocity);
	Vec3V angState0 = V3LoadA(b0.angularState);
	Vec3V angState1 = V3LoadA(b1.angularState);

	PxU8* PX_RESTRICT ptr = desc.constraint;
	PxU8* PX_RESTRICT currPtr = ptr;

	const PxU8* PX_RESTRICT last = ptr + getConstraintLength(desc);


	while(currPtr < last)
	{
		const SolverFrictionHeader* PX_RESTRICT frictionHeader = reinterpret_cast<SolverFrictionHeader*>(currPtr);
		currPtr += sizeof(SolverFrictionHeader);
		PxF32* appliedImpulse = reinterpret_cast<PxF32*>(currPtr);
		currPtr += frictionHeader->getAppliedForcePaddingSize();

		SolverContactFriction* PX_RESTRICT frictions = reinterpret_cast<SolverContactFriction*>(currPtr);
		const PxU32 numFrictionConstr = frictionHeader->numFrictionConstr;
		const PxU32 numNormalConstr = frictionHeader->numNormalConstr;

		const PxU32 numFrictionPerPoint = numFrictionConstr/numNormalConstr;

		currPtr += numFrictionConstr * sizeof(SolverContactFriction);
		const FloatV staticFriction = frictionHeader->getStaticFriction();

		const FloatV invMass0D0 = FLoad(frictionHeader->invMass0D0);
		const FloatV invMass1D1 = FLoad(frictionHeader->invMass1D1);

		
		const FloatV angD0 = FLoad(frictionHeader->angDom0);
		const FloatV angD1 = FLoad(frictionHeader->angDom1);

		for(PxU32 i=0, j = 0;i<numFrictionConstr;j++)
		{
			for(PxU32 p = 0; p < numFrictionPerPoint; p++, i++)
			{
		
				SolverContactFriction& f = frictions[i];
				Ps::prefetchLine(&frictions[i], 128);

				const Vec3V t0 = Vec3V_From_Vec4V(f.normalXYZ_appliedForceW);
				const Vec3V raXt0 = Vec3V_From_Vec4V(f.raXnXYZ_velMultiplierW);
				const Vec3V rbXt0 = Vec3V_From_Vec4V(f.rbXnXYZ_biasW);

				const FloatV appliedForce = V4GetW(f.normalXYZ_appliedForceW);
				const FloatV velMultiplier = V4GetW(f.raXnXYZ_velMultiplierW);

				const FloatV targetVel = FLoad(f.targetVel);

				const FloatV normalImpulse = FLoad(appliedImpulse[j]);
				const FloatV maxFriction = FMul(staticFriction, normalImpulse);
				const FloatV nMaxFriction = FNeg(maxFriction);

				//Compute the normal velocity of the constraint.

				const FloatV t0Vel1 = V3Dot(t0, linVel0);
				const FloatV t0Vel2 = V3Dot(raXt0, angState0);
				const FloatV t0Vel3 = V3Dot(t0, linVel1);
				const FloatV t0Vel4 = V3Dot(rbXt0, angState1);


				const FloatV t0Vel = FSub(FAdd(t0Vel1, t0Vel2), FAdd(t0Vel3, t0Vel4));

				const Vec3V delLinVel0 = V3Scale(t0, invMass0D0);
				const Vec3V delLinVel1 = V3Scale(t0, invMass1D1);

				// still lots to do here: using loop pipelining we can interweave this code with the
				// above - the code here has a lot of stalls that we would thereby eliminate
				
				const FloatV tmp = FNegScaleSub(targetVel,velMultiplier,appliedForce);
				FloatV newForce = FScaleAdd(t0Vel, velMultiplier, tmp);
				newForce = FClamp(newForce, nMaxFriction, maxFriction);
				FloatV deltaF = FSub(newForce, appliedForce);

				linVel0 = V3ScaleAdd(delLinVel0, deltaF, linVel0);
				linVel1 = V3NegScaleSub(delLinVel1, deltaF, linVel1);
				angState0 = V3ScaleAdd(raXt0, FMul(deltaF, angD0), angState0);
				angState1 = V3NegScaleSub(rbXt0, FMul(deltaF, angD1), angState1);

				f.setAppliedForce(newForce);
			}
		}
	}

	// Write back
	V3StoreA(linVel0, b0.linearVelocity);
	V3StoreA(linVel1, b1.linearVelocity);
	V3StoreA(angState0, b0.angularState);
	V3StoreA(angState1, b1.angularState);


	PX_ASSERT(currPtr == last);
}

void solveContactCoulomb_BStatic(const PxSolverConstraintDesc& desc, SolverContext& /*cache*/)
{
	PxSolverBody& b0 = *desc.bodyA;


	Vec3V linVel0 = V3LoadA(b0.linearVelocity);
	Vec3V angState0 = V3LoadA(b0.angularState);

	SolverContactCoulombHeader* firstHeader = reinterpret_cast<SolverContactCoulombHeader*>(desc.constraint);
	const PxU8* PX_RESTRICT last = desc.constraint + firstHeader->frictionOffset;//getConstraintLength(desc);

	//hopefully pointer aliasing doesn't bite.
	PxU8* PX_RESTRICT currPtr = desc.constraint;

	//const FloatV zero = FZero();

	while(currPtr < last)
	{
		SolverContactCoulombHeader* PX_RESTRICT hdr = reinterpret_cast<SolverContactCoulombHeader*>(currPtr);
		currPtr += sizeof(SolverContactCoulombHeader);

		const PxU32 numNormalConstr = hdr->numNormalConstr;

		SolverContactPoint* PX_RESTRICT contacts = reinterpret_cast<SolverContactPoint*>(currPtr);
		Ps::prefetchLine(contacts);
		currPtr += numNormalConstr * sizeof(SolverContactPoint);

		PxF32* appliedImpulse = reinterpret_cast<PxF32*> ((reinterpret_cast<PxU8*>(hdr)) + hdr->frictionOffset + sizeof(SolverFrictionHeader));
		Ps::prefetchLine(appliedImpulse);

		const Vec3V normal = hdr->getNormal();

		const FloatV invMassDom0 = FLoad(hdr->dominance0);

		const FloatV angD0 = FLoad(hdr->angDom0);
		
		solveStaticContacts(contacts, numNormalConstr, normal, invMassDom0, 
			angD0, linVel0, angState0, appliedImpulse); 
	}

	// Write back
	V3StoreA(linVel0, b0.linearVelocity);
	V3StoreA(angState0, b0.angularState);

	PX_ASSERT(currPtr == last);
}

void solveFriction_BStatic(const PxSolverConstraintDesc& desc, SolverContext& /*cache*/)
{
	PxSolverBody& b0 = *desc.bodyA;

	Vec3V linVel0 = V3LoadA(b0.linearVelocity);
	Vec3V angState0 = V3LoadA(b0.angularState);

	PxU8* PX_RESTRICT currPtr = desc.constraint;

	const PxU8* PX_RESTRICT last = currPtr + getConstraintLength(desc);

	while(currPtr < last)
	{

		const SolverFrictionHeader* PX_RESTRICT frictionHeader = reinterpret_cast<SolverFrictionHeader*>(currPtr);
		const PxU32 numFrictionConstr = frictionHeader->numFrictionConstr;
		const PxU32 numNormalConstr = frictionHeader->numNormalConstr;
		const PxU32 numFrictionPerPoint = numFrictionConstr/numNormalConstr;
		currPtr +=sizeof(SolverFrictionHeader);
		PxF32* appliedImpulse = reinterpret_cast<PxF32*>(currPtr);
		currPtr +=frictionHeader->getAppliedForcePaddingSize();

		SolverContactFriction* PX_RESTRICT frictions = reinterpret_cast<SolverContactFriction*>(currPtr);
		currPtr += numFrictionConstr * sizeof(SolverContactFriction);

		const FloatV invMass0 = FLoad(frictionHeader->invMass0D0);
		const FloatV angD0 = FLoad(frictionHeader->angDom0);
		//const FloatV angD1 = FLoad(frictionHeader->angDom1);


		const FloatV staticFriction = frictionHeader->getStaticFriction();

		for(PxU32 i=0, j = 0;i<numFrictionConstr;j++)
		{
			for(PxU32 p = 0; p < numFrictionPerPoint; p++, i++)
			{
				SolverContactFriction& f = frictions[i];
				Ps::prefetchLine(&frictions[i+1]);

				const Vec3V t0 = Vec3V_From_Vec4V(f.normalXYZ_appliedForceW);
				const Vec3V raXt0 = Vec3V_From_Vec4V(f.raXnXYZ_velMultiplierW);

				const FloatV appliedForce = V4GetW(f.normalXYZ_appliedForceW);
				const FloatV velMultiplier = V4GetW(f.raXnXYZ_velMultiplierW);

				const FloatV targetVel = FLoad(f.targetVel);
				
				//const FloatV normalImpulse = contacts[f.contactIndex].getAppliedForce();
				const FloatV normalImpulse = FLoad(appliedImpulse[j]);
				const FloatV maxFriction = FMul(staticFriction, normalImpulse);
				const FloatV nMaxFriction = FNeg(maxFriction);

				//Compute the normal velocity of the constraint.

				const FloatV t0Vel1 = V3Dot(t0, linVel0);
				const FloatV t0Vel2 = V3Dot(raXt0, angState0);

				const FloatV t0Vel = FAdd(t0Vel1, t0Vel2);

				const Vec3V delangState0 = V3Scale(raXt0, angD0);
				const Vec3V delLinVel0 = V3Scale(t0, invMass0);

				// still lots to do here: using loop pipelining we can interweave this code with the
				// above - the code here has a lot of stalls that we would thereby eliminate

				const FloatV tmp = FNegScaleSub(targetVel,velMultiplier,appliedForce);
				FloatV newForce = FScaleAdd(t0Vel, velMultiplier, tmp);
				newForce = FClamp(newForce, nMaxFriction, maxFriction);
				const FloatV deltaF = FSub(newForce, appliedForce);

				linVel0 = V3ScaleAdd(delLinVel0, deltaF, linVel0);
				angState0 = V3ScaleAdd(delangState0, deltaF, angState0);

				f.setAppliedForce(newForce);
			}
		}
	}

	// Write back
	V3StoreA(linVel0, b0.linearVelocity);
	V3StoreA(angState0, b0.angularState);

	PX_ASSERT(currPtr == last);
}


void concludeContactCoulomb(const PxSolverConstraintDesc& desc, SolverContext& /*cache*/)
{
	PxU8* PX_RESTRICT cPtr = desc.constraint;

	const SolverContactCoulombHeader* PX_RESTRICT firstHeader = reinterpret_cast<const SolverContactCoulombHeader*>(cPtr);
	PxU8* PX_RESTRICT last = desc.constraint + firstHeader->frictionOffset;//getConstraintLength(desc);
	while(cPtr < last)
	{
		const SolverContactCoulombHeader* PX_RESTRICT hdr = reinterpret_cast<const SolverContactCoulombHeader*>(cPtr);
		cPtr += sizeof(SolverContactCoulombHeader);

		const PxU32 numNormalConstr = hdr->numNormalConstr;
		
		//if(cPtr < last)
		//Ps::prefetchLine(cPtr, 512);
		Ps::prefetchLine(cPtr,128);
		Ps::prefetchLine(cPtr,256);
		Ps::prefetchLine(cPtr,384);

		const PxU32 pointStride = hdr->type == DY_SC_TYPE_EXT_CONTACT ? sizeof(SolverContactPointExt)
																	   : sizeof(SolverContactPoint);
		for(PxU32 i=0;i<numNormalConstr;i++)
		{
			SolverContactPoint *c = reinterpret_cast<SolverContactPoint*>(cPtr);
			cPtr += pointStride;
			//c->scaledBias = PxMin(c->scaledBias, 0.f);
			c->biasedErr = c->unbiasedErr;
		}
	}
	PX_ASSERT(cPtr == last);
}

void  writeBackContactCoulomb(const PxSolverConstraintDesc& desc, SolverContext& cache,
					  PxSolverBodyData& bd0, PxSolverBodyData& bd1)
{

	PxReal normalForce = 0.f;

	PxU8* PX_RESTRICT cPtr = desc.constraint;
	PxReal* PX_RESTRICT vForceWriteback = reinterpret_cast<PxReal*>(desc.writeBack);
	const SolverContactCoulombHeader* PX_RESTRICT firstHeader = reinterpret_cast<const SolverContactCoulombHeader*>(cPtr);
	PxU8* PX_RESTRICT last = desc.constraint + firstHeader->frictionOffset;

	const PxU32 pointStride = firstHeader->type == DY_SC_TYPE_EXT_CONTACT ? sizeof(SolverContactPointExt)
																	   : sizeof(SolverContactPoint);

	bool hasForceThresholds = false;
	while(cPtr < last)
	{
		const SolverContactCoulombHeader* PX_RESTRICT hdr = reinterpret_cast<const SolverContactCoulombHeader*>(cPtr);
		cPtr += sizeof(SolverContactCoulombHeader);

		PxF32* appliedImpulse = reinterpret_cast<PxF32*> (const_cast<PxU8*>((reinterpret_cast<const PxU8*>(hdr)) + hdr->frictionOffset + sizeof(SolverFrictionHeader)));

		hasForceThresholds = hdr->flags & SolverContactHeader::eHAS_FORCE_THRESHOLDS;

		const PxU32 numNormalConstr = hdr->numNormalConstr;

		Ps::prefetchLine(cPtr, 256);
		Ps::prefetchLine(cPtr, 384);

		if(vForceWriteback!=NULL)
		{
			for(PxU32 i=0; i<numNormalConstr; i++)
			{
				PxF32 imp = appliedImpulse[i];
				*vForceWriteback = imp;
				vForceWriteback++;
				normalForce += imp;
			}
		}
		cPtr += numNormalConstr * pointStride;
	}
	PX_ASSERT(cPtr == last);

	if(hasForceThresholds && desc.linkIndexA == PxSolverConstraintDesc::NO_LINK && desc.linkIndexB == PxSolverConstraintDesc::NO_LINK &&
		normalForce !=0 && (bd0.reportThreshold < PX_MAX_REAL  || bd1.reportThreshold < PX_MAX_REAL))
	{
		ThresholdStreamElement elt;
		elt.normalForce = normalForce;
		elt.threshold = PxMin<float>(bd0.reportThreshold, bd1.reportThreshold);
		elt.nodeIndexA = IG::NodeIndex(bd0.nodeIndex);
		elt.nodeIndexB = IG::NodeIndex(bd1.nodeIndex);
		elt.shapeInteraction = (reinterpret_cast<SolverContactCoulombHeader*>(desc.constraint))->shapeInteraction;
		Ps::order(elt.nodeIndexA, elt.nodeIndexB);
		PX_ASSERT(elt.nodeIndexA < elt.nodeIndexB);

		PX_ASSERT(cache.mThresholdStreamIndex<cache.mThresholdStreamLength);
		cache.mThresholdStream[cache.mThresholdStreamIndex++] = elt;
	}

}


void solveFrictionBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 0; a < constraintCount; ++a)
	{
		solveFriction(desc[a], cache);
	}
}


void solveFrictionBlockWriteBack(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 0; a < constraintCount; ++a)
	{
		solveFriction(desc[a], cache);
	}
}

void solveFriction_BStaticBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 0; a < constraintCount; ++a)
	{
		solveFriction_BStatic(desc[a], cache);
	}
}


void solveFriction_BStaticConcludeBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 0; a < constraintCount; ++a)
	{
		solveFriction_BStatic(desc[a], cache);
	}
}

void solveFriction_BStaticBlockWriteBack(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 0; a < constraintCount; ++a)
	{
		solveFriction_BStatic(desc[a], cache);
	}
}


void solveContactCoulombBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 0; a < constraintCount; ++a)
	{
		solveContactCoulomb(desc[a], cache);
	}
}

void solveContactCoulombConcludeBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 0; a < constraintCount; ++a)
	{
		solveContactCoulomb(desc[a], cache);
		concludeContactCoulomb(desc[a], cache);
	}
}

void solveContactCoulombBlockWriteBack(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 0; a < constraintCount; ++a)
	{
		PxSolverBodyData& bd0 = cache.solverBodyArray[desc[a].bodyADataIndex];
		PxSolverBodyData& bd1 = cache.solverBodyArray[desc[a].bodyBDataIndex];
		solveContactCoulomb(desc[a], cache);
		writeBackContactCoulomb(desc[a], cache, bd0, bd1);
	}

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

void solveContactCoulomb_BStaticBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 0; a < constraintCount; ++a)
	{
		solveContactCoulomb_BStatic(desc[a], cache);
	}
}

void solveContactCoulomb_BStaticConcludeBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 0; a < constraintCount; ++a)
	{
		solveContactCoulomb_BStatic(desc[a], cache);
		concludeContactCoulomb(desc[a], cache);
	}
}

void solveContactCoulomb_BStaticBlockWriteBack(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 0; a < constraintCount; ++a)
	{
		PxSolverBodyData& bd0 = cache.solverBodyArray[desc[a].bodyADataIndex];
		PxSolverBodyData& bd1 = cache.solverBodyArray[desc[a].bodyBDataIndex];
		solveContactCoulomb_BStatic(desc[a], cache);
		writeBackContactCoulomb(desc[a], cache, bd0, bd1);
	}

	if(cache.mThresholdStreamIndex > (cache.mThresholdStreamLength - 4))
	{
		//Not enough space to write 4 more thresholds back!
		//Write back to global buffer
		PxI32 threshIndex = physx::shdfnd::atomicAdd(cache.mSharedOutThresholdPairs, PxI32(cache.mThresholdStreamIndex)) - PxI32(cache.mThresholdStreamIndex);
		for(PxU32 a = 0; a < cache.mThresholdStreamIndex; ++a)
		{
			cache.mSharedThresholdStream[a + threshIndex] = cache.mThresholdStream[a];
		}
		cache.mThresholdStreamIndex = 0;
	}
}

void solveExtContactCoulomb(const PxSolverConstraintDesc& desc, SolverContext& cache)
{
	//We'll need this.
//	const FloatV zero	= FZero();
//	const FloatV one	= FOne();

	Vec3V linVel0, angVel0, linVel1, angVel1;

	if(desc.linkIndexA == PxSolverConstraintDesc::NO_LINK)
	{
		linVel0 = V3LoadA(desc.bodyA->linearVelocity);
		angVel0 = V3LoadA(desc.bodyA->angularState);
	}
	else
	{
		Cm::SpatialVectorV v = desc.articulationA->pxcFsGetVelocity(desc.linkIndexA);
		linVel0 = v.linear;
		angVel0 = v.angular;
	}

	if(desc.linkIndexB == PxSolverConstraintDesc::NO_LINK)
	{
		linVel1 = V3LoadA(desc.bodyB->linearVelocity);
		angVel1 = V3LoadA(desc.bodyB->angularState);
	}
	else
	{
		Cm::SpatialVectorV v = desc.articulationB->pxcFsGetVelocity(desc.linkIndexB);
		linVel1 = v.linear;
		angVel1 = v.angular;
	}

	//const PxU8* PX_RESTRICT last = desc.constraint + desc.constraintLengthOver16*16;

	PxU8* PX_RESTRICT currPtr = desc.constraint;

	const SolverContactCoulombHeader* PX_RESTRICT firstHeader = reinterpret_cast<SolverContactCoulombHeader*>(currPtr);

	const PxU8* PX_RESTRICT last = desc.constraint + firstHeader->frictionOffset;

	//hopefully pointer aliasing doesn't bite.

	Vec3V linImpulse0 = V3Zero(), linImpulse1 = V3Zero(), angImpulse0 = V3Zero(), angImpulse1 = V3Zero();

	while(currPtr < last)
	{
		const SolverContactCoulombHeader* PX_RESTRICT hdr = reinterpret_cast<SolverContactCoulombHeader*>(currPtr);
		currPtr += sizeof(SolverContactCoulombHeader);

		const PxU32 numNormalConstr = hdr->numNormalConstr;

		PxF32* appliedImpulse = reinterpret_cast<PxF32*>(const_cast<PxU8*>(((reinterpret_cast<const PxU8*>(hdr)) + hdr->frictionOffset + sizeof(SolverFrictionHeader))));
		Ps::prefetchLine(appliedImpulse);
		
		SolverContactPointExt* PX_RESTRICT contacts = reinterpret_cast<SolverContactPointExt*>(currPtr);
		Ps::prefetchLine(contacts);
		currPtr += numNormalConstr * sizeof(SolverContactPointExt);

		Vec3V li0 = V3Zero(), li1 = V3Zero(), ai0 = V3Zero(), ai1 = V3Zero();

		const Vec3V normal = hdr->getNormal();

		solveExtContacts(contacts, numNormalConstr, normal, linVel0, angVel0, linVel1, angVel1, li0, ai0, li1, ai1, appliedImpulse);

		linImpulse0 = V3ScaleAdd(li0, FLoad(hdr->dominance0), linImpulse0);		
		angImpulse0 = V3ScaleAdd(ai0, FLoad(hdr->angDom0), angImpulse0);
		linImpulse1 = V3NegScaleSub(li1, FLoad(hdr->dominance1), linImpulse1);	
		angImpulse1 = V3NegScaleSub(ai1, FLoad(hdr->angDom1), angImpulse1);
	}

	if(desc.linkIndexA == PxSolverConstraintDesc::NO_LINK)
	{
		V3StoreA(linVel0, desc.bodyA->linearVelocity);
		V3StoreA(angVel0, desc.bodyA->angularState);
	}
	else
	{
		Cm::SpatialVectorF Z[64];
		Cm::SpatialVectorF deltaV[64];
		desc.articulationA->pxcFsApplyImpulse(desc.linkIndexA, linImpulse0, 
			angImpulse0, Z, deltaV);
	}

	if(desc.linkIndexB == PxSolverConstraintDesc::NO_LINK)
	{
		V3StoreA(linVel1, desc.bodyB->linearVelocity);
		V3StoreA(angVel1, desc.bodyB->angularState);
	}
	else
	{
		desc.articulationB->pxcFsApplyImpulse(desc.linkIndexB, linImpulse1,
			angImpulse1, cache.Z, cache.deltaV);
	}

	PX_ASSERT(currPtr == last);
}

void solveExtFriction(const PxSolverConstraintDesc& desc, SolverContext& cache)
{
	Vec3V linVel0, angVel0, linVel1, angVel1;

	if(desc.linkIndexA == PxSolverConstraintDesc::NO_LINK)
	{
		linVel0 = V3LoadA(desc.bodyA->linearVelocity);
		angVel0 = V3LoadA(desc.bodyA->angularState);
	}
	else
	{
		Cm::SpatialVectorV v = desc.articulationA->pxcFsGetVelocity(desc.linkIndexA);
		linVel0 = v.linear;
		angVel0 = v.angular;
	}

	if(desc.linkIndexB == PxSolverConstraintDesc::NO_LINK)
	{
		linVel1 = V3LoadA(desc.bodyB->linearVelocity);
		angVel1 = V3LoadA(desc.bodyB->angularState);
	}
	else
	{
		Cm::SpatialVectorV v = desc.articulationB->pxcFsGetVelocity(desc.linkIndexB);
		linVel1 = v.linear;
		angVel1 = v.angular;
	}


	//hopefully pointer aliasing doesn't bite.
	PxU8* PX_RESTRICT currPtr = desc.constraint;

	const PxU8* PX_RESTRICT last = currPtr + desc.constraintLengthOver16*16;

	Vec3V linImpulse0 = V3Zero(), linImpulse1 = V3Zero(), angImpulse0 = V3Zero(), angImpulse1 = V3Zero();

	while(currPtr < last)
	{
	
		const SolverFrictionHeader* PX_RESTRICT frictionHeader = reinterpret_cast<SolverFrictionHeader*>(currPtr);
		currPtr += sizeof(SolverFrictionHeader);
		PxF32* appliedImpulse = reinterpret_cast<PxF32*>(currPtr);
		currPtr += frictionHeader->getAppliedForcePaddingSize();

		SolverContactFrictionExt* PX_RESTRICT frictions = reinterpret_cast<SolverContactFrictionExt*>(currPtr);
		const PxU32 numFrictionConstr = frictionHeader->numFrictionConstr;

		currPtr += numFrictionConstr * sizeof(SolverContactFrictionExt);
		const FloatV staticFriction = frictionHeader->getStaticFriction();
	
	
		Vec3V li0 = V3Zero(), li1 = V3Zero(), ai0 = V3Zero(), ai1 = V3Zero();

		PxU32 numNormalConstr = frictionHeader->numNormalConstr;
		PxU32 nbFrictionsPerPoint = numFrictionConstr/numNormalConstr;




		for(PxU32 i = 0, j = 0; i < numFrictionConstr; j++)
		{
			for(PxU32 p=0;p<nbFrictionsPerPoint;p++, i++)
			{
				SolverContactFrictionExt& f = frictions[i];
				Ps::prefetchLine(&frictions[i+1]);
			

				const Vec3V t0 = Vec3V_From_Vec4V(f.normalXYZ_appliedForceW);
				const Vec3V raXt0 = Vec3V_From_Vec4V(f.raXnXYZ_velMultiplierW);
				const Vec3V rbXt0 = Vec3V_From_Vec4V(f.rbXnXYZ_biasW);

				const FloatV appliedForce = V4GetW(f.normalXYZ_appliedForceW);
				const FloatV velMultiplier = V4GetW(f.raXnXYZ_velMultiplierW);
				const FloatV targetVel = FLoad(f.targetVel);

				const FloatV normalImpulse = FLoad(appliedImpulse[j]);//contacts[f.contactIndex].getAppliedForce();
				const FloatV maxFriction = FMul(staticFriction, normalImpulse);
				const FloatV nMaxFriction = FNeg(maxFriction);

				//Compute the normal velocity of the constraint.

				Vec3V rVel = V3MulAdd(linVel0, t0, V3Mul(angVel0, raXt0));
				rVel = V3Sub(rVel, V3MulAdd(linVel1, t0, V3Mul(angVel1, rbXt0)));
				const FloatV t0Vel = FAdd(V3SumElems(rVel), targetVel);

				FloatV deltaF = FNeg(FMul(t0Vel, velMultiplier));
				FloatV newForce = FAdd(appliedForce, deltaF);
				newForce = FClamp(newForce, nMaxFriction, maxFriction);
				deltaF = FSub(newForce, appliedForce);

				linVel0 = V3ScaleAdd(f.linDeltaVA, deltaF, linVel0);	
				angVel0 = V3ScaleAdd(f.angDeltaVA, deltaF, angVel0);
				linVel1 = V3ScaleAdd(f.linDeltaVB, deltaF, linVel1);	
				angVel1 = V3ScaleAdd(f.angDeltaVB, deltaF, angVel1);

				li0 = V3ScaleAdd(t0, deltaF, li0);	ai0 = V3ScaleAdd(raXt0, deltaF, ai0);
				li1 = V3ScaleAdd(t0, deltaF, li1);	ai1 = V3ScaleAdd(rbXt0, deltaF, ai1);

				f.setAppliedForce(newForce);
			}
		}


		linImpulse0 = V3ScaleAdd(li0, FLoad(frictionHeader->invMass0D0), linImpulse0);		
		angImpulse0 = V3ScaleAdd(ai0, FLoad(frictionHeader->angDom0), angImpulse0);
		linImpulse1 = V3NegScaleSub(li1, FLoad(frictionHeader->invMass1D1), linImpulse1);	
		angImpulse1 = V3NegScaleSub(ai1, FLoad(frictionHeader->angDom1), angImpulse1);
	}

	if(desc.linkIndexA == PxSolverConstraintDesc::NO_LINK)
	{
		V3StoreA(linVel0, desc.bodyA->linearVelocity);
		V3StoreA(angVel0, desc.bodyA->angularState);
	}
	else
	{
		desc.articulationA->pxcFsApplyImpulse(desc.linkIndexA, 
			linImpulse0, angImpulse0, cache.Z, cache.deltaV);
	}

	if(desc.linkIndexB == PxSolverConstraintDesc::NO_LINK)
	{
		V3StoreA(linVel1, desc.bodyB->linearVelocity);
		V3StoreA(angVel1, desc.bodyB->angularState);
	}
	else
	{
		desc.articulationB->pxcFsApplyImpulse(desc.linkIndexB, 
			linImpulse1, angImpulse1, cache.Z, cache.deltaV);
	}

	PX_ASSERT(currPtr == last);

}

void solveExtFrictionBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 0; a < constraintCount; ++a)
	{
		solveExtFriction(desc[a], cache);
	}
}

void solveExtFrictionConcludeBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 0; a < constraintCount; ++a)
	{
		solveExtFriction(desc[a], cache);
	}
}

void solveExtFrictionBlockWriteBack(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 0; a < constraintCount; ++a)
	{
		solveExtFriction(desc[a], cache);
	}
}


void solveConcludeExtContactCoulomb		(const PxSolverConstraintDesc& desc, SolverContext& cache)
{
	solveExtContactCoulomb(desc, cache);
	concludeContactCoulomb(desc, cache);
}

void solveExtContactCoulombBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 0; a < constraintCount; ++a)
	{
		solveExtContactCoulomb(desc[a], cache);
	}
}

void solveExtContactCoulombConcludeBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 0; a < constraintCount; ++a)
	{
		solveExtContactCoulomb(desc[a], cache);
		concludeContactCoulomb(desc[a], cache);
	}
}

void solveExtContactCoulombBlockWriteBack(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 0; a < constraintCount; ++a)
	{
		PxSolverBodyData& bd0 = cache.solverBodyArray[desc[a].linkIndexA != PxSolverConstraintDesc::NO_LINK ? 0 : desc[a].bodyADataIndex];
		PxSolverBodyData& bd1 = cache.solverBodyArray[desc[a].linkIndexB != PxSolverConstraintDesc::NO_LINK ? 0 : desc[a].bodyBDataIndex];

		solveExtContactCoulomb(desc[a], cache);
		writeBackContactCoulomb(desc[a], cache, bd0, bd1);
	}
	if(cache.mThresholdStreamIndex > 0)
	{
		//Not enough space to write 4 more thresholds back!
		//Write back to global buffer
		PxI32 threshIndex = physx::shdfnd::atomicAdd(cache.mSharedOutThresholdPairs, PxI32(cache.mThresholdStreamIndex)) - PxI32(cache.mThresholdStreamIndex);
		for(PxU32 a = 0; a < cache.mThresholdStreamIndex; ++a)
		{
			cache.mSharedThresholdStream[a + threshIndex] = cache.mThresholdStream[a];
		}
		cache.mThresholdStreamIndex = 0;
	}
}


void solveConcludeContactCoulomb			(const PxSolverConstraintDesc& desc, SolverContext& cache)
{
	solveContactCoulomb(desc, cache);
	concludeContactCoulomb(desc, cache);
}


void solveConcludeContactCoulomb_BStatic	(const PxSolverConstraintDesc& desc, SolverContext& cache)
{
	solveContactCoulomb_BStatic(desc, cache);
	concludeContactCoulomb(desc, cache);
}



}

}

