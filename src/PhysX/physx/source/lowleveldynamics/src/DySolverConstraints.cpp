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
#include "DySolverConstraint1D.h"
#include "DySolverConstraintDesc.h"
#include "DyThresholdTable.h"
#include "DySolverContext.h"
#include "PsUtilities.h"
#include "DyConstraint.h"
#include "PsAtomic.h"
#include "DySolverConstraintsShared.h"

namespace physx
{

namespace Dy
{

//Port of scalar implementation to SIMD maths with some interleaving of instructions
void solve1D(const PxSolverConstraintDesc& desc, SolverContext& cache)
{
	PX_UNUSED(cache);
	PxSolverBody& b0 = *desc.bodyA;
	PxSolverBody& b1 = *desc.bodyB;

	PxU8* PX_RESTRICT bPtr = desc.constraint;
	if (bPtr == NULL)
		return;
	//PxU32 length = desc.constraintLength;

	const SolverConstraint1DHeader* PX_RESTRICT  header = reinterpret_cast<const SolverConstraint1DHeader*>(bPtr);
	SolverConstraint1D* PX_RESTRICT base = reinterpret_cast<SolverConstraint1D*>(bPtr + sizeof(SolverConstraint1DHeader));

	Vec3V linVel0 = V3LoadA(b0.linearVelocity);
	Vec3V linVel1 = V3LoadA(b1.linearVelocity);
	Vec3V angState0 = V3LoadA(b0.angularState);
	Vec3V angState1 = V3LoadA(b1.angularState);

	const FloatV invMass0 = FLoad(header->invMass0D0);
	const FloatV invMass1 = FLoad(header->invMass1D1);
	const FloatV invInertiaScale0 = FLoad(header->angularInvMassScale0);
	const FloatV invInertiaScale1 = FLoad(header->angularInvMassScale1);


	for(PxU32 i=0; i<header->count;++i, base++)
	{
		Ps::prefetchLine(base+1);
		SolverConstraint1D& c = *base;

		const Vec3V clinVel0 = V3LoadA(c.lin0);
		const Vec3V clinVel1 = V3LoadA(c.lin1);
		const Vec3V cangVel0 = V3LoadA(c.ang0);
		const Vec3V cangVel1 = V3LoadA(c.ang1);

		const FloatV constant = FLoad(c.constant);
		const FloatV vMul = FLoad(c.velMultiplier);
		const FloatV iMul = FLoad(c.impulseMultiplier);
		const FloatV appliedForce = FLoad(c.appliedForce);
		//const FloatV targetVel = FLoad(c.targetVelocity);
		
		const FloatV maxImpulse = FLoad(c.maxImpulse);
		const FloatV minImpulse = FLoad(c.minImpulse);

		const Vec3V v0 = V3MulAdd(linVel0, clinVel0, V3Mul(angState0, cangVel0));
		const Vec3V v1 = V3MulAdd(linVel1, clinVel1, V3Mul(angState1, cangVel1));

		const FloatV normalVel = V3SumElems(V3Sub(v0, v1));
		const FloatV unclampedForce = FScaleAdd(iMul, appliedForce, FScaleAdd(vMul, normalVel, constant));
		const FloatV clampedForce = FMin(maxImpulse, (FMax(minImpulse, unclampedForce)));
		const FloatV deltaF = FSub(clampedForce, appliedForce);
			
		FStore(clampedForce, &c.appliedForce);
		linVel0 = V3ScaleAdd(clinVel0, FMul(deltaF, invMass0), linVel0);			
		linVel1 = V3NegScaleSub(clinVel1, FMul(deltaF, invMass1), linVel1);
		angState0 = V3ScaleAdd(cangVel0, FMul(deltaF, invInertiaScale0), angState0);
		//This should be negScaleSub but invInertiaScale1 is negated already
		angState1 = V3ScaleAdd(cangVel1, FMul(deltaF, invInertiaScale1), angState1);

	}

	V3StoreA(linVel0, b0.linearVelocity);
	V3StoreA(angState0, b0.angularState);
	V3StoreA(linVel1, b1.linearVelocity);
	V3StoreA(angState1, b1.angularState);
	
	PX_ASSERT(b0.linearVelocity.isFinite());
	PX_ASSERT(b0.angularState.isFinite());
	PX_ASSERT(b1.linearVelocity.isFinite());
	PX_ASSERT(b1.angularState.isFinite());
}

void conclude1D(const PxSolverConstraintDesc& desc, SolverContext& /*cache*/)
{
	SolverConstraint1DHeader* header = reinterpret_cast<SolverConstraint1DHeader*>(desc.constraint);
	if (header == NULL)
		return;
	PxU8* base = desc.constraint + sizeof(SolverConstraint1DHeader);
	PxU32 stride = header->type == DY_SC_TYPE_EXT_1D ? sizeof(SolverConstraint1DExt) : sizeof(SolverConstraint1D);

	for(PxU32 i=0; i<header->count; i++)
	{
		SolverConstraint1D& c = *reinterpret_cast<SolverConstraint1D*>(base);

		c.constant = c.unbiasedConstant;

		base += stride;
	}
	PX_ASSERT(desc.constraint + getConstraintLength(desc) == base);
}

// ==============================================================

void solveContact(const PxSolverConstraintDesc& desc, SolverContext& cache)
{
	PxSolverBody& b0 = *desc.bodyA;
	PxSolverBody& b1 = *desc.bodyB;

	Vec3V linVel0 = V3LoadA(b0.linearVelocity);
	Vec3V linVel1 = V3LoadA(b1.linearVelocity);
	Vec3V angState0 = V3LoadA(b0.angularState);
	Vec3V angState1 = V3LoadA(b1.angularState);

	const PxU8* PX_RESTRICT last = desc.constraint + getConstraintLength(desc);

	//hopefully pointer aliasing doesn't bite.
	PxU8* PX_RESTRICT currPtr = desc.constraint;

	while(currPtr < last)
	{
		SolverContactHeader* PX_RESTRICT hdr = reinterpret_cast<SolverContactHeader*>(currPtr);
		currPtr += sizeof(SolverContactHeader);

		const PxU32 numNormalConstr = hdr->numNormalConstr;
		const PxU32	numFrictionConstr = hdr->numFrictionConstr;

		SolverContactPoint* PX_RESTRICT contacts = reinterpret_cast<SolverContactPoint*>(currPtr);
		Ps::prefetchLine(contacts);
		currPtr += numNormalConstr * sizeof(SolverContactPoint);

		PxF32* forceBuffer = reinterpret_cast<PxF32*>(currPtr);
		currPtr += sizeof(PxF32) * ((numNormalConstr + 3) & (~3));

		SolverContactFriction* PX_RESTRICT frictions = reinterpret_cast<SolverContactFriction*>(currPtr);
		currPtr += numFrictionConstr * sizeof(SolverContactFriction);

		const FloatV invMassA = FLoad(hdr->invMass0);
		const FloatV invMassB = FLoad(hdr->invMass1);

		const FloatV angDom0 = FLoad(hdr->angDom0);
		const FloatV angDom1 = FLoad(hdr->angDom1);

		const Vec3V contactNormal = Vec3V_From_Vec4V_WUndefined(hdr->normal_minAppliedImpulseForFrictionW);

		const FloatV accumulatedNormalImpulse = solveDynamicContacts(contacts, numNormalConstr, contactNormal, invMassA, invMassB, 
			angDom0, angDom1, linVel0, angState0, linVel1, angState1, forceBuffer); 

		if(cache.doFriction && numFrictionConstr)
		{
			const FloatV staticFrictionCof = hdr->getStaticFriction();
			const FloatV dynamicFrictionCof = hdr->getDynamicFriction();
			const FloatV maxFrictionImpulse = FMul(staticFrictionCof, accumulatedNormalImpulse);
			const FloatV maxDynFrictionImpulse = FMul(dynamicFrictionCof, accumulatedNormalImpulse);
			const FloatV negMaxDynFrictionImpulse = FNeg(maxDynFrictionImpulse);

			BoolV broken = BFFFF();

			if(cache.writeBackIteration)
				Ps::prefetchLine(hdr->frictionBrokenWritebackByte);

			for(PxU32 i=0;i<numFrictionConstr;i++)
			{
				SolverContactFriction& f = frictions[i];
				Ps::prefetchLine(&frictions[i],128);


				const Vec4V normalXYZ_appliedForceW = f.normalXYZ_appliedForceW;
				const Vec4V raXnXYZ_velMultiplierW = f.raXnXYZ_velMultiplierW;
				const Vec4V rbXnXYZ_biasW = f.rbXnXYZ_biasW;

				const Vec3V normal = Vec3V_From_Vec4V(normalXYZ_appliedForceW);
				const Vec3V raXn = Vec3V_From_Vec4V(raXnXYZ_velMultiplierW);
				const Vec3V rbXn = Vec3V_From_Vec4V(rbXnXYZ_biasW);

				const FloatV appliedForce = V4GetW(normalXYZ_appliedForceW);
				const FloatV bias = V4GetW(rbXnXYZ_biasW);
				const FloatV velMultiplier = V4GetW(raXnXYZ_velMultiplierW);
				
				const FloatV targetVel = FLoad(f.targetVel);

				const Vec3V delLinVel0 = V3Scale(normal, invMassA);
				const Vec3V delLinVel1 = V3Scale(normal, invMassB);

				const Vec3V v0 = V3MulAdd(linVel0, normal, V3Mul(angState0, raXn));
				const Vec3V v1 = V3MulAdd(linVel1, normal, V3Mul(angState1, rbXn));
				const FloatV normalVel = V3SumElems(V3Sub(v0, v1));



				// appliedForce -bias * velMultiplier - a hoisted part of the total impulse computation
				const FloatV tmp1 = FNegScaleSub(FSub(bias, targetVel),velMultiplier,appliedForce);				

				// Algorithm:
				// if abs(appliedForce + deltaF) > maxFrictionImpulse
				//    clamp newAppliedForce + deltaF to [-maxDynFrictionImpulse, maxDynFrictionImpulse]
				//      (i.e. clamp deltaF to [-maxDynFrictionImpulse-appliedForce, maxDynFrictionImpulse-appliedForce]
				//    set broken flag to true || broken flag

				// FloatV deltaF = FMul(FAdd(bias, normalVel), minusVelMultiplier);
				// FloatV potentialSumF = FAdd(appliedForce, deltaF);

				const FloatV totalImpulse = FNegScaleSub(normalVel, velMultiplier, tmp1);

				// On XBox this clamping code uses the vector simple pipe rather than vector float,
				// which eliminates a lot of stall cycles

				const BoolV clamp = FIsGrtr(FAbs(totalImpulse), maxFrictionImpulse);
				
				const FloatV totalClamped = FMin(maxDynFrictionImpulse, FMax(negMaxDynFrictionImpulse, totalImpulse));

				const FloatV newAppliedForce = FSel(clamp, totalClamped,totalImpulse);

				broken = BOr(broken, clamp);

				FloatV deltaF = FSub(newAppliedForce, appliedForce);

				// we could get rid of the stall here by calculating and clamping delta separately, but
				// the complexity isn't really worth it.

				linVel0 = V3ScaleAdd(delLinVel0, deltaF, linVel0);
				linVel1 = V3NegScaleSub(delLinVel1, deltaF, linVel1);
				angState0 = V3ScaleAdd(raXn, FMul(deltaF, angDom0), angState0);
				angState1 = V3NegScaleSub(rbXn, FMul(deltaF, angDom1), angState1);

				f.setAppliedForce(newAppliedForce);

				
			}
			Store_From_BoolV(broken, &hdr->broken);
		}

	}

	PX_ASSERT(b0.linearVelocity.isFinite());
	PX_ASSERT(b0.angularState.isFinite());
	PX_ASSERT(b1.linearVelocity.isFinite());
	PX_ASSERT(b1.angularState.isFinite());

	// Write back
	V3StoreU(linVel0, b0.linearVelocity);
	V3StoreU(linVel1, b1.linearVelocity);
	V3StoreU(angState0, b0.angularState);
	V3StoreU(angState1, b1.angularState);

	PX_ASSERT(b0.linearVelocity.isFinite());
	PX_ASSERT(b0.angularState.isFinite());
	PX_ASSERT(b1.linearVelocity.isFinite());
	PX_ASSERT(b1.angularState.isFinite());

	PX_ASSERT(currPtr == last);
}

void solveContact_BStatic(const PxSolverConstraintDesc& desc, SolverContext& cache)
{
	PxSolverBody& b0 = *desc.bodyA;
	//PxSolverBody& b1 = *desc.bodyB;

	Vec3V linVel0 = V3LoadA(b0.linearVelocity);
	Vec3V angState0 = V3LoadA(b0.angularState);

	const PxU8* PX_RESTRICT last = desc.constraint + getConstraintLength(desc);

	//hopefully pointer aliasing doesn't bite.
	PxU8* PX_RESTRICT currPtr = desc.constraint;

	while(currPtr < last)
	{
		SolverContactHeader* PX_RESTRICT hdr = reinterpret_cast<SolverContactHeader*>(currPtr);
		currPtr += sizeof(SolverContactHeader);

		const PxU32 numNormalConstr = hdr->numNormalConstr;
		const PxU32	numFrictionConstr = hdr->numFrictionConstr;

		SolverContactPoint* PX_RESTRICT contacts = reinterpret_cast<SolverContactPoint*>(currPtr);
		//Ps::prefetchLine(contacts);
		currPtr += numNormalConstr * sizeof(SolverContactPoint);

		PxF32* forceBuffer = reinterpret_cast<PxF32*>(currPtr);
		currPtr += sizeof(PxF32) * ((numNormalConstr + 3) & (~3));

		SolverContactFriction* PX_RESTRICT frictions = reinterpret_cast<SolverContactFriction*>(currPtr);
		currPtr += numFrictionConstr * sizeof(SolverContactFriction);

		

		const FloatV invMassA = FLoad(hdr->invMass0);

		const Vec3V contactNormal = Vec3V_From_Vec4V_WUndefined(hdr->normal_minAppliedImpulseForFrictionW);
		const FloatV angDom0 = FLoad(hdr->angDom0);


		const FloatV accumulatedNormalImpulse = solveStaticContacts(contacts, numNormalConstr, contactNormal,
			invMassA, angDom0, linVel0, angState0, forceBuffer);

		if(cache.doFriction && numFrictionConstr)
		{
			const FloatV maxFrictionImpulse = FMul(hdr->getStaticFriction(), accumulatedNormalImpulse);
			const FloatV maxDynFrictionImpulse = FMul(hdr->getDynamicFriction(), accumulatedNormalImpulse);

			BoolV broken = BFFFF();
			if(cache.writeBackIteration)
				Ps::prefetchLine(hdr->frictionBrokenWritebackByte);

			for(PxU32 i=0;i<numFrictionConstr;i++)
			{
				SolverContactFriction& f = frictions[i];
				Ps::prefetchLine(&frictions[i],128);
				

				const Vec4V normalXYZ_appliedForceW = f.normalXYZ_appliedForceW;
				const Vec4V raXnXYZ_velMultiplierW = f.raXnXYZ_velMultiplierW;
				const Vec4V rbXnXYZ_biasW = f.rbXnXYZ_biasW;

				const Vec3V normal = Vec3V_From_Vec4V(normalXYZ_appliedForceW);
				const Vec3V raXn = Vec3V_From_Vec4V(raXnXYZ_velMultiplierW);

				const FloatV appliedForce = V4GetW(normalXYZ_appliedForceW);
				const FloatV bias = V4GetW(rbXnXYZ_biasW);
				const FloatV velMultiplier = V4GetW(raXnXYZ_velMultiplierW);

				const FloatV targetVel = FLoad(f.targetVel);
	
				const FloatV negMaxDynFrictionImpulse = FNeg(maxDynFrictionImpulse);

				const Vec3V delLinVel0 = V3Scale(normal, invMassA);
				//const FloatV negMaxFrictionImpulse = FNeg(maxFrictionImpulse);

				const Vec3V v0 = V3MulAdd(linVel0, normal, V3Mul(angState0, raXn));
				const FloatV normalVel = V3SumElems(v0);


				// appliedForce -bias * velMultiplier - a hoisted part of the total impulse computation
				const FloatV tmp1 = FNegScaleSub(FSub(bias, targetVel),velMultiplier,appliedForce); 

				// Algorithm:
				// if abs(appliedForce + deltaF) > maxFrictionImpulse
				//    clamp newAppliedForce + deltaF to [-maxDynFrictionImpulse, maxDynFrictionImpulse]
				//      (i.e. clamp deltaF to [-maxDynFrictionImpulse-appliedForce, maxDynFrictionImpulse-appliedForce]
				//    set broken flag to true || broken flag

				// FloatV deltaF = FMul(FAdd(bias, normalVel), minusVelMultiplier);
				// FloatV potentialSumF = FAdd(appliedForce, deltaF);

				const FloatV totalImpulse = FNegScaleSub(normalVel, velMultiplier, tmp1);

				// On XBox this clamping code uses the vector simple pipe rather than vector float,
				// which eliminates a lot of stall cycles

				const BoolV clamp = FIsGrtr(FAbs(totalImpulse), maxFrictionImpulse);
				
				const FloatV totalClamped = FMin(maxDynFrictionImpulse, FMax(negMaxDynFrictionImpulse, totalImpulse));
				
				broken = BOr(broken, clamp);

				const FloatV newAppliedForce = FSel(clamp, totalClamped,totalImpulse);

				FloatV deltaF = FSub(newAppliedForce, appliedForce);

				// we could get rid of the stall here by calculating and clamping delta separately, but
				// the complexity isn't really worth it.

				linVel0 = V3ScaleAdd(delLinVel0, deltaF, linVel0);
				angState0 = V3ScaleAdd(raXn, FMul(deltaF, angDom0), angState0);

				f.setAppliedForce(newAppliedForce);

			}
			Store_From_BoolV(broken, &hdr->broken);
		}

	}

	PX_ASSERT(b0.linearVelocity.isFinite());
	PX_ASSERT(b0.angularState.isFinite());

	// Write back
	V3StoreA(linVel0, b0.linearVelocity);
	V3StoreA(angState0, b0.angularState);

	PX_ASSERT(b0.linearVelocity.isFinite());
	PX_ASSERT(b0.angularState.isFinite());

	PX_ASSERT(currPtr == last);
}


void concludeContact(const PxSolverConstraintDesc& desc, SolverContext& /*cache*/)
{
	PxU8* PX_RESTRICT cPtr = desc.constraint;

	const FloatV zero = FZero();

	PxU8* PX_RESTRICT last = desc.constraint + getConstraintLength(desc);
	while(cPtr < last)
	{
		const SolverContactHeader* PX_RESTRICT hdr = reinterpret_cast<const SolverContactHeader*>(cPtr);
		cPtr += sizeof(SolverContactHeader);

		const PxU32 numNormalConstr = hdr->numNormalConstr;
		const PxU32	numFrictionConstr = hdr->numFrictionConstr;

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

		cPtr += sizeof(PxF32) * ((numNormalConstr + 3) & (~3)); //Jump over force buffers

		const PxU32 frictionStride = hdr->type == DY_SC_TYPE_EXT_CONTACT ? sizeof(SolverContactFrictionExt)
																		  : sizeof(SolverContactFriction);
		for(PxU32 i=0;i<numFrictionConstr;i++)
		{
			SolverContactFriction *f = reinterpret_cast<SolverContactFriction*>(cPtr);
			cPtr += frictionStride;
			f->setBias(zero);
		}
	}
	PX_ASSERT(cPtr == last);
}

void writeBackContact(const PxSolverConstraintDesc& desc, SolverContext& cache,
					  PxSolverBodyData& bd0, PxSolverBodyData& bd1)
{

	PxReal normalForce = 0;

	PxU8* PX_RESTRICT cPtr = desc.constraint;
	PxReal* PX_RESTRICT vForceWriteback = reinterpret_cast<PxReal*>(desc.writeBack);
	PxU8* PX_RESTRICT last = desc.constraint + getConstraintLength(desc);

	bool forceThreshold = false;

	while(cPtr < last)
	{
		const SolverContactHeader* PX_RESTRICT hdr = reinterpret_cast<const SolverContactHeader*>(cPtr);
		cPtr += sizeof(SolverContactHeader);

		forceThreshold = hdr->flags & SolverContactHeader::eHAS_FORCE_THRESHOLDS;
		const PxU32 numNormalConstr = hdr->numNormalConstr;
		const PxU32	numFrictionConstr = hdr->numFrictionConstr;

		//if(cPtr < last)
		Ps::prefetchLine(cPtr, 256);
		Ps::prefetchLine(cPtr, 384);

		const PxU32 pointStride = hdr->type == DY_SC_TYPE_EXT_CONTACT ? sizeof(SolverContactPointExt)
																	   : sizeof(SolverContactPoint);

		cPtr += pointStride * numNormalConstr;
		PxF32* forceBuffer = reinterpret_cast<PxF32*>(cPtr);
		cPtr += sizeof(PxF32) * ((numNormalConstr + 3) & (~3));

		if(vForceWriteback!=NULL)
		{
			for(PxU32 i=0; i<numNormalConstr; i++)
			{
				PxReal appliedForce = forceBuffer[i];
				*vForceWriteback++ = appliedForce;
				normalForce += appliedForce;
			}
		}

		const PxU32 frictionStride = hdr->type == DY_SC_TYPE_EXT_CONTACT ? sizeof(SolverContactFrictionExt)
																		  : sizeof(SolverContactFriction);

		if(hdr->broken && hdr->frictionBrokenWritebackByte != NULL)
		{
			*hdr->frictionBrokenWritebackByte = 1;
		}

		cPtr += frictionStride * numFrictionConstr;

	}
	PX_ASSERT(cPtr == last);

	

	if(forceThreshold && desc.linkIndexA == PxSolverConstraintDesc::NO_LINK && desc.linkIndexB == PxSolverConstraintDesc::NO_LINK &&
		normalForce !=0 && (bd0.reportThreshold < PX_MAX_REAL  || bd1.reportThreshold < PX_MAX_REAL))
	{
		ThresholdStreamElement elt;
		elt.normalForce = normalForce;
		elt.threshold = PxMin<float>(bd0.reportThreshold, bd1.reportThreshold);
		elt.nodeIndexA = IG::NodeIndex(bd0.nodeIndex);
		elt.nodeIndexB = IG::NodeIndex(bd1.nodeIndex);
		elt.shapeInteraction  = reinterpret_cast<const SolverContactHeader*>(desc.constraint)->shapeInteraction;
		Ps::order(elt.nodeIndexA, elt.nodeIndexB);
		PX_ASSERT(elt.nodeIndexA < elt.nodeIndexB);
		PX_ASSERT(cache.mThresholdStreamIndex<cache.mThresholdStreamLength);
		cache.mThresholdStream[cache.mThresholdStreamIndex++] = elt;
	}
}

// adjust from CoM to joint

void writeBack1D(const PxSolverConstraintDesc& desc, SolverContext&, PxSolverBodyData&, PxSolverBodyData&)
{
	ConstraintWriteback* writeback = reinterpret_cast<ConstraintWriteback*>(desc.writeBack);
	if(writeback)
	{
		SolverConstraint1DHeader* header = reinterpret_cast<SolverConstraint1DHeader*>(desc.constraint);
		PxU8* base = desc.constraint + sizeof(SolverConstraint1DHeader);
		PxU32 stride = header->type == DY_SC_TYPE_EXT_1D ? sizeof(SolverConstraint1DExt) : sizeof(SolverConstraint1D);

		PxVec3 lin(0), ang(0);
		for(PxU32 i=0; i<header->count; i++)
		{
			const SolverConstraint1D* c = reinterpret_cast<SolverConstraint1D*>(base);
			if(c->flags & DY_SC_FLAG_OUTPUT_FORCE)
			{
				lin += c->lin0 * c->appliedForce;
				ang += c->ang0Writeback * c->appliedForce;
			}
			base += stride;
		}

		ang -= header->body0WorldOffset.cross(lin);
		writeback->linearImpulse = lin;
		writeback->angularImpulse = ang;
		writeback->broken = header->breakable ? PxU32(lin.magnitude()>header->linBreakImpulse || ang.magnitude()>header->angBreakImpulse) : 0;

		PX_ASSERT(desc.constraint + getConstraintLength(desc) == base);
	}
}


void solve1DBlock (const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 1; a < constraintCount; ++a)
	{
		Ps::prefetchLine(desc[a].constraint);
		Ps::prefetchLine(desc[a].constraint, 128);
		Ps::prefetchLine(desc[a].constraint, 256);
		solve1D(desc[a-1], cache);
	}
	solve1D(desc[constraintCount-1], cache);
}

void solve1DConcludeBlock (const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 1; a < constraintCount; ++a)
	{
		Ps::prefetchLine(desc[a].constraint);
		Ps::prefetchLine(desc[a].constraint, 128);
		Ps::prefetchLine(desc[a].constraint, 256);
		solve1D(desc[a-1], cache);
		conclude1D(desc[a-1], cache);
	}
	solve1D(desc[constraintCount-1], cache);
	conclude1D(desc[constraintCount-1], cache);
}

void solve1DBlockWriteBack (const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 1; a < constraintCount; ++a)
	{
		Ps::prefetchLine(desc[a].constraint);
		Ps::prefetchLine(desc[a].constraint, 128);
		Ps::prefetchLine(desc[a].constraint, 256);
		PxSolverBodyData& bd0 = cache.solverBodyArray[desc[a-1].bodyADataIndex];
		PxSolverBodyData& bd1 = cache.solverBodyArray[desc[a-1].bodyBDataIndex];
		solve1D(desc[a-1], cache);
		writeBack1D(desc[a-1], cache, bd0, bd1);
	}
	PxSolverBodyData& bd0 = cache.solverBodyArray[desc[constraintCount-1].bodyADataIndex];
	PxSolverBodyData& bd1 = cache.solverBodyArray[desc[constraintCount-1].bodyBDataIndex];
	solve1D(desc[constraintCount-1], cache);
	writeBack1D(desc[constraintCount-1], cache, bd0, bd1);
}

void writeBack1DBlock (const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 1; a < constraintCount; ++a)
	{
		Ps::prefetchLine(desc[a].constraint);
		Ps::prefetchLine(desc[a].constraint, 128);
		Ps::prefetchLine(desc[a].constraint, 256);
		PxSolverBodyData& bd0 = cache.solverBodyArray[desc[a-1].bodyADataIndex];
		PxSolverBodyData& bd1 = cache.solverBodyArray[desc[a-1].bodyBDataIndex];
		writeBack1D(desc[a-1], cache, bd0, bd1);
	}
	PxSolverBodyData& bd0 = cache.solverBodyArray[desc[constraintCount-1].bodyADataIndex];
	PxSolverBodyData& bd1 = cache.solverBodyArray[desc[constraintCount-1].bodyBDataIndex];
	writeBack1D(desc[constraintCount-1], cache, bd0, bd1);
}

void solveContactBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 1; a < constraintCount; ++a)
	{
		Ps::prefetchLine(desc[a].constraint);
		Ps::prefetchLine(desc[a].constraint, 128);
		Ps::prefetchLine(desc[a].constraint, 256);
		solveContact(desc[a-1], cache);
	}
	solveContact(desc[constraintCount-1], cache);
}

void solveContactConcludeBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 1; a < constraintCount; ++a)
	{
		Ps::prefetchLine(desc[a].constraint);
		Ps::prefetchLine(desc[a].constraint, 128);
		Ps::prefetchLine(desc[a].constraint, 256);
		solveContact(desc[a-1], cache);
		concludeContact(desc[a-1], cache);
	}
	solveContact(desc[constraintCount-1], cache);
	concludeContact(desc[constraintCount-1], cache);
}

void solveContactBlockWriteBack(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 1; a < constraintCount; ++a)
	{
		Ps::prefetchLine(desc[a].constraint);
		Ps::prefetchLine(desc[a].constraint, 128);
		Ps::prefetchLine(desc[a].constraint, 256);
		PxSolverBodyData& bd0 = cache.solverBodyArray[desc[a-1].bodyADataIndex];
		PxSolverBodyData& bd1 = cache.solverBodyArray[desc[a-1].bodyBDataIndex];
		solveContact(desc[a-1], cache);
		writeBackContact(desc[a-1], cache, bd0, bd1);
	}
	PxSolverBodyData& bd0 = cache.solverBodyArray[desc[constraintCount-1].bodyADataIndex];
	PxSolverBodyData& bd1 = cache.solverBodyArray[desc[constraintCount-1].bodyBDataIndex];
	solveContact(desc[constraintCount-1], cache);
	writeBackContact(desc[constraintCount-1], cache, bd0, bd1);

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

void solveContact_BStaticBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 1; a < constraintCount; ++a)
	{
		Ps::prefetchLine(desc[a].constraint);
		Ps::prefetchLine(desc[a].constraint, 128);
		Ps::prefetchLine(desc[a].constraint, 256);
		solveContact_BStatic(desc[a-1], cache);
	}
	solveContact_BStatic(desc[constraintCount-1], cache);
}

void solveContact_BStaticConcludeBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 1; a < constraintCount; ++a)
	{
		Ps::prefetchLine(desc[a].constraint);
		Ps::prefetchLine(desc[a].constraint, 128);
		Ps::prefetchLine(desc[a].constraint, 256);
		solveContact_BStatic(desc[a-1], cache);
		concludeContact(desc[a-1], cache);
	}
	solveContact_BStatic(desc[constraintCount-1], cache);
	concludeContact(desc[constraintCount-1], cache);
}

void solveContact_BStaticBlockWriteBack(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 1; a < constraintCount; ++a)
	{
		Ps::prefetchLine(desc[a].constraint);
		Ps::prefetchLine(desc[a].constraint, 128);
		Ps::prefetchLine(desc[a].constraint, 256);
		PxSolverBodyData& bd0 = cache.solverBodyArray[desc[a-1].bodyADataIndex];
		PxSolverBodyData& bd1 = cache.solverBodyArray[desc[a-1].bodyBDataIndex];
		solveContact_BStatic(desc[a-1], cache);
		writeBackContact(desc[a-1], cache, bd0, bd1);
	}
	PxSolverBodyData& bd0 = cache.solverBodyArray[desc[constraintCount-1].bodyADataIndex];
	PxSolverBodyData& bd1 = cache.solverBodyArray[desc[constraintCount-1].bodyBDataIndex];
	solveContact_BStatic(desc[constraintCount-1], cache);
	writeBackContact(desc[constraintCount-1], cache, bd0, bd1);

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

void clearExt1D(const PxSolverConstraintDesc& desc, SolverContext& /*cache*/)
{
	PxU8* PX_RESTRICT bPtr = desc.constraint;
	const SolverConstraint1DHeader* PX_RESTRICT  header = reinterpret_cast<const SolverConstraint1DHeader*>(bPtr);
	SolverConstraint1DExt* PX_RESTRICT base = reinterpret_cast<SolverConstraint1DExt*>(bPtr + sizeof(SolverConstraint1DHeader));

	for (PxU32 i = 0; i < header->count; ++i, base++)
	{
		base->appliedForce = 0.f;
	}
}

//Port of scalar implementation to SIMD maths with some interleaving of instructions
void solveExt1D(const PxSolverConstraintDesc& desc, SolverContext& cache)
{
	PxU8* PX_RESTRICT bPtr = desc.constraint;
	//PxU32 length = desc.constraintLength;

	const SolverConstraint1DHeader* PX_RESTRICT  header = reinterpret_cast<const SolverConstraint1DHeader*>(bPtr);
	SolverConstraint1DExt* PX_RESTRICT base = reinterpret_cast<SolverConstraint1DExt*>(bPtr + sizeof(SolverConstraint1DHeader));

	Vec3V linVel0, angVel0, linVel1, angVel1;

	if (desc.articulationA == desc.articulationB)
	{
		Cm::SpatialVectorV v0, v1;
		desc.articulationA->pxcFsGetVelocities(desc.linkIndexA, desc.linkIndexB, v0, v1);
		linVel0 = v0.linear;
		angVel0 = v0.angular;
		linVel1 = v1.linear;
		angVel1 = v1.angular;
	}
	else
	{

		if (desc.linkIndexA == PxSolverConstraintDesc::NO_LINK)
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

		if (desc.linkIndexB == PxSolverConstraintDesc::NO_LINK)
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
	}

	Vec3V li0 = V3Zero(), li1 = V3Zero(), ai0 = V3Zero(), ai1 = V3Zero();

	for(PxU32 i=0; i<header->count;++i, base++)
	{
		Ps::prefetchLine(base+1);

		const Vec4V lin0XYZ_constantW						= V4LoadA(&base->lin0.x);	
		const Vec4V lin1XYZ_unbiasedConstantW				= V4LoadA(&base->lin1.x);
		const Vec4V ang0XYZ_velMultiplierW					= V4LoadA(&base->ang0.x);
		const Vec4V ang1XYZ_impulseMultiplierW				= V4LoadA(&base->ang1.x);	
		const Vec4V minImpulseX_maxImpulseY_appliedForceZ	= V4LoadA(&base->minImpulse);

		const Vec3V lin0 = Vec3V_From_Vec4V(lin0XYZ_constantW);				FloatV constant = V4GetW(lin0XYZ_constantW);
		const Vec3V lin1 = Vec3V_From_Vec4V(lin1XYZ_unbiasedConstantW);
		const Vec3V ang0 = Vec3V_From_Vec4V(ang0XYZ_velMultiplierW);		FloatV vMul = V4GetW(ang0XYZ_velMultiplierW);
		const Vec3V ang1 = Vec3V_From_Vec4V(ang1XYZ_impulseMultiplierW);	FloatV iMul = V4GetW(ang1XYZ_impulseMultiplierW);

		const FloatV minImpulse		= V4GetX(minImpulseX_maxImpulseY_appliedForceZ);
		const FloatV maxImpulse		= V4GetY(minImpulseX_maxImpulseY_appliedForceZ);
		const FloatV appliedForce	= V4GetZ(minImpulseX_maxImpulseY_appliedForceZ);

		const Vec3V v0 = V3MulAdd(linVel0, lin0, V3Mul(angVel0, ang0));
		const Vec3V v1 = V3MulAdd(linVel1, lin1, V3Mul(angVel1, ang1));
		const FloatV normalVel = V3SumElems(V3Sub(v0, v1));

		const FloatV unclampedForce = FScaleAdd(iMul, appliedForce, FScaleAdd(vMul, normalVel, constant));
		const FloatV clampedForce = FMin(maxImpulse, (FMax(minImpulse, unclampedForce)));
		const FloatV deltaF = FSub(clampedForce, appliedForce);

		FStore(clampedForce, &base->appliedForce);
		li0 = V3ScaleAdd(lin0, deltaF, li0);	ai0 = V3ScaleAdd(ang0, deltaF, ai0);
		li1 = V3ScaleAdd(lin1, deltaF, li1);	ai1 = V3ScaleAdd(ang1, deltaF, ai1);

		linVel0 = V3ScaleAdd(base->deltaVA.linear, deltaF, linVel0); 		angVel0 = V3ScaleAdd(base->deltaVA.angular, deltaF, angVel0);
		linVel1 = V3ScaleAdd(base->deltaVB.linear, deltaF, linVel1); 		angVel1 = V3ScaleAdd(base->deltaVB.angular, deltaF, angVel1);

#if 0
		PxVec3 lv0, lv1, av0, av1;
		V3StoreU(linVel0, lv0); V3StoreU(linVel1, lv1);
		V3StoreU(angVel0, av0); V3StoreU(angVel1, av1);

		PX_ASSERT(lv0.magnitude() < 30.f);
		PX_ASSERT(lv1.magnitude() < 30.f);
		PX_ASSERT(av0.magnitude() < 30.f);
		PX_ASSERT(av1.magnitude() < 30.f);
#endif

	}

	if (desc.articulationA == desc.articulationB)
	{
		desc.articulationA->pxcFsApplyImpulses(desc.linkIndexA, V3Scale(li0, FLoad(header->linearInvMassScale0)), V3Scale(ai0, FLoad(header->angularInvMassScale0)),
			desc.linkIndexB, V3Scale(li1, FLoad(header->linearInvMassScale1)), V3Scale(ai1, FLoad(header->angularInvMassScale1)), cache.Z, cache.deltaV);
	}
	else
	{

		if (desc.linkIndexA == PxSolverConstraintDesc::NO_LINK)
		{
			V3StoreA(linVel0, desc.bodyA->linearVelocity);
			V3StoreA(angVel0, desc.bodyA->angularState);
		}
		else
		{
			desc.articulationA->pxcFsApplyImpulse(desc.linkIndexA, V3Scale(li0, FLoad(header->linearInvMassScale0)),
				V3Scale(ai0, FLoad(header->angularInvMassScale0)), cache.Z, cache.deltaV);


		}

		if (desc.linkIndexB == PxSolverConstraintDesc::NO_LINK)
		{
			V3StoreA(linVel1, desc.bodyB->linearVelocity);
			V3StoreA(angVel1, desc.bodyB->angularState);
		}
		else
		{
			desc.articulationB->pxcFsApplyImpulse(desc.linkIndexB, V3Scale(li1, FLoad(header->linearInvMassScale1)),
				V3Scale(ai1, FLoad(header->angularInvMassScale1)), cache.Z, cache.deltaV);
		}
	}

}

FloatV solveExtContacts(SolverContactPointExt* contacts, const PxU32 nbContactPoints, const Vec3VArg contactNormal,
	Vec3V& linVel0, Vec3V& angVel0,
	Vec3V& linVel1, Vec3V& angVel1,
	Vec3V& li0, Vec3V& ai0,
	Vec3V& li1, Vec3V& ai1,
	PxF32* PX_RESTRICT appliedForceBuffer)
{

	FloatV accumulatedNormalImpulse = FZero();
	for (PxU32 i = 0; i<nbContactPoints; i++)
	{
		SolverContactPointExt& c = contacts[i];
		Ps::prefetchLine(&contacts[i + 1]);

		const Vec3V raXn = c.raXn;
		const Vec3V rbXn = c.rbXn;

		const FloatV appliedForce = FLoad(appliedForceBuffer[i]);
		const FloatV velMultiplier = c.getVelMultiplier();

		/*const FloatV targetVel = c.getTargetVelocity();
		const FloatV scaledBias = c.getScaledBias();*/

		//Compute the normal velocity of the constraint.

		Vec3V v = V3MulAdd(linVel0, contactNormal, V3Mul(angVel0, raXn));
		v = V3Sub(v, V3MulAdd(linVel1, contactNormal, V3Mul(angVel1, rbXn)));
		const FloatV normalVel = V3SumElems(v);

		const FloatV biasedErr = c.getBiasedErr();//FNeg(scaledBias);

												  // still lots to do here: using loop pipelining we can interweave this code with the
												  // above - the code here has a lot of stalls that we would thereby eliminate

		const FloatV deltaF = FMax(FNegScaleSub(normalVel, velMultiplier, biasedErr), FNeg(appliedForce));

		linVel0 = V3ScaleAdd(c.linDeltaVA, deltaF, linVel0);
		angVel0 = V3ScaleAdd(c.angDeltaVA, deltaF, angVel0);
		linVel1 = V3ScaleAdd(c.linDeltaVB, deltaF, linVel1);
		angVel1 = V3ScaleAdd(c.angDeltaVB, deltaF, angVel1);

		li0 = V3ScaleAdd(contactNormal, deltaF, li0);	ai0 = V3ScaleAdd(raXn, deltaF, ai0);
		li1 = V3ScaleAdd(contactNormal, deltaF, li1);	ai1 = V3ScaleAdd(rbXn, deltaF, ai1);

		const FloatV newAppliedForce = FAdd(appliedForce, deltaF);

		FStore(newAppliedForce, &appliedForceBuffer[i]);

		accumulatedNormalImpulse = FAdd(accumulatedNormalImpulse, newAppliedForce);

#if 0
		PxVec3 lv0, lv1, av0, av1;
		V3StoreU(linVel0, lv0); V3StoreU(linVel1, lv1);
		V3StoreU(angVel0, av0); V3StoreU(angVel1, av1);

		PX_ASSERT(lv0.magnitude() < 30.f);
		PX_ASSERT(lv1.magnitude() < 30.f);
		PX_ASSERT(av0.magnitude() < 30.f);
		PX_ASSERT(av1.magnitude() < 30.f);
#endif

	}
	return accumulatedNormalImpulse;
}

void solveExtContact(const PxSolverConstraintDesc& desc, SolverContext& cache)
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

	//Vec3V origLin0 = linVel0, origAng0 = angVel0, origLin1 = linVel1, origAng1 = angVel1;

	const PxU8* PX_RESTRICT last = desc.constraint + desc.constraintLengthOver16*16;

	//hopefully pointer aliasing doesn't bite.
	PxU8* PX_RESTRICT currPtr = desc.constraint;

	Vec3V linImpulse0 = V3Zero(), linImpulse1 = V3Zero(), angImpulse0 = V3Zero(), angImpulse1 = V3Zero();

	while(currPtr < last)
	{
		SolverContactHeader* PX_RESTRICT hdr = reinterpret_cast<SolverContactHeader*>(currPtr);
		currPtr += sizeof(SolverContactHeader);

		const PxU32 numNormalConstr = hdr->numNormalConstr;
		const PxU32	numFrictionConstr = hdr->numFrictionConstr;

		SolverContactPointExt* PX_RESTRICT contacts = reinterpret_cast<SolverContactPointExt*>(currPtr);
		Ps::prefetchLine(contacts);
		currPtr += numNormalConstr * sizeof(SolverContactPointExt);

		PxF32* appliedForceBuffer = reinterpret_cast<PxF32*>(currPtr);
		currPtr += sizeof(PxF32) * ((numNormalConstr + 3) & (~3));

		SolverContactFrictionExt* PX_RESTRICT frictions = reinterpret_cast<SolverContactFrictionExt*>(currPtr);
		currPtr += numFrictionConstr * sizeof(SolverContactFrictionExt);

		

		Vec3V li0 = V3Zero(), li1 = V3Zero(), ai0 = V3Zero(), ai1 = V3Zero();

		const Vec3V contactNormal = Vec3V_From_Vec4V(hdr->normal_minAppliedImpulseForFrictionW);
		const FloatV minNorImpulse = V4GetW(hdr->normal_minAppliedImpulseForFrictionW);

		const FloatV accumulatedNormalImpulse = FMax(solveExtContacts(contacts, numNormalConstr, contactNormal, linVel0, angVel0, linVel1, 
			angVel1, li0, ai0, li1, ai1, appliedForceBuffer), minNorImpulse);


		if(cache.doFriction && numFrictionConstr)
		{
			Ps::prefetchLine(frictions);
			const FloatV maxFrictionImpulse = FMul(hdr->getStaticFriction(), accumulatedNormalImpulse);
			const FloatV maxDynFrictionImpulse = FMul(hdr->getDynamicFriction(), accumulatedNormalImpulse);

			BoolV broken = BFFFF();

			for(PxU32 i=0;i<numFrictionConstr;i++)
			{
				SolverContactFrictionExt& f = frictions[i];
				Ps::prefetchLine(&frictions[i+1]);

				const Vec4V normalXYZ_appliedForceW = f.normalXYZ_appliedForceW;
				const Vec4V raXnXYZ_velMultiplierW = f.raXnXYZ_velMultiplierW;
				const Vec4V rbXnXYZ_biasW = f.rbXnXYZ_biasW;

				const Vec3V normal = Vec3V_From_Vec4V(normalXYZ_appliedForceW);
				/*const Vec3V normal0 = V3Scale(normal, sqrtInvMass0);
				const Vec3V normal1 = V3Scale(normal, sqrtInvMass1);*/
				const Vec3V raXn = Vec3V_From_Vec4V(raXnXYZ_velMultiplierW);
				const Vec3V rbXn = Vec3V_From_Vec4V(rbXnXYZ_biasW);

				const FloatV appliedForce = V4GetW(normalXYZ_appliedForceW);
				const FloatV bias = V4GetW(rbXnXYZ_biasW);
				const FloatV velMultiplier = V4GetW(raXnXYZ_velMultiplierW);

				const FloatV targetVel = FLoad(f.targetVel);

				const FloatV negMaxDynFrictionImpulse = FNeg(maxDynFrictionImpulse);
				const FloatV negMaxFrictionImpulse = FNeg(maxFrictionImpulse);

				const Vec3V v0 = V3MulAdd(linVel0, normal, V3Mul(angVel0, raXn));
				const Vec3V v1 = V3MulAdd(linVel1, normal, V3Mul(angVel1, rbXn));
				const FloatV normalVel = V3SumElems(V3Sub(v0, v1));

				// appliedForce -bias * velMultiplier - a hoisted part of the total impulse computation
				const FloatV tmp1 = FNegScaleSub(FSub(bias, targetVel),velMultiplier,appliedForce); 

				// Algorithm:
				// if abs(appliedForce + deltaF) > maxFrictionImpulse
				//    clamp newAppliedForce + deltaF to [-maxDynFrictionImpulse, maxDynFrictionImpulse]
				//      (i.e. clamp deltaF to [-maxDynFrictionImpulse-appliedForce, maxDynFrictionImpulse-appliedForce]
				//    set broken flag to true || broken flag

				// FloatV deltaF = FMul(FAdd(bias, normalVel), minusVelMultiplier);
				// FloatV potentialSumF = FAdd(appliedForce, deltaF);

				const FloatV totalImpulse = FNegScaleSub(normalVel, velMultiplier, tmp1);

				// On XBox this clamping code uses the vector simple pipe rather than vector float,
				// which eliminates a lot of stall cycles

				const BoolV clampLow = FIsGrtr(negMaxFrictionImpulse, totalImpulse);
				const BoolV clampHigh = FIsGrtr(totalImpulse, maxFrictionImpulse);

				const FloatV totalClampedLow = FMax(negMaxDynFrictionImpulse, totalImpulse);
				const FloatV totalClampedHigh = FMin(maxDynFrictionImpulse, totalImpulse);

				const FloatV newAppliedForce = FSel(clampLow, totalClampedLow,
															  FSel(clampHigh, totalClampedHigh, totalImpulse));

				broken = BOr(broken, BOr(clampLow, clampHigh));

				FloatV deltaF = FSub(newAppliedForce, appliedForce);

				linVel0 = V3ScaleAdd(f.linDeltaVA, deltaF, linVel0);	
				angVel0 = V3ScaleAdd(f.angDeltaVA, deltaF, angVel0);
				linVel1 = V3ScaleAdd(f.linDeltaVB, deltaF, linVel1);	
				angVel1 = V3ScaleAdd(f.angDeltaVB, deltaF, angVel1);

				li0 = V3ScaleAdd(normal, deltaF, li0);	ai0 = V3ScaleAdd(raXn, deltaF, ai0);
				li1 = V3ScaleAdd(normal, deltaF, li1);	ai1 = V3ScaleAdd(rbXn, deltaF, ai1);

#if 0
				PxVec3 lv0, lv1, av0, av1;
				V3StoreU(linVel0, lv0); V3StoreU(linVel1, lv1);
				V3StoreU(angVel0, av0); V3StoreU(angVel1, av1);

				PX_ASSERT(lv0.magnitude() < 30.f);
				PX_ASSERT(lv1.magnitude() < 30.f);
				PX_ASSERT(av0.magnitude() < 30.f);
				PX_ASSERT(av1.magnitude() < 30.f);
#endif

				f.setAppliedForce(newAppliedForce);
			}
			Store_From_BoolV(broken, &hdr->broken);
		}

		linImpulse0 = V3ScaleAdd(li0, hdr->getDominance0(), linImpulse0);		
		angImpulse0 = V3ScaleAdd(ai0, FLoad(hdr->angDom0), angImpulse0);
		linImpulse1 = V3NegScaleSub(li1, hdr->getDominance1(), linImpulse1);	
		angImpulse1 = V3NegScaleSub(ai1, FLoad(hdr->angDom1), angImpulse1);
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


void solveExtContactBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 0; a < constraintCount; ++a)
	{
		solveExtContact(desc[a], cache);
	}
}

void solveExtContactConcludeBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 0; a < constraintCount; ++a)
	{
		solveExtContact(desc[a], cache);
		concludeContact(desc[a], cache);
	}
}

void solveExtContactBlockWriteBack(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 0; a < constraintCount; ++a)
	{
		PxSolverBodyData& bd0 = cache.solverBodyArray[desc[a].linkIndexA != PxSolverConstraintDesc::NO_LINK ? 0 : desc[a].bodyADataIndex];
		PxSolverBodyData& bd1 = cache.solverBodyArray[desc[a].linkIndexB != PxSolverConstraintDesc::NO_LINK ? 0 : desc[a].bodyBDataIndex];

		solveExtContact(desc[a], cache);
		writeBackContact(desc[a], cache, bd0, bd1);
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

void solveExt1DBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 0; a < constraintCount; ++a)
	{
		solveExt1D(desc[a], cache);
	}
}

void solveExt1DConcludeBlock(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 0; a < constraintCount; ++a)
	{
		solveExt1D(desc[a], cache);
		conclude1D(desc[a], cache);
	}
}

void solveExt1DBlockWriteBack(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 0; a < constraintCount; ++a)
	{
		PxSolverBodyData& bd0 = cache.solverBodyArray[desc[a].linkIndexA != PxSolverConstraintDesc::NO_LINK ? 0 : desc[a].bodyADataIndex];
		PxSolverBodyData& bd1 = cache.solverBodyArray[desc[a].linkIndexB != PxSolverConstraintDesc::NO_LINK ? 0 : desc[a].bodyBDataIndex];
		solveExt1D(desc[a], cache);
		writeBack1D(desc[a], cache, bd0, bd1);
	}
}

void ext1DBlockWriteBack(const PxSolverConstraintDesc* PX_RESTRICT desc, const PxU32 constraintCount, SolverContext& cache)
{
	for(PxU32 a = 0; a < constraintCount; ++a)
	{
		PxSolverBodyData& bd0 = cache.solverBodyArray[desc[a].linkIndexA != PxSolverConstraintDesc::NO_LINK ? 0 : desc[a].bodyADataIndex];
		PxSolverBodyData& bd1 = cache.solverBodyArray[desc[a].linkIndexB != PxSolverConstraintDesc::NO_LINK ? 0 : desc[a].bodyBDataIndex];
		writeBack1D(desc[a], cache, bd0, bd1);
	}
}

void solveConcludeExtContact		(const PxSolverConstraintDesc& desc, SolverContext& cache)
{
	solveExtContact(desc, cache);
	concludeContact(desc, cache);
}

void solveConcludeExt1D				(const PxSolverConstraintDesc& desc, SolverContext& cache)
{
	solveExt1D(desc, cache);
	conclude1D(desc, cache);
}


void solveConclude1D(const PxSolverConstraintDesc& desc, SolverContext& cache)
{
	solve1D(desc, cache);
	conclude1D(desc, cache);
}

void solveConcludeContact			(const PxSolverConstraintDesc& desc, SolverContext& cache)
{
	solveContact(desc, cache);
	concludeContact(desc, cache);
}

void solveConcludeContact_BStatic	(const PxSolverConstraintDesc& desc, SolverContext& cache)
{
	solveContact_BStatic(desc, cache);
	concludeContact(desc, cache);
}


}

}

