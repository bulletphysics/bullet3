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
#include "DyArticulationContactPrep.h"
#include "DySolverConstraintDesc.h"
#include "DySolverConstraint1D.h"
#include "DyArticulationHelper.h"
#include "PxcNpWorkUnit.h"
#include "PxsMaterialManager.h"
#include "PxsMaterialCombiner.h"
#include "DyCorrelationBuffer.h"
#include "DySolverConstraintExtShared.h"
#include "DyConstraintPrep.h"

using namespace physx::Gu;

namespace physx
{

namespace Dy
{

// constraint-gen only, since these use getVelocity methods
// which aren't valid during the solver phase

PX_INLINE void computeFrictionTangents(const PxVec3& vrel,const PxVec3& unitNormal, PxVec3& t0, PxVec3& t1)
{
	PX_ASSERT(PxAbs(unitNormal.magnitude()-1)<1e-3f);

	t0 = vrel - unitNormal * unitNormal.dot(vrel);
	PxReal ll = t0.magnitudeSquared();

	if (ll > 1e10f)										//can set as low as 0.
	{
		t0 *= PxRecipSqrt(ll);
		t1 = unitNormal.cross(t0);
	}
	else
		Ps::normalToTangents(unitNormal, t0, t1);		//fallback
}

PxReal SolverExtBody::projectVelocity(const PxVec3& linear, const PxVec3& angular) const
{
	if(mLinkIndex == PxSolverConstraintDesc::NO_LINK)
	{
		return mBodyData->projectVelocity(linear, angular);
	}
	else
	{
		const Cm::SpatialVectorV velocity = mArticulation->getLinkVelocity(mLinkIndex);

		FloatV fv = velocity.dot(Cm::SpatialVector(linear, angular));

		PxF32 f;
		FStore(fv, &f);
		return f;
		/*PxF32 f;
		FStore(getVelocity(*mFsData)[mLinkIndex].dot(Cm::SpatialVector(linear, angular)), &f);
		return f;*/
	}
}

PxVec3 SolverExtBody::getLinVel() const
{
	if(mLinkIndex == PxSolverConstraintDesc::NO_LINK)
		return mBodyData->linearVelocity;
	else
	{
		Vec3V linear = mArticulation->getLinkVelocity(mLinkIndex).linear;

		PxVec3 result;
		V3StoreU(linear, result);
		/*PxVec3 result;
		V3StoreU(getVelocity(*mFsData)[mLinkIndex].linear, result);*/
		return result;
	}
}


PxVec3 SolverExtBody::getAngVel() const
{
	if(mLinkIndex == PxSolverConstraintDesc::NO_LINK)
		return mBodyData->angularVelocity;
	else
	{

		Vec3V angular = mArticulation->getLinkVelocity(mLinkIndex).angular;
		
		PxVec3 result;
		V3StoreU(angular, result);
		/*PxVec3 result;
		V3StoreU(getVelocity(*mFsData)[mLinkIndex].angular, result);*/
		return result;
	}
}

Cm::SpatialVector createImpulseResponseVector(const PxVec3& linear, const PxVec3& angular, const SolverExtBody& body)
{
	if(body.mLinkIndex == PxSolverConstraintDesc::NO_LINK)
	{
		return Cm::SpatialVector(linear, body.mBodyData->sqrtInvInertia * angular);
	}
	return Cm::SpatialVector(linear, angular);
}

PxReal getImpulseResponse(const SolverExtBody& b0, const Cm::SpatialVector& impulse0, Cm::SpatialVector& deltaV0, PxReal dom0, PxReal angDom0, 
	const SolverExtBody& b1, const Cm::SpatialVector& impulse1, Cm::SpatialVector& deltaV1, PxReal dom1, PxReal angDom1, 
	Cm::SpatialVectorF* Z, bool allowSelfCollision)
{
	PxReal response;
	allowSelfCollision = true;
	// right now self-collision with contacts crashes the solver
	
	//KS - knocked this out to save some space on SPU
	if(allowSelfCollision && b0.mLinkIndex!=PxSolverConstraintDesc::NO_LINK && b0.mArticulation == b1.mArticulation && 0)
	{
		/*ArticulationHelper::getImpulseSelfResponse(*b0.mFsData,b0.mLinkIndex, impulse0, deltaV0, 
													  b1.mLinkIndex, impulse1, deltaV1);*/

		b0.mArticulation->getImpulseSelfResponse(b0.mLinkIndex, b1.mLinkIndex, Z, impulse0.scale(dom0, angDom0), impulse1.scale(dom1, angDom1), 
			deltaV0, deltaV1);
		
		response = impulse0.dot(deltaV0) + impulse1.dot(deltaV1);

	}
	else 
	{
		
		if(b0.mLinkIndex == PxSolverConstraintDesc::NO_LINK)
		{
			deltaV0.linear = impulse0.linear * b0.mBodyData->invMass * dom0;
			deltaV0.angular = impulse0.angular * angDom0;
		}
		else
		{
			b0.mArticulation->getImpulseResponse(b0.mLinkIndex, Z, impulse0.scale(dom0, angDom0), deltaV0);
		}

		response = impulse0.dot(deltaV0);
		if(b1.mLinkIndex == PxSolverConstraintDesc::NO_LINK)
		{
			deltaV1.linear = impulse1.linear * b1.mBodyData->invMass * dom1;
			deltaV1.angular = impulse1.angular * angDom1;
		}
		else
		{
			b1.mArticulation->getImpulseResponse( b1.mLinkIndex, Z, impulse1.scale(dom1, angDom1), deltaV1);
		}
		response += impulse1.dot(deltaV1);
	}

	return response;
}


void setupFinalizeExtSolverContacts(
	const ContactPoint* buffer,
	const CorrelationBuffer& c,
	const PxTransform& bodyFrame0,
	const PxTransform& bodyFrame1,
	PxU8* workspace,
	const SolverExtBody& b0,
	const SolverExtBody& b1,
	const PxReal invDtF32,
	PxReal bounceThresholdF32,
	PxReal invMassScale0, PxReal invInertiaScale0,
	PxReal invMassScale1, PxReal invInertiaScale1,
	const PxReal restDist,
	PxU8* frictionDataPtr,
	PxReal ccdMaxContactDist,
	Cm::SpatialVectorF* Z)
{
	// NOTE II: the friction patches are sparse (some of them have no contact patches, and
	// therefore did not get written back to the cache) but the patch addresses are dense,
	// corresponding to valid patches

	/*const bool haveFriction = PX_IR(n.staticFriction) > 0 || PX_IR(n.dynamicFriction) > 0;*/

	const FloatV ccdMaxSeparation = FLoad(ccdMaxContactDist);

	PxU8* PX_RESTRICT ptr = workspace;

	const FloatV zero = FZero();

	//KS - TODO - this should all be done in SIMD to avoid LHS
	//const PxF32 maxPenBias0 = b0.mLinkIndex == PxSolverConstraintDesc::NO_LINK ? b0.mBodyData->penBiasClamp : getMaxPenBias(*b0.mFsData)[b0.mLinkIndex];
	//const PxF32 maxPenBias1 = b1.mLinkIndex == PxSolverConstraintDesc::NO_LINK ? b1.mBodyData->penBiasClamp : getMaxPenBias(*b1.mFsData)[b1.mLinkIndex];

	PxF32 maxPenBias0 = b0.mBodyData->penBiasClamp;
	PxF32 maxPenBias1 = b1.mBodyData->penBiasClamp;

	if (b0.mLinkIndex != PxSolverConstraintDesc::NO_LINK)
	{
		maxPenBias0 = b0.mArticulation->getLinkMaxPenBias(b0.mLinkIndex);
	}

	if (b1.mLinkIndex != PxSolverConstraintDesc::NO_LINK)
	{
		maxPenBias1 = b1.mArticulation->getLinkMaxPenBias(b1.mLinkIndex);
	}

	const FloatV maxPenBias = FLoad(PxMax(maxPenBias0, maxPenBias1));


	const PxReal d0 = invMassScale0;
	const PxReal d1 = invMassScale1;

	const PxReal angD0 = invInertiaScale0;
	const PxReal angD1 = invInertiaScale1;

	Vec4V staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4Zero();
	staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W=V4SetZ(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, FLoad(d0));
	staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W=V4SetW(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, FLoad(d1));

	const FloatV restDistance = FLoad(restDist); 

	PxU32 frictionPatchWritebackAddrIndex = 0;
	PxU32 contactWritebackCount = 0;

	Ps::prefetchLine(c.contactID);
	Ps::prefetchLine(c.contactID, 128);

	const FloatV invDt = FLoad(invDtF32);
	const FloatV p8 = FLoad(0.8f);
	const FloatV bounceThreshold = FLoad(bounceThresholdF32);

	const FloatV invDtp8 = FMul(invDt, p8);

	PxU8 flags = 0;

	for(PxU32 i=0;i<c.frictionPatchCount;i++)
	{
		PxU32 contactCount = c.frictionPatchContactCounts[i];
		if(contactCount == 0)
			continue;

		const FrictionPatch& frictionPatch = c.frictionPatches[i];
		PX_ASSERT(frictionPatch.anchorCount <= 2);  //0==anchorCount is allowed if all the contacts in the manifold have a large offset. 

		const Gu::ContactPoint* contactBase0 = buffer + c.contactPatches[c.correlationListHeads[i]].start;
		const PxReal combinedRestitution = contactBase0->restitution;

		PxReal coefficient = (contactBase0->materialFlags & PxMaterialFlag::eIMPROVED_PATCH_FRICTION && frictionPatch.anchorCount == 2) ? 0.5f : 1.f;

		const PxReal staticFriction = contactBase0->staticFriction * coefficient;
		const PxReal dynamicFriction = contactBase0->dynamicFriction * coefficient;
		const bool disableStrongFriction = !!(contactBase0->materialFlags & PxMaterialFlag::eDISABLE_FRICTION);
		staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W=V4SetX(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, FLoad(staticFriction));
		staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W=V4SetY(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, FLoad(dynamicFriction));
	
		SolverContactHeader* PX_RESTRICT header = reinterpret_cast<SolverContactHeader*>(ptr);
		ptr += sizeof(SolverContactHeader);		


		Ps::prefetchLine(ptr + 128);
		Ps::prefetchLine(ptr + 256);
		Ps::prefetchLine(ptr + 384);
		
		const bool haveFriction = (disableStrongFriction == 0) ;//PX_IR(n.staticFriction) > 0 || PX_IR(n.dynamicFriction) > 0;
		header->numNormalConstr		= Ps::to8(contactCount);
		header->numFrictionConstr	= Ps::to8(haveFriction ? frictionPatch.anchorCount*2 : 0);
	
		header->type				= Ps::to8(DY_SC_TYPE_EXT_CONTACT);

		header->flags = flags;

		const FloatV restitution = FLoad(combinedRestitution);
	
		header->staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W;

		header->angDom0 = angD0;
		header->angDom1 = angD1;
	
		const PxU32 pointStride = sizeof(SolverContactPointExt);
		const PxU32 frictionStride = sizeof(SolverContactFrictionExt);

		const Vec3V normal = V3LoadU(buffer[c.contactPatches[c.correlationListHeads[i]].start].normal);
		
		FloatV accumImpulse = FZero();

		for(PxU32 patch=c.correlationListHeads[i]; 
			patch!=CorrelationBuffer::LIST_END; 
			patch = c.contactPatches[patch].next)
		{
			const PxU32 count = c.contactPatches[patch].count;
			const Gu::ContactPoint* contactBase = buffer + c.contactPatches[patch].start;
				
			PxU8* p = ptr;

			

			for(PxU32 j=0;j<count;j++)
			{
				const Gu::ContactPoint& contact = contactBase[j];

				SolverContactPointExt* PX_RESTRICT solverContact = reinterpret_cast<SolverContactPointExt*>(p);
				p += pointStride;

				accumImpulse = FAdd(accumImpulse, setupExtSolverContact(b0, b1, d0, d1, angD0, angD1, bodyFrame0, bodyFrame1, normal, invDt, invDtp8, restDistance, maxPenBias, restitution,
					bounceThreshold, contact, *solverContact, ccdMaxSeparation, Z));
			
			}

			ptr = p;
		}
		contactWritebackCount += contactCount;

		accumImpulse = FDiv(accumImpulse, FLoad(PxF32(contactCount)));

		header->normal_minAppliedImpulseForFrictionW = V4SetW(Vec4V_From_Vec3V(normal), accumImpulse);

		PxF32* forceBuffer = reinterpret_cast<PxF32*>(ptr);
		PxMemZero(forceBuffer, sizeof(PxF32) * contactCount);
		ptr += sizeof(PxF32) * ((contactCount + 3) & (~3));

		header->broken = 0;

		if(haveFriction)
		{
			//const Vec3V normal = Vec3V_From_PxVec3(buffer.contacts[c.contactPatches[c.correlationListHeads[i]].start].normal);
			PxVec3 normalS = buffer[c.contactPatches[c.correlationListHeads[i]].start].normal;

			PxVec3 t0, t1;
			computeFrictionTangents(b0.getLinVel() - b1.getLinVel(), normalS, t0, t1);

			Vec3V vT0 = V3LoadU(t0);
			Vec3V vT1 = V3LoadU(t1);
			
			//We want to set the writeBack ptr to point to the broken flag of the friction patch.
			//On spu we have a slight problem here because the friction patch array is 
			//in local store rather than in main memory. The good news is that the address of the friction 
			//patch array in main memory is stored in the work unit. These two addresses will be equal 
			//except on spu where one is local store memory and the other is the effective address in main memory.
			//Using the value stored in the work unit guarantees that the main memory address is used on all platforms.
			PxU8* PX_RESTRICT writeback = frictionDataPtr + frictionPatchWritebackAddrIndex*sizeof(FrictionPatch);

			header->frictionBrokenWritebackByte = writeback;			

			for(PxU32 j = 0; j < frictionPatch.anchorCount; j++)
			{
				SolverContactFrictionExt* PX_RESTRICT f0 = reinterpret_cast<SolverContactFrictionExt*>(ptr);
				ptr += frictionStride;
				SolverContactFrictionExt* PX_RESTRICT f1 = reinterpret_cast<SolverContactFrictionExt*>(ptr);
				ptr += frictionStride;

				PxVec3 ra = bodyFrame0.q.rotate(frictionPatch.body0Anchors[j]);
				PxVec3 rb = bodyFrame1.q.rotate(frictionPatch.body1Anchors[j]);
				PxVec3 error = (ra + bodyFrame0.p) - (rb + bodyFrame1.p);

				{
					const PxVec3 raXn = ra.cross(t0);
					const PxVec3 rbXn = rb.cross(t0);

					Cm::SpatialVector deltaV0, deltaV1;

					const Cm::SpatialVector resp0 = createImpulseResponseVector(t0, raXn, b0);
					const Cm::SpatialVector resp1 = createImpulseResponseVector(-t0, -rbXn, b1);
					FloatV resp = FLoad(getImpulseResponse(b0, resp0, deltaV0, d0, angD0,
															 b1, resp1, deltaV1, d1, angD1, Z));

					const FloatV velMultiplier = FSel(FIsGrtr(resp, FLoad(DY_ARTICULATION_MIN_RESPONSE)), FDiv(p8, resp), zero);

					PxU32 index = c.contactPatches[c.correlationListHeads[i]].start;
					PxF32 targetVel = buffer[index].targetVel.dot(t0);

					if(b0.mLinkIndex == PxSolverConstraintDesc::NO_LINK)
						targetVel -= b0.projectVelocity(t0, raXn);
					else if(b1.mLinkIndex == PxSolverConstraintDesc::NO_LINK)
						targetVel += b1.projectVelocity(t0, rbXn);

					f0->normalXYZ_appliedForceW = V4SetW(vT0, zero);
					f0->raXnXYZ_velMultiplierW = V4SetW(V4LoadA(&resp0.angular.x), velMultiplier);
					f0->rbXnXYZ_biasW = V4SetW(V4Neg(V4LoadA(&resp1.angular.x)), FLoad(t0.dot(error) * invDtF32));
					f0->linDeltaVA = V3LoadA(deltaV0.linear);
					f0->angDeltaVA = V3LoadA(deltaV0.angular);
					f0->linDeltaVB = V3LoadA(deltaV1.linear);
					f0->angDeltaVB = V3LoadA(deltaV1.angular);
					f0->targetVel = targetVel;
				}

				{

					const PxVec3 raXn = ra.cross(t1);
					const PxVec3 rbXn = rb.cross(t1);

					Cm::SpatialVector deltaV0, deltaV1;


					const Cm::SpatialVector resp0 = createImpulseResponseVector(t1, raXn, b0);
					const Cm::SpatialVector resp1 = createImpulseResponseVector(-t1, -rbXn, b1);

					FloatV resp = FLoad(getImpulseResponse(b0, resp0, deltaV0, d0, angD0,
														   b1, resp1, deltaV1, d1, angD1, Z));

					//const FloatV velMultiplier = FSel(FIsGrtr(resp, FLoad(DY_ARTICULATION_MIN_RESPONSE)), FMul(p8, FRecip(resp)), zero);

					const FloatV velMultiplier = FSel(FIsGrtr(resp, FLoad(DY_ARTICULATION_MIN_RESPONSE)), FDiv(p8, resp), zero);

					PxU32 index = c.contactPatches[c.correlationListHeads[i]].start;
					PxF32 targetVel = buffer[index].targetVel.dot(t1);

					if(b0.mLinkIndex == PxSolverConstraintDesc::NO_LINK)
						targetVel -= b0.projectVelocity(t1, raXn);
					else if(b1.mLinkIndex == PxSolverConstraintDesc::NO_LINK)
						targetVel += b1.projectVelocity(t1, rbXn);

					f1->normalXYZ_appliedForceW = V4SetW(vT1, zero);
					f1->raXnXYZ_velMultiplierW = V4SetW(V4LoadA(&resp0.angular.x), velMultiplier);
					f1->rbXnXYZ_biasW = V4SetW(V4Neg(V4LoadA(&resp1.angular.x)), FLoad(t1.dot(error) * invDtF32));
					f1->linDeltaVA = V3LoadA(deltaV0.linear);
					f1->angDeltaVA = V3LoadA(deltaV0.angular);
					f1->linDeltaVB = V3LoadA(deltaV1.linear);
					f1->angDeltaVB = V3LoadA(deltaV1.angular);
					f1->targetVel = targetVel;
				}
			}
		}

		frictionPatchWritebackAddrIndex++;
	}
}

}


}
