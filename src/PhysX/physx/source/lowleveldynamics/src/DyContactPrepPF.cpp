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
#include "PsMathUtils.h"
#include "DySolverContact.h"
#include "DySolverContactPF.h"
#include "DySolverConstraintTypes.h"
#include "PxcNpWorkUnit.h"
#include "DyThreadContext.h"
#include "DyContactPrep.h"
#include "PxcNpContactPrepShared.h"
//#include "PxvGeometry.h"
#include "PxvDynamics.h"
#include "DyCorrelationBuffer.h"
#include "DySolverConstraintDesc.h"
#include "DySolverBody.h"
#include "DySolverContact4.h"
#include "DySolverContactPF4.h"


#include "PsVecMath.h"
#include "PxContactModifyCallback.h"
#include "PxsMaterialManager.h"
#include "PxsMaterialCombiner.h"
#include "DySolverExt.h"
#include "DyArticulationContactPrep.h"
#include "DyContactPrepShared.h"

#include "PsFoundation.h"

using namespace physx::Gu;
using namespace physx::shdfnd::aos;

namespace physx
{
namespace Dy
{

bool createFinalizeSolverContactsCoulomb(PxSolverContactDesc& contactDesc,
		PxsContactManagerOutput& output,
		ThreadContext& threadContext,
		const PxReal invDtF32,
		PxReal bounceThresholdF32,
		PxReal frictionOffsetThreshold,
		PxReal correlationDistance,
		PxReal solverOffsetSlop,
		PxConstraintAllocator& constraintAllocator,
		PxFrictionType::Enum frictionType,
		Cm::SpatialVectorF* Z);

static bool setupFinalizeSolverConstraintsCoulomb(
												  Sc::ShapeInteraction* shapeInteraction,
						    const ContactBuffer& buffer,
							const CorrelationBuffer& c,
							const PxTransform& bodyFrame0,
							const PxTransform& bodyFrame1,
							PxU8* workspace,
							const PxSolverBodyData& data0,
							const PxSolverBodyData& data1,
							const PxReal invDtF32,
							PxReal bounceThresholdF32,
							PxU32 frictionPerPointCount,
							const bool hasForceThresholds,
							const bool staticBody,
							PxReal invMassScale0, PxReal invInertiaScale0, 
							PxReal invMassScale1, PxReal invInertiaScale1,
							PxReal restDist,
							const PxReal maxCCDSeparation,
							const PxReal solverOffsetSlopF32)
{   
	const FloatV ccdMaxSeparation = FLoad(maxCCDSeparation);
	const Vec3V solverOffsetSlop = V3Load(solverOffsetSlopF32);
	PxU8* PX_RESTRICT ptr = workspace;
	const FloatV zero=FZero();

	PxU8 flags = PxU8(hasForceThresholds ? SolverContactHeader::eHAS_FORCE_THRESHOLDS : 0);

	const FloatV restDistance = FLoad(restDist);

	const Vec3V bodyFrame0p = V3LoadU(bodyFrame0.p);
	const Vec3V bodyFrame1p = V3LoadU(bodyFrame1.p);

	Ps::prefetchLine(c.contactID);
	Ps::prefetchLine(c.contactID, 128);
	
	const PxU32 frictionPatchCount = c.frictionPatchCount;

	const PxU32 pointStride = sizeof(SolverContactPoint);
	const PxU32 frictionStride = sizeof(SolverContactFriction);
	const PxU8 pointHeaderType = Ps::to8(staticBody ? DY_SC_TYPE_STATIC_CONTACT : DY_SC_TYPE_RB_CONTACT);
	const PxU8 frictionHeaderType = Ps::to8(staticBody ? DY_SC_TYPE_STATIC_FRICTION : DY_SC_TYPE_FRICTION);


	const Vec3V linVel0 = V3LoadU(data0.linearVelocity);
	const Vec3V linVel1 = V3LoadU(data1.linearVelocity);
	const Vec3V angVel0 = V3LoadU(data0.angularVelocity);
	const Vec3V angVel1 = V3LoadU(data1.angularVelocity);


	const FloatV invMass0 = FLoad(data0.invMass);
	const FloatV invMass1 = FLoad(data1.invMass);

	const FloatV maxPenBias = FMax(FLoad(data0.penBiasClamp), FLoad(data1.penBiasClamp));

	// PT: the matrix is symmetric so we can read it as a PxMat33! Gets rid of 25000+ LHS.
	const PxMat33& invIn0 = reinterpret_cast<const PxMat33&>(data0.sqrtInvInertia);
	PX_ALIGN(16, const Mat33V invSqrtInertia0)
	(
		V3LoadU(invIn0.column0),
		V3LoadU(invIn0.column1),
		V3LoadU(invIn0.column2)
	);
	const PxMat33& invIn1 = reinterpret_cast<const PxMat33&>(data1.sqrtInvInertia);
	PX_ALIGN(16, const Mat33V invSqrtInertia1)
	(
		V3LoadU(invIn1.column0),
		V3LoadU(invIn1.column1),
		V3LoadU(invIn1.column2)
	);

	const FloatV invDt = FLoad(invDtF32);
	const FloatV p8 = FLoad(0.8f);
	const FloatV bounceThreshold = FLoad(bounceThresholdF32);
	const FloatV orthoThreshold = FLoad(0.70710678f);
	const FloatV eps = FLoad(0.00001f);

	const FloatV invDtp8 = FMul(invDt, p8);

	const FloatV d0 = FLoad(invMassScale0);
	const FloatV d1 = FLoad(invMassScale1);
	const FloatV nDom1fV = FNeg(d1);
	const FloatV angD0 = FLoad(invInertiaScale0);
	const FloatV angD1 = FLoad(invInertiaScale1);

	const FloatV invMass0_dom0fV = FMul(d0, invMass0);
	const FloatV invMass1_dom1fV = FMul(nDom1fV, invMass1);


	for(PxU32 i=0;i< frictionPatchCount;i++)
	{
		const PxU32 contactCount = c.frictionPatchContactCounts[i];
		if(contactCount == 0)
			continue;

		const Gu::ContactPoint* contactBase0 = buffer.contacts + c.contactPatches[c.correlationListHeads[i]].start;

		const Vec3V normal = Ps::aos::V3LoadA(contactBase0->normal);

		const FloatV normalLenSq = V3LengthSq(normal);
		const VecCrossV norCross = V3PrepareCross(normal);

		const FloatV restitution = FLoad(contactBase0->restitution);

		const FloatV norVel = V3SumElems(V3NegMulSub(normal, linVel1, V3Mul(normal, linVel0)));
		/*const FloatV norVel0 = V3Dot(normal, linVel0);
		const FloatV norVel1 = V3Dot(normal, linVel1);
		const FloatV norVel = FSub(norVel0, norVel1);*/

		const FloatV invMassNorLenSq0 = FMul(invMass0_dom0fV, normalLenSq);
		const FloatV invMassNorLenSq1 = FMul(invMass1_dom1fV, normalLenSq);
	
		
		SolverContactCoulombHeader* PX_RESTRICT header = reinterpret_cast<SolverContactCoulombHeader*>(ptr);
		ptr += sizeof(SolverContactCoulombHeader);

		Ps::prefetchLine(ptr, 128);
		Ps::prefetchLine(ptr, 256);
		Ps::prefetchLine(ptr, 384);


		header->numNormalConstr		= PxU8(contactCount);
		header->type				= pointHeaderType;
		//header->setRestitution(n.restitution);
		//header->setRestitution(contactBase0->restitution);
		
		header->setDominance0(invMass0_dom0fV);
		header->setDominance1(FNeg(invMass1_dom1fV));
		FStore(angD0, &header->angDom0);
		FStore(angD1, &header->angDom1);
		header->setNormal(normal);
		header->flags = flags;
		header->shapeInteraction = shapeInteraction;

		
		for(PxU32 patch=c.correlationListHeads[i]; 
			patch!=CorrelationBuffer::LIST_END; 
			patch = c.contactPatches[patch].next)
		{
			const PxU32 count = c.contactPatches[patch].count;
			const Gu::ContactPoint* contactBase = buffer.contacts + c.contactPatches[patch].start;

				
			PxU8* p = ptr;
			for(PxU32 j=0;j<count;j++)
			{
				const Gu::ContactPoint& contact = contactBase[j];

				SolverContactPoint* PX_RESTRICT solverContact = reinterpret_cast<SolverContactPoint*>(p);
				p += pointStride;

				constructContactConstraint(invSqrtInertia0, invSqrtInertia1, invMassNorLenSq0, 
					invMassNorLenSq1, angD0, angD1, bodyFrame0p, bodyFrame1p,
					normal, norVel, norCross, angVel0, angVel1,
					invDt, invDtp8, restDistance, maxPenBias,  restitution,
					bounceThreshold, contact, *solverContact, ccdMaxSeparation, solverOffsetSlop);
			}			
			ptr = p;
		}
	}

	//construct all the frictions

	PxU8* PX_RESTRICT ptr2 = workspace;

	bool hasFriction = false;
	for(PxU32 i=0;i< frictionPatchCount;i++)
	{
		const PxU32 contactCount = c.frictionPatchContactCounts[i];
		if(contactCount == 0)
			continue;

		const Gu::ContactPoint* contactBase0 = buffer.contacts + c.contactPatches[c.correlationListHeads[i]].start;

		SolverContactCoulombHeader* header = reinterpret_cast<SolverContactCoulombHeader*>(ptr2); 
		header->frictionOffset = PxU16(ptr - ptr2);// + sizeof(SolverFrictionHeader);
		ptr2 += sizeof(SolverContactCoulombHeader) + header->numNormalConstr * pointStride;

		const PxReal staticFriction = contactBase0->staticFriction;
		const bool disableStrongFriction = !!(contactBase0->materialFlags & PxMaterialFlag::eDISABLE_FRICTION);
		const bool haveFriction = (disableStrongFriction == 0);
	
		SolverFrictionHeader* frictionHeader = reinterpret_cast<SolverFrictionHeader*>(ptr);
		frictionHeader->numNormalConstr = Ps::to8(c.frictionPatchContactCounts[i]);
		frictionHeader->numFrictionConstr = Ps::to8(haveFriction ? c.frictionPatchContactCounts[i] * frictionPerPointCount : 0);
		ptr += sizeof(SolverFrictionHeader);
		PxF32* appliedForceBuffer = reinterpret_cast<PxF32*>(ptr);
		ptr += frictionHeader->getAppliedForcePaddingSize(c.frictionPatchContactCounts[i]);
		PxMemZero(appliedForceBuffer, sizeof(PxF32)*contactCount*frictionPerPointCount);
		Ps::prefetchLine(ptr, 128);
		Ps::prefetchLine(ptr, 256);
		Ps::prefetchLine(ptr, 384);

		const Vec3V normal = V3LoadU(buffer.contacts[c.contactPatches[c.correlationListHeads[i]].start].normal);

		const FloatV normalX = V3GetX(normal);
		const FloatV normalY = V3GetY(normal);
		const FloatV normalZ = V3GetZ(normal);
		
		const Vec3V t0Fallback1 = V3Merge(zero, FNeg(normalZ), normalY);
		const Vec3V t0Fallback2 = V3Merge(FNeg(normalY), normalX, zero) ;

		const BoolV con = FIsGrtr(orthoThreshold, FAbs(normalX));
		const Vec3V tFallback1 = V3Sel(con, t0Fallback1, t0Fallback2);

		const Vec3V linVrel = V3Sub(linVel0, linVel1);
		const Vec3V t0_ = V3Sub(linVrel, V3Scale(normal, V3Dot(normal, linVrel)));
		const FloatV sqDist = V3Dot(t0_,t0_);
		const BoolV con1 = FIsGrtr(sqDist, eps);
		const Vec3V tDir0 =V3Normalize(V3Sel(con1, t0_, tFallback1));
		const Vec3V tDir1 = V3Cross(tDir0, normal);

		Vec3V tFallback = tDir0;
		Vec3V tFallbackAlt = tDir1;

		if(haveFriction)
		{
			//frictionHeader->setStaticFriction(n.staticFriction);
			frictionHeader->setStaticFriction(staticFriction);
			FStore(invMass0_dom0fV, &frictionHeader->invMass0D0);
			FStore(FNeg(invMass1_dom1fV), &frictionHeader->invMass1D1);
			FStore(angD0, &frictionHeader->angDom0);
			FStore(angD1, &frictionHeader->angDom1);
			frictionHeader->type			= frictionHeaderType;
			
			PxU32 totalPatchContactCount = 0;
		
			for(PxU32 patch=c.correlationListHeads[i]; 
				patch!=CorrelationBuffer::LIST_END; 
				patch = c.contactPatches[patch].next)
			{
				const PxU32 count = c.contactPatches[patch].count;
				const PxU32 start = c.contactPatches[patch].start;
				const Gu::ContactPoint* contactBase = buffer.contacts + start;
					
				PxU8* p = ptr;
				for(PxU32 j =0; j < count; j++)
				{
					hasFriction = true;
					const Gu::ContactPoint& contact = contactBase[j];
					const Vec3V point = V3LoadU(contact.point);
					Vec3V ra = V3Sub(point, bodyFrame0p);
					Vec3V rb = V3Sub(point, bodyFrame1p);
					ra = V3Sel(V3IsGrtr(solverOffsetSlop, V3Abs(ra)), V3Zero(), ra);
					rb = V3Sel(V3IsGrtr(solverOffsetSlop, V3Abs(rb)), V3Zero(), rb);
					const Vec3V targetVel = V3LoadU(contact.targetVel);

					for(PxU32 k = 0; k < frictionPerPointCount; ++k)
					{
						const Vec3V t0 = tFallback;
						tFallback = tFallbackAlt;
						tFallbackAlt = t0;

						SolverContactFriction* PX_RESTRICT f0 = reinterpret_cast<SolverContactFriction*>(p);
						p += frictionStride;
						//f0->brokenOrContactIndex = contactId;

						const Vec3V raXn = V3Cross(ra, t0);
						const Vec3V rbXn = V3Cross(rb, t0);

						const Vec3V delAngVel0 = M33MulV3(invSqrtInertia0, raXn);
						const Vec3V delAngVel1 = M33MulV3(invSqrtInertia1, rbXn);

						const FloatV resp0 = FAdd(invMass0_dom0fV, FMul(angD0, V3Dot(delAngVel0, delAngVel0)));
						const FloatV resp1 = FSub(FMul(angD1, V3Dot(delAngVel1, delAngVel1)), invMass1_dom1fV);
						const FloatV resp = FAdd(resp0, resp1);

						const FloatV velMultiplier = FNeg(FSel(FIsGrtr(resp, zero), FRecip(resp), zero));

						const FloatV vrel1 = FAdd(V3Dot(t0, linVel0), V3Dot(raXn, angVel0));
						const FloatV vrel2 = FAdd(V3Dot(t0, linVel1), V3Dot(rbXn, angVel1));
						const FloatV vrel = FSub(vrel1, vrel2);


						f0->normalXYZ_appliedForceW = V4SetW(Vec4V_From_Vec3V(t0), zero);
						f0->raXnXYZ_velMultiplierW = V4SetW(Vec4V_From_Vec3V(delAngVel0), velMultiplier);
						//f0->rbXnXYZ_targetVelocityW = V4SetW(Vec4V_From_Vec3V(delAngVel1), FSub(V3Dot(targetVel, t0), vrel));
						f0->rbXnXYZ_biasW = Vec4V_From_Vec3V(delAngVel1);
						FStore(FSub(V3Dot(targetVel, t0), vrel), &f0->targetVel);
					}
				}

				totalPatchContactCount += c.contactPatches[patch].count;
				
				ptr = p;	
			}
		}
	}
	*ptr = 0;
	return hasFriction;
}



static void computeBlockStreamByteSizesCoulomb(const CorrelationBuffer& c,
													 const PxU32 frictionCountPerPoint, PxU32& _solverConstraintByteSize,
													 PxU32& _axisConstraintCount,
													 bool useExtContacts)
{
	PX_ASSERT(0 == _solverConstraintByteSize);
	PX_ASSERT(0 == _axisConstraintCount);

	// PT: use local vars to remove LHS
	PxU32 solverConstraintByteSize = 0;
	PxU32 numFrictionPatches = 0;
	PxU32 axisConstraintCount = 0;

	for(PxU32 i = 0; i < c.frictionPatchCount; i++)
	{
		//Friction patches.
		if(c.correlationListHeads[i] != CorrelationBuffer::LIST_END)
			numFrictionPatches++;


		const FrictionPatch& frictionPatch = c.frictionPatches[i];
		const bool haveFriction = (frictionPatch.materialFlags & PxMaterialFlag::eDISABLE_FRICTION) == 0;

		//Solver constraint data.
		if(c.frictionPatchContactCounts[i]!=0)
		{
			solverConstraintByteSize += sizeof(SolverContactCoulombHeader);
			
			solverConstraintByteSize += useExtContacts ? c.frictionPatchContactCounts[i] * sizeof(SolverContactPointExt) 
				: c.frictionPatchContactCounts[i] * sizeof(SolverContactPoint);

			axisConstraintCount += c.frictionPatchContactCounts[i];

			//We always need the friction headers to write the accumulated 
			if(haveFriction)
			{
				//4 bytes
				solverConstraintByteSize += sizeof(SolverFrictionHeader);
				//buffer to store applied forces in
				solverConstraintByteSize += SolverFrictionHeader::getAppliedForcePaddingSize(c.frictionPatchContactCounts[i]);

				const PxU32 nbFrictionConstraints = c.frictionPatchContactCounts[i] * frictionCountPerPoint;

				solverConstraintByteSize += useExtContacts ? nbFrictionConstraints * sizeof(SolverContactFrictionExt)
					: nbFrictionConstraints * sizeof(SolverContactFriction);
				axisConstraintCount += c.frictionPatchContactCounts[i];
			}
			else
			{
				//reserve buffers for storing accumulated impulses
				solverConstraintByteSize += sizeof(SolverFrictionHeader);
				solverConstraintByteSize += SolverFrictionHeader::getAppliedForcePaddingSize(c.frictionPatchContactCounts[i]);
			}
		}
	}  
	_axisConstraintCount = axisConstraintCount;

	//16-byte alignment.
	_solverConstraintByteSize =  ((solverConstraintByteSize + 0x0f) & ~0x0f);
	PX_ASSERT(0 == (_solverConstraintByteSize & 0x0f));
}

static bool reserveBlockStreamsCoulomb(const CorrelationBuffer& c,
						PxU8*& solverConstraint, PxU32 frictionCountPerPoint,
						PxU32& solverConstraintByteSize,
						PxU32& axisConstraintCount, PxConstraintAllocator& constraintAllocator,
						bool useExtContacts)
{
	PX_ASSERT(NULL == solverConstraint);
	PX_ASSERT(0 == solverConstraintByteSize);
	PX_ASSERT(0 == axisConstraintCount);
	

	//From constraintBlockStream we need to reserve contact points, contact forces, and a char buffer for the solver constraint data (already have a variable for this).
	//From frictionPatchStream we just need to reserve a single buffer.

	//Compute the sizes of all the buffers.
	computeBlockStreamByteSizesCoulomb(
		c,
		frictionCountPerPoint, solverConstraintByteSize,
		axisConstraintCount, useExtContacts);

	//Reserve the buffers.

	//First reserve the accumulated buffer size for the constraint block.
	PxU8* constraintBlock = NULL;
	const PxU32 constraintBlockByteSize = solverConstraintByteSize;
	if(constraintBlockByteSize > 0)
	{
		constraintBlock = constraintAllocator.reserveConstraintData(constraintBlockByteSize + 16u);

		if(0==constraintBlock || (reinterpret_cast<PxU8*>(-1))==constraintBlock)
		{
			if(0==constraintBlock)
			{
				PX_WARN_ONCE(
					"Reached limit set by PxSceneDesc::maxNbContactDataBlocks - ran out of buffer space for constraint prep. "
					"Either accept dropped contacts or increase buffer size allocated for narrow phase by increasing PxSceneDesc::maxNbContactDataBlocks.");
			}
			else
			{
				PX_WARN_ONCE(
					"Attempting to allocate more than 16K of contact data for a single contact pair in constraint prep. "
					"Either accept dropped contacts or simplify collision geometry.");
				constraintBlock=NULL;
			}
		}
	}

	//Patch up the individual ptrs to the buffer returned by the constraint block reservation (assuming the reservation didn't fail).
	if(0==constraintBlockByteSize || constraintBlock)
	{
		if(solverConstraintByteSize)
		{
			solverConstraint = constraintBlock;
			PX_ASSERT(0==(uintptr_t(solverConstraint) & 0x0f));
		}
	}

	//Return true if neither of the two block reservations failed.
	return ((0==constraintBlockByteSize || constraintBlock));
}

bool createFinalizeSolverContactsCoulomb1D(PxSolverContactDesc& contactDesc,
	PxsContactManagerOutput& output,
	ThreadContext& threadContext,
	const PxReal invDtF32,
	PxReal bounceThresholdF32,
	PxReal frictionOffsetThreshold,
	PxReal correlationDistance,
	PxReal solverOffsetSlop,
	PxConstraintAllocator& constraintAllocator,
	Cm::SpatialVectorF* Z)
{
	return createFinalizeSolverContactsCoulomb(contactDesc, output, threadContext, invDtF32, bounceThresholdF32, frictionOffsetThreshold, correlationDistance, solverOffsetSlop,
		constraintAllocator, PxFrictionType::eONE_DIRECTIONAL, Z);
}

bool createFinalizeSolverContactsCoulomb2D(PxSolverContactDesc& contactDesc,
	PxsContactManagerOutput& output,
	ThreadContext& threadContext,
	const PxReal invDtF32,
	PxReal bounceThresholdF32,
	PxReal frictionOffsetThreshold,
	PxReal correlationDistance,
	PxReal solverOffsetSlop,
	PxConstraintAllocator& constraintAllocator,
	Cm::SpatialVectorF* Z)

{
	return createFinalizeSolverContactsCoulomb(contactDesc, output, threadContext, invDtF32, bounceThresholdF32, frictionOffsetThreshold, correlationDistance, solverOffsetSlop,
		constraintAllocator, PxFrictionType::eTWO_DIRECTIONAL, Z);
}

bool createFinalizeSolverContactsCoulomb(PxSolverContactDesc& contactDesc,
									PxsContactManagerOutput& output,
								 ThreadContext& threadContext,
								 const PxReal invDtF32,
								 PxReal bounceThresholdF32,
								 PxReal frictionOffsetThreshold,
								 PxReal correlationDistance,
								 PxReal solverOffsetSlop,
								 PxConstraintAllocator& constraintAllocator,
								 PxFrictionType::Enum frictionType,
								 Cm::SpatialVectorF* Z)
{
	PX_UNUSED(frictionOffsetThreshold);
	PX_UNUSED(correlationDistance);

	PxSolverConstraintDesc& desc = *contactDesc.desc;

	desc.constraintLengthOver16 = 0;
	
	ContactBuffer& buffer = threadContext.mContactBuffer;

	buffer.count = 0;

	// We pull the friction patches out of the cache to remove the dependency on how
	// the cache is organized. Remember original addrs so we can write them back 
	// efficiently.

	Ps::prefetchLine(contactDesc.frictionPtr);

	PxReal invMassScale0 = 1.f;
	PxReal invMassScale1 = 1.f;
	PxReal invInertiaScale0 = 1.f;
	PxReal invInertiaScale1 = 1.f;

	bool hasMaxImpulse = false, hasTargetVelocity = false;
	
	PxU32 numContacts = extractContacts(buffer, output, hasMaxImpulse, hasTargetVelocity, invMassScale0, invMassScale1, 
			invInertiaScale0, invInertiaScale1, PxMin(contactDesc.data0->maxContactImpulse, contactDesc.data1->maxContactImpulse));

	if(numContacts == 0)
	{
		contactDesc.frictionPtr = NULL;
		contactDesc.frictionCount = 0;
		return true;
	}

	Ps::prefetchLine(contactDesc.body0);
	Ps::prefetchLine(contactDesc.body1);
	Ps::prefetchLine(contactDesc.data0);
	Ps::prefetchLine(contactDesc.data1);

	CorrelationBuffer& c = threadContext.mCorrelationBuffer;
	c.frictionPatchCount = 0;
	c.contactPatchCount = 0;

	createContactPatches(c, buffer.contacts, buffer.count, PXC_SAME_NORMAL);	

	PxU32 numFrictionPerPatch = PxU32(frictionType == PxFrictionType::eONE_DIRECTIONAL ? 1 : 2);
	
	bool overflow = correlatePatches(c, buffer.contacts, contactDesc.bodyFrame0, contactDesc.bodyFrame1, PXC_SAME_NORMAL, 0, 0);
	PX_UNUSED(overflow);
#if PX_CHECKED
	if(overflow)
	{
		Ps::getFoundation().error(physx::PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, 
					"Dropping contacts in solver because we exceeded limit of 32 friction patches.");
	}
#endif


	//PX_ASSERT(patchCount == c.frictionPatchCount);

	PxU8* solverConstraint = NULL;
	PxU32 solverConstraintByteSize = 0;
	PxU32 axisConstraintCount = 0;

	bool useExtContacts = !!((contactDesc.bodyState0 | contactDesc.bodyState1) & PxSolverContactDesc::eARTICULATION);

	const bool successfulReserve = reserveBlockStreamsCoulomb(
		c,
		solverConstraint, numFrictionPerPatch,
		solverConstraintByteSize,
		axisConstraintCount,
		constraintAllocator,
		useExtContacts);

	// initialise the work unit's ptrs to the various buffers.

	contactDesc.frictionPtr = NULL;
	desc.constraint = NULL;
	desc.constraintLengthOver16 = 0;
	contactDesc.frictionCount = 0;
	
	// patch up the work unit with the reserved buffers and set the reserved buffer data as appropriate.

	if(successfulReserve)
	{
		desc.constraint = solverConstraint;
		output.nbContacts = Ps::to8(numContacts);
		desc.constraintLengthOver16 = Ps::to16(solverConstraintByteSize/16);

		//Initialise solverConstraint buffer.
		if(solverConstraint)
		{
			bool hasFriction = false;
			if(useExtContacts)
			{
				const PxSolverBodyData& data0 = *contactDesc.data0;
				const PxSolverBodyData& data1 = *contactDesc.data1;

				const SolverExtBody b0(reinterpret_cast<const void*>(contactDesc.body0), reinterpret_cast<const void*>(&data0), desc.linkIndexA);
				const SolverExtBody b1(reinterpret_cast<const void*>(contactDesc.body1), reinterpret_cast<const void*>(&data1), desc.linkIndexB);

				hasFriction = setupFinalizeExtSolverContactsCoulomb(buffer, c, contactDesc.bodyFrame0, contactDesc.bodyFrame1, solverConstraint,
					invDtF32, bounceThresholdF32, b0, b1, numFrictionPerPatch,
					invMassScale0, invInertiaScale0, invMassScale1, invInertiaScale1, contactDesc.restDistance, contactDesc.maxCCDSeparation, Z);
			}
			else
			{
				const PxSolverBodyData& data0 = *contactDesc.data0;
				const PxSolverBodyData& data1 = *contactDesc.data1;

				hasFriction = setupFinalizeSolverConstraintsCoulomb(contactDesc.shapeInteraction, buffer, c, contactDesc.bodyFrame0, contactDesc.bodyFrame1, solverConstraint,
					data0, data1, invDtF32, bounceThresholdF32, numFrictionPerPatch, contactDesc.hasForceThresholds, contactDesc.bodyState1 == PxSolverContactDesc::eSTATIC_BODY,
					invMassScale0, invInertiaScale0, invMassScale1, invInertiaScale1, contactDesc.restDistance, contactDesc.maxCCDSeparation, solverOffsetSlop);
			}
			*(reinterpret_cast<PxU32*>(solverConstraint + solverConstraintByteSize)) = 0;
			*(reinterpret_cast<PxU32*>(solverConstraint + solverConstraintByteSize + 4)) = hasFriction ? 0xFFFFFFFF : 0;
		}
	}

	return successfulReserve;
}

}
}


