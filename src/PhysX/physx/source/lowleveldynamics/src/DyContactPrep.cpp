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
#include "PxSceneDesc.h"
#include "PsVecMath.h"
#include "PsMathUtils.h"
#include "DySolverContact.h"
#include "DySolverContact4.h"
#include "DySolverConstraintTypes.h"
#include "PxcNpWorkUnit.h"
#include "DyThreadContext.h"
#include "DyContactPrep.h"
#include "PxcNpContactPrepShared.h"
#include "PxvDynamics.h"
#include "DyCorrelationBuffer.h"
#include "DyDynamics.h"
#include "DyArticulationContactPrep.h"
#include "PxsContactManager.h"
#include "PsFoundation.h"
#include "DyConstraintPrep.h"

using namespace physx;
using namespace Gu;


#include "PsVecMath.h"
#include "PxContactModifyCallback.h"
#include "PxsMaterialManager.h"
#include "PxsMaterialCombiner.h"
#include "DyContactPrepShared.h"

using namespace Ps::aos;

namespace physx
{
namespace Dy
{

PxcCreateFinalizeSolverContactMethod createFinalizeMethods[3] =
{
	createFinalizeSolverContacts,
	createFinalizeSolverContactsCoulomb1D,
	createFinalizeSolverContactsCoulomb2D
};



static void setupFinalizeSolverConstraints(Sc::ShapeInteraction* shapeInteraction,
						    const ContactPoint* buffer,
							const CorrelationBuffer& c,
							const PxTransform& bodyFrame0,
							const PxTransform& bodyFrame1,
							PxU8* workspace,
							const PxSolverBodyData& data0,
							const PxSolverBodyData& data1,
							const PxReal invDtF32,
							PxReal bounceThresholdF32,
							PxReal invMassScale0, PxReal invInertiaScale0, 
							PxReal invMassScale1, PxReal invInertiaScale1, 
							bool hasForceThreshold, bool staticOrKinematicBody,
							const PxReal restDist, PxU8* frictionDataPtr,
							const PxReal maxCCDSeparation,
							const PxReal solverOffsetSlopF32)
{
	// NOTE II: the friction patches are sparse (some of them have no contact patches, and
	// therefore did not get written back to the cache) but the patch addresses are dense,
	// corresponding to valid patches

	const Vec3V solverOffsetSlop = V3Load(solverOffsetSlopF32);

	const FloatV ccdMaxSeparation = FLoad(maxCCDSeparation);

	PxU8 flags = PxU8(hasForceThreshold ? SolverContactHeader::eHAS_FORCE_THRESHOLDS : 0);

	PxU8* PX_RESTRICT ptr = workspace;

	PxU8 type = Ps::to8(staticOrKinematicBody ? DY_SC_TYPE_STATIC_CONTACT
									                 : DY_SC_TYPE_RB_CONTACT);

	const FloatV zero=FZero();
	const Vec3V v3Zero = V3Zero();

	const FloatV d0 = FLoad(invMassScale0);
	const FloatV d1 = FLoad(invMassScale1);
	const FloatV angD0 = FLoad(invInertiaScale0);
	const FloatV angD1 = FLoad(invInertiaScale1);
	
	const FloatV nDom1fV = FNeg(d1);

	const FloatV invMass0 = FLoad(data0.invMass);
	const FloatV invMass1 = FLoad(data1.invMass);

	const FloatV invMass0_dom0fV = FMul(d0, invMass0);
	const FloatV invMass1_dom1fV = FMul(nDom1fV, invMass1);


	Vec4V staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = V4Zero();
	staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W=V4SetZ(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, invMass0_dom0fV);
	staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W=V4SetW(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, invMass1_dom1fV);

	const FloatV restDistance = FLoad(restDist); 

	const FloatV maxPenBias = FMax(FLoad(data0.penBiasClamp), FLoad(data1.penBiasClamp));

	const QuatV bodyFrame0q = QuatVLoadU(&bodyFrame0.q.x);
	const Vec3V bodyFrame0p = V3LoadU(bodyFrame0.p);
	
	const QuatV bodyFrame1q = QuatVLoadU(&bodyFrame1.q.x);
	const Vec3V bodyFrame1p = V3LoadU(bodyFrame1.p);

	PxU32 frictionPatchWritebackAddrIndex = 0;
	PxU32 contactWritebackCount = 0;

	Ps::prefetchLine(c.contactID);
	Ps::prefetchLine(c.contactID, 128);

	const Vec3V linVel0 = V3LoadU_SafeReadW(data0.linearVelocity);	// PT: safe because 'invMass' follows 'initialLinVel' in PxSolverBodyData
	const Vec3V linVel1 = V3LoadU_SafeReadW(data1.linearVelocity);	// PT: safe because 'invMass' follows 'initialLinVel' in PxSolverBodyData
	const Vec3V angVel0 = V3LoadU_SafeReadW(data0.angularVelocity);	// PT: safe because 'reportThreshold' follows 'initialAngVel' in PxSolverBodyData
	const Vec3V angVel1 = V3LoadU_SafeReadW(data1.angularVelocity);	// PT: safe because 'reportThreshold' follows 'initialAngVel' in PxSolverBodyData

	PX_ALIGN(16, const Mat33V invSqrtInertia0)
	(
		V3LoadU_SafeReadW(data0.sqrtInvInertia.column0),	// PT: safe because 'column1' follows 'column0' in PxMat33
		V3LoadU_SafeReadW(data0.sqrtInvInertia.column1),	// PT: safe because 'column2' follows 'column1' in PxMat33
		V3LoadU(data0.sqrtInvInertia.column2)
	);
	
	PX_ALIGN(16, const Mat33V invSqrtInertia1)
	(
		V3LoadU_SafeReadW(data1.sqrtInvInertia.column0),	// PT: safe because 'column1' follows 'column0' in PxMat33
		V3LoadU_SafeReadW(data1.sqrtInvInertia.column1),	// PT: safe because 'column2' follows 'column1' in PxMat33
		V3LoadU(data1.sqrtInvInertia.column2)
	);

	const FloatV invDt = FLoad(invDtF32);
	const FloatV p8 = FLoad(0.8f);
	const FloatV bounceThreshold = FLoad(bounceThresholdF32);

	const FloatV invDtp8 = FMul(invDt, p8);


	for(PxU32 i=0;i<c.frictionPatchCount;i++)
	{
		PxU32 contactCount = c.frictionPatchContactCounts[i];
		if(contactCount == 0)
			continue;

		const FrictionPatch& frictionPatch = c.frictionPatches[i];
		PX_ASSERT(frictionPatch.anchorCount <= 2);

		PxU32 firstPatch = c.correlationListHeads[i];
		const Gu::ContactPoint* contactBase0 = buffer + c.contactPatches[firstPatch].start;

		const PxReal combinedRestitution = contactBase0->restitution;
		
		SolverContactHeader* PX_RESTRICT header = reinterpret_cast<SolverContactHeader*>(ptr);
		ptr += sizeof(SolverContactHeader);		


		Ps::prefetchLine(ptr, 128);
		Ps::prefetchLine(ptr, 256);

		header->shapeInteraction = shapeInteraction;
		header->flags = flags;
		FStore(invMass0_dom0fV, &header->invMass0);
		FStore(FNeg(invMass1_dom1fV), &header->invMass1);
		const FloatV restitution = FLoad(combinedRestitution);
	
		PxU32 pointStride = sizeof(SolverContactPoint);
		PxU32 frictionStride = sizeof(SolverContactFriction);

		const Vec3V normal = V3LoadA(buffer[c.contactPatches[c.correlationListHeads[i]].start].normal);
		const FloatV normalLenSq = V3LengthSq(normal);
		const VecCrossV norCross = V3PrepareCross(normal);
		const FloatV norVel = V3SumElems(V3NegMulSub(normal, linVel1, V3Mul(normal, linVel0)));

		const FloatV invMassNorLenSq0 = FMul(invMass0_dom0fV, normalLenSq);
		const FloatV invMassNorLenSq1 = FMul(invMass1_dom1fV, normalLenSq);

		header->normal_minAppliedImpulseForFrictionW = Vec4V_From_Vec3V(normal);
		
		for(PxU32 patch=c.correlationListHeads[i]; 
			patch!=CorrelationBuffer::LIST_END; 
			patch = c.contactPatches[patch].next)
		{
			const PxU32 count = c.contactPatches[patch].count;
			const Gu::ContactPoint* contactBase = buffer + c.contactPatches[patch].start;
				
			PxU8* p = ptr;
			
			for(PxU32 j=0;j<count;j++)
			{
				Ps::prefetchLine(p, 256);
				const Gu::ContactPoint& contact = contactBase[j];

				SolverContactPoint* PX_RESTRICT solverContact = reinterpret_cast<SolverContactPoint*>(p);
				p += pointStride;

				constructContactConstraint(invSqrtInertia0, invSqrtInertia1, invMassNorLenSq0, 
					invMassNorLenSq1, angD0, angD1, bodyFrame0p, bodyFrame1p,
					normal, norVel, norCross, angVel0, angVel1,
					invDt, invDtp8, restDistance, maxPenBias,  restitution,
					bounceThreshold, contact, *solverContact,
					ccdMaxSeparation, solverOffsetSlop);
			}

			ptr = p;
		}
		contactWritebackCount += contactCount;

		PxF32* forceBuffers = reinterpret_cast<PxF32*>(ptr);
		PxMemZero(forceBuffers, sizeof(PxF32) * contactCount);
		ptr += ((contactCount + 3) & (~3)) * sizeof(PxF32); // jump to next 16-byte boundary

		const PxReal staticFriction = contactBase0->staticFriction;
		const PxReal dynamicFriction = contactBase0->dynamicFriction;
		const bool disableStrongFriction = !!(contactBase0->materialFlags & PxMaterialFlag::eDISABLE_FRICTION);
		staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W=V4SetX(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, FLoad(staticFriction));
		staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W=V4SetY(staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W, FLoad(dynamicFriction));

		const bool haveFriction = (disableStrongFriction == 0 && frictionPatch.anchorCount != 0) ;//PX_IR(n.staticFriction) > 0 || PX_IR(n.dynamicFriction) > 0;
		header->numNormalConstr		= Ps::to8(contactCount);
		header->numFrictionConstr	= Ps::to8(haveFriction ? frictionPatch.anchorCount*2 : 0);
	
		header->type				= type;

		header->staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W = staticFrictionX_dynamicFrictionY_dominance0Z_dominance1W;
		FStore(angD0, &header->angDom0);
		FStore(angD1, &header->angDom1);

		header->broken = 0;

		if(haveFriction)
		{
			const Vec3V linVrel = V3Sub(linVel0, linVel1);
			//const Vec3V normal = Vec3V_From_PxVec3_Aligned(buffer.contacts[c.contactPatches[c.correlationListHeads[i]].start].normal);

			const FloatV orthoThreshold = FLoad(0.70710678f);
			const FloatV p1 = FLoad(0.1f);
			// fallback: normal.cross((1,0,0)) or normal.cross((0,0,1))
			const FloatV normalX = V3GetX(normal);
			const FloatV normalY = V3GetY(normal);
			const FloatV normalZ = V3GetZ(normal);
			
			Vec3V t0Fallback1 = V3Merge(zero, FNeg(normalZ), normalY);
			Vec3V t0Fallback2 = V3Merge(FNeg(normalY), normalX, zero) ;
			Vec3V t0Fallback = V3Sel(FIsGrtr(orthoThreshold, FAbs(normalX)), t0Fallback1, t0Fallback2);

			Vec3V t0 = V3Sub(linVrel, V3Scale(normal, V3Dot(normal, linVrel)));
			t0 = V3Sel(FIsGrtr(V3LengthSq(t0), p1), t0, t0Fallback);
			t0 = V3Normalize(t0);

			const VecCrossV t0Cross = V3PrepareCross(t0);

			const Vec3V t1 = V3Cross(norCross, t0Cross);
			const VecCrossV t1Cross = V3PrepareCross(t1);

			
			// since we don't even have the body velocities we can't compute the tangent dirs, so 
			// the only thing we can do right now is to write the geometric information (which is the
			// same for both axis constraints of an anchor) We put ra in the raXn field, rb in the rbXn
			// field, and the error in the normal field. See corresponding comments in
			// completeContactFriction()

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
				Ps::prefetchLine(ptr, 256);
				Ps::prefetchLine(ptr, 384);
				SolverContactFriction* PX_RESTRICT f0 = reinterpret_cast<SolverContactFriction*>(ptr);
				ptr += frictionStride;
				SolverContactFriction* PX_RESTRICT f1 = reinterpret_cast<SolverContactFriction*>(ptr);
				ptr += frictionStride;

				Vec3V body0Anchor = V3LoadU(frictionPatch.body0Anchors[j]);
				Vec3V body1Anchor = V3LoadU(frictionPatch.body1Anchors[j]);

				const Vec3V ra = QuatRotate(bodyFrame0q, body0Anchor);
				const Vec3V rb = QuatRotate(bodyFrame1q, body1Anchor);

				/*ra = V3Sel(V3IsGrtr(solverOffsetSlop, V3Abs(ra)), v3Zero, ra);
				rb = V3Sel(V3IsGrtr(solverOffsetSlop, V3Abs(rb)), v3Zero, rb);*/

				PxU32 index = c.contactID[i][j];
				Vec3V error = V3Sub(V3Add(ra, bodyFrame0p), V3Add(rb, bodyFrame1p));
				error = V3Sel(V3IsGrtr(solverOffsetSlop, V3Abs(error)), v3Zero, error);

				index = index == 0xFFFF ? c.contactPatches[c.correlationListHeads[i]].start : index;

				const Vec3V tvel = V3LoadA(buffer[index].targetVel);
				
				{
					Vec3V raXn = V3Cross(ra, t0Cross);
					Vec3V rbXn = V3Cross(rb, t0Cross);

					raXn = V3Sel(V3IsGrtr(solverOffsetSlop, V3Abs(raXn)), V3Zero(), raXn);
					rbXn = V3Sel(V3IsGrtr(solverOffsetSlop, V3Abs(rbXn)), V3Zero(), rbXn);

					const Vec3V raXnSqrtInertia = M33MulV3(invSqrtInertia0, raXn);
					const Vec3V rbXnSqrtInertia = M33MulV3(invSqrtInertia1, rbXn);	


					const FloatV resp0 = FAdd(invMass0_dom0fV, FMul(angD0, V3Dot(raXnSqrtInertia, raXnSqrtInertia)));
					const FloatV resp1 = FSub(FMul(angD1, V3Dot(rbXnSqrtInertia, rbXnSqrtInertia)), invMass1_dom1fV);
					const FloatV resp = FAdd(resp0, resp1);

					const FloatV velMultiplier = FSel(FIsGrtr(resp, zero), FDiv(p8, resp), zero);

					FloatV targetVel = V3Dot(tvel, t0);

					const FloatV vrel1 = FAdd(V3Dot(t0, linVel0), V3Dot(raXn, angVel0));
					const FloatV vrel2 = FAdd(V3Dot(t0, linVel1), V3Dot(rbXn, angVel1));
					const FloatV vrel = FSub(vrel1, vrel2);

					targetVel = FSub(targetVel, vrel);

					f0->normalXYZ_appliedForceW = V4SetW(t0, zero);
					f0->raXnXYZ_velMultiplierW = V4SetW(raXnSqrtInertia, velMultiplier);
					f0->rbXnXYZ_biasW = V4SetW(rbXnSqrtInertia, FMul(V3Dot(t0, error), invDt));
					FStore(targetVel, &f0->targetVel);
				}

				{

					Vec3V raXn = V3Cross(ra, t1Cross);
					Vec3V rbXn = V3Cross(rb, t1Cross);

					raXn = V3Sel(V3IsGrtr(solverOffsetSlop, V3Abs(raXn)), V3Zero(), raXn);
					rbXn = V3Sel(V3IsGrtr(solverOffsetSlop, V3Abs(rbXn)), V3Zero(), rbXn);

					const Vec3V raXnSqrtInertia = M33MulV3(invSqrtInertia0, raXn);
					const Vec3V rbXnSqrtInertia = M33MulV3(invSqrtInertia1, rbXn);	

					const FloatV resp0 = FAdd(invMass0_dom0fV, FMul(angD0, V3Dot(raXnSqrtInertia, raXnSqrtInertia)));
					const FloatV resp1 = FSub(FMul(angD1, V3Dot(rbXnSqrtInertia, rbXnSqrtInertia)), invMass1_dom1fV);
					const FloatV resp = FAdd(resp0, resp1);

					const FloatV velMultiplier = FSel(FIsGrtr(resp, zero), FDiv(p8, resp), zero);

					FloatV targetVel = V3Dot(tvel, t1);

					const FloatV vrel1 = FAdd(V3Dot(t1, linVel0), V3Dot(raXn, angVel0));
					const FloatV vrel2 = FAdd(V3Dot(t1, linVel1), V3Dot(rbXn, angVel1));
					const FloatV vrel = FSub(vrel1, vrel2);

					targetVel = FSub(targetVel, vrel);

					f1->normalXYZ_appliedForceW = V4SetW(t1, zero);
					f1->raXnXYZ_velMultiplierW = V4SetW(raXnSqrtInertia, velMultiplier);
					f1->rbXnXYZ_biasW = V4SetW(rbXnSqrtInertia, FMul(V3Dot(t1, error), invDt));
					FStore(targetVel, &f1->targetVel);
				}
			}
		}

		frictionPatchWritebackAddrIndex++;
	}
}


PX_FORCE_INLINE void computeBlockStreamByteSizes(const bool useExtContacts, const CorrelationBuffer& c,
								PxU32& _solverConstraintByteSize, PxU32& _frictionPatchByteSize, PxU32& _numFrictionPatches,
								PxU32& _axisConstraintCount)
{
	PX_ASSERT(0 == _solverConstraintByteSize);
	PX_ASSERT(0 == _frictionPatchByteSize);
	PX_ASSERT(0 == _numFrictionPatches);
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
			solverConstraintByteSize += sizeof(SolverContactHeader);
			solverConstraintByteSize += useExtContacts ? c.frictionPatchContactCounts[i] * sizeof(SolverContactPointExt) 
				: c.frictionPatchContactCounts[i] * sizeof(SolverContactPoint);
			solverConstraintByteSize += sizeof(PxF32) * ((c.frictionPatchContactCounts[i] + 3)&(~3)); //Add on space for applied impulses

			axisConstraintCount += c.frictionPatchContactCounts[i];

			if(haveFriction)
			{
				solverConstraintByteSize += useExtContacts ? c.frictionPatches[i].anchorCount * 2 * sizeof(SolverContactFrictionExt)
					: c.frictionPatches[i].anchorCount * 2 * sizeof(SolverContactFriction);
				axisConstraintCount += c.frictionPatches[i].anchorCount * 2;

			}
		}
	}
	PxU32 frictionPatchByteSize = numFrictionPatches*sizeof(FrictionPatch);

	_numFrictionPatches = numFrictionPatches;
	_axisConstraintCount = axisConstraintCount;

	//16-byte alignment.
	_frictionPatchByteSize = ((frictionPatchByteSize + 0x0f) & ~0x0f);
	_solverConstraintByteSize =  ((solverConstraintByteSize + 0x0f) & ~0x0f);
	PX_ASSERT(0 == (_solverConstraintByteSize & 0x0f));
	PX_ASSERT(0 == (_frictionPatchByteSize & 0x0f));
}

static bool reserveBlockStreams(const bool useExtContacts, Dy::CorrelationBuffer& cBuffer,
						PxU8*& solverConstraint,
						FrictionPatch*& _frictionPatches,
						PxU32& numFrictionPatches, PxU32& solverConstraintByteSize,
						PxU32& axisConstraintCount, PxConstraintAllocator& constraintAllocator)
{
	PX_ASSERT(NULL == solverConstraint);
	PX_ASSERT(NULL == _frictionPatches);
	PX_ASSERT(0 == numFrictionPatches);
	PX_ASSERT(0 == solverConstraintByteSize);
	PX_ASSERT(0 == axisConstraintCount);

	//From frictionPatchStream we just need to reserve a single buffer.
	PxU32 frictionPatchByteSize = 0;
	//Compute the sizes of all the buffers.
	computeBlockStreamByteSizes(
		useExtContacts, cBuffer,
		solverConstraintByteSize, frictionPatchByteSize, numFrictionPatches,
		axisConstraintCount);

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
		PX_ASSERT((size_t(constraintBlock) & 0xF) == 0);
	}

	FrictionPatch* frictionPatches = NULL;
	//If the constraint block reservation didn't fail then reserve the friction buffer too.
	if(frictionPatchByteSize >0 && (0==constraintBlockByteSize || constraintBlock))
	{
		frictionPatches = reinterpret_cast<FrictionPatch*>(constraintAllocator.reserveFrictionData(frictionPatchByteSize));

		if(0==frictionPatches || (reinterpret_cast<FrictionPatch*>(-1))==frictionPatches)
		{
			if(0==frictionPatches)
			{
				PX_WARN_ONCE(
					"Reached limit set by PxSceneDesc::maxNbContactDataBlocks - ran out of buffer space for constraint prep. "
					"Either accept dropped contacts or increase buffer size allocated for narrow phase by increasing PxSceneDesc::maxNbContactDataBlocks.");
			}
			else
			{
				PX_WARN_ONCE(
					"Attempting to allocate more than 16K of friction data for a single contact pair in constraint prep. "
					"Either accept dropped contacts or simplify collision geometry.");
				frictionPatches=NULL;
			}
		}
	}

	_frictionPatches = frictionPatches;

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
	return ((0==constraintBlockByteSize || constraintBlock) && (0==frictionPatchByteSize || frictionPatches));
}


bool createFinalizeSolverContacts(
	PxSolverContactDesc& contactDesc,
	CorrelationBuffer& c,
	const PxReal invDtF32,
	PxReal bounceThresholdF32,
	PxReal frictionOffsetThreshold,
	PxReal correlationDistance,
	PxReal solverOffsetSlop,
	PxConstraintAllocator& constraintAllocator,
	Cm::SpatialVectorF* Z)
{
	Ps::prefetchLine(contactDesc.body0);
	Ps::prefetchLine(contactDesc.body1);
	Ps::prefetchLine(contactDesc.data0);
	Ps::prefetchLine(contactDesc.data1);

	c.frictionPatchCount = 0;
	c.contactPatchCount = 0;

	const bool hasForceThreshold = contactDesc.hasForceThresholds;
	const bool staticOrKinematicBody = contactDesc.bodyState1 == PxSolverContactDesc::eKINEMATIC_BODY || contactDesc.bodyState1 == PxSolverContactDesc::eSTATIC_BODY;

	const bool disableStrongFriction = contactDesc.disableStrongFriction;
	const bool useExtContacts = ((contactDesc.bodyState0 | contactDesc.bodyState1) & PxSolverContactDesc::eARTICULATION) != 0;

	PxSolverConstraintDesc& desc = *contactDesc.desc;

	desc.constraintLengthOver16 = 0;


	if (contactDesc.numContacts == 0)
	{
		contactDesc.frictionPtr = NULL;
		contactDesc.frictionCount = 0;
		desc.constraint = NULL;
		return true;
	}

	if (!disableStrongFriction)
	{
		getFrictionPatches(c, contactDesc.frictionPtr, contactDesc.frictionCount, contactDesc.bodyFrame0, contactDesc.bodyFrame1, correlationDistance);
	}

	bool overflow = !createContactPatches(c, contactDesc.contacts, contactDesc.numContacts, PXC_SAME_NORMAL);
	overflow = correlatePatches(c, contactDesc.contacts, contactDesc.bodyFrame0, contactDesc.bodyFrame1, PXC_SAME_NORMAL, 0, 0) || overflow;
	PX_UNUSED(overflow);

#if PX_CHECKED
	if (overflow)
	{
		Ps::getFoundation().error(physx::PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__,
			"Dropping contacts in solver because we exceeded limit of 32 friction patches.");
	}
#endif

	growPatches(c, contactDesc.contacts, contactDesc.bodyFrame0, contactDesc.bodyFrame1, correlationDistance, 0, frictionOffsetThreshold + contactDesc.restDistance);

	//PX_ASSERT(patchCount == c.frictionPatchCount);

	FrictionPatch* frictionPatches = NULL;
	PxU8* solverConstraint = NULL;
	PxU32 numFrictionPatches = 0;
	PxU32 solverConstraintByteSize = 0;
	PxU32 axisConstraintCount = 0;

	const bool successfulReserve = reserveBlockStreams(
		useExtContacts, c,
		solverConstraint, frictionPatches,
		numFrictionPatches,
		solverConstraintByteSize,
		axisConstraintCount,
		constraintAllocator);
	// initialise the work unit's ptrs to the various buffers.

	contactDesc.frictionPtr = NULL;
	contactDesc.frictionCount = 0;
	desc.constraint = NULL;
	desc.constraintLengthOver16 = 0;
	// patch up the work unit with the reserved buffers and set the reserved buffer data as appropriate.

	if (successfulReserve)
	{
		PxU8* frictionDataPtr = reinterpret_cast<PxU8*>(frictionPatches);
		contactDesc.frictionPtr = frictionDataPtr;
		desc.constraint = solverConstraint;
		//output.nbContacts = Ps::to8(numContacts);
		contactDesc.frictionCount = Ps::to8(numFrictionPatches);
		desc.constraintLengthOver16 = Ps::to16(solverConstraintByteSize / 16);
		desc.writeBack = contactDesc.contactForces;
		desc.writeBackLengthOver4 = PxU16(contactDesc.contactForces ? contactDesc.numContacts : 0);

		//Initialise friction buffer.
		if (frictionPatches)
		{
			// PT: TODO: revisit this... not very satisfying
			//const PxU32 maxSize = numFrictionPatches*sizeof(FrictionPatch);
			Ps::prefetchLine(frictionPatches);
			Ps::prefetchLine(frictionPatches, 128);
			Ps::prefetchLine(frictionPatches, 256);

			for (PxU32 i = 0; i<c.frictionPatchCount; i++)
			{
				//if(c.correlationListHeads[i]!=CorrelationBuffer::LIST_END)
				if (c.frictionPatchContactCounts[i])
				{
					*frictionPatches++ = c.frictionPatches[i];
					Ps::prefetchLine(frictionPatches, 256);
				}
			}
		}

		//Initialise solverConstraint buffer.
		if (solverConstraint)
		{
			if (useExtContacts)
			{
				const PxSolverBodyData& data0 = *contactDesc.data0;
				const PxSolverBodyData& data1 = *contactDesc.data1;

				const SolverExtBody b0(reinterpret_cast<const void*>(contactDesc.body0), reinterpret_cast<const void*>(&data0), desc.linkIndexA);
				const SolverExtBody b1(reinterpret_cast<const void*>(contactDesc.body1), reinterpret_cast<const void*>(&data1), desc.linkIndexB);

				setupFinalizeExtSolverContacts(contactDesc.contacts, c, contactDesc.bodyFrame0, contactDesc.bodyFrame1, solverConstraint,
					b0, b1, invDtF32, bounceThresholdF32,
					contactDesc.mInvMassScales.linear0, contactDesc.mInvMassScales.angular0, contactDesc.mInvMassScales.linear1, contactDesc.mInvMassScales.angular1, 
					contactDesc.restDistance, frictionDataPtr, contactDesc.maxCCDSeparation, Z);
			}
			else
			{
				const PxSolverBodyData& data0 = *contactDesc.data0;
				const PxSolverBodyData& data1 = *contactDesc.data1;
				setupFinalizeSolverConstraints(contactDesc.shapeInteraction, contactDesc.contacts, c, contactDesc.bodyFrame0, contactDesc.bodyFrame1, solverConstraint,
					data0, data1, invDtF32, bounceThresholdF32,
					contactDesc.mInvMassScales.linear0, contactDesc.mInvMassScales.angular0, contactDesc.mInvMassScales.linear1, contactDesc.mInvMassScales.angular1, 
					hasForceThreshold, staticOrKinematicBody, contactDesc.restDistance, frictionDataPtr, contactDesc.maxCCDSeparation, solverOffsetSlop);
			}
			//KS - set to 0 so we have a counter for the number of times we solved the constraint
			//only going to be used on SPU but might as well set on all platforms because this code is shared
			*(reinterpret_cast<PxU32*>(solverConstraint + solverConstraintByteSize)) = 0;
		}
	}

	return successfulReserve;
}

FloatV setupExtSolverContact(const SolverExtBody& b0, const SolverExtBody& b1,
	const PxF32 d0, const PxF32 d1, const PxF32 angD0, const PxF32 angD1, const PxTransform& bodyFrame0, const PxTransform& bodyFrame1,
	const Vec3VArg normal, const FloatVArg invDt, const FloatVArg invDtp8, const FloatVArg restDistance, const FloatVArg maxPenBias, const FloatVArg restitution,
	const FloatVArg bounceThreshold, const Gu::ContactPoint& contact, SolverContactPointExt& solverContact, const FloatVArg ccdMaxSeparation, Cm::SpatialVectorF* zVector)
{
	const FloatV zero = FZero();
	const FloatV separation = FLoad(contact.separation);

	const FloatV penetration = FSub(separation, restDistance);

	const PxVec3 ra = contact.point - bodyFrame0.p;
	const PxVec3 rb = contact.point - bodyFrame1.p;

	const PxVec3 raXn = ra.cross(contact.normal);
	const PxVec3 rbXn = rb.cross(contact.normal);

	Cm::SpatialVector deltaV0, deltaV1;

	const Cm::SpatialVector resp0 = createImpulseResponseVector(contact.normal, raXn, b0);
	const Cm::SpatialVector resp1 = createImpulseResponseVector(-contact.normal, -rbXn, b1);

	const FloatV unitResponse = FLoad(getImpulseResponse(b0, resp0, deltaV0, d0, angD0,
		b1, resp1, deltaV1, d1, angD1, zVector));


	const FloatV vel0 = FLoad(b0.projectVelocity(contact.normal, raXn));
	const FloatV vel1 = FLoad(b1.projectVelocity(contact.normal, rbXn));

	const FloatV vrel = FSub(vel0, vel1);

	FloatV velMultiplier = FSel(FIsGrtr(FLoad(DY_ARTICULATION_MIN_RESPONSE), unitResponse), zero, FRecip(unitResponse));
	//FloatV velMultiplier = FSel(FIsGrtr(FEps(), unitResponse), zero, FRecip(unitResponse));
	FloatV scaledBias = FMul(velMultiplier, FMax(maxPenBias, FMul(penetration, invDtp8)));
	const FloatV penetrationInvDt = FMul(penetration, invDt);

	const BoolV isGreater2 = BAnd(BAnd(FIsGrtr(restitution, zero), FIsGrtr(bounceThreshold, vrel)), FIsGrtr(FNeg(vrel), penetrationInvDt));

	const BoolV ccdSeparationCondition = FIsGrtrOrEq(ccdMaxSeparation, penetration);

	scaledBias = FSel(BAnd(ccdSeparationCondition, isGreater2), zero, scaledBias);

	FloatV targetVelocity = FSel(isGreater2, FMul(FNeg(vrel), restitution), zero);

	//Get the rigid body's current velocity and embed into the constraint target velocities
	if (b0.mLinkIndex == PxSolverConstraintDesc::NO_LINK)
		targetVelocity = FSub(targetVelocity, vel0);
	else if (b1.mLinkIndex == PxSolverConstraintDesc::NO_LINK)
		targetVelocity = FAdd(targetVelocity, vel1);

	targetVelocity = FAdd(targetVelocity, V3Dot(V3LoadA(contact.targetVel), normal));

	const FloatV biasedErr = FScaleAdd(targetVelocity, velMultiplier, FNeg(scaledBias));
	const FloatV unbiasedErr = FScaleAdd(targetVelocity, velMultiplier, FSel(isGreater2, zero, FNeg(FMax(scaledBias, zero))));

	const FloatV deltaF = FMax(FNegScaleSub(vrel, velMultiplier, biasedErr), zero);


	FStore(velMultiplier, &solverContact.velMultiplier);
	FStore(biasedErr, &solverContact.biasedErr);
	FStore(unbiasedErr, &solverContact.unbiasedErr);
	solverContact.maxImpulse = contact.maxImpulse;

	solverContact.raXn = V3LoadA(resp0.angular);
	solverContact.rbXn = V3Neg(V3LoadA(resp1.angular));
	solverContact.linDeltaVA = V3LoadA(deltaV0.linear);
	solverContact.angDeltaVA = V3LoadA(deltaV0.angular);
	solverContact.linDeltaVB = V3LoadA(deltaV1.linear);
	solverContact.angDeltaVB = V3LoadA(deltaV1.angular);

	return deltaF;
}


bool createFinalizeSolverContacts(PxSolverContactDesc& contactDesc,
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
	ContactBuffer& buffer = threadContext.mContactBuffer;

	

	buffer.count = 0;

	// We pull the friction patches out of the cache to remove the dependency on how
	// the cache is organized. Remember original addrs so we can write them back 
	// efficiently.

	PxU32 numContacts = 0;
	{
		PxReal invMassScale0 = 1.f;
		PxReal invMassScale1 = 1.f;
		PxReal invInertiaScale0 = 1.f;
		PxReal invInertiaScale1 = 1.f;

		bool hasMaxImpulse = false, hasTargetVelocity = false;

		numContacts = extractContacts(buffer, output, hasMaxImpulse, hasTargetVelocity, invMassScale0, invMassScale1,
			invInertiaScale0, invInertiaScale1, PxMin(contactDesc.data0->maxContactImpulse, contactDesc.data1->maxContactImpulse));

		contactDesc.contacts = buffer.contacts;
		contactDesc.numContacts = numContacts;
		contactDesc.disableStrongFriction = contactDesc.disableStrongFriction || hasTargetVelocity;
		contactDesc.hasMaxImpulse = hasMaxImpulse;
		contactDesc.mInvMassScales.linear0 *= invMassScale0;
		contactDesc.mInvMassScales.linear1 *= invMassScale1;
		contactDesc.mInvMassScales.angular0 *= invInertiaScale0;
		contactDesc.mInvMassScales.angular1 *= invInertiaScale1;
	}
	
	CorrelationBuffer& c = threadContext.mCorrelationBuffer;

	return createFinalizeSolverContacts(contactDesc, c, invDtF32, bounceThresholdF32, frictionOffsetThreshold, 
		correlationDistance, solverOffsetSlop, constraintAllocator, Z);
}
  
PxU32 getContactManagerConstraintDesc(const PxsContactManagerOutput& cmOutput, const PxsContactManager& /*cm*/, PxSolverConstraintDesc& desc)
{
	desc.writeBackLengthOver4 = cmOutput.nbContacts;
	desc.writeBack = cmOutput.contactForces;
	return cmOutput.nbContacts;// cm.getWorkUnit().axisConstraintCount;
}

}

}

