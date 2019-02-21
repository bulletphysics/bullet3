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


#include "PxcNpBatch.h"
#include "PxcNpWorkUnit.h"
#include "PxsContactManager.h"
#include "GuGeometryUnion.h"
#include "PxcContactCache.h"
#include "PxcMaterialMethodImpl.h"
#include "PxcNpContactPrepShared.h"
#include "PxvDynamics.h"			// for PxsBodyCore
#include "PxvGeometry.h"			// for PxsShapeCore
#include "CmFlushPool.h"
#include "CmTask.h"
#include "PxTriangleMesh.h"
#include "PxsMaterialManager.h"
#include "PxsTransformCache.h"
#include "GuPersistentContactManifold.h"
#include "PxsContactManagerState.h"
#include "PsFoundation.h"

using namespace physx;
using namespace Gu;


static void startContacts(PxsContactManagerOutput& output, PxcNpThreadContext& context)
{
	context.mContactBuffer.reset();

	output.contactForces = NULL;
	output.contactPatches = NULL;
	output.contactPoints = NULL;
	output.nbContacts = 0;
	output.nbPatches = 0;
	output.statusFlag = 0;	
}

static void flipContacts(PxcNpThreadContext& threadContext, PxsMaterialInfo* PX_RESTRICT materialInfo)
{
	ContactBuffer& buffer = threadContext.mContactBuffer;
	for(PxU32 i=0; i<buffer.count; ++i)
	{
		Gu::ContactPoint& contactPoint = buffer.contacts[i];
		contactPoint.normal = -contactPoint.normal;
		Ps::swap(materialInfo[i].mMaterialIndex0, materialInfo[i].mMaterialIndex1);
	}
}

static PX_FORCE_INLINE void updateDiscreteContactStats(PxcNpThreadContext& context, PxGeometryType::Enum type0, PxGeometryType::Enum type1)
{
#if PX_ENABLE_SIM_STATS
	PX_ASSERT(type0<=type1);
	context.mDiscreteContactPairs[type0][type1]++;
#endif
}

static bool copyBuffers(PxsContactManagerOutput& cmOutput, Gu::Cache& cache, PxcNpThreadContext& context, const bool useContactCache, const bool isMeshType)
{
	bool ret = false;
	//Copy the contact stream from previous buffer to current buffer...
	PxU32 oldSize = sizeof(PxContact) * cmOutput.nbContacts + sizeof(PxContactPatch)*cmOutput.nbPatches;
	if(oldSize)
	{
		ret = true;
		PxU8* oldPatches = cmOutput.contactPatches;
		PxU8* oldContacts = cmOutput.contactPoints;
		PxReal* oldForces = cmOutput.contactForces;

		PxU32 forceSize = cmOutput.nbContacts * sizeof(PxReal);
		if(isMeshType)
			forceSize += cmOutput.nbContacts * sizeof(PxU32);

		PxU8* PX_RESTRICT contactPatches = NULL;
		PxU8* PX_RESTRICT contactPoints = NULL;

		PxReal* forceBuffer = NULL;

		bool isOverflown = false;

		//ML: if we are using contactStreamPool, which means we are running the GPU codepath
		if(context.mContactStreamPool)
		{
			const PxU32 patchSize = cmOutput.nbPatches * sizeof(PxContactPatch);
			const PxU32 contactSize = cmOutput.nbContacts * sizeof(PxContact);

			PxU32 index = PxU32(Ps::atomicAdd(&context.mContactStreamPool->mSharedDataIndex, PxI32(contactSize)));
			
			if(context.mContactStreamPool->isOverflown())
			{
				PX_WARN_ONCE("Contact buffer overflow detected, please increase its size in the scene desc!\n");
				isOverflown = true;
			}
			contactPoints = context.mContactStreamPool->mDataStream + context.mContactStreamPool->mDataStreamSize - index;

			const PxU32 patchIndex = PxU32(Ps::atomicAdd(&context.mPatchStreamPool->mSharedDataIndex, PxI32(patchSize)));
			
			if(context.mPatchStreamPool->isOverflown())
			{
				PX_WARN_ONCE("Patch buffer overflow detected, please increase its size in the scene desc!\n");
				isOverflown = true;
			}
			contactPatches = context.mPatchStreamPool->mDataStream + context.mPatchStreamPool->mDataStreamSize - patchIndex;

			if(forceSize)
			{
				index = PxU32(Ps::atomicAdd(&context.mForceAndIndiceStreamPool->mSharedDataIndex, PxI32(forceSize)));
			
				if(context.mForceAndIndiceStreamPool->isOverflown())
				{
					PX_WARN_ONCE("Force buffer overflow detected, please increase its size in the scene desc!\n");
					isOverflown = true;
				}
				forceBuffer = reinterpret_cast<PxReal*>(context.mForceAndIndiceStreamPool->mDataStream + context.mForceAndIndiceStreamPool->mDataStreamSize - index);
			}

			if(isOverflown)
			{
				contactPatches = NULL;
				contactPoints = NULL;
				forceBuffer = NULL;
				cmOutput.nbContacts = cmOutput.nbPatches = 0;
			}
			else
			{
				PxMemCopy(contactPatches, oldPatches, patchSize);
				PxMemCopy(contactPoints, oldContacts, contactSize);
				if (isMeshType)
				{
					PxMemCopy(forceBuffer + cmOutput.nbContacts, oldForces + cmOutput.nbContacts, sizeof(PxU32) * cmOutput.nbContacts);
				}
			}
		}
		else
		{
			const PxU32 alignedOldSize = (oldSize + 0xf) & 0xfffffff0;

			PxU8* data = context.mContactBlockStream.reserve(alignedOldSize + forceSize);
			if(forceSize)
				forceBuffer = reinterpret_cast<PxReal*>(data + alignedOldSize);

			contactPatches = data;
			contactPoints = data + cmOutput.nbPatches * sizeof(PxContactPatch);

			PxMemCopy(data, oldPatches, oldSize);
			if (isMeshType)
			{
				PxMemCopy(forceBuffer + cmOutput.nbContacts, oldForces + cmOutput.nbContacts, sizeof(PxU32) * cmOutput.nbContacts);
			}
		}

		if(forceSize)
			PxMemZero(forceBuffer, forceSize);
		
		cmOutput.contactPatches= contactPatches;
		cmOutput.contactPoints = contactPoints;
		cmOutput.contactForces = forceBuffer;
	}

	if(cache.mCachedSize)
	{
		if(cache.isMultiManifold())
		{
			PX_ASSERT((cache.mCachedSize & 0xF) == 0);
			PxU8* newData = context.mNpCacheStreamPair.reserve(cache.mCachedSize);
			PX_ASSERT((reinterpret_cast<uintptr_t>(newData)& 0xF) == 0);
			PxMemCopy(newData, & cache.getMultipleManifold(), cache.mCachedSize);
			cache.setMultiManifold(newData);
		}
		else if(useContactCache)
		{
			//Copy cache information as well...
			const PxU8* cachedData = cache.mCachedData;
			PxU8* newData = context.mNpCacheStreamPair.reserve(PxU32(cache.mCachedSize + 0xf) & 0xfff0);
			PxMemCopy(newData, cachedData, cache.mCachedSize);
			cache.mCachedData = newData;
		}
	}
	return ret;
}

//ML: isMeshType is used in the GPU codepath. If the collision pair is mesh/heightfield vs primitives, we need to allocate enough memory for the mForceAndIndiceStreamPool in the threadContext. 
static bool finishContacts(const PxcNpWorkUnit& input, PxsContactManagerOutput& npOutput, PxcNpThreadContext& threadContext, PxsMaterialInfo* PX_RESTRICT pMaterials, const bool isMeshType)
{
	ContactBuffer& buffer = threadContext.mContactBuffer;

	PX_ASSERT((npOutput.statusFlag & PxsContactManagerStatusFlag::eTOUCH_KNOWN) != PxsContactManagerStatusFlag::eTOUCH_KNOWN);
	PxU8 statusFlags = PxU16(npOutput.statusFlag & (~PxsContactManagerStatusFlag::eTOUCH_KNOWN));
	if (buffer.count != 0)
		statusFlags |= PxsContactManagerStatusFlag::eHAS_TOUCH;
	else
		statusFlags |= PxsContactManagerStatusFlag::eHAS_NO_TOUCH;

	npOutput.nbContacts = Ps::to8(buffer.count);

	if(buffer.count==0)
	{
		npOutput.statusFlag = statusFlags;
		npOutput.nbContacts = 0;
		npOutput.nbPatches = 0;
		return true;
	}

#if PX_ENABLE_SIM_STATS
	if(buffer.count)
		threadContext.mNbDiscreteContactPairsWithContacts++;
#endif

	npOutput.statusFlag = statusFlags;

	PxU32 contactForceByteSize = buffer.count * sizeof(PxReal);

	//Regardless of the flags, we need to now record the compressed contact stream

	PxU16 compressedContactSize;

	const bool createReports =
		input.flags & PxcNpWorkUnitFlag::eOUTPUT_CONTACTS
		|| threadContext.mCreateContactStream
		|| (input.flags & PxcNpWorkUnitFlag::eFORCE_THRESHOLD);

	if(!buffer.count || (!isMeshType && !createReports))
		contactForceByteSize = 0;

	bool res = (writeCompressedContact(buffer.contacts, buffer.count, &threadContext, npOutput.nbContacts, npOutput.contactPatches, npOutput.contactPoints, compressedContactSize,
		reinterpret_cast<PxReal*&>(npOutput.contactForces), contactForceByteSize, threadContext.mMaterialManager, ((input.flags & PxcNpWorkUnitFlag::eMODIFIABLE_CONTACT) != 0), 
		false, pMaterials, npOutput.nbPatches, 0, NULL, NULL, threadContext.mCreateAveragePoint, threadContext.mContactStreamPool, 
		threadContext.mPatchStreamPool, threadContext.mForceAndIndiceStreamPool, isMeshType) != 0) || (buffer.count == 0);

	//handle buffer overflow
	if (buffer.count && !npOutput.nbContacts)
	{
		PxU8 thisStatusFlags = PxU16(npOutput.statusFlag & (~PxsContactManagerStatusFlag::eTOUCH_KNOWN));
		thisStatusFlags |= PxsContactManagerStatusFlag::eHAS_NO_TOUCH;

		npOutput.statusFlag = thisStatusFlags;
		npOutput.nbContacts = 0;
		npOutput.nbPatches = 0;
#if PX_ENABLE_SIM_STATS
		if(buffer.count)
			threadContext.mNbDiscreteContactPairsWithContacts--;
#endif
	}
	return res;
}

template<bool useContactCacheT>
static PX_FORCE_INLINE bool checkContactsMustBeGenerated(PxcNpThreadContext& context, const PxcNpWorkUnit& input, Gu::Cache& cache, PxsContactManagerOutput& output,
										 const PxsCachedTransform* cachedTransform0, const PxsCachedTransform* cachedTransform1,
										 const bool flip, PxGeometryType::Enum type0, PxGeometryType::Enum type1)
{
	PX_ASSERT(cachedTransform0->transform.isSane() && cachedTransform1->transform.isSane());

	//ML : if user doesn't raise the eDETECT_DISCRETE_CONTACT, we should not generate contacts
	if(!(input.flags & PxcNpWorkUnitFlag::eDETECT_DISCRETE_CONTACT))
		return false;

	if(!(output.statusFlag & PxcNpWorkUnitStatusFlag::eDIRTY_MANAGER) && !(input.flags & PxcNpWorkUnitFlag::eMODIFIABLE_CONTACT))
	{
		const PxU32 body0Dynamic = PxU32(input.flags & PxcNpWorkUnitFlag::eDYNAMIC_BODY0);
		const PxU32 body1Dynamic = PxU32(input.flags & PxcNpWorkUnitFlag::eDYNAMIC_BODY1);

		const PxU32 active0 = PxU32(body0Dynamic && !cachedTransform0->isFrozen());
		const PxU32 active1 = PxU32(body1Dynamic && !cachedTransform1->isFrozen());

		if(!(active0 || active1))
		{
			if(flip)
				Ps::swap(type0, type1);

			const bool useContactCache = useContactCacheT ? context.mContactCache && g_CanUseContactCache[type0][type1] : false;
			
#if PX_ENABLE_SIM_STATS
			if(output.nbContacts)
				context.mNbDiscreteContactPairsWithContacts++;
#endif
			const bool isMeshType = type1 > PxGeometryType::eCONVEXMESH;
			copyBuffers(output, cache, context, useContactCache, isMeshType);
			return false;
		}
	}

	output.statusFlag &= (~PxcNpWorkUnitStatusFlag::eDIRTY_MANAGER);

	const PxReal contactDist0 = context.mContactDistance[input.mTransformCache0];
	const PxReal contactDist1 = context.mContactDistance[input.mTransformCache1];
	//context.mNarrowPhaseParams.mContactDistance = shape0->contactOffset + shape1->contactOffset;
	context.mNarrowPhaseParams.mContactDistance = contactDist0 + contactDist1;

	return true;
}

template<bool useLegacyCodepath>
static PX_FORCE_INLINE void discreteNarrowPhase(PxcNpThreadContext& context, const PxcNpWorkUnit& input, Gu::Cache& cache, PxsContactManagerOutput& output)
{
	PxGeometryType::Enum type0 = static_cast<PxGeometryType::Enum>(input.geomType0);
	PxGeometryType::Enum type1 = static_cast<PxGeometryType::Enum>(input.geomType1);

	const bool flip = (type1<type0);

	const PxsCachedTransform* cachedTransform0 = &context.mTransformCache->getTransformCache(input.mTransformCache0);
	const PxsCachedTransform* cachedTransform1 = &context.mTransformCache->getTransformCache(input.mTransformCache1);

	if(!checkContactsMustBeGenerated<useLegacyCodepath>(context, input, cache, output, cachedTransform0, cachedTransform1, flip, type0, type1))
		return;

	PxsShapeCore* shape0 = const_cast<PxsShapeCore*>(input.shapeCore0);
	PxsShapeCore* shape1 = const_cast<PxsShapeCore*>(input.shapeCore1);

	if(flip)
	{
		Ps::swap(type0, type1);
		Ps::swap(shape0, shape1);
		Ps::swap(cachedTransform0, cachedTransform1);
	}

	PxsMaterialInfo materialInfo[ContactBuffer::MAX_CONTACTS];

	Gu::MultiplePersistentContactManifold& manifold = context.mTempManifold;
	bool isMultiManifold = false;

	if(!useLegacyCodepath)
	{
		if(cache.isMultiManifold())
		{
			//We are using a multi-manifold. This is cached in a reduced npCache...
			isMultiManifold = true;
			uintptr_t address = uintptr_t(&cache.getMultipleManifold());
			manifold.fromBuffer(reinterpret_cast<PxU8*>(address));
			cache.setMultiManifold(&manifold);
		}
		else if(cache.isManifold())
		{
			void* address = reinterpret_cast<void*>(&cache.getManifold());
			Ps::prefetch(address);
			Ps::prefetch(address, 128);
			Ps::prefetch(address, 256);
		}
	}

	updateDiscreteContactStats(context, type0, type1);

	startContacts(output, context);

	const PxTransform* tm0 = &cachedTransform0->transform;
	const PxTransform* tm1 = &cachedTransform1->transform;
	PX_ASSERT(tm0->isSane() && tm1->isSane());

	if(useLegacyCodepath)
	{
		// PT: many cache misses here...
	
		Ps::prefetchLine(shape1, 0);	// PT: at least get rid of L2s for shape1

		const PxcContactMethod conMethod = g_ContactMethodTable[type0][type1];
		PX_ASSERT(conMethod);

		const bool useContactCache = context.mContactCache && g_CanUseContactCache[type0][type1];
		if(useContactCache)
		{
#if PX_ENABLE_SIM_STATS
			if(PxcCacheLocalContacts(context, cache, *tm0, *tm1, conMethod, shape0->geometry, shape1->geometry))
				context.mNbDiscreteContactPairsWithCacheHits++;
#else
			PxcCacheLocalContacts(context, n.pairCache, *tm0, *tm1, conMethod, shape0->geometry, shape1->geometry);
#endif
		}
		else
		{
			conMethod(shape0->geometry, shape1->geometry, *tm0, *tm1, context.mNarrowPhaseParams, cache, context.mContactBuffer, &context.mRenderOutput);
		}
	}
	else
	{
		const PxcContactMethod conMethod = g_PCMContactMethodTable[type0][type1];
		PX_ASSERT(conMethod);

		conMethod(shape0->geometry, shape1->geometry, *tm0, *tm1, context.mNarrowPhaseParams, cache, context.mContactBuffer, &context.mRenderOutput);
	}

	const PxcGetMaterialMethod materialMethod = g_GetMaterialMethodTable[type0][type1];
	PX_ASSERT(materialMethod);

	materialMethod(shape0, shape1, context,  materialInfo);

	if(flip)
		flipContacts(context, materialInfo);

	if(!useLegacyCodepath)
	{
		if(isMultiManifold)
		{
			//Store the manifold back...
			const PxU32 size = (sizeof(MultiPersistentManifoldHeader) +
				manifold.mNumManifolds * sizeof(SingleManifoldHeader) +
				manifold.mNumTotalContacts * sizeof(Gu::CachedMeshPersistentContact));

			PxU8* buffer = context.mNpCacheStreamPair.reserve(size);

			PX_ASSERT((reinterpret_cast<uintptr_t>(buffer)& 0xf) == 0);
			manifold.toBuffer(buffer);
			cache.setMultiManifold(buffer);
			cache.mCachedSize = Ps::to16(size);
		}
	}

	const bool isMeshType = type1 > PxGeometryType::eCONVEXMESH; 
	finishContacts(input, output, context, materialInfo, isMeshType);
}

void physx::PxcDiscreteNarrowPhase(PxcNpThreadContext& context, const PxcNpWorkUnit& input, Gu::Cache& cache, PxsContactManagerOutput& output)
{
	discreteNarrowPhase<true>(context, input, cache, output);
}

void physx::PxcDiscreteNarrowPhasePCM(PxcNpThreadContext& context, const PxcNpWorkUnit& input, Gu::Cache& cache, PxsContactManagerOutput& output)
{
	discreteNarrowPhase<false>(context, input, cache, output);
}
