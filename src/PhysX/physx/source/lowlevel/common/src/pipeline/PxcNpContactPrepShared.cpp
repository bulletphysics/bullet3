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
#include "PsMathUtils.h"
#include "PxcNpWorkUnit.h"
#include "PxvDynamics.h"

using namespace physx;
using namespace Gu;

#include "PxsMaterialManager.h"
#include "PxsMaterialCombiner.h"

#include "PxcNpContactPrepShared.h"
#include "PsAtomic.h"
#include "PxsContactManagerState.h"

#include "PsVecMath.h"
using namespace physx;
using namespace Ps::aos;

static PX_FORCE_INLINE void copyContactPoint(PxContact* PX_RESTRICT point, const Gu::ContactPoint* PX_RESTRICT cp)
{
	// PT: TODO: consider moving "separation" right after "point" in both structures, to copy both at the same time.
//	point->contact = cp->point;
	const Vec4V contactV = V4LoadA(&cp->point.x);	// PT: V4LoadA safe because 'point' is aligned.
	V4StoreU(contactV, &point->contact.x);

	point->separation = cp->separation;
}

struct StridePatch
{
	PxU8 startIndex;
	PxU8 endIndex;
	PxU8 nextIndex;
	PxU8 totalCount;
	bool isRoot;
};

PxU32 physx::writeCompressedContact(const Gu::ContactPoint* const PX_RESTRICT contactPoints, const PxU32 numContactPoints, PxcNpThreadContext* threadContext,
									PxU8& writtenContactCount, PxU8*& outContactPatches, PxU8*& outContactPoints, PxU16& compressedContactSize, PxReal*& outContactForces, PxU32 contactForceByteSize,
									const PxsMaterialManager* materialManager, bool hasModifiableContacts, bool forceNoResponse, PxsMaterialInfo* PX_RESTRICT pMaterial, PxU8& numPatches,
									PxU32 additionalHeaderSize,  PxsConstraintBlockManager* manager, PxcConstraintBlockStream* blockStream, bool insertAveragePoint,
									PxcDataStreamPool* contactStreamPool, PxcDataStreamPool* patchStreamPool, PxcDataStreamPool* forceStreamPool, const bool isMeshType)
{
	if(numContactPoints == 0)
	{
		writtenContactCount = 0;
		outContactPatches = NULL;
		outContactPoints = NULL;
		outContactForces = NULL;
		compressedContactSize = 0;
		numPatches = 0;
		return 0;
	}

	//Calculate the size of the contact buffer...
	PX_ALLOCA(strPatches, StridePatch, numContactPoints);

	StridePatch* stridePatches = &strPatches[0];
	
	PxU32 numStrideHeaders = 1;

	/*const bool hasInternalFaceIndex = contactPoints[0].internalFaceIndex0 != PXC_CONTACT_NO_FACE_INDEX ||
										contactPoints[0].internalFaceIndex1 != PXC_CONTACT_NO_FACE_INDEX;*/
	const bool isModifiable = !forceNoResponse && hasModifiableContacts;

	PxU32 totalUniquePatches = 1;

	PxU32 totalContactPoints = numContactPoints;

	PxU32 strideStart = 0;
	bool root = true;
	StridePatch* parentRootPatch = NULL;
	{
		const PxReal closeNormalThresh = PXC_SAME_NORMAL;
		//Go through and tag how many patches we have...
		PxVec3 normal = contactPoints[0].normal;
		PxU16 mat0 = pMaterial[0].mMaterialIndex0;
		PxU16 mat1 = pMaterial[0].mMaterialIndex1;

		for(PxU32 a = 1; a < numContactPoints; ++a)
		{
			if(normal.dot(contactPoints[a].normal) < closeNormalThresh || 
				pMaterial[a].mMaterialIndex0 != mat0 || pMaterial[a].mMaterialIndex1 != mat1)
			{
				StridePatch& patch = stridePatches[numStrideHeaders-1];

				patch.startIndex = PxU8(strideStart);
				patch.endIndex = PxU8(a);
				patch.nextIndex = 0xFF;
				patch.totalCount = PxU8(a - strideStart);
				patch.isRoot = root;
				if(parentRootPatch)
					parentRootPatch->totalCount += PxU8(a - strideStart);

				root = true;
				parentRootPatch = NULL;
				for(PxU32 b = 1; b < numStrideHeaders; ++b)
				{
					StridePatch& thisPatch = stridePatches[b-1];
					if(thisPatch.isRoot)
					{
						PxU32 ind = thisPatch.startIndex;
						PxReal dp2 = contactPoints[a].normal.dot(contactPoints[ind].normal);
						if(dp2 >= closeNormalThresh && pMaterial[a].mMaterialIndex0 == pMaterial[ind].mMaterialIndex0 && 
							pMaterial[a].mMaterialIndex1 == pMaterial[ind].mMaterialIndex1)
						{
							PxU32 nextInd = b-1;
							while(stridePatches[nextInd].nextIndex != 0xFF)
								nextInd = stridePatches[nextInd].nextIndex;
							stridePatches[nextInd].nextIndex = PxU8(numStrideHeaders);
							root = false;
							parentRootPatch = &stridePatches[b-1];
							break;
						}
					}
				}

				normal = contactPoints[a].normal;

				mat0 = pMaterial[a].mMaterialIndex0;
				mat1 = pMaterial[a].mMaterialIndex1;
				totalContactPoints = insertAveragePoint && (a - strideStart) > 1 ? totalContactPoints + 1 : totalContactPoints;
				strideStart = a;
				numStrideHeaders++;
				if(root)
					totalUniquePatches++;
			}
		}
		totalContactPoints = insertAveragePoint &&(numContactPoints - strideStart) > 1 ? totalContactPoints + 1 : totalContactPoints;
		contactForceByteSize = insertAveragePoint && contactForceByteSize != 0 ? contactForceByteSize + sizeof(PxF32) * (totalContactPoints - numContactPoints) : contactForceByteSize;
	}
	{
		StridePatch& patch = stridePatches[numStrideHeaders-1];
		patch.startIndex = PxU8(strideStart);
		patch.endIndex = PxU8(numContactPoints);
		patch.nextIndex = 0xFF;
		patch.totalCount = PxU8(numContactPoints - strideStart);
		patch.isRoot = root;
		if(parentRootPatch)
			parentRootPatch->totalCount += PxU8(numContactPoints - strideStart);
	}

	numPatches = PxU8(totalUniquePatches);

	//Calculate the number of patches/points required

	const PxU32 patchHeaderSize = sizeof(PxContactPatch) * (isModifiable ? totalContactPoints : totalUniquePatches) + additionalHeaderSize;
	const PxU32 pointSize = totalContactPoints * (isModifiable ? sizeof(PxModifiableContact) : sizeof(PxContact));

	PxU32 requiredContactSize = pointSize;
	PxU32 requiredPatchSize = patchHeaderSize;
	PxU32 totalRequiredSize;

	PxU8* PX_RESTRICT contactData = NULL;
	PxU8* PX_RESTRICT patchData = NULL;
	PxReal* PX_RESTRICT forceData = NULL;
	PxU32* PX_RESTRICT triangleIndice = NULL;

	if(contactStreamPool && !isModifiable && additionalHeaderSize == 0) //If the contacts are modifiable, we **DON'T** allocate them in GPU pinned memory. This will be handled later when they're modified
	{
		bool isOverflown = false;

		PxU32 contactIndex = PxU32(Ps::atomicAdd(&contactStreamPool->mSharedDataIndex, PxI32(requiredContactSize)));
		
		if (contactStreamPool->isOverflown())
		{
			PX_WARN_ONCE("Contact buffer overflow detected, please increase its size in the scene desc!\n");
			isOverflown = true;
		}
		
		contactData = contactStreamPool->mDataStream + contactStreamPool->mDataStreamSize - contactIndex;
	
		PxU32 patchIndex = PxU32(Ps::atomicAdd(&patchStreamPool->mSharedDataIndex, PxI32(requiredPatchSize)));
		
		if (patchStreamPool->isOverflown())
		{
			PX_WARN_ONCE("Patch buffer overflow detected, please increase its size in the scene desc!\n");
			isOverflown = true;
		}

		patchData = patchStreamPool->mDataStream + patchStreamPool->mDataStreamSize - patchIndex;
	
		if(contactForceByteSize)
		{
			contactForceByteSize = isMeshType ? contactForceByteSize * 2 : contactForceByteSize;
			contactIndex = PxU32(Ps::atomicAdd(&forceStreamPool->mSharedDataIndex, PxI32(contactForceByteSize)));
			
			if (forceStreamPool->isOverflown())
			{
				PX_WARN_ONCE("Force buffer overflow detected, please increase its size in the scene desc!\n");
				isOverflown = true;
			}
			forceData = reinterpret_cast<PxReal*>(forceStreamPool->mDataStream + forceStreamPool->mDataStreamSize - contactIndex);
			if (isMeshType)
				triangleIndice = reinterpret_cast<PxU32*>(forceData + numContactPoints);
		}

		totalRequiredSize = requiredContactSize + requiredPatchSize;

		if (isOverflown)
		{
			patchData = NULL;
			contactData = NULL;
			forceData = NULL;
			triangleIndice = NULL;
		}
	}
	else
	{
		PxU32 alignedRequiredSize = (requiredContactSize + requiredPatchSize + 0xf) & 0xfffffff0;
		contactForceByteSize = (isMeshType ? contactForceByteSize * 2 : contactForceByteSize);
		PxU32 totalSize = alignedRequiredSize + contactForceByteSize;
		PxU8* data = manager ? blockStream->reserve(totalSize, *manager) : threadContext->mContactBlockStream.reserve(totalSize);

		patchData = data;
		contactData = patchData + requiredPatchSize;

		if(contactForceByteSize)
		{
			forceData = reinterpret_cast<PxReal*>((data + alignedRequiredSize));

			if (isMeshType)
				triangleIndice = reinterpret_cast<PxU32*>(forceData + numContactPoints);

			if(data)
			{
				PxMemZero(forceData, contactForceByteSize);
			}
		}

		totalRequiredSize = alignedRequiredSize;

	}
	
	Ps::prefetchLine(patchData);
	Ps::prefetchLine(contactData);

	if(patchData == NULL)
	{
		writtenContactCount = 0;
		outContactPatches = NULL;
		outContactPoints = NULL;
		outContactForces = NULL;
		compressedContactSize = 0;
		numPatches = 0;
		return 0;
	}

#if PX_ENABLE_SIM_STATS
	if(threadContext)
	{
		threadContext->mCompressedCacheSize += totalRequiredSize;
		threadContext->mTotalCompressedCacheSize += totalRequiredSize;
	}
#endif
	compressedContactSize = Ps::to16(totalRequiredSize);

	

	//PxU32 startIndex = 0;

	//Extract first material
	PxU16 origMatIndex0 = pMaterial[0].mMaterialIndex0;
	PxU16 origMatIndex1 = pMaterial[0].mMaterialIndex1;

	PxReal staticFriction, dynamicFriction, combinedRestitution;
	PxU32 materialFlags;
	{
		const PxsMaterialData& data0 = *materialManager->getMaterial(origMatIndex0);
		const PxsMaterialData& data1 = *materialManager->getMaterial(origMatIndex1);

		combinedRestitution = PxsMaterialCombiner::combineRestitution(data0, data1);
		PxsMaterialCombiner combiner(1.0f, 1.0f);
		PxsMaterialCombiner::PxsCombinedMaterial combinedMat = combiner.combineIsotropicFriction(data0, data1);
		staticFriction  = combinedMat.staFriction;
		dynamicFriction = combinedMat.dynFriction;
		materialFlags = combinedMat.flags;
	}


	PxU8* PX_RESTRICT dataPlusOffset = patchData + additionalHeaderSize;
	PxContactPatch* PX_RESTRICT patches = reinterpret_cast<PxContactPatch*>(dataPlusOffset);
	PxU32* PX_RESTRICT faceIndice = triangleIndice;

	outContactPatches = patchData;
	outContactPoints = contactData;
	outContactForces = forceData;

	if(isModifiable)
	{
		
		PxU32 flags = PxU32(isModifiable ? PxContactPatch::eMODIFIABLE : 0) |
			(forceNoResponse ? PxContactPatch::eFORCE_NO_RESPONSE : 0) |
			(isMeshType ? PxContactPatch::eHAS_FACE_INDICES : 0);

		PxU32 currentIndex = 0;

		PxModifiableContact* PX_RESTRICT point = reinterpret_cast<PxModifiableContact*>(contactData);

		for(PxU32 a = 0; a < numStrideHeaders; ++a)
		{
			StridePatch& rootPatch = stridePatches[a];
			if(rootPatch.isRoot)
			{
				PxContactPatch* PX_RESTRICT patch = patches++;
				
				PxU32 startIndex = rootPatch.startIndex;

				const PxU16 matIndex0 = pMaterial[startIndex].mMaterialIndex0;
				const PxU16 matIndex1 = pMaterial[startIndex].mMaterialIndex1;
				if(matIndex0 != origMatIndex0 || matIndex1 != origMatIndex1)
				{
					const PxsMaterialData& data0 = *materialManager->getMaterial(matIndex0);
					const PxsMaterialData& data1 = *materialManager->getMaterial(matIndex1);

					combinedRestitution = PxsMaterialCombiner::combineRestitution(data0, data1);
					PxsMaterialCombiner combiner(1.0f, 1.0f);
					PxsMaterialCombiner::PxsCombinedMaterial combinedMat = combiner.combineIsotropicFriction(data0, data1);
					staticFriction = combinedMat.staFriction;
					dynamicFriction = combinedMat.dynFriction;
					materialFlags = combinedMat.flags;
					origMatIndex0 = matIndex0;
					origMatIndex1 = matIndex1;
				}

				patch->nbContacts = rootPatch.totalCount;

				patch->startContactIndex = Ps::to8(currentIndex);
				patch->materialFlags = PxU8(materialFlags);
				patch->staticFriction = staticFriction;
				patch->dynamicFriction = dynamicFriction;
				patch->restitution = combinedRestitution;
				patch->materialIndex0 = matIndex0;
				patch->materialIndex1 = matIndex1;
				patch->normal = contactPoints[0].normal;
				patch->mMassModification.mInvMassScale0 = 1.0f;
				patch->mMassModification.mInvMassScale1 = 1.0f;
				patch->mMassModification.mInvInertiaScale0 = 1.0f;
				patch->mMassModification.mInvInertiaScale1 = 1.0f;
				patch->internalFlags = PxU8(flags);

				//const PxU32 endIndex = strideHeader[a];
				const PxU32 totalCountThisPatch = rootPatch.totalCount;
				if(insertAveragePoint && totalCountThisPatch > 1)
				{
					PxVec3 avgPt(0.0f);
					PxF32 avgPen(0.0f);
					PxF32 recipCount = 1.0f/(PxF32(rootPatch.totalCount));

					PxU32 index = a;
					while(index != 0xFF)
					{
						StridePatch& p = stridePatches[index];
						for(PxU32 b = p.startIndex; b < p.endIndex; ++b)
						{
							avgPt += contactPoints[b].point;
							avgPen += contactPoints[b].separation;
						}
						index = p.nextIndex;
					}

					if (faceIndice)
					{
						StridePatch& p = stridePatches[index];
						*faceIndice = contactPoints[p.startIndex].internalFaceIndex1;
						faceIndice++;
					}

					patch->nbContacts++;
					point->contact = avgPt * recipCount;
					point->separation = avgPen * recipCount;
					point->normal = contactPoints[0].normal;
					point->maxImpulse = PX_MAX_REAL;
					point->targetVelocity = PxVec3(0.0f);
					point->staticFriction = staticFriction;
					point->dynamicFriction = dynamicFriction;
					point->restitution = combinedRestitution;
					point->materialFlags = materialFlags;
					point->materialIndex0 = matIndex0;
					point->materialIndex1 = matIndex1;
					point++;
					currentIndex++;
					Ps::prefetchLine(point, 128);
				}

				PxU32 index = a;
				while(index != 0xFF)
				{
					StridePatch& p = stridePatches[index];
						
					for(PxU32 b = p.startIndex; b < p.endIndex; ++b)
					{
						copyContactPoint(point, &contactPoints[b]);
						point->normal = contactPoints[b].normal;
						point->maxImpulse = PX_MAX_REAL;
						point->targetVelocity = PxVec3(0.0f);
						point->staticFriction = staticFriction;
						point->dynamicFriction = dynamicFriction;
						point->restitution = combinedRestitution;
						point->materialFlags = materialFlags;
						point->materialIndex0 = matIndex0;
						point->materialIndex1 = matIndex1;
						if (faceIndice)
						{
							*faceIndice = contactPoints[b].internalFaceIndex1;
							faceIndice++;
						}
						point++;
						currentIndex++;
						Ps::prefetchLine(point, 128);
					}
					index = p.nextIndex;
				}				
			}
		}
	}
	else 
	{
		PxU32 flags = PxU32(isMeshType ? PxContactPatch::eHAS_FACE_INDICES : 0);

		PxContact* PX_RESTRICT point = reinterpret_cast<PxContact*>(contactData);
		
		PxU32 currentIndex = 0;
		{
			for(PxU32 a = 0; a < numStrideHeaders; ++a)
			{
				StridePatch& rootPatch = stridePatches[a];

				if(rootPatch.isRoot)
				{
					const PxU16 matIndex0 = pMaterial[rootPatch.startIndex].mMaterialIndex0;
					const PxU16 matIndex1 = pMaterial[rootPatch.startIndex].mMaterialIndex1;
					if(matIndex0 != origMatIndex0 || matIndex1 != origMatIndex1)
					{
						const PxsMaterialData& data0 = *materialManager->getMaterial(matIndex0);
						const PxsMaterialData& data1 = *materialManager->getMaterial(matIndex1);

						combinedRestitution = PxsMaterialCombiner::combineRestitution(data0, data1);
						PxsMaterialCombiner combiner(1.0f, 1.0f);
						PxsMaterialCombiner::PxsCombinedMaterial combinedMat = combiner.combineIsotropicFriction(data0, data1);
						staticFriction = combinedMat.staFriction;
						dynamicFriction = combinedMat.dynFriction;
						materialFlags = combinedMat.flags;
						origMatIndex0 = matIndex0;
						origMatIndex1 = matIndex1;
					}

					PxContactPatch* PX_RESTRICT patch = patches++;
					patch->normal = contactPoints[rootPatch.startIndex].normal;
					patch->nbContacts = rootPatch.totalCount;
					patch->startContactIndex = Ps::to8(currentIndex);
					//KS - we could probably compress this further into the header but the complexity might not be worth it
					patch->staticFriction = staticFriction;
					patch->dynamicFriction = dynamicFriction;
					patch->restitution = combinedRestitution;
					patch->materialIndex0 = matIndex0;
					patch->materialIndex1 = matIndex1;
					patch->materialFlags = PxU8(materialFlags);
					patch->internalFlags = PxU8(flags);
					patch->mMassModification.mInvMassScale0 = 1.0f;
					patch->mMassModification.mInvMassScale1 = 1.0f;
					patch->mMassModification.mInvInertiaScale0 = 1.0f;
					patch->mMassModification.mInvInertiaScale1 = 1.0f;
					if(insertAveragePoint && (rootPatch.totalCount) > 1)
					{
						patch->nbContacts++;
						PxVec3 avgPt(0.0f);
						PxF32 avgPen(0.0f);
						PxF32 recipCount = 1.0f/(PxF32(rootPatch.totalCount));
						PxU32 index = a;
						
						while(index != 0xFF)
						{
							StridePatch& p = stridePatches[index];
							for(PxU32 b = p.startIndex; b < p.endIndex; ++b)
							{
								avgPt += contactPoints[b].point;
								avgPen += contactPoints[b].separation;
							}
							index = stridePatches[index].nextIndex;
						}

						if (faceIndice)
						{
							StridePatch& p = stridePatches[index];
							*faceIndice = contactPoints[p.startIndex].internalFaceIndex1;
							faceIndice++;
						}
						point->contact = avgPt * recipCount;
						point->separation = avgPen * recipCount;
						
						point++;
						currentIndex++;
						Ps::prefetchLine(point, 128);
					}

					PxU32 index = a;
					while(index != 0xFF)
					{
						StridePatch& p = stridePatches[index];
						for(PxU32 b = p.startIndex; b < p.endIndex; ++b)
						{
							copyContactPoint(point, &contactPoints[b]);
							if (faceIndice)
							{
								*faceIndice = contactPoints[b].internalFaceIndex1;
								faceIndice++;
							}
							point++;
							currentIndex++;
							Ps::prefetchLine(point, 128);
						}
						index = stridePatches[index].nextIndex;
					}
				}
			}
		}
	}

	writtenContactCount = Ps::to8(totalContactPoints);

	return totalRequiredSize;
}
