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

#ifndef GU_PCM_CONTACT_CONVEX_COMMON_H
#define GU_PCM_CONTACT_CONVEX_COMMON_H

#define	PCM_MAX_CONTACTPATCH_SIZE	32

#include "GuContactBuffer.h"
#include "GuVecCapsule.h"
#include "GuPCMTriangleContactGen.h"
#include "PsInlineArray.h"
#include "GuTriangleCache.h"

namespace physx
{

namespace Gu
{

#define MAX_CACHE_SIZE	128

class PCMMeshContactGeneration
{
	PX_NOCOPY(PCMMeshContactGeneration)
public:
	PCMContactPatch										mContactPatch[PCM_MAX_CONTACTPATCH_SIZE];
	PCMContactPatch*									mContactPatchPtr[PCM_MAX_CONTACTPATCH_SIZE];
	const Ps::aos::FloatV								mContactDist;
	const Ps::aos::FloatV								mReplaceBreakingThreshold;
	const Ps::aos::PsTransformV&						mConvexTransform;
	const Ps::aos::PsTransformV&						mMeshTransform;
	Gu::MultiplePersistentContactManifold&				mMultiManifold;
	Gu::ContactBuffer&									mContactBuffer;

	Ps::aos::FloatV										mAcceptanceEpsilon;
	Ps::aos::FloatV										mSqReplaceBreakingThreshold;
	Ps::aos::PsMatTransformV							mMeshToConvex;
	Gu::MeshPersistentContact*							mManifoldContacts;
	PxU32												mNumContacts;
	PxU32												mNumContactPatch;
	PxU32												mNumCalls;
	Gu::CacheMap<Gu::CachedEdge, MAX_CACHE_SIZE>		mEdgeCache;
	Ps::InlineArray<PxU32, LOCAL_CONTACTS_SIZE>*		mDeferredContacts;
	Cm::RenderOutput*									mRenderOutput;

	PCMMeshContactGeneration(
		const Ps::aos::FloatVArg	contactDist,
		const Ps::aos::FloatVArg	replaceBreakingThreshold,
		const Ps::aos::PsTransformV& convexTransform,
		const Ps::aos::PsTransformV& meshTransform,
		Gu::MultiplePersistentContactManifold& multiManifold,
		Gu::ContactBuffer& contactBuffer,
		Ps::InlineArray<PxU32, LOCAL_CONTACTS_SIZE>* deferredContacts,
		Cm::RenderOutput*  renderOutput

	) :
		mContactDist(contactDist),
		mReplaceBreakingThreshold(replaceBreakingThreshold),
		mConvexTransform(convexTransform),
		mMeshTransform(meshTransform),
		mMultiManifold(multiManifold),
		mContactBuffer(contactBuffer),
		mDeferredContacts(deferredContacts),
		mRenderOutput(renderOutput)
		
	{
		using namespace Ps::aos;
		mNumContactPatch = 0;
		mNumContacts = 0;
		mNumCalls = 0;

		mMeshToConvex = mConvexTransform.transformInv(mMeshTransform);

		//Assign the PCMContactPatch to the PCMContactPathPtr
		for(PxU32 i=0; i<PCM_MAX_CONTACTPATCH_SIZE; ++i)
		{
			mContactPatchPtr[i] = &mContactPatch[i];
		}
		mManifoldContacts = PX_CP_TO_MPCP(contactBuffer.contacts);

		mSqReplaceBreakingThreshold = FMul(replaceBreakingThreshold, replaceBreakingThreshold);

		mAcceptanceEpsilon = FLoad(0.996f);//5 degree
		//mAcceptanceEpsilon = FloatV_From_F32(0.9999);//5 degree
	}

	template <PxU32 TriangleCount, typename Derived>
	bool processTriangleCache(Gu::TriangleCache<TriangleCount>& cache)
	{
		PxU32 count = cache.mNumTriangles;
		PxVec3* verts = cache.mVertices;
		PxU32* vertInds = cache.mIndices;
		PxU32* triInds = cache.mTriangleIndex;
		PxU8* edgeFlags = cache.mEdgeFlags;
		while(count--)
		{
			(static_cast<Derived*>(this))->processTriangle(verts, *triInds, *edgeFlags, vertInds);
			verts += 3;
			vertInds += 3;
			triInds++;
			edgeFlags++;
		}
		return true;
	}
	void prioritizeContactPatches();
	void addManifoldPointToPatch(const Ps::aos::Vec3VArg currentPatchNormal, const Ps::aos::FloatVArg maxPen, const PxU32 previousNumContacts);
	void processContacts(const PxU8 maxContactPerManifold, const bool isNotLastPatch = true);
};

/*
	This function is based on the current patch normal to either create a new patch or merge the manifold contacts in this patch with the manifold contacts in the last existing
	patch. This means there might be more than GU_SINGLE_MANIFOLD_CACHE_SIZE in a SinglePersistentContactManifold.
*/
PX_FORCE_INLINE void PCMMeshContactGeneration::addManifoldPointToPatch(const Ps::aos::Vec3VArg currentPatchNormal, const Ps::aos::FloatVArg maxPen, const PxU32 previousNumContacts)
{
	using namespace Ps::aos;

	bool foundPatch = false;
	//we have existing patch
	if(mNumContactPatch > 0)
	{
		//if the direction between the last existing patch normal and the current patch normal are within acceptance epsilon, which means we will be
		//able to merge the last patch's contacts with the current patch's contacts. This is just to avoid to create an extra patch. We have some logic
		//later to refine the patch again
		if(FAllGrtr(V3Dot(mContactPatch[mNumContactPatch-1].mPatchNormal, currentPatchNormal), mAcceptanceEpsilon))
		{
			//get the last patch
			PCMContactPatch& patch = mContactPatch[mNumContactPatch-1];

			//remove duplicate contacts
			for(PxU32 i = patch.mStartIndex; i<patch.mEndIndex; ++i)
			{
				for(PxU32 j = previousNumContacts; j<mNumContacts; ++j)
				{
					Vec3V dif = V3Sub(mManifoldContacts[j].mLocalPointB, mManifoldContacts[i].mLocalPointB);
					FloatV d = V3Dot(dif, dif);
					if(FAllGrtr(mSqReplaceBreakingThreshold, d))
					{
						if(FAllGrtr(V4GetW(mManifoldContacts[i].mLocalNormalPen), V4GetW(mManifoldContacts[j].mLocalNormalPen)))
						{
							//The new contact is deeper than the old contact so we keep the deeper contact
							mManifoldContacts[i] = mManifoldContacts[j];
						}
						mManifoldContacts[j] = mManifoldContacts[mNumContacts-1];
						mNumContacts--;
						j--;
					}
				}
			}
			patch.mEndIndex = mNumContacts;
			patch.mPatchMaxPen = FMin(patch.mPatchMaxPen, maxPen);
			foundPatch = true;
		}
	}

	//If there are no existing patch which match the currentPatchNormal, we will create a new patch
	if(!foundPatch)
	{
		mContactPatch[mNumContactPatch].mStartIndex = previousNumContacts;
		mContactPatch[mNumContactPatch].mEndIndex = mNumContacts;
		mContactPatch[mNumContactPatch].mPatchMaxPen = maxPen;
		mContactPatch[mNumContactPatch++].mPatchNormal = currentPatchNormal;
	}
}

/*
	This function sort the contact patch based on the max penetration so that deepest penetration contact patch will be in front of the less penetration contact
	patch
*/
PX_FORCE_INLINE  void PCMMeshContactGeneration::prioritizeContactPatches()
{
	//we are using insertion sort to prioritize contact patchs
	using namespace Ps::aos;
	//sort the contact patch based on the max penetration
	for(PxU32 i=1; i<mNumContactPatch; ++i)
	{
		const PxU32 indexi = i-1;
		if(FAllGrtr(mContactPatchPtr[indexi]->mPatchMaxPen, mContactPatchPtr[i]->mPatchMaxPen))
		{
			//swap
			PCMContactPatch* tmp = mContactPatchPtr[indexi];
			mContactPatchPtr[indexi] = mContactPatchPtr[i];
			mContactPatchPtr[i] = tmp;

			for(PxI32 j=PxI32(i-2); j>=0; j--)
			{
				const PxU32 indexj = PxU32(j+1);
				if(FAllGrtrOrEq(mContactPatchPtr[indexj]->mPatchMaxPen, mContactPatchPtr[j]->mPatchMaxPen))
					break;
				//swap
				PCMContactPatch* temp = mContactPatchPtr[indexj];
				mContactPatchPtr[indexj] = mContactPatchPtr[j];
				mContactPatchPtr[j] = temp;
			}
		}
	}
}


PX_FORCE_INLINE void PCMMeshContactGeneration::processContacts(const PxU8 maxContactPerManifold, bool isNotLastPatch)
{
	using namespace Ps::aos;
	
	if(mNumContacts != 0)
	{
		//reorder the contact patches based on the max penetration
		prioritizeContactPatches();
		//connect the patches which's angle between patch normals are within 5 degree
		mMultiManifold.refineContactPatchConnective(mContactPatchPtr, mNumContactPatch, mManifoldContacts, mAcceptanceEpsilon);
		//get rid of duplicate manifold contacts in connected contact patches
		mMultiManifold.reduceManifoldContactsInDifferentPatches(mContactPatchPtr, mNumContactPatch, mManifoldContacts, mNumContacts, mSqReplaceBreakingThreshold);
		//add the manifold contact to the corresponding manifold
		mMultiManifold.addManifoldContactPoints(mManifoldContacts, mNumContacts, mContactPatchPtr, mNumContactPatch, mSqReplaceBreakingThreshold, mAcceptanceEpsilon, maxContactPerManifold);
		
		mNumContacts = 0;
		mNumContactPatch = 0;

		if(isNotLastPatch)
		{
			//remap the contact patch pointer to contact patch
			for(PxU32 i=0; i<PCM_MAX_CONTACTPATCH_SIZE; ++i)
			{
				mContactPatchPtr[i] = &mContactPatch[i];
			}
		}
	}
}

struct PCMDeferredPolyData
{
public:
	PxVec3	mVerts[3];			//36
	PxU32	mInds[3];			//48
	PxU32	mTriangleIndex;		//52
	PxU32	mFeatureIndex;		//56
	PxU8	triFlags;			//57
};

#if PX_VC 
    #pragma warning(push)
	#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
#endif

class PCMConvexVsMeshContactGeneration : public PCMMeshContactGeneration
{
	PCMConvexVsMeshContactGeneration &operator=(PCMConvexVsMeshContactGeneration  &);

public:
	Gu::CacheMap<Gu::CachedVertex, MAX_CACHE_SIZE>		mVertexCache;
	Ps::aos::Vec3V										mHullCenterMesh;

	const Gu::PolygonalData&							mPolyData;
	SupportLocal*										mPolyMap;
	const Cm::FastVertex2ShapeScaling&					mConvexScaling;  
	bool												mIdtConvexScale;
	bool												mSilhouetteEdgesAreActive;
				
	PCMConvexVsMeshContactGeneration(
		const Ps::aos::FloatVArg						contactDistance,
		const Ps::aos::FloatVArg						replaceBreakingThreshold,
		const Ps::aos::PsTransformV&					convexTransform,
		const Ps::aos::PsTransformV&					meshTransform,
		Gu::MultiplePersistentContactManifold&			multiManifold,
		Gu::ContactBuffer&								contactBuffer,

		const Gu::PolygonalData&						polyData,
		SupportLocal*									polyMap,
		Ps::InlineArray<PxU32, LOCAL_CONTACTS_SIZE>*	delayedContacts,
		const Cm::FastVertex2ShapeScaling&				convexScaling,
		bool											idtConvexScale,
		bool											silhouetteEdgesAreActive,
		Cm::RenderOutput*								renderOutput
		
	) : PCMMeshContactGeneration(contactDistance, replaceBreakingThreshold, convexTransform, meshTransform, multiManifold, contactBuffer, 
		delayedContacts, renderOutput),
		mPolyData(polyData),
		mPolyMap(polyMap),
		mConvexScaling(convexScaling),
		mIdtConvexScale(idtConvexScale),
		mSilhouetteEdgesAreActive(silhouetteEdgesAreActive)
	{
		using namespace Ps::aos;
	
		// Hull center in local space
		const Vec3V hullCenterLocal = V3LoadU(mPolyData.mCenter);
		// Hull center in mesh space
		mHullCenterMesh = mMeshToConvex.transformInv(hullCenterLocal);

	}

	bool generateTriangleFullContactManifold(Gu::TriangleV& localTriangle, const PxU32 triangleIndex, const PxU32* triIndices, const PxU8 triFlags, const Gu::PolygonalData& polyData,  Gu::SupportLocalImpl<Gu::TriangleV>* localTriMap, Gu::SupportLocal* polyMap, Gu::MeshPersistentContact* manifoldContacts, PxU32& numContacts,
		const Ps::aos::FloatVArg contactDist, Ps::aos::Vec3V& patchNormal);

	bool generatePolyDataContactManifold(Gu::TriangleV& localTriangle, const PxU32 featureIndex, const PxU32 triangleIndex, const PxU8 triFlags, Gu::MeshPersistentContact* manifoldContacts, PxU32& numContacts, const Ps::aos::FloatVArg contactDist, Ps::aos::Vec3V& patchNormal);
	void generateLastContacts();
	void addContactsToPatch(const Ps::aos::Vec3VArg patchNormal, const PxU32 previousNumContacts);

	bool processTriangle(const PxVec3* verts, PxU32 triangleIndex, PxU8 triFlags, const PxU32* vertInds); 

	static bool generateTriangleFullContactManifold(Gu::TriangleV& localTriangle, const PxU32 triangleIndex, const PxU8 triFlags, const Gu::PolygonalData& polyData,  Gu::SupportLocalImpl<Gu::TriangleV>* localTriMap, Gu::SupportLocal* polyMap, Gu::MeshPersistentContact* manifoldContacts, PxU32& numContacts,
		const Ps::aos::FloatVArg contactDist, Ps::aos::Vec3V& patchNormal, Cm::RenderOutput* renderOutput = NULL);

	static bool processTriangle(const Gu::PolygonalData& polyData, SupportLocal* polyMap, const PxVec3* verts, const PxU32 triangleIndex, PxU8 triFlags, const Ps::aos::FloatVArg inflation, const bool isDoubleSided, 
		const Ps::aos::PsTransformV& convexTransform, const Ps::aos::PsMatTransformV& meshToConvex, Gu::MeshPersistentContact* manifoldContact, PxU32& numContacts); 
};

#if PX_VC 
     #pragma warning(pop) 
#endif

struct SortedTriangle
{
	Ps::aos::FloatV		mSquareDist;
	PxU32				mIndex;

	PX_FORCE_INLINE bool operator < (const SortedTriangle& data) const
	{
		return Ps::aos::FAllGrtrOrEq(mSquareDist, data.mSquareDist) ==0;
	}
};

class PCMSphereVsMeshContactGeneration : public PCMMeshContactGeneration
{
public:
	Ps::aos::Vec3V									mSphereCenter;
	Ps::aos::FloatV									mSphereRadius;
	Ps::aos::FloatV									mSqInflatedSphereRadius;
	Ps::InlineArray<SortedTriangle, 64>				mSortedTriangle;

	PCMSphereVsMeshContactGeneration(
		const Ps::aos::Vec3VArg							sphereCenter,
		const Ps::aos::FloatVArg						sphereRadius,
		const Ps::aos::FloatVArg						contactDist,
		const Ps::aos::FloatVArg						replaceBreakingThreshold,
		const Ps::aos::PsTransformV&					sphereTransform,
		const Ps::aos::PsTransformV&					meshTransform,
		MultiplePersistentContactManifold&				multiManifold,
		ContactBuffer&									contactBuffer,
		Ps::InlineArray<PxU32, LOCAL_CONTACTS_SIZE>*	deferredContacts,
		Cm::RenderOutput*			renderOutput = NULL

	) : PCMMeshContactGeneration(contactDist, replaceBreakingThreshold, sphereTransform, meshTransform, multiManifold, 
		contactBuffer, deferredContacts, renderOutput),
		mSphereCenter(sphereCenter),
		mSphereRadius(sphereRadius)
	{
		using namespace Ps::aos;
		const FloatV inflatedSphereRadius = FAdd(sphereRadius, contactDist);
		mSqInflatedSphereRadius = FMul(inflatedSphereRadius, inflatedSphereRadius);
	}

	bool processTriangle(const PxVec3* verts, PxU32 triangleIndex, PxU8 triFlags, const PxU32* vertInds);
	void generateLastContacts();
	void addToPatch(const Ps::aos::Vec3VArg contactP, const Ps::aos::Vec3VArg patchNormal, 
		const Ps::aos::FloatV pen, const PxU32 triangleIndex);
};

class PCMCapsuleVsMeshContactGeneration : public PCMMeshContactGeneration
{
	PCMCapsuleVsMeshContactGeneration &operator=(PCMCapsuleVsMeshContactGeneration &);
public:
	Ps::aos::FloatV mInflatedRadius;
	Ps::aos::FloatV	mSqInflatedRadius;
	const CapsuleV&	mCapsule;

				
	PCMCapsuleVsMeshContactGeneration( 
		const CapsuleV&									capsule,
		const Ps::aos::FloatVArg						contactDist,
		const Ps::aos::FloatVArg						replaceBreakingThreshold,
		const Ps::aos::PsTransformV&					sphereTransform,
		const Ps::aos::PsTransformV&					meshTransform,
		Gu::MultiplePersistentContactManifold&			multiManifold,
		Gu::ContactBuffer&								contactBuffer,
		Ps::InlineArray<PxU32, LOCAL_CONTACTS_SIZE>*	deferredContacts,
		Cm::RenderOutput*								renderOutput = NULL

	) : PCMMeshContactGeneration(contactDist, replaceBreakingThreshold, sphereTransform, meshTransform, multiManifold, contactBuffer, 
		deferredContacts, renderOutput),
		mCapsule(capsule)
	{
		using namespace Ps::aos;
		mInflatedRadius = FAdd(capsule.radius, contactDist);
		mSqInflatedRadius = FMul(mInflatedRadius, mInflatedRadius);
	}

	void generateEEContacts(const Ps::aos::Vec3VArg a, const Ps::aos::Vec3VArg b,const Ps::aos::Vec3VArg c, const Ps::aos::Vec3VArg normal, const PxU32 triangleIndex, 
		const Ps::aos::Vec3VArg p, const Ps::aos::Vec3VArg q, const Ps::aos::FloatVArg sqInflatedRadius, Gu::MeshPersistentContact* manifoldContacts, PxU32& numContacts);

	void generateEE(const Ps::aos::Vec3VArg p, const Ps::aos::Vec3VArg q,  const Ps::aos::FloatVArg sqInflatedRadius, const Ps::aos::Vec3VArg normal, const PxU32 triangleIndex,
		const Ps::aos::Vec3VArg a, const Ps::aos::Vec3VArg b, Gu::MeshPersistentContact* manifoldContacts, PxU32& numContacts);
	
	static bool generateContacts(const Ps::aos::Vec3VArg a, const Ps::aos::Vec3VArg b,const Ps::aos::Vec3VArg c, const Ps::aos::Vec3VArg planeNormal, const Ps::aos::Vec3VArg normal,  
		const PxU32 triangleIndex, const Ps::aos::Vec3VArg p, const Ps::aos::Vec3VArg q, const Ps::aos::FloatVArg inflatedRadius, Gu::MeshPersistentContact* manifoldContacts, PxU32& numContacts);

	static void generateEEContactsMTD(const Ps::aos::Vec3VArg a, const Ps::aos::Vec3VArg b,const Ps::aos::Vec3VArg c, const Ps::aos::Vec3VArg normal, const PxU32 triangleIndex,
		const Ps::aos::Vec3VArg p, const Ps::aos::Vec3VArg q, const Ps::aos::FloatVArg inflatedRadius, Gu::MeshPersistentContact* manifoldContacts, PxU32& numContacts);

	static void generateEEMTD(const Ps::aos::Vec3VArg p, const Ps::aos::Vec3VArg q,  const Ps::aos::FloatVArg inflatedRadius, const Ps::aos::Vec3VArg normal, const PxU32 trianlgeIndex, 
		const Ps::aos::Vec3VArg a, const Ps::aos::Vec3VArg b, Gu::MeshPersistentContact* manifoldContacts, PxU32& numContacts);

	bool processTriangle(const PxVec3* verts, const PxU32 triangleIndex, PxU8 triFlags, const PxU32* vertInds);

	static bool processTriangle(const TriangleV& triangle, const PxU32 triangleIndex, const CapsuleV& capsule, const Ps::aos::FloatVArg inflatedRadius, const PxU8 triFlag, Gu::MeshPersistentContact* manifoldContacts, PxU32& numContacts);
};

}
}

#endif
