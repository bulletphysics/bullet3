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

#ifndef GU_PCM_CONTACT_MESH_CALLBACK_H
#define GU_PCM_CONTACT_MESH_CALLBACK_H

#include "GuMidphaseInterface.h"
#include "GuEntityReport.h"
#include "GuHeightFieldUtil.h"
#include "GuTriangleCache.h"
#include "GuConvexEdgeFlags.h"

namespace physx
{

namespace Gu
{

template <typename Derived>
struct PCMMeshContactGenerationCallback : MeshHitCallback<PxRaycastHit>
{
public:
	const Cm::FastVertex2ShapeScaling&		mMeshScaling;
	const PxU8* PX_RESTRICT					mExtraTrigData;
	bool									mIdtMeshScale;
	static const PxU32 CacheSize = 16;
	Gu::TriangleCache<CacheSize>			mCache;

	PCMMeshContactGenerationCallback(const Cm::FastVertex2ShapeScaling& meshScaling, const PxU8* extraTrigData, bool idtMeshScale)
	:	MeshHitCallback<PxRaycastHit>(CallbackMode::eMULTIPLE),
		mMeshScaling(meshScaling), mExtraTrigData(extraTrigData), mIdtMeshScale(idtMeshScale)
	{
	}

	void flushCache() 
	{
		if (!mCache.isEmpty())
		{
			(static_cast<Derived*>(this))->template processTriangleCache< CacheSize >(mCache);
			mCache.reset();
		}
	}

	virtual PxAgain processHit(
		const PxRaycastHit& hit, const PxVec3& v0, const PxVec3& v1, const PxVec3& v2, PxReal&, const PxU32* vinds)
	{
		if (!(static_cast<Derived*>(this))->doTest(v0, v1, v2))
			return true;

		PxVec3 v[3];
		if(mIdtMeshScale)
		{
			v[0] = v0;
			v[1] = v1;
			v[2] = v2;
		}
		else
		{
			const PxI32 winding = mMeshScaling.flipsNormal() ? 1 : 0;
			v[0] = mMeshScaling * v0;
			v[1 + winding] = mMeshScaling * v1;
			v[2 - winding] = mMeshScaling * v2;
		}

		const PxU32 triangleIndex = hit.faceIndex;
		const PxU8 extraData = getConvexEdgeFlags(mExtraTrigData, triangleIndex);

		if (mCache.isFull())
		{
			(static_cast<Derived*>(this))->template processTriangleCache< CacheSize >(mCache);
			mCache.reset();
		}
		mCache.addTriangle(v, vinds, triangleIndex, extraData);

		return true;
	}

protected:
	PCMMeshContactGenerationCallback& operator=(const PCMMeshContactGenerationCallback&);
};

template <typename Derived>
struct PCMHeightfieldContactGenerationCallback : Gu::EntityReport<PxU32>
{
public:
	const Gu::HeightFieldUtil&	mHfUtil;
	const PxTransform&			mHeightfieldTransform;
	bool						mBoundaryCollisions;

	PCMHeightfieldContactGenerationCallback(const Gu::HeightFieldUtil& hfUtil, const PxTransform& heightfieldTransform)	:
		mHfUtil(hfUtil), mHeightfieldTransform(heightfieldTransform)
	{
		mBoundaryCollisions = !(hfUtil.getHeightField().getFlags() & PxHeightFieldFlag::eNO_BOUNDARY_EDGES);
	}

	// PT: TODO: refactor/unify with similar code in other places
	virtual PxAgain onEvent(PxU32 nb, PxU32* indices)
	{
		const PxU32 CacheSize = 16;
		Gu::TriangleCache<CacheSize> cache;

		const PxU32 nbPasses = (nb+(CacheSize-1))/CacheSize;
		PxU32 nbTrigs = nb;
		PxU32* inds0 = indices;

		const PxU8 nextInd[] = {2,0,1};

		for(PxU32 i = 0; i < nbPasses; ++i)
		{
			cache.mNumTriangles = 0;
			PxU32 trigCount = PxMin(nbTrigs, CacheSize);
			nbTrigs -= trigCount;
			while(trigCount--)
			{
				PxU32 triangleIndex = *(inds0++);
				PxU32 vertIndices[3];

				PxTriangle currentTriangle;	// in world space

				PxU32 adjInds[3];
				mHfUtil.getTriangle(mHeightfieldTransform, currentTriangle, vertIndices, adjInds, triangleIndex, false, false);

				PxVec3 normal;
				currentTriangle.normal(normal);

				PxU8 triFlags = 0; //KS - temporary until we can calculate triFlags for HF

				for(PxU32 a = 0; a < 3; ++a)
				{

					if (adjInds[a] != 0xFFFFFFFF)
					{
						PxTriangle adjTri;
						PxU32 inds[3];
						mHfUtil.getTriangle(mHeightfieldTransform, adjTri, inds, NULL, adjInds[a], false, false);
						//We now compare the triangles to see if this edge is active

						PX_ASSERT(inds[0] == vertIndices[a] || inds[1] == vertIndices[a] || inds[2] == vertIndices[a]);
						PX_ASSERT(inds[0] == vertIndices[(a + 1) % 3] || inds[1] == vertIndices[(a + 1) % 3] || inds[2] == vertIndices[(a + 1) % 3]);


						PxVec3 adjNormal;
						adjTri.denormalizedNormal(adjNormal);
						PxU32 otherIndex = nextInd[a];
						PxF32 projD = adjNormal.dot(currentTriangle.verts[otherIndex] - adjTri.verts[0]);

						if (projD < 0.f)
						{
							adjNormal.normalize();

							PxF32 proj = adjNormal.dot(normal);

							if (proj < 0.997f)
							{
								triFlags |= (1 << (a + 3));
							}
						}
					}
					else if (mBoundaryCollisions)
					{
						triFlags |= (1 << (a + 3)); //Mark boundary edge active
					}
					else
						triFlags |= (1 << a); //Mark as silhouette edge
				}

				cache.addTriangle(currentTriangle.verts, vertIndices, triangleIndex, triFlags);
			}
			PX_ASSERT(cache.mNumTriangles <= 16);

			(static_cast<Derived*>(this))->template processTriangleCache< CacheSize >(cache);
		}
		return true;
	}	
protected:
	PCMHeightfieldContactGenerationCallback& operator=(const PCMHeightfieldContactGenerationCallback&);
};

}//Gu
}//physx

#endif
