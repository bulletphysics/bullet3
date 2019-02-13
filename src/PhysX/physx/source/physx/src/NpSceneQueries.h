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

#ifndef PX_PHYSICS_NP_SCENEQUERIES
#define PX_PHYSICS_NP_SCENEQUERIES


#include "PxQueryReport.h"
#include "PsIntrinsics.h"
#include "CmPhysXCommon.h"
#include "SqSceneQueryManager.h"
#include "GuTriangleMesh.h"
#include "GuRaycastTests.h"
#include "GuSweepTests.h"
#include "GuOverlapTests.h"
#include "NpSceneAccessor.h"
#include "ScbScene.h"

#if PX_SUPPORT_PVD
#include "NpPvdSceneQueryCollector.h"
#endif

namespace physx { namespace Sq {

	struct QueryID { enum Enum {
		QUERY_RAYCAST_ANY_OBJECT,
		QUERY_RAYCAST_CLOSEST_OBJECT,
		QUERY_RAYCAST_ALL_OBJECTS,

		QUERY_OVERLAP_SPHERE_ALL_OBJECTS,
		QUERY_OVERLAP_AABB_ALL_OBJECTS,
		QUERY_OVERLAP_OBB_ALL_OBJECTS,
		QUERY_OVERLAP_CAPSULE_ALL_OBJECTS,
		QUERY_OVERLAP_CONVEX_ALL_OBJECTS,

		QUERY_LINEAR_OBB_SWEEP_CLOSEST_OBJECT,
		QUERY_LINEAR_CAPSULE_SWEEP_CLOSEST_OBJECT,
		QUERY_LINEAR_CONVEX_SWEEP_CLOSEST_OBJECT
	}; };
}

struct MultiQueryInput
{
	const PxVec3* rayOrigin; // only valid for raycasts
	const PxVec3* unitDir; // only valid for raycasts and sweeps
	PxReal maxDistance; // only valid for raycasts and sweeps
	const PxGeometry* geometry; // only valid for overlaps and sweeps
	const PxTransform* pose; // only valid for overlaps and sweeps
	PxReal inflation; // only valid for sweeps

	// Raycast constructor
	MultiQueryInput(const PxVec3& aRayOrigin, const PxVec3& aUnitDir, PxReal aMaxDist)
	{
		Ps::prefetchLine(&aRayOrigin);
		Ps::prefetchLine(&aUnitDir);
		rayOrigin = &aRayOrigin;
		unitDir = &aUnitDir;
		maxDistance = aMaxDist;
		geometry = NULL;
		pose = NULL;
		inflation = 0.0f;
	}

	// Overlap constructor
	MultiQueryInput(const PxGeometry* aGeometry, const PxTransform* aPose)
	{
		Ps::prefetchLine(aGeometry);
		Ps::prefetchLine(aPose);
		geometry = aGeometry;
		pose = aPose;
		inflation = 0.0f;
		rayOrigin = unitDir = NULL;
	}

	// Sweep constructor
	MultiQueryInput(
		const PxGeometry* aGeometry, const PxTransform* aPose,
		const PxVec3& aUnitDir, const PxReal aMaxDist, const PxReal aInflation)
	{
		Ps::prefetchLine(aGeometry);
		Ps::prefetchLine(aPose);
		Ps::prefetchLine(&aUnitDir);
		rayOrigin = NULL;
		maxDistance = aMaxDist;
		unitDir = &aUnitDir;
		geometry = aGeometry;
		pose = aPose;
		inflation = aInflation;
	}

	PX_FORCE_INLINE const PxVec3& getDir() const { PX_ASSERT(unitDir); return *unitDir; }
	PX_FORCE_INLINE const PxVec3& getOrigin() const { PX_ASSERT(rayOrigin); return *rayOrigin; }
};

struct BatchQueryFilterData
{
	void*							filterShaderData;
	PxU32							filterShaderDataSize;
	PxBatchQueryPreFilterShader		preFilterShader;	
	PxBatchQueryPostFilterShader	postFilterShader;	
	#if PX_SUPPORT_PVD
	Vd::PvdSceneQueryCollector*	collector; // gets set to bq collector
	#endif
	BatchQueryFilterData(void* fsData, PxU32 fsSize, PxBatchQueryPreFilterShader preFs, PxBatchQueryPostFilterShader postFs)
		: filterShaderData(fsData), filterShaderDataSize(fsSize), preFilterShader(preFs), postFilterShader(postFs)
	{
		#if PX_SUPPORT_PVD
		collector = NULL;
		#endif
	}
};

class PxGeometry;

class NpSceneQueries : public NpSceneAccessor
{
	PX_NOCOPY(NpSceneQueries)

public:
	NpSceneQueries(const PxSceneDesc& desc);
	~NpSceneQueries();

	template<typename QueryHit>
					bool							multiQuery(
														const MultiQueryInput& in,
														PxHitCallback<QueryHit>& hits, PxHitFlags hitFlags, const PxQueryCache* cache,
														const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
														BatchQueryFilterData* bqFd) const;

	// Synchronous scene queries
	virtual			bool							raycast(
														const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,	// Ray data
														PxRaycastCallback& hitCall, PxHitFlags hitFlags,
														const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
														const PxQueryCache* cache) const;

	virtual			bool							sweep(
														const PxGeometry& geometry, const PxTransform& pose,	// GeomObject data
														const PxVec3& unitDir, const PxReal distance,	// Ray data
														PxSweepCallback& hitCall, PxHitFlags hitFlags,
														const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
														const PxQueryCache* cache, const PxReal inflation) const;

	virtual			bool							overlap(
														const PxGeometry& geometry, const PxTransform& transform,	// GeomObject data
														PxOverlapCallback& hitCall, 
														const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall) const;

	PX_FORCE_INLINE	PxU64							getContextId()				const	{ return PxU64(reinterpret_cast<size_t>(this)); }
	PX_FORCE_INLINE	Scb::Scene&						getScene()							{ return mScene; }
	PX_FORCE_INLINE	const Scb::Scene&				getScene()					const	{ return mScene; }
	PX_FORCE_INLINE	PxU32							getFlagsFast()				const	{ return mScene.getFlags();						}
	PX_FORCE_INLINE	Sq::SceneQueryManager&			getSceneQueryManagerFast()			{ return mSQManager;							}
	PX_FORCE_INLINE	PxSceneQueryUpdateMode::Enum	getSceneQueryUpdateModeFast() const	{ return mSceneQueryUpdateMode;					}

					void							sceneQueriesStaticPrunerUpdate(PxBaseTask* continuation);
					void							sceneQueriesDynamicPrunerUpdate(PxBaseTask* continuation);

					Scb::Scene						mScene;

					Sq::SceneQueryManager			mSQManager;

					const Gu::GeomRaycastTable&		mCachedRaycastFuncs;
					const Gu::GeomSweepFuncs&		mCachedSweepFuncs;
					const Gu::GeomOverlapTable*		mCachedOverlapFuncs;

					typedef Cm::DelegateTask<NpSceneQueries, &NpSceneQueries::sceneQueriesStaticPrunerUpdate> SceneQueriesStaticPrunerUpdate;
					typedef Cm::DelegateTask<NpSceneQueries, &NpSceneQueries::sceneQueriesDynamicPrunerUpdate> SceneQueriesDynamicPrunerUpdate;
					SceneQueriesStaticPrunerUpdate	mSceneQueriesStaticPrunerUpdate;
					SceneQueriesDynamicPrunerUpdate	mSceneQueriesDynamicPrunerUpdate;

					PxSceneQueryUpdateMode::Enum    mSceneQueryUpdateMode;

#if PX_SUPPORT_PVD
public:
					//Scene query and hits for pvd, collected in current frame
					mutable Vd::PvdSceneQueryCollector		mSingleSqCollector;
					mutable Vd::PvdSceneQueryCollector		mBatchedSqCollector;

PX_FORCE_INLINE				Vd::PvdSceneQueryCollector&	getSingleSqCollector() const {return mSingleSqCollector;}
PX_FORCE_INLINE				Vd::PvdSceneQueryCollector&	getBatchedSqCollector() const {return mBatchedSqCollector;}
#endif // PX_SUPPORT_PVD
};

#if PX_SUPPORT_EXTERN_TEMPLATE
//explicit template instantiation declaration
extern template
bool NpSceneQueries::multiQuery<PxRaycastHit>(const MultiQueryInput&, PxHitCallback<PxRaycastHit>&, PxHitFlags, const PxQueryCache*, const PxQueryFilterData&, PxQueryFilterCallback*, BatchQueryFilterData*) const;

extern template
bool NpSceneQueries::multiQuery<PxOverlapHit>(const MultiQueryInput&, PxHitCallback<PxOverlapHit>&, PxHitFlags, const PxQueryCache*, const PxQueryFilterData&, PxQueryFilterCallback*, BatchQueryFilterData*) const;

extern template
bool NpSceneQueries::multiQuery<PxSweepHit>(const MultiQueryInput&, PxHitCallback<PxSweepHit>&, PxHitFlags, const PxQueryCache*, const PxQueryFilterData&, PxQueryFilterCallback*, BatchQueryFilterData*) const;
#endif

namespace Sq { class AABBPruner; class AABBTreeRuntimeNode; class AABBTree; }

#if PX_VC 
    #pragma warning(push)
	#pragma warning( disable : 4324 ) // Padding was added at the end of a structure because of a __declspec(align) value.
#endif

#if PX_VC 
     #pragma warning(pop) 
#endif

} // namespace physx, sq

#endif
