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

#include "GuIntersectionRayBox.h"
#include "PxGeometryQuery.h"
#include "NpRigidDynamic.h"
#include "NpQueryShared.h"
#include "SqPruner.h"
#include "GuBounds.h"
#include "GuIntersectionRay.h"
#include "common/PxProfileZone.h"

// Synchronous scene queries

using namespace physx;
using namespace Sq;
using namespace Gu;

#if PX_SUPPORT_PVD
#include "NpPvdSceneQueryCollector.h"
#endif

namespace local
{
	// helper class to encapsulate Scb::Actor and Shape together with PxActorShape
	struct ActorShape : PxActorShape
	{
		const Scb::Shape* scbShape;
		const Scb::Actor* scbActor;

		ActorShape() : PxActorShape() {}

		ActorShape(PxRigidActor* eaActor, PxShape* eaShape, Scb::Shape* sShape, Scb::Actor* sActor) : PxActorShape(eaActor, eaShape)
		{
			scbShape = sShape;
			scbActor = sActor;
		}
	};

	// fill the helper actor shape
	static PX_FORCE_INLINE void populate(const PrunerPayload& payload, ActorShape& as)
	{
		Scb::Shape* localShape = reinterpret_cast<Scb::Shape*>(payload.data[0]);
		Scb::Actor* localActor = reinterpret_cast<Scb::Actor*>(payload.data[1]);

		as.scbShape = localShape;
		as.scbActor = localActor;

		as.actor = static_cast<PxRigidActor*>(static_cast<const Sc::RigidCore&>(localActor->getActorCore()).getPxActor());
		as.shape = localShape->getScShape().getPxShape();
	}
}

///////////////////////////////////////////////////////////////////////////////
bool NpSceneQueries::raycast(
	const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,
	PxHitCallback<PxRaycastHit>& hits, PxHitFlags hitFlags, const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
	const PxQueryCache* cache) const
{
	PX_PROFILE_ZONE("SceneQuery.raycast", getContextId());
	NP_READ_CHECK(this);	
	PX_SIMD_GUARD;

	MultiQueryInput input(origin, unitDir, distance);
	return multiQuery<PxRaycastHit>(input, hits, hitFlags, cache, filterData, filterCall, NULL);
}

//////////////////////////////////////////////////////////////////////////
bool NpSceneQueries::overlap(
	const PxGeometry& geometry, const PxTransform& pose, PxOverlapCallback& hits,
	const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall) const
{
	PX_PROFILE_ZONE("SceneQuery.overlap", getContextId());
	NP_READ_CHECK(this);	
	PX_SIMD_GUARD;

	MultiQueryInput input(&geometry, &pose);
	// we are not supporting cache for overlaps for some reason
	return multiQuery<PxOverlapHit>(input, hits, PxHitFlags(), NULL, filterData, filterCall, NULL);
}

///////////////////////////////////////////////////////////////////////////////
bool NpSceneQueries::sweep(
	const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, const PxReal distance,
	PxHitCallback<PxSweepHit>& hits, PxHitFlags hitFlags, const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
	const PxQueryCache* cache, const PxReal inflation) const
{
	PX_PROFILE_ZONE("SceneQuery.sweep", getContextId());
	NP_READ_CHECK(this);	
	PX_SIMD_GUARD;

#if PX_CHECKED
	if(!PxGeometryQuery::isValid(geometry))
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, "Provided geometry is not valid");
		return false;
	}
#endif // PX_CHECKED


	if((hitFlags & PxHitFlag::ePRECISE_SWEEP) && (hitFlags & PxHitFlag::eMTD))
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, " Precise sweep doesn't support MTD. Perform MTD with default sweep");
		hitFlags &= ~PxHitFlag::ePRECISE_SWEEP;
	}

	if((hitFlags & PxHitFlag::eASSUME_NO_INITIAL_OVERLAP) && (hitFlags & PxHitFlag::eMTD))
	{
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, " eMTD cannot be used in conjunction with eASSUME_NO_INITIAL_OVERLAP. eASSUME_NO_INITIAL_OVERLAP will be ignored");
		hitFlags &= ~PxHitFlag::eASSUME_NO_INITIAL_OVERLAP;
	}

	PxReal realInflation = inflation;
	if((hitFlags & PxHitFlag::ePRECISE_SWEEP)&& inflation > 0.f)
	{
		realInflation = 0.f;
		Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, " Precise sweep doesn't support inflation, inflation will be overwritten to be zero");
	}
	MultiQueryInput input(&geometry, &pose, unitDir, distance, realInflation);
	return multiQuery<PxSweepHit>(input, hits, hitFlags, cache, filterData, filterCall, NULL);
}

///////////////////////////////////////////////////////////////////////////////
//========================================================================================================================

static PX_FORCE_INLINE bool applyAllPreFiltersSQ(
	const local::ActorShape* as, PxQueryHitType::Enum& hitType, const PxQueryFlags& inFilterFlags,
	const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
	BatchQueryFilterData* bfd, PxHitFlags& queryFlags/*, PxU32 maxNbTouches*/)
{
	// AP: the !bfd clause is here because there's no other way to pass data to BQ pre/post filter shaders
	// For normal query the data can be passed with inherited callback instance
	// So if for BQ SPU filter shader the user tries to pass data via FD, the equation will always cut it out
	// AP scaffold TODO: once SPU is officially phased out we can remove the !bfd clause, fix broken UTs (that are wrong)
	// and also remove support for filter shaders
	if(!bfd && !applyFilterEquation(*as->scbShape, filterData.data))
		return false;

	if((inFilterFlags & PxQueryFlag::ePREFILTER) && (filterCall || bfd))
	{
		PxHitFlags outQueryFlags = queryFlags;

		if(filterCall)
			hitType = filterCall->preFilter(filterData.data, as->shape, as->actor, outQueryFlags);
		else if(bfd->preFilterShader)
			hitType = bfd->preFilterShader(
				filterData.data, as->scbShape->getScShape().getQueryFilterData(),
				bfd->filterShaderData, bfd->filterShaderDataSize, outQueryFlags);

		// AP: at this point the callback might return eTOUCH but the touch buffer can be empty, the hit will be discarded
		//PX_CHECK_MSG(hitType == PxQueryHitType::eTOUCH ? maxNbTouches > 0 : true,
		//	"SceneQuery: preFilter returned eTOUCH but empty touch buffer was provided, hit discarded.");

		queryFlags = (queryFlags & ~PxHitFlag::eMODIFIABLE_FLAGS) | (outQueryFlags & PxHitFlag::eMODIFIABLE_FLAGS);
	}
	// test passed, continue to return as;
	return true;
}

//========================================================================================================================
// performs a single geometry query for any HitType (PxSweepHit, PxOverlapHit, PxRaycastHit)
template<typename HitType>
struct GeomQueryAny
{
	static PX_FORCE_INLINE PxU32 geomHit(
		const NpSceneQueries& sceneQueries, const MultiQueryInput& input, const ShapeData& sd,
		const PxGeometry& sceneGeom, const PxTransform& pose, PxHitFlags hitFlags,
		PxU32 maxHits, HitType* hits, const PxReal shrunkMaxDistance, PxBounds3* precomputedBounds)
	{
		const PxGeometry& geom0 = *input.geometry;
		const PxTransform& pose0 = *input.pose;
		const PxGeometry& geom1 = sceneGeom;
		const PxTransform& pose1 = pose;

		// Handle raycasts
		if(HitTypeSupport<HitType>::IsRaycast)
		{
			// the test for mesh AABB is archived in //sw/physx/dev/apokrovsky/graveyard/sqMeshAABBTest.cpp
			// TODO: investigate performance impact (see US12801)
			PX_CHECK_AND_RETURN_VAL(input.getDir().isFinite(), "PxScene::raycast(): rayDir is not valid.", 0);
			PX_CHECK_AND_RETURN_VAL(input.getOrigin().isFinite(), "PxScene::raycast(): rayOrigin is not valid.", 0);
			PX_CHECK_AND_RETURN_VAL(pose1.isValid(), "PxScene::raycast(): pose is not valid.", 0);
			PX_CHECK_AND_RETURN_VAL(shrunkMaxDistance >= 0.0f, "PxScene::raycast(): maxDist is negative.", 0);
			PX_CHECK_AND_RETURN_VAL(PxIsFinite(shrunkMaxDistance), "PxScene::raycast(): maxDist is not valid.", 0);
			PX_CHECK_AND_RETURN_VAL(PxAbs(input.getDir().magnitudeSquared()-1)<1e-4f,
				"PxScene::raycast(): ray direction must be unit vector.", 0);

			// PT: TODO: investigate perf difference
			const RaycastFunc func = sceneQueries.mCachedRaycastFuncs[geom1.getType()];
			return func(geom1, pose1, input.getOrigin(), input.getDir(), shrunkMaxDistance,
						hitFlags, maxHits, reinterpret_cast<PxRaycastHit*>(hits));
		}
		// Handle sweeps
		else if(HitTypeSupport<HitType>::IsSweep)
		{
			PX_ASSERT(precomputedBounds != NULL);
			// b0 = query shape bounds
			// b1 = scene shape bounds
			// AP: Here we clip the sweep to bounds with sum of extents. This is needed for GJK stability.
			// because sweep is equivalent to a raycast vs a scene shape with inflated bounds.
			// This also may (or may not) provide an optimization for meshes because top level of rtree has multiple boxes
			// and there is no bounds test for the whole mesh elsewhere
			PxBounds3 b0 = *precomputedBounds, b1;
			// compute the scene geometry bounds
			// PT: TODO: avoid recomputing the bounds here
			Gu::computeBounds(b1, sceneGeom, pose, 0.0f, NULL, 1.0f);
			const PxVec3 combExt = (b0.getExtents() + b1.getExtents())*1.01f;

			PxF32 tnear, tfar;
			if(!intersectRayAABB2(-combExt, combExt, b0.getCenter() - b1.getCenter(), input.getDir(), shrunkMaxDistance, tnear, tfar)) // returns (tnear<tfar)
				if(tnear>tfar) // this second test is needed because shrunkMaxDistance can be 0 for 0 length sweep
					return 0;
			PX_ASSERT(input.getDir().isNormalized());
			// tfar is now the t where the ray exits the AABB. input.getDir() is normalized

			const PxVec3& unitDir = input.getDir();
			PxSweepHit& sweepHit = reinterpret_cast<PxSweepHit&>(hits[0]);
			
			// if we don't start inside the AABB box, offset the start pos, because of precision issues with large maxDist
			const bool offsetPos = (tnear > GU_RAY_SURFACE_OFFSET);
			const PxReal offset = offsetPos ? (tnear - GU_RAY_SURFACE_OFFSET) : 0.0f;
			const PxVec3 offsetVec(offsetPos ? (unitDir*offset) : PxVec3(0.0f));
			// we move the geometry we sweep against, so that we avoid the Gu::Capsule/Box recomputation
			const PxTransform pose1Offset(pose1.p - offsetVec, pose1.q);
            
			const PxReal distance = PxMin(tfar, shrunkMaxDistance) - offset;
			const PxReal inflation = input.inflation;
			PX_CHECK_AND_RETURN_VAL(pose0.isValid(), "PxScene::sweep(): pose0 is not valid.", 0);
			PX_CHECK_AND_RETURN_VAL(pose1Offset.isValid(), "PxScene::sweep(): pose1 is not valid.", 0);
			PX_CHECK_AND_RETURN_VAL(unitDir.isFinite(), "PxScene::sweep(): unitDir is not valid.", 0);
			PX_CHECK_AND_RETURN_VAL(PxIsFinite(distance), "PxScene::sweep(): distance is not valid.", 0);
			PX_CHECK_AND_RETURN_VAL((distance >= 0.0f && !(hitFlags & PxHitFlag::eASSUME_NO_INITIAL_OVERLAP)) || distance > 0.0f,
				"PxScene::sweep(): sweep distance must be >=0 or >0 with eASSUME_NO_INITIAL_OVERLAP.", 0);

			PxU32 retVal = 0;
			const GeomSweepFuncs& sf = sceneQueries.mCachedSweepFuncs;
			switch(geom0.getType())
			{
				case PxGeometryType::eSPHERE:
				{
					const PxSphereGeometry& sphereGeom = static_cast<const PxSphereGeometry&>(geom0);
					// PT: TODO: technically this capsule with 0.0 half-height is invalid ("isValid" returns false)
					const PxCapsuleGeometry capsuleGeom(sphereGeom.radius, 0.0f);
					const Capsule worldCapsule(pose0.p, pose0.p, sphereGeom.radius); // AP: precompute?
					const bool precise = hitFlags & PxHitFlag::ePRECISE_SWEEP;
					const SweepCapsuleFunc func = precise ? sf.preciseCapsuleMap[geom1.getType()] : sf.capsuleMap[geom1.getType()];
					retVal = PxU32(func(geom1, pose1Offset, capsuleGeom, pose0, worldCapsule, unitDir, distance, sweepHit, hitFlags, inflation));
				}
				break;

				case PxGeometryType::eCAPSULE:
				{
					const bool precise = hitFlags & PxHitFlag::ePRECISE_SWEEP;
					const SweepCapsuleFunc func = precise ? sf.preciseCapsuleMap[geom1.getType()] : sf.capsuleMap[geom1.getType()];
					retVal = PxU32(func(geom1, pose1Offset, static_cast<const PxCapsuleGeometry&>(geom0), pose0, sd.getGuCapsule(), unitDir, distance, sweepHit, hitFlags, inflation));
				}
				break;
	
				case PxGeometryType::eBOX:
				{
					const bool precise = hitFlags & PxHitFlag::ePRECISE_SWEEP;
					const SweepBoxFunc func = precise ? sf.preciseBoxMap[geom1.getType()] : sf.boxMap[geom1.getType()];
					retVal = PxU32(func(geom1, pose1Offset, static_cast<const PxBoxGeometry&>(geom0), pose0, sd.getGuBox(), unitDir, distance, sweepHit, hitFlags, inflation));
				}
				break;
	
				case PxGeometryType::eCONVEXMESH:
				{
					const PxConvexMeshGeometry& convexGeom = static_cast<const PxConvexMeshGeometry&>(geom0);
					const SweepConvexFunc func = sf.convexMap[geom1.getType()];
					retVal = PxU32(func(geom1, pose1Offset, convexGeom, pose0, unitDir, distance, sweepHit, hitFlags, inflation));
				}
				break;
				case PxGeometryType::ePLANE:
				case PxGeometryType::eTRIANGLEMESH:
				case PxGeometryType::eHEIGHTFIELD:
				case PxGeometryType::eGEOMETRY_COUNT:
				case PxGeometryType::eINVALID:
					physx::shdfnd::getFoundation().error(physx::PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__,
							"PxScene::sweep(): first geometry object parameter must be sphere, capsule, box or convex geometry.");
				break;
			}
			if (retVal)
			{
				// we need to offset the distance back
				sweepHit.distance += offset;
				// we need to offset the hit position back as we moved the geometry we sweep against
				sweepHit.position += offsetVec;
			}
			return retVal;
		}
		// Handle overlaps
		else if(HitTypeSupport<HitType>::IsOverlap)
		{
			const GeomOverlapTable* overlapFuncs = sceneQueries.mCachedOverlapFuncs;
			return PxU32(Gu::overlap(geom0, pose0, geom1, pose1, overlapFuncs));
		}
		else
		{
			PX_ALWAYS_ASSERT_MESSAGE("Unexpected template expansion in GeomQueryAny::geomHit");
			return 0;
		}
	}
};

// struct to access protected data members in the public PxHitCallback API
template<typename HitType>
struct MultiQueryCallback : public PrunerCallback
{
	const NpSceneQueries&		mScene;
	const MultiQueryInput&		mInput;
	PxHitCallback<HitType>&		mHitCall;
	const PxHitFlags			mHitFlags;
	const PxQueryFilterData&	mFilterData;
	PxQueryFilterCallback*		mFilterCall;
	PxReal						mShrunkDistance;
	BatchQueryFilterData*		mBfd; // only not NULL for batch queries
	const PxHitFlags			mMeshAnyHitFlags;
	bool						mReportTouchesAgain;
	bool						mFarBlockFound; // this is to prevent repeated searches for far block
	bool						mNoBlock;
	const bool					mAnyHit;
	bool						mIsCached; // is this call coming as a callback from the pruner or a single item cached callback?

	// The reason we need these bounds is because we need to know combined(inflated shape) bounds to clip the sweep path
	// to be tolerable by GJK precision issues. This test is done for (queryShape vs touchedShapes)
	// So it makes sense to cache the bounds for sweep query shape, otherwise we'd have to recompute them every time
	// Currently only used for sweeps.
	PxBounds3					mQueryShapeBounds;
	bool						mQueryShapeBoundsValid;
	const ShapeData*			mShapeData;

	MultiQueryCallback(
		const NpSceneQueries& scene, const MultiQueryInput& input, bool anyHit, PxHitCallback<HitType>& hitCall, PxHitFlags hitFlags,
		const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall, PxReal shrunkDistance, BatchQueryFilterData* aBfd) :
			mScene					(scene),
			mInput					(input),
			mHitCall				(hitCall),
			mHitFlags				(hitFlags),
			mFilterData				(filterData),
			mFilterCall				(filterCall),
			mShrunkDistance			(shrunkDistance),
			mBfd					(aBfd),
			mMeshAnyHitFlags		((hitFlags.isSet(PxHitFlag::eMESH_ANY) || anyHit) ? PxHitFlag::eMESH_ANY : PxHitFlag::Enum(0)),
			mReportTouchesAgain		(true),
			mFarBlockFound			(filterData.flags & PxQueryFlag::eNO_BLOCK),
			mNoBlock				(filterData.flags & PxQueryFlag::eNO_BLOCK),
			mAnyHit					(anyHit),
			mIsCached				(false),
			mQueryShapeBoundsValid	(false),
			mShapeData				(NULL)
	{
	}
	
	virtual PxAgain invoke(PxReal& aDist, const PrunerPayload& aPayload)
	{
		const PxU32 tempCount = 1;
		HitType tempBuf[tempCount];

		// PT: TODO: do we need actorShape.actor/actorShape.shape immediately?
		local::ActorShape actorShape;
		local::populate(aPayload, actorShape);

		const PxQueryFlags filterFlags = mFilterData.flags;

		// for no filter callback, default to eTOUCH for MULTIPLE, eBLOCK otherwise
		// also always treat as eBLOCK if currently tested shape is cached
		// Using eRESERVED flag as a special condition to default to eTOUCH hits while only looking for a single blocking hit
		// from a nested query (see other comments containing #LABEL1)
		PxQueryHitType::Enum shapeHitType =
			((mHitCall.maxNbTouches || (mFilterData.flags & PxQueryFlag::eRESERVED)) && !mIsCached)
				? PxQueryHitType::eTOUCH
				: PxQueryHitType::eBLOCK;

		// apply pre-filter
		PxHitFlags filteredHitFlags = mHitFlags;
		if(!mIsCached) // don't run filters on single item cache
			if(!applyAllPreFiltersSQ(&actorShape, shapeHitType/*in&out*/, filterFlags, mFilterData, mFilterCall,
					mBfd, filteredHitFlags/*, mHitCall.maxNbTouches*/))
				return true; // skip this shape from reporting if prefilter said to do so
		if(shapeHitType == PxQueryHitType::eNONE)
			return true;

		PX_ASSERT(actorShape.actor && actorShape.shape);
		const Scb::Shape* shape = actorShape.scbShape;
		const Scb::Actor* actor = actorShape.scbActor;

		// compute the global pose for the cached shape and actor
		PX_ALIGN(16, PxTransform) globalPose;
		NpActor::getGlobalPose(globalPose, *shape, *actor);

		const PxGeometry& shapeGeom = shape->getGeometry();

		// Here we decide whether to use the user provided buffer in place or a local stack buffer
		// see if we have more room left in the callback results buffer than in the parent stack buffer
		// if so get subHits in-place in the hit buffer instead of the parent stack buffer
		// nbTouches is the number of accumulated touch hits so far
		// maxNbTouches is the size of the user buffer
		PxU32 maxSubHits1 = mHitCall.maxNbTouches - mHitCall.nbTouches; // how much room is left in the user buffer
		HitType* subHits1 = mHitCall.touches + mHitCall.nbTouches; // pointer to the first free hit in the user buffer
		if(mHitCall.nbTouches >= mHitCall.maxNbTouches)
		// if there's no room left in the user buffer, use a stack buffer
		{
			// tried using 64 here - causes check stack code to get generated on xbox, perhaps because of guard page
			// need this buffer in case the input buffer is full but we still want to correctly merge results from later hits
			maxSubHits1 = tempCount;
			subHits1 = reinterpret_cast<HitType*>(tempBuf);
		}

		// limit number of hits to 1 for meshes if eMESH_MULTIPLE wasn't specified. this tells geomQuery to only look for a closest hit
		if(shapeGeom.getType() == PxGeometryType::eTRIANGLEMESH && !(filteredHitFlags & PxHitFlag::eMESH_MULTIPLE))
			maxSubHits1 = 1; // required to only receive 1 hit to pass UTs
		// call the geometry specific intersection template
		PxU32 nbSubHits = GeomQueryAny<HitType>::geomHit(
			mScene, mInput, *mShapeData, shapeGeom, globalPose,
			filteredHitFlags | mMeshAnyHitFlags,
			maxSubHits1, subHits1, mShrunkDistance, mQueryShapeBoundsValid ? &mQueryShapeBounds : NULL);

		// ------------------------- iterate over geometry subhits -----------------------------------
		for (PxU32 iSubHit = 0; iSubHit < nbSubHits; iSubHit++)
		{
			HitType& hit = subHits1[iSubHit];
			hit.actor = actorShape.actor;
			hit.shape = actorShape.shape;

			// some additional processing only for sweep hits with initial overlap
			if(HitTypeSupport<HitType>::IsSweep && HITDIST(hit) == 0.0f && !(filteredHitFlags & PxHitFlag::eMTD))
				// PT: necessary as some leaf routines are called with reversed params, thus writing +unitDir there.
				// AP: apparently still necessary to also do in Gu because Gu can be used standalone (without SQ)
				reinterpret_cast<PxSweepHit&>(hit).normal = -mInput.getDir();

			// start out with hitType for this cached shape set to a pre-filtered hit type
			PxQueryHitType::Enum hitType = shapeHitType;

			// run the post-filter if specified in filterFlags and filterCall is non-NULL
			if(!mIsCached && (mFilterCall || mBfd) && (filterFlags & PxQueryFlag::ePOSTFILTER))
			{
				if(mFilterCall)
					hitType = mFilterCall->postFilter(mFilterData.data, hit);
				else if(mBfd->postFilterShader)
					hitType = mBfd->postFilterShader(
						mFilterData.data, actorShape.scbShape->getScShape().getQueryFilterData(),
						mBfd->filterShaderData, mBfd->filterShaderDataSize, hit);
			}

			// early out on any hit if eANY_HIT was specified, regardless of hit type
			if(mAnyHit && hitType != PxQueryHitType::eNONE)
			{
				// block or touch qualifies for qType=ANY type hit => return it as blocking according to spec. Ignore eNONE.
				mHitCall.block = hit;
				mHitCall.hasBlock = true;
				return false; // found a hit for ANY qType, can early exit now
			}

			if(mNoBlock)
				hitType = PxQueryHitType::eTOUCH;

			PX_WARN_ONCE_IF(HitTypeSupport<HitType>::IsOverlap && hitType == PxQueryHitType::eBLOCK, 
				"eBLOCK returned from user filter for overlap() query. This may cause undesired behavior. "
				"Consider using PxQueryFlag::eNO_BLOCK for overlap queries.");

			if(hitType == PxQueryHitType::eTOUCH)
			{
				// -------------------------- handle eTOUCH hits ---------------------------------
				// for qType=multiple, store the hit. For other qTypes ignore it.
				// <= is important for initially overlapping sweeps
				#if PX_CHECKED
				if(mHitCall.maxNbTouches == 0 && !mBfd && !mFilterData.flags.isSet(PxQueryFlag::eRESERVED))
					// issue a warning if eTOUCH was returned by the prefilter, we have 0 touch buffer and not a batch query
					// not doing for BQ because the touches buffer can be overflown and thats ok by spec
					// eRESERVED to avoid a warning from nested callback (closest blocking hit recursive search)
					Ps::getFoundation().error(PxErrorCode::eINVALID_OPERATION, __FILE__, __LINE__,
											"User filter returned PxQueryHitType::eTOUCH but the touches buffer was empty. Hit was discarded.");
				#endif

				if(mHitCall.maxNbTouches && mReportTouchesAgain && HITDIST(hit) <= mShrunkDistance)
				{
					// Buffer full: need to find the closest blocking hit, clip touch hits and flush the buffer
					if(mHitCall.nbTouches == mHitCall.maxNbTouches)
					{
						// issue a second nested query just looking for the closest blocking hit
						// could do better perf-wise by saving traversal state (start looking for blocking from this point)
						// but this is not a perf critical case because users can provide a bigger buffer
						// that covers non-degenerate cases
						// far block search doesn't apply to overlaps because overlaps don't work with blocking hits
						if(HitTypeSupport<HitType>::IsOverlap == 0)
						{
							// AP: the use of eRESERVED is a bit tricky, see other comments containing #LABEL1
							PxQueryFilterData fd1 = mFilterData; fd1.flags |= PxQueryFlag::eRESERVED;
							PxHitBuffer<HitType> buf1; // create a temp callback buffer for a single blocking hit
							if(!mFarBlockFound && mHitCall.maxNbTouches > 0 && mScene.NpSceneQueries::multiQuery<HitType>(
								mInput, buf1, mHitFlags, NULL, fd1, mFilterCall, mBfd))
							{
								mHitCall.block = buf1.block;
								mHitCall.hasBlock = true;
								mHitCall.nbTouches =
									clipHitsToNewMaxDist<HitType>(mHitCall.touches, mHitCall.nbTouches, HITDIST(buf1.block));
								mShrunkDistance = HITDIST(buf1.block);
								aDist = mShrunkDistance;
							}
							mFarBlockFound = true;
						}
						if(mHitCall.nbTouches == mHitCall.maxNbTouches)
						{
							mReportTouchesAgain = mHitCall.processTouches(mHitCall.touches, mHitCall.nbTouches);
							if(!mReportTouchesAgain)
								return false; // optimization - buffer is full 
							else
								mHitCall.nbTouches = 0; // reset nbTouches so we can continue accumulating again
						}
					}

					//if(hitCall.nbTouches < hitCall.maxNbTouches) // can be true if maxNbTouches is 0
					mHitCall.touches[mHitCall.nbTouches++] = hit;
				} // if(hitCall.maxNbTouches && reportTouchesAgain && HITDIST(hit) <= shrunkDistance)
			} // if(hitType == PxQueryHitType::eTOUCH)
			else if(hitType == PxQueryHitType::eBLOCK)
			{
				// -------------------------- handle eBLOCK hits ----------------------------------
				// only eBLOCK qualifies as a closest hit candidate => compare against best distance and store
				// <= is needed for eTOUCH hits to be recorded correctly vs same eBLOCK distance for overlaps
				if(HITDIST(hit) <= mShrunkDistance)
				{
					if(HitTypeSupport<HitType>::IsOverlap == 0)
					{
						mShrunkDistance = HITDIST(hit);
						aDist = mShrunkDistance;
					}
					mHitCall.block = hit;
					mHitCall.hasBlock = true;
				}
			} // if(hitType == eBLOCK)
			else {
				PX_ASSERT(hitType == PxQueryHitType::eNONE);
			}
		} // for iSubHit
		return true;
	}

private:
	MultiQueryCallback<HitType>& operator=(const MultiQueryCallback<HitType>&);
};

//========================================================================================================================
#if PX_SUPPORT_PVD
template<typename HitType>
struct CapturePvdOnReturn : public PxHitCallback<HitType>
{
	// copy the arguments of multiQuery into a struct, this is strictly for PVD recording
	const NpSceneQueries*		mSQ;
	const MultiQueryInput&		mInput;
	PxHitFlags					mHitFlags;		// PT: TODO: this is not used!
	const PxQueryCache*			mCache;			// PT: TODO: this is not used!
	const PxQueryFilterData&	mFilterData;
	PxQueryFilterCallback*		mFilterCall;	// PT: TODO: this is not used!
	BatchQueryFilterData*		mBFD;			// PT: TODO: check if this is sometimes not NULL
	Ps::Array<HitType>			mAllHits;
	PxHitCallback<HitType>&		mParentCallback;

	CapturePvdOnReturn(
		const NpSceneQueries* sq, const MultiQueryInput& input, PxHitFlags hitFlags,
		const PxQueryCache* cache, const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall,
		BatchQueryFilterData* bfd, PxHitCallback<HitType>& parentCallback) :
		PxHitCallback<HitType>	(parentCallback.touches, parentCallback.maxNbTouches),
		mSQ						(sq),
		mInput					(input),
		mHitFlags				(hitFlags),
		mCache					(cache),
		mFilterData				(filterData),
		mFilterCall				(filterCall),
		mBFD					(bfd),
		mParentCallback			(parentCallback)
	{}

	virtual PxAgain processTouches(const HitType* hits, PxU32 nbHits)
	{
		const PxAgain again = mParentCallback.processTouches(hits, nbHits);
		for(PxU32 i=0; i<nbHits; i++)
			mAllHits.pushBack(hits[i]);
		return again;
	}

	~CapturePvdOnReturn()
	{
		const physx::Vd::ScbScenePvdClient& pvdClient = mSQ->getScene().getScenePvdClient();
		if(!(pvdClient.checkPvdDebugFlag() && (pvdClient.getScenePvdFlagsFast() & PxPvdSceneFlag::eTRANSMIT_SCENEQUERIES)))
			return;

		physx::Vd::PvdSceneQueryCollector& collector = mBFD ? mSQ->getBatchedSqCollector() : mSQ->getSingleSqCollector();

		if(mParentCallback.nbTouches)
		{
			for(PxU32 i = 0; i < mParentCallback.nbTouches; i++)
				mAllHits.pushBack(mParentCallback.touches[i]);
		}

		if(mParentCallback.hasBlock)
			mAllHits.pushBack(mParentCallback.block);

		// PT: TODO: why do we need reinterpret_casts below?
		if(HitTypeSupport<HitType>::IsRaycast)
			collector.raycast			(mInput.getOrigin(), mInput.getDir(), mInput.maxDistance, reinterpret_cast<PxRaycastHit*>(mAllHits.begin()), mAllHits.size(), mFilterData, this->maxNbTouches!=0);
		else if(HitTypeSupport<HitType>::IsOverlap)
			collector.overlapMultiple	(*mInput.geometry, *mInput.pose, reinterpret_cast<PxOverlapHit*>(mAllHits.begin()), mAllHits.size(), mFilterData);
		else if(HitTypeSupport<HitType>::IsSweep)
			collector.sweep				(*mInput.geometry, *mInput.pose, mInput.getDir(), mInput.maxDistance, reinterpret_cast<PxSweepHit*>(mAllHits.begin()), mAllHits.size(), mFilterData, this->maxNbTouches!=0);
	}

private:
	CapturePvdOnReturn<HitType>& operator=(const CapturePvdOnReturn<HitType>&);
};
#endif // PX_SUPPORT_PVD

//========================================================================================================================
template<typename HitType>
struct IssueCallbacksOnReturn
{
	PxHitCallback<HitType>& hits;
	PxAgain again;	// query was stopped by previous processTouches. This means that nbTouches is still non-zero
					// but we don't need to issue processTouches again
	PX_FORCE_INLINE IssueCallbacksOnReturn(PxHitCallback<HitType>& aHits) : hits(aHits)
	{
		again = true;
	}

	~IssueCallbacksOnReturn()
	{
		if(again)
			// only issue processTouches if query wasn't stopped
			// this is because nbTouches doesn't get reset to 0 in this case (according to spec)
			// and the touches in touches array were already processed by the callback
		{
			if(hits.hasBlock && hits.nbTouches)
				hits.nbTouches = clipHitsToNewMaxDist<HitType>(hits.touches, hits.nbTouches, HITDIST(hits.block));
			if(hits.nbTouches)
			{
				bool again_ = hits.processTouches(hits.touches, hits.nbTouches);
				if(again_)
					hits.nbTouches = 0;
			}
		}
		hits.finalizeQuery();
	}

private:
	IssueCallbacksOnReturn<HitType>& operator=(const IssueCallbacksOnReturn<HitType>&);
};

#undef HITDIST

//========================================================================================================================
template<typename HitType>
bool NpSceneQueries::multiQuery(
	const MultiQueryInput& input, PxHitCallback<HitType>& hits, PxHitFlags hitFlags, const PxQueryCache* cache,
	const PxQueryFilterData& filterData, PxQueryFilterCallback* filterCall, BatchQueryFilterData* bfd) const
{
	const bool anyHit = (filterData.flags & PxQueryFlag::eANY_HIT) == PxQueryFlag::eANY_HIT;

	PxI32 retval = 0; PX_UNUSED(retval);

	if(HitTypeSupport<HitType>::IsRaycast == 0)
	{
		PX_CHECK_AND_RETURN_VAL(input.pose != NULL, "NpSceneQueries::overlap/sweep pose is NULL.", 0);
		PX_CHECK_AND_RETURN_VAL(input.pose->isValid(), "NpSceneQueries::overlap/sweep pose is not valid.", 0);
	}
	else
	{
		PX_CHECK_AND_RETURN_VAL(input.getOrigin().isFinite(), "NpSceneQueries::raycast pose is not valid.", 0);
	}

	if(HitTypeSupport<HitType>::IsOverlap == 0)
	{
		PX_CHECK_AND_RETURN_VAL(input.getDir().isFinite(), "NpSceneQueries multiQuery input check: unitDir is not valid.", 0);
		PX_CHECK_AND_RETURN_VAL(input.getDir().isNormalized(), "NpSceneQueries multiQuery input check: direction must be normalized", 0);
	}

	if(HitTypeSupport<HitType>::IsRaycast)
	{
		PX_CHECK_AND_RETURN_VAL(input.maxDistance > 0.0f, "NpSceneQueries::multiQuery input check: distance cannot be negative or zero", 0);
	}

	if(HitTypeSupport<HitType>::IsOverlap && !anyHit)
	{
		PX_CHECK_AND_RETURN_VAL(hits.maxNbTouches > 0, "PxScene::overlap() and PxBatchQuery::overlap() calls without eANY_HIT flag require a touch hit buffer for return results.", 0);
	}

	if(HitTypeSupport<HitType>::IsSweep)
	{
		PX_CHECK_AND_RETURN_VAL(input.maxDistance >= 0.0f, "NpSceneQueries multiQuery input check: distance cannot be negative", 0);
		PX_CHECK_AND_RETURN_VAL(input.maxDistance != 0.0f || !(hitFlags & PxHitFlag::eASSUME_NO_INITIAL_OVERLAP),
			"NpSceneQueries multiQuery input check: zero-length sweep only valid without the PxHitFlag::eASSUME_NO_INITIAL_OVERLAP flag", 0);
	}

	PX_CHECK_MSG(!cache || (cache && cache->shape && cache->actor), "Raycast cache specified but shape or actor pointer is NULL!");
	PxU32 cachedCompoundId = INVALID_PRUNERHANDLE;
	const PrunerData cacheData = cache ? NpActor::getShapeManager(*cache->actor)->findSceneQueryData(*static_cast<NpShape*>(cache->shape), cachedCompoundId) : SQ_INVALID_PRUNER_DATA;

	// this function is logically const for the SDK user, as flushUpdates() will not have an API-visible effect on this object
	// internally however, flushUpdates() changes the states of the Pruners in mSQManager
	// because here is the only place we need this, const_cast instead of making SQM mutable
	const_cast<NpSceneQueries*>(this)->mSQManager.flushUpdates();

#if PX_SUPPORT_PVD
	CapturePvdOnReturn<HitType> pvdCapture(this, input, hitFlags, cache, filterData, filterCall, bfd, hits);
#endif

	IssueCallbacksOnReturn<HitType> cbr(hits); // destructor will execute callbacks on return from this function
	hits.hasBlock = false;
	hits.nbTouches = 0;

	PxReal shrunkDistance = HitTypeSupport<HitType>::IsOverlap ? PX_MAX_REAL : input.maxDistance; // can be progressively shrunk as we go over the list of shapes
	if(HitTypeSupport<HitType>::IsSweep)
		shrunkDistance = PxMin(shrunkDistance, PX_MAX_SWEEP_DISTANCE);
	MultiQueryCallback<HitType> pcb(*this, input, anyHit, hits, hitFlags, filterData, filterCall, shrunkDistance, bfd);

	if(cacheData!=SQ_INVALID_PRUNER_DATA && hits.maxNbTouches == 0) // don't use cache for queries that can return touch hits
	{
		// this block is only executed for single shape cache
		const PrunerPayload& cachedPayload = mSQManager.getPayload(cachedCompoundId, cacheData);
		pcb.mIsCached = true;
		PxReal dummyDist;
	
		PxAgain againAfterCache;
		if(HitTypeSupport<HitType>::IsSweep)
		{
			// AP: for sweeps we cache the bounds because we need to know them for the test to clip the sweep to bounds
			// otherwise GJK becomes unstable. The bounds can be used multiple times so this is an optimization.
			const ShapeData sd(*input.geometry, *input.pose, input.inflation);
			pcb.mQueryShapeBounds = sd.getPrunerInflatedWorldAABB();
			pcb.mQueryShapeBoundsValid = true;
			pcb.mShapeData = &sd;
			againAfterCache = pcb.invoke(dummyDist, cachedPayload);
			pcb.mShapeData = NULL;
		} else
			againAfterCache = pcb.invoke(dummyDist, cachedPayload);
		pcb.mIsCached = false;
		if(!againAfterCache) // if PxAgain result for cached shape was false (abort query), return here
			return hits.hasAnyHits();
	}

	const Pruner* staticPruner = mSQManager.get(PruningIndex::eSTATIC).pruner();
	const Pruner* dynamicPruner = mSQManager.get(PruningIndex::eDYNAMIC).pruner();
	const CompoundPruner* compoundPruner = mSQManager.getCompoundPruner().pruner();

	const PxU32 doStatics = filterData.flags & PxQueryFlag::eSTATIC;
	const PxU32 doDynamics = filterData.flags & PxQueryFlag::eDYNAMIC;

	if(HitTypeSupport<HitType>::IsRaycast)
	{
		bool again = doStatics ? staticPruner->raycast(input.getOrigin(), input.getDir(), pcb.mShrunkDistance, pcb) : true;
		if(!again)
			return hits.hasAnyHits();
		
		if(doDynamics)
			again = dynamicPruner->raycast(input.getOrigin(), input.getDir(), pcb.mShrunkDistance, pcb);

		if(again)
			again = compoundPruner->raycast(input.getOrigin(), input.getDir(), pcb.mShrunkDistance, pcb, filterData.flags);

		cbr.again = again; // update the status to avoid duplicate processTouches()
		return hits.hasAnyHits();
	}
	else if(HitTypeSupport<HitType>::IsOverlap)
	{
		PX_ASSERT(input.geometry);

		const ShapeData sd(*input.geometry, *input.pose, input.inflation);
		pcb.mShapeData = &sd;
		PxAgain again = doStatics ? staticPruner->overlap(sd, pcb) : true;
		if(!again) // && (filterData.flags & PxQueryFlag::eANY_HIT))
			return hits.hasAnyHits();
		
		if(doDynamics)
			again = dynamicPruner->overlap(sd, pcb);

		if(again)
			again = compoundPruner->overlap(sd, pcb, filterData.flags);

		cbr.again = again; // update the status to avoid duplicate processTouches()
		return hits.hasAnyHits();
	}
	else
	{
		PX_ASSERT(HitTypeSupport<HitType>::IsSweep);
		PX_ASSERT(input.geometry);

		const ShapeData sd(*input.geometry, *input.pose, input.inflation);
		pcb.mQueryShapeBounds = sd.getPrunerInflatedWorldAABB();
		pcb.mQueryShapeBoundsValid = true;
		pcb.mShapeData = &sd;
		PxAgain again = doStatics ? staticPruner->sweep(sd, input.getDir(), pcb.mShrunkDistance, pcb) : true;
		if(!again)
			return hits.hasAnyHits();
		
		if(doDynamics)
			again = dynamicPruner->sweep(sd, input.getDir(), pcb.mShrunkDistance, pcb);

		if(again)
			again = compoundPruner->sweep(sd, input.getDir(), pcb.mShrunkDistance, pcb, filterData.flags);
		
		cbr.again = again; // update the status to avoid duplicate processTouches()
		return hits.hasAnyHits();
	}
}

void NpSceneQueries::sceneQueriesStaticPrunerUpdate(PxBaseTask* )
{
	PX_PROFILE_ZONE("SceneQuery.sceneQueriesStaticPrunerUpdate", getContextId());
	// run pruner build only, this will build the new tree only, no commit happens
	mSQManager.sceneQueryBuildStep(PruningIndex::eSTATIC);
}

void NpSceneQueries::sceneQueriesDynamicPrunerUpdate(PxBaseTask*)
{
	PX_PROFILE_ZONE("SceneQuery.sceneQueriesDynamicPrunerUpdate", getContextId());
	// run pruner build only, this will build the new tree only, no commit happens
	mSQManager.sceneQueryBuildStep(PruningIndex::eDYNAMIC);
}

//explicit template instantiation
template bool NpSceneQueries::multiQuery<PxRaycastHit>(const MultiQueryInput&, PxHitCallback<PxRaycastHit>&, PxHitFlags, const PxQueryCache*, const PxQueryFilterData&, PxQueryFilterCallback*, BatchQueryFilterData*) const; 
template bool NpSceneQueries::multiQuery<PxOverlapHit>(const MultiQueryInput&, PxHitCallback<PxOverlapHit>&, PxHitFlags, const PxQueryCache*, const PxQueryFilterData&, PxQueryFilterCallback*, BatchQueryFilterData*) const;
template bool NpSceneQueries::multiQuery<PxSweepHit>(const MultiQueryInput&, PxHitCallback<PxSweepHit>&, PxHitFlags, const PxQueryCache*, const PxQueryFilterData&, PxQueryFilterCallback*, BatchQueryFilterData*) const;

