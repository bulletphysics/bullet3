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


#ifndef PX_PHYSICS_NX_SCENEQUERY
#define PX_PHYSICS_NX_SCENEQUERY
/** \addtogroup scenequery 
@{ */

#include "PxPhysXConfig.h"
#include "PxShape.h"
#include "PxBatchQueryDesc.h"
#include "PxQueryFiltering.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxBoxGeometry;
class PxSphereGeometry;
struct PxQueryCache;

/**
\brief Batched queries object. This is used to perform several queries at the same time. 

\deprecated The batched query feature has been deprecated in PhysX version 3.4

@see PxScene, PxScene.createBatchQuery
*/
class PX_DEPRECATED PxBatchQuery
{
	public:

	/**
	\brief Executes batched queries.
	*/
	virtual	void							execute() = 0;

	/**
	\brief Gets the prefilter shader in use for this scene query.

	\return Prefilter shader.

	@see PxBatchQueryDesc.preFilterShade PxBatchQueryPreFilterShader
	*/
	virtual	PxBatchQueryPreFilterShader getPreFilterShader() const = 0;

	/**
	\brief Gets the postfilter shader in use for this scene query.

	\return Postfilter shader.

	@see PxBatchQueryDesc.preFilterShade PxBatchQueryPostFilterShader
	*/
	virtual	PxBatchQueryPostFilterShader getPostFilterShader() const = 0;


	/**
	\brief Gets the shared global filter data in use for this scene query.

	\return Shared filter data for filter shader.

	@see getFilterShaderDataSize() PxBatchQueryDesc.filterShaderData PxBatchQueryPreFilterShader, PxBatchQueryPostFilterShader
	*/
	virtual	const void*						getFilterShaderData() const	= 0;

	/**
	\brief Gets the size of the shared global filter data (#PxSceneDesc.filterShaderData)

	\return Size of shared filter data [bytes].

	@see getFilterShaderData() PxBatchQueryDesc.filterShaderDataSize PxBatchQueryPreFilterShader, PxBatchQueryPostFilterShader
	*/
	virtual	PxU32							getFilterShaderDataSize() const	= 0;

	/**
 	\brief Sets new user memory pointers.
 
 	It is not possible to change the memory during query execute.
 
 	@see PxBatchQueryDesc
 	*/
 	virtual	void							setUserMemory(const PxBatchQueryMemory&) = 0;

	/**
 	\brief Gets the user memory pointers. 	
 
 	@see PxBatchQueryDesc
 	*/
 	virtual	const PxBatchQueryMemory&		getUserMemory() = 0;

	/**
	\brief Releases PxBatchQuery from PxScene

	@see PxScene, PxScene.createBatchQuery
	*/
	virtual	void							release() = 0;

	/**
	\brief Performs a raycast against objects in the scene, returns results in PxBatchQueryMemory::userRaycastResultBuffer
	specified at PxBatchQuery creation time or via PxBatchQuery::setUserMemory call.

	\note	Touching hits are not ordered.
	\note	Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in article SceneQuery. User can ignore such objects by using one of the provided filter mechanisms.

	\param[in] origin		Origin of the ray.
	\param[in] unitDir		Normalized direction of the ray.
	\param[in] distance		Length of the ray. Needs to be larger than 0.
	\param[in] maxTouchHits	Maximum number of hits to record in the touch buffer for this query. Default=0 reports a single blocking hit. If maxTouchHits is set to 0 all hits are treated as blocking by default.
	\param[in] hitFlags		Specifies which properties per hit should be computed and returned in hit array and blocking hit.
	\param[in] filterData	Filtering data passed to the filer shader. See #PxQueryFilterData #PxBatchQueryPreFilterShader, #PxBatchQueryPostFilterShader
	\param[in] userData		User can pass any value in this argument, usually to identify this particular query
	\param[in] cache		Cached hit shape (optional). Query is tested against cached shape first. If no hit is found the ray gets queried against the scene.
							Note: Filtering is not executed for a cached shape if supplied; instead, if a hit is found, it is assumed to be a blocking hit.
							Note: Using past touching hits as cache will produce incorrect behavior since the cached hit will always be treated as blocking.
	
	\note This query call writes to a list associated with the query object and is NOT thread safe (for performance reasons there is no lock
		and overlapping writes from different threads may result in undefined behavior).

	@see PxQueryFilterData PxBatchQueryPreFilterShader PxBatchQueryPostFilterShader PxRaycastHit PxScene::raycast
	*/
	virtual void raycast(
		const PxVec3& origin, const PxVec3& unitDir, PxReal distance = PX_MAX_F32, PxU16 maxTouchHits = 0,
		PxHitFlags hitFlags = PxHitFlag::eDEFAULT,
		const PxQueryFilterData& filterData = PxQueryFilterData(),
		void* userData = NULL, const PxQueryCache* cache = NULL) = 0;


	/**
	\brief Performs an overlap test of a given geometry against objects in the scene, returns results in PxBatchQueryMemory::userOverlapResultBuffer
	specified at PxBatchQuery creation time or via PxBatchQuery::setUserMemory call.
	
	\note Filtering: returning eBLOCK from user filter for overlap queries will cause a warning (see #PxQueryHitType).

	\param[in] geometry		Geometry of object to check for overlap (supported types are: box, sphere, capsule, convex).
	\param[in] pose			Pose of the object.
	\param[in] maxTouchHits	Maximum number of hits to record in the touch buffer for this query. Default=0 reports a single blocking hit. If maxTouchHits is set to 0 all hits are treated as blocking by default.
	\param[in] filterData	Filtering data and simple logic. See #PxQueryFilterData #PxBatchQueryPreFilterShader, #PxBatchQueryPostFilterShader
	\param[in] userData		User can pass any value in this argument, usually to identify this particular query
	\param[in] cache		Cached hit shape (optional). Query is tested against cached shape first. If no hit is found the ray gets queried against the scene.
							Note: Filtering is not executed for a cached shape if supplied; instead, if a hit is found, it is assumed to be a blocking hit.
							Note: Using past touching hits as cache will produce incorrect behavior since the cached hit will always be treated as blocking.

	\note eBLOCK should not be returned from user filters for overlap(). Doing so will result in undefined behavior, and a warning will be issued.
	\note If the PxQueryFlag::eNO_BLOCK flag is set, the eBLOCK will instead be automatically converted to an eTOUCH and the warning suppressed.
	\note This query call writes to a list associated with the query object and is NOT thread safe (for performance reasons there is no lock
		and overlapping writes from different threads may result in undefined behavior).

	@see PxQueryFilterData PxBatchQueryPreFilterShader PxBatchQueryPostFilterShader 
	*/
	virtual void overlap(
		const PxGeometry& geometry, const PxTransform& pose, PxU16 maxTouchHits = 0,
		const PxQueryFilterData& filterData = PxQueryFilterData(), void* userData=NULL, const PxQueryCache* cache = NULL) = 0;

	/**
	\brief Performs a sweep test against objects in the scene, returns results in PxBatchQueryMemory::userSweepResultBuffer
	specified at PxBatchQuery creation time or via PxBatchQuery::setUserMemory call.
	
	\note	Touching hits are not ordered.
	\note	If a shape from the scene is already overlapping with the query shape in its starting position,
			the hit is returned unless eASSUME_NO_INITIAL_OVERLAP was specified.

	\param[in] geometry		Geometry of object to sweep (supported types are: box, sphere, capsule, convex).
	\param[in] pose			Pose of the sweep object.
	\param[in] unitDir		Normalized direction of the sweep.
	\param[in] distance		Sweep distance. Needs to be larger than 0. Will be clamped to PX_MAX_SWEEP_DISTANCE.
	\param[in] maxTouchHits	Maximum number of hits to record in the touch buffer for this query. Default=0 reports a single blocking hit. If maxTouchHits is set to 0 all hits are treated as blocking by default.
	\param[in] hitFlags		Specifies which properties per hit should be computed and returned in hit array and blocking hit.
	\param[in] filterData	Filtering data and simple logic. See #PxQueryFilterData #PxBatchQueryPreFilterShader, #PxBatchQueryPostFilterShader
	\param[in] userData		User can pass any value in this argument, usually to identify this particular query
	\param[in] cache		Cached hit shape (optional). Query is tested against cached shape first. If no hit is found the ray gets queried against the scene.
							Note: Filtering is not executed for a cached shape if supplied; instead, if a hit is found, it is assumed to be a blocking hit.
							Note: Using past touching hits as cache will produce incorrect behavior since the cached hit will always be treated as blocking.
	\param[in] inflation	This parameter creates a skin around the swept geometry which increases its extents for sweeping. The sweep will register a hit as soon as the skin touches a shape, and will return the corresponding distance and normal.
							Note: ePRECISE_SWEEP doesn't support inflation. Therefore the sweep will be performed with zero inflation.

	\note This query call writes to a list associated with the query object and is NOT thread safe (for performance reasons there is no lock
		and overlapping writes from different threads may result in undefined behavior).

	@see PxHitFlags PxQueryFilterData PxBatchQueryPreFilterShader PxBatchQueryPostFilterShader PxSweepHit
	*/
	virtual void sweep(
		const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, const PxReal distance,
		PxU16 maxTouchHits = 0, PxHitFlags hitFlags = PxHitFlag::eDEFAULT,
		const PxQueryFilterData& filterData = PxQueryFilterData(), void* userData=NULL, const PxQueryCache* cache = NULL,
		const PxReal inflation = 0.f) = 0;

protected:
	virtual	~PxBatchQuery() {}
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
