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


#ifndef PX_PHYSICS_EXTENSIONS_SCENE_QUERY_H
#define PX_PHYSICS_EXTENSIONS_SCENE_QUERY_H
/** \addtogroup extensions
  @{
*/

#include "PxPhysXConfig.h"

#include "PxScene.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

// These types have been deprecated (removed) in PhysX 3.4. We typedef them to the new types here for easy migration from 3.3 to 3.4.
typedef PxQueryHit				PxSceneQueryHit;
typedef PxQueryFilterData		PxSceneQueryFilterData;
typedef PxQueryFilterCallback	PxSceneQueryFilterCallback;
typedef PxQueryCache			PxSceneQueryCache;
typedef PxHitFlag				PxSceneQueryFlag;
typedef PxHitFlags				PxSceneQueryFlags;

/**
\brief utility functions for use with PxScene, related to scene queries.

Some of these functions have been deprecated (removed) in PhysX 3.4. We re-implement them here for easy migration from 3.3 to 3.4.

@see PxShape
*/

class PxSceneQueryExt
{
public:

	/**
	\brief Raycast returning any blocking hit, not necessarily the closest.
	
	Returns whether any rigid actor is hit along the ray.

	\note Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in article SceneQuery. User can ignore such objects by using one of the provided filter mechanisms.

	\param[in] scene		The scene
	\param[in] origin		Origin of the ray.
	\param[in] unitDir		Normalized direction of the ray.
	\param[in] distance		Length of the ray. Needs to be larger than 0.
	\param[out] hit			Raycast hit information.
	\param[in] filterData	Filtering data and simple logic.
	\param[in] filterCall	Custom filtering logic (optional). Only used if the corresponding #PxHitFlag flags are set. If NULL, all hits are assumed to be blocking.
	\param[in] cache		Cached hit shape (optional). Ray is tested against cached shape first. If no hit is found the ray gets queried against the scene.
							Note: Filtering is not executed for a cached shape if supplied; instead, if a hit is found, it is assumed to be a blocking hit.
							Note: Using past touching hits as cache will produce incorrect behavior since the cached hit will always be treated as blocking.
	\return True if a blocking hit was found.

	@see PxSceneQueryFilterData PxSceneQueryFilterCallback PxSceneQueryCache PxSceneQueryHit
	*/
	static bool raycastAny(	const PxScene& scene,
							const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,
							PxSceneQueryHit& hit, const PxSceneQueryFilterData& filterData = PxSceneQueryFilterData(),
							PxSceneQueryFilterCallback* filterCall = NULL, const PxSceneQueryCache* cache = NULL);

	/**
	\brief Raycast returning a single result.
	
	Returns the first rigid actor that is hit along the ray. Data for a blocking hit will be returned as specified by the outputFlags field. Touching hits will be ignored.

	\note Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in article SceneQuery. User can ignore such objects by using one of the provided filter mechanisms.

	\param[in] scene		The scene
	\param[in] origin		Origin of the ray.
	\param[in] unitDir		Normalized direction of the ray.
	\param[in] distance		Length of the ray. Needs to be larger than 0.
	\param[in] outputFlags	Specifies which properties should be written to the hit information
	\param[out] hit			Raycast hit information.
	\param[in] filterData	Filtering data and simple logic.
	\param[in] filterCall	Custom filtering logic (optional). Only used if the corresponding #PxHitFlag flags are set. If NULL, all hits are assumed to be blocking.
	\param[in] cache		Cached hit shape (optional). Ray is tested against cached shape first then against the scene.
							Note: Filtering is not executed for a cached shape if supplied; instead, if a hit is found, it is assumed to be a blocking hit.
							Note: Using past touching hits as cache will produce incorrect behavior since the cached hit will always be treated as blocking.
	\return True if a blocking hit was found.

	@see PxSceneQueryFlags PxRaycastHit PxSceneQueryFilterData PxSceneQueryFilterCallback PxSceneQueryCache
	*/
	static bool raycastSingle(	const PxScene& scene,
								const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,
								PxSceneQueryFlags outputFlags, PxRaycastHit& hit,
								const PxSceneQueryFilterData& filterData = PxSceneQueryFilterData(),
								PxSceneQueryFilterCallback* filterCall = NULL, const PxSceneQueryCache* cache = NULL);

	/**
	\brief Raycast returning multiple results.
	
	Find all rigid actors that get hit along the ray. Each result contains data as specified by the outputFlags field.

	\note Touching hits are not ordered.

	\note Shooting a ray from within an object leads to different results depending on the shape type. Please check the details in article SceneQuery. User can ignore such objects by using one of the provided filter mechanisms.

	\param[in] scene			The scene
	\param[in] origin			Origin of the ray.
	\param[in] unitDir			Normalized direction of the ray.
	\param[in] distance			Length of the ray. Needs to be larger than 0.
	\param[in] outputFlags		Specifies which properties should be written to the hit information
	\param[out] hitBuffer		Raycast hit information buffer. If the buffer overflows, the blocking hit is returned as the last entry together with an arbitrary subset
								of the nearer touching hits (typically the query should be restarted with a larger buffer).
	\param[in] hitBufferSize	Size of the hit buffer.
	\param[out] blockingHit		True if a blocking hit was found. If found, it is the last in the buffer, preceded by any touching hits which are closer. Otherwise the touching hits are listed.
	\param[in] filterData		Filtering data and simple logic.
	\param[in] filterCall		Custom filtering logic (optional). Only used if the corresponding #PxHitFlag flags are set. If NULL, all hits are assumed to be touching.
	\param[in] cache			Cached hit shape (optional). Ray is tested against cached shape first then against the scene.
								Note: Filtering is not executed for a cached shape if supplied; instead, if a hit is found, it is assumed to be a blocking hit.
								Note: Using past touching hits as cache will produce incorrect behavior since the cached hit will always be treated as blocking.
	\return Number of hits in the buffer, or -1 if the buffer overflowed.

	@see PxSceneQueryFlags PxRaycastHit PxSceneQueryFilterData PxSceneQueryFilterCallback PxSceneQueryCache
	*/
	static PxI32 raycastMultiple(	const PxScene& scene,
									const PxVec3& origin, const PxVec3& unitDir, const PxReal distance,
									PxSceneQueryFlags outputFlags,
									PxRaycastHit* hitBuffer, PxU32 hitBufferSize, bool& blockingHit,
									const PxSceneQueryFilterData& filterData = PxSceneQueryFilterData(),
									PxSceneQueryFilterCallback* filterCall = NULL, const PxSceneQueryCache* cache = NULL);

	/**
	\brief Sweep returning any blocking hit, not necessarily the closest.
	
	Returns whether any rigid actor is hit along the sweep path.

	\note If a shape from the scene is already overlapping with the query shape in its starting position, behavior is controlled by the PxSceneQueryFlag::eINITIAL_OVERLAP flag.

	\param[in] scene		The scene
	\param[in] geometry		Geometry of object to sweep (supported types are: box, sphere, capsule, convex).
	\param[in] pose			Pose of the sweep object.
	\param[in] unitDir		Normalized direction of the sweep.
	\param[in] distance		Sweep distance. Needs to be larger than 0. Will be clamped to PX_MAX_SWEEP_DISTANCE.
	\param[in] queryFlags	Combination of PxSceneQueryFlag defining the query behavior
	\param[out] hit			Sweep hit information.
	\param[in] filterData	Filtering data and simple logic.
	\param[in] filterCall	Custom filtering logic (optional). Only used if the corresponding #PxHitFlag flags are set. If NULL, all hits are assumed to be blocking.
	\param[in] cache		Cached hit shape (optional). Sweep is performed against cached shape first. If no hit is found the sweep gets queried against the scene.
							Note: Filtering is not executed for a cached shape if supplied; instead, if a hit is found, it is assumed to be a blocking hit.
							Note: Using past touching hits as cache will produce incorrect behavior since the cached hit will always be treated as blocking.
	\param[in] inflation	This parameter creates a skin around the swept geometry which increases its extents for sweeping. The sweep will register a hit as soon as the skin touches a shape, and will return the corresponding distance and normal.
	\return True if a blocking hit was found.

	@see PxSceneQueryFilterData PxSceneQueryFilterCallback PxSceneQueryHit PxSceneQueryCache
	*/
	static bool sweepAny(	const PxScene& scene,
							const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, const PxReal distance,
							PxSceneQueryFlags queryFlags,
							PxSceneQueryHit& hit,
							const PxSceneQueryFilterData& filterData = PxSceneQueryFilterData(),
							PxSceneQueryFilterCallback* filterCall = NULL,
							const PxSceneQueryCache* cache = NULL,
							PxReal inflation = 0.0f);

	/**
	\brief Sweep returning a single result.
	
	Returns the first rigid actor that is hit along the ray. Data for a blocking hit will be returned as specified by the outputFlags field. Touching hits will be ignored.

	\note If a shape from the scene is already overlapping with the query shape in its starting position, behavior is controlled by the PxSceneQueryFlag::eINITIAL_OVERLAP flag.

	\param[in] scene		The scene
	\param[in] geometry		Geometry of object to sweep (supported types are: box, sphere, capsule, convex).
	\param[in] pose			Pose of the sweep object.
	\param[in] unitDir		Normalized direction of the sweep.
	\param[in] distance		Sweep distance. Needs to be larger than 0. Will be clamped to PX_MAX_SWEEP_DISTANCE.
	\param[in] outputFlags	Specifies which properties should be written to the hit information.
	\param[out] hit			Sweep hit information.
	\param[in] filterData	Filtering data and simple logic.
	\param[in] filterCall	Custom filtering logic (optional). Only used if the corresponding #PxHitFlag flags are set. If NULL, all hits are assumed to be blocking.
	\param[in] cache		Cached hit shape (optional). Sweep is performed against cached shape first then against the scene.
							Note: Filtering is not executed for a cached shape if supplied; instead, if a hit is found, it is assumed to be a blocking hit.
							Note: Using past touching hits as cache will produce incorrect behavior since the cached hit will always be treated as blocking.
	\param[in] inflation	This parameter creates a skin around the swept geometry which increases its extents for sweeping. The sweep will register a hit as soon as the skin touches a shape, and will return the corresponding distance and normal.
	\return True if a blocking hit was found.

	@see PxSceneQueryFlags PxSweepHit PxSceneQueryFilterData PxSceneQueryFilterCallback PxSceneQueryCache
	*/
	static bool sweepSingle(const PxScene& scene,
							const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, const PxReal distance,
							PxSceneQueryFlags outputFlags,
							PxSweepHit& hit,
							const PxSceneQueryFilterData& filterData = PxSceneQueryFilterData(),
							PxSceneQueryFilterCallback* filterCall = NULL,
							const PxSceneQueryCache* cache = NULL,
							PxReal inflation=0.0f);

	/**
	\brief Sweep returning multiple results.
	
	Find all rigid actors that get hit along the sweep. Each result contains data as specified by the outputFlags field.

	\note Touching hits are not ordered.

	\note If a shape from the scene is already overlapping with the query shape in its starting position, behavior is controlled by the PxSceneQueryFlag::eINITIAL_OVERLAP flag.

	\param[in] scene			The scene
	\param[in] geometry			Geometry of object to sweep (supported types are: box, sphere, capsule, convex).
	\param[in] pose				Pose of the sweep object.
	\param[in] unitDir			Normalized direction of the sweep.
	\param[in] distance			Sweep distance. Needs to be larger than 0. Will be clamped to PX_MAX_SWEEP_DISTANCE.
	\param[in] outputFlags		Specifies which properties should be written to the hit information.
	\param[out] hitBuffer		Sweep hit information buffer. If the buffer overflows, the blocking hit is returned as the last entry together with an arbitrary subset
								of the nearer touching hits (typically the query should be restarted with a larger buffer).
	\param[in] hitBufferSize	Size of the hit buffer.
	\param[out] blockingHit		True if a blocking hit was found. If found, it is the last in the buffer, preceded by any touching hits which are closer. Otherwise the touching hits are listed.
	\param[in] filterData		Filtering data and simple logic.
	\param[in] filterCall		Custom filtering logic (optional). Only used if the corresponding #PxHitFlag flags are set. If NULL, all hits are assumed to be touching.
	\param[in] cache			Cached hit shape (optional). Sweep is performed against cached shape first then against the scene.
								Note: Filtering is not executed for a cached shape if supplied; instead, if a hit is found, it is assumed to be a blocking hit.
								Note: Using past touching hits as cache will produce incorrect behavior since the cached hit will always be treated as blocking.
	\param[in] inflation		This parameter creates a skin around the swept geometry which increases its extents for sweeping. The sweep will register a hit as soon as the skin touches a shape, and will return the corresponding distance and normal.
	\return Number of hits in the buffer, or -1 if the buffer overflowed.

	@see PxSceneQueryFlags PxSweepHit PxSceneQueryFilterData PxSceneQueryFilterCallback PxSceneQueryCache
	*/
	static PxI32 sweepMultiple(	const PxScene& scene,
								const PxGeometry& geometry, const PxTransform& pose, const PxVec3& unitDir, const PxReal distance,
								PxSceneQueryFlags outputFlags, PxSweepHit* hitBuffer, PxU32 hitBufferSize, bool& blockingHit,
								const PxSceneQueryFilterData& filterData = PxSceneQueryFilterData(),
								PxSceneQueryFilterCallback* filterCall = NULL, const PxSceneQueryCache* cache = NULL,
								PxReal inflation = 0.0f);

	/**
	\brief Test overlap between a geometry and objects in the scene.
	
	\note Filtering: Overlap tests do not distinguish between touching and blocking hit types. Both get written to the hit buffer.

	\note PxHitFlag::eMESH_MULTIPLE and PxHitFlag::eMESH_BOTH_SIDES have no effect in this case

	\param[in] scene			The scene
	\param[in] geometry			Geometry of object to check for overlap (supported types are: box, sphere, capsule, convex).
	\param[in] pose				Pose of the object.
	\param[out] hitBuffer		Buffer to store the overlapping objects to. If the buffer overflows, an arbitrary subset of overlapping objects is stored (typically the query should be restarted with a larger buffer).
	\param[in] hitBufferSize	Size of the hit buffer. 
	\param[in] filterData		Filtering data and simple logic.
	\param[in] filterCall		Custom filtering logic (optional). Only used if the corresponding #PxHitFlag flags are set. If NULL, all hits are assumed to overlap.
	\return Number of hits in the buffer, or -1 if the buffer overflowed.

	@see PxSceneQueryFlags PxSceneQueryFilterData PxSceneQueryFilterCallback
	*/
	static PxI32 overlapMultiple(	const PxScene& scene,
									const PxGeometry& geometry, const PxTransform& pose,
									PxOverlapHit* hitBuffer, PxU32 hitBufferSize,
									const PxSceneQueryFilterData& filterData = PxSceneQueryFilterData(),
									PxSceneQueryFilterCallback* filterCall = NULL);

	/**
	\brief Test returning, for a given geometry, any overlapping object in the scene.
	
	\note Filtering: Overlap tests do not distinguish between touching and blocking hit types. Both trigger a hit.

	\note PxHitFlag::eMESH_MULTIPLE and PxHitFlag::eMESH_BOTH_SIDES have no effect in this case
	
	\param[in] scene			The scene
	\param[in] geometry			Geometry of object to check for overlap (supported types are: box, sphere, capsule, convex).
	\param[in] pose				Pose of the object.
	\param[out] hit				Pointer to store the overlapping object to.
	\param[in] filterData		Filtering data and simple logic.
	\param[in] filterCall		Custom filtering logic (optional). Only used if the corresponding #PxHitFlag flags are set. If NULL, all hits are assumed to overlap.
	\return True if an overlap was found.

	@see PxSceneQueryFlags PxSceneQueryFilterData PxSceneQueryFilterCallback
	*/
	static bool	overlapAny(	const PxScene& scene,
							const PxGeometry& geometry, const PxTransform& pose,
							PxOverlapHit& hit,
							const PxSceneQueryFilterData& filterData = PxSceneQueryFilterData(),
							PxSceneQueryFilterCallback* filterCall = NULL);
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
