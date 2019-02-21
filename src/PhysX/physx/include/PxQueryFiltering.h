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


#ifndef PX_PHYSICS_NX_SCENE_QUERY_FILTERING
#define PX_PHYSICS_NX_SCENE_QUERY_FILTERING
/** \addtogroup scenequery
@{
*/

#include "PxPhysXConfig.h"
#include "PxFiltering.h"
#include "PxQueryReport.h"
#include "PxClient.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxShape;
class PxRigidActor;
struct PxQueryHit;


/**
\brief Filtering flags for scene queries.

@see PxQueryFilterData.flags
*/
struct PxQueryFlag
{
	enum Enum
	{
		eSTATIC				= (1<<0),	//!< Traverse static shapes

		eDYNAMIC			= (1<<1),	//!< Traverse dynamic shapes

		ePREFILTER			= (1<<2),	//!< Run the pre-intersection-test filter (see #PxQueryFilterCallback::preFilter())

		ePOSTFILTER			= (1<<3),	//!< Run the post-intersection-test filter (see #PxQueryFilterCallback::postFilter())

		eANY_HIT			= (1<<4),	//!< Abort traversal as soon as any hit is found and return it via callback.block.
										//!< Helps query performance. Both eTOUCH and eBLOCK hitTypes are considered hits with this flag.

		eNO_BLOCK			= (1<<5),	//!< All hits are reported as touching. Overrides eBLOCK returned from user filters with eTOUCH.
										//!< This is also an optimization hint that may improve query performance.

		eRESERVED			= (1<<15)	//!< Reserved for internal use
	};
};
PX_COMPILE_TIME_ASSERT(PxQueryFlag::eSTATIC==(1<<0));
PX_COMPILE_TIME_ASSERT(PxQueryFlag::eDYNAMIC==(1<<1));

/**
\brief Flags typedef for the set of bits defined in PxQueryFlag.

*/
typedef PxFlags<PxQueryFlag::Enum,PxU16> PxQueryFlags;
PX_FLAGS_OPERATORS(PxQueryFlag::Enum,PxU16)

/**
\brief Classification of scene query hits (intersections).

 - eNONE: Returning this hit type means that the hit should not be reported.
 - eBLOCK: For all raycast, sweep and overlap queries the nearest eBLOCK type hit will always be returned in PxHitCallback::block member.
 - eTOUCH: Whenever a raycast, sweep or overlap query was called with non-zero PxHitCallback::nbTouches and PxHitCallback::touches
		   parameters, eTOUCH type hits that are closer or same distance (touchDistance <= blockDistance condition)
		   as the globally nearest eBLOCK type hit, will be reported.
 - For example, to record all hits from a raycast query, always return eTOUCH.

All hits in overlap() queries are treated as if the intersection distance were zero.
This means the hits are unsorted and all eTOUCH hits are recorded by the callback even if an eBLOCK overlap hit was encountered.
Even though all overlap() blocking hits have zero length, only one (arbitrary) eBLOCK overlap hit is recorded in PxHitCallback::block.
All overlap() eTOUCH type hits are reported (zero touchDistance <= zero blockDistance condition).

For raycast/sweep/overlap calls with zero touch buffer or PxHitCallback::nbTouches member,
only the closest hit of type eBLOCK is returned. All eTOUCH hits are discarded.

@see PxQueryFilterCallback.preFilter PxQueryFilterCallback.postFilter PxScene.raycast PxScene.sweep PxScene.overlap
*/
struct PxQueryHitType
{
	enum Enum
	{
		eNONE	= 0,	//!< the query should ignore this shape
		eTOUCH	= 1,	//!< a hit on the shape touches the intersection geometry of the query but does not block it
		eBLOCK	= 2		//!< a hit on the shape blocks the query (does not block overlap queries)
	};
};

/**
\brief Scene query filtering data.

Whenever the scene query intersects a shape, filtering is performed in the following order:

\li For non-batched queries only:<br>If the data field is non-zero, and the bitwise-AND value of data AND the shape's
queryFilterData is zero, the shape is skipped
\li If filter callbacks are enabled in flags field (see #PxQueryFlags) they will get invoked accordingly.
\li If neither #PxQueryFlag::ePREFILTER or #PxQueryFlag::ePOSTFILTER is set, the hit defaults
to type #PxQueryHitType::eBLOCK when the value of PxHitCallback::nbTouches provided with the query is zero and to type
#PxQueryHitType::eTOUCH when PxHitCallback::nbTouches is positive.

@see PxScene.raycast PxScene.sweep PxScene.overlap PxBatchQuery.raycast PxBatchQuery.sweep PxBatchQuery.overlap PxQueryFlag::eANY_HIT
*/
struct PxQueryFilterData
{
	/** \brief default constructor */
	explicit PX_INLINE PxQueryFilterData() : flags(PxQueryFlag::eDYNAMIC | PxQueryFlag::eSTATIC)		{}

	/** \brief constructor to set both filter data and filter flags */
	explicit PX_INLINE PxQueryFilterData(const PxFilterData& fd, PxQueryFlags f) : data(fd), flags(f)	{}

	/** \brief constructor to set filter flags only */
	explicit PX_INLINE PxQueryFilterData(PxQueryFlags f) : flags(f)										{}

	PxFilterData	data;		//!< Filter data associated with the scene query
	PxQueryFlags	flags;		//!< Filter flags (see #PxQueryFlags)
};

/**
\brief Scene query filtering callbacks.

Custom filtering logic for scene query intersection candidates. If an intersection candidate object passes the data based filter
(see #PxQueryFilterData), filtering callbacks are executed if requested (see #PxQueryFilterData.flags)

\li If #PxQueryFlag::ePREFILTER is set, the preFilter function runs before exact intersection tests.
If this function returns #PxQueryHitType::eTOUCH or #PxQueryHitType::eBLOCK, exact testing is performed to
determine the intersection location.

The preFilter function may overwrite the copy of queryFlags it receives as an argument to specify any of #PxHitFlag::eMODIFIABLE_FLAGS
on a per-shape basis. Changes apply only to the shape being filtered, and changes to other flags are ignored.

\li If #PxQueryFlag::ePREFILTER is not set, precise intersection testing is performed using the original query's filterData.flags.

\li If #PxQueryFlag::ePOSTFILTER is set, the postFilter function is called for each intersection to determine the touch/block status.
This overrides any touch/block status previously returned from the preFilter function for this shape.

Filtering calls are not guaranteed to be sorted along the ray or sweep direction.

@see PxScene.raycast PxScene.sweep PxScene.overlap PxQueryFlags PxHitFlags
*/
class PxQueryFilterCallback
{
public:

	/**
	\brief This filter callback is executed before the exact intersection test if PxQueryFlag::ePREFILTER flag was set.

	\param[in] filterData custom filter data specified as the query's filterData.data parameter.
	\param[in] shape A shape that has not yet passed the exact intersection test.
	\param[in] actor The shape's actor.
	\param[in,out] queryFlags scene query flags from the query's function call (only flags from PxHitFlag::eMODIFIABLE_FLAGS bitmask can be modified)
	\return the updated type for this hit  (see #PxQueryHitType)
	*/
	virtual PxQueryHitType::Enum preFilter(
		const PxFilterData& filterData, const PxShape* shape, const PxRigidActor* actor, PxHitFlags& queryFlags) = 0;

	/**
	\brief This filter callback is executed if the exact intersection test returned true and PxQueryFlag::ePOSTFILTER flag was set.

	\param[in] filterData custom filter data of the query
	\param[in] hit Scene query hit information. faceIndex member is not valid for overlap queries. For sweep and raycast queries the hit information can be cast to #PxSweepHit and #PxRaycastHit respectively.
	\return the updated hit type for this hit  (see #PxQueryHitType)
	*/
	virtual PxQueryHitType::Enum postFilter(const PxFilterData& filterData, const PxQueryHit& hit) = 0;

	/**
	\brief virtual destructor
	*/
	virtual ~PxQueryFilterCallback() {}
};

/**
\brief Batched query pre-filter shader.

Custom filtering logic for batched query intersection candidates. If an intersection candidate object passes the data based filter (see #PxQueryFilterData),
filtering shader runs if specified in filtering flags (see #PxQueryFilterData.flags)

\li If #PxQueryFlag::ePREFILTER is set, the preFilter shader runs before exact intersection tests.
If the shader returns #PxQueryHitType::eTOUCH or #PxQueryHitType::eBLOCK, exact testing is performed to
determine the intersection location.

The preFilter shader may overwrite the copy of queryFlags it receives as an argument to specify any of #PxHitFlag::eMODIFIABLE_FLAGS
on a per-shape basis. Changes apply only to the shape being filtered, and changes to other flags are ignored.

\li If #PxQueryFlag::ePREFILTER is not set, precise intersection testing is performed using the original query's filterData.flags.

Filtering calls are not guaranteed to be sorted along the ray or sweep direction.

\deprecated The batched query feature has been deprecated in PhysX version 3.4

@see PxBatchQueryDesc.preFilterShader PxQueryFilterCallback.preFilter PxBatchQueryPostFilterShader

*/

/**
\param[in] queryFilterData Query filter data
\param[in] objectFilterData Object filter data
\param[in] constantBlock Global constant filter data (see #PxBatchQuery)
\param[in] constantBlockSize Size of global filter data (see #PxBatchQuery)
\param[in,out] hitFlags Per-object modifiable hit flags (only flags from PxHitFlag::eMODIFIABLE_FLAGS mask can be modified)
\return the updated hit type for this hit (see #PxQueryHitType)

@see PxBatchQueryPostFilterShader
*/
typedef PX_DEPRECATED PxQueryHitType::Enum (*PxBatchQueryPreFilterShader)(
	PxFilterData queryFilterData, PxFilterData objectFilterData,
	const void* constantBlock, PxU32 constantBlockSize,
	PxHitFlags& hitFlags);

/**
\brief Batched query post-filter shader.

Custom filtering logic for batched query intersection candidates. If an intersection candidate object passes the data based filter (see #PxQueryFilterData),
the filtering shader run on request (see #PxQueryFilterData.flags)

\li If #PxQueryFlag::ePOSTFILTER is set, the postFilter shader is called for each intersection to determine the touch/block status.
This overrides any touch/block status previously returned from the preFilter function for this shape.

Filtering shaders are not in order along the query direction, rather they are processed in the order in which
candidate shapes for testing are found by PhysX' scene traversal algorithms.

\deprecated The batched query feature has been deprecated in PhysX version 3.4

@see PxBatchQueryDesc.postFilterShader PxQueryFilterCallback.postFilter PxBatchQueryPreFilterShader
*/

/**
\param[in] queryFilterData Query filter data
\param[in] objectFilterData Object filter data
\param[in] constantBlock Global constant filter data (see #PxBatchQuery)
\param[in] constantBlockSize Size of global filter data (see #PxBatchQuery)
\param[in] hit Hit data from the prior exact intersection test.
\return the new hit type for this hit (see #PxQueryHitType)

@see PxBatchQueryPreFilterShader
*/

typedef PX_DEPRECATED PxQueryHitType::Enum (*PxBatchQueryPostFilterShader)(
	PxFilterData queryFilterData, PxFilterData objectFilterData,
	const void* constantBlock, PxU32 constantBlockSize,
	const PxQueryHit& hit);

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
