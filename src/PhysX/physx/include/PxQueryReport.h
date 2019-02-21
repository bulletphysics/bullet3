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


#ifndef PX_PHYSICS_NX_SCENEQUERYREPORT
#define PX_PHYSICS_NX_SCENEQUERYREPORT
/** \addtogroup scenequery
@{
*/
#include "PxPhysXConfig.h"
#include "foundation/PxVec3.h"
#include "foundation/PxFlags.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

class PxShape;
class PxRigidActor;

/**
\brief Scene query and geometry query behavior flags.

PxHitFlags are used for 3 different purposes:

1) To request hit fields to be filled in by scene queries (such as hit position, normal, face index or UVs).
2) Once query is completed, to indicate which fields are valid (note that a query may produce more valid fields than requested).
3) To specify additional options for the narrow phase and mid-phase intersection routines.

All these flags apply to both scene queries and geometry queries (PxGeometryQuery).

@see PxRaycastHit PxSweepHit PxOverlapHit PxScene.raycast PxScene.sweep PxScene.overlap PxGeometryQuery PxFindFaceIndex
*/
struct PxHitFlag
{
	enum Enum
	{
		ePOSITION					= (1<<0),	//!< "position" member of #PxQueryHit is valid
		eNORMAL						= (1<<1),	//!< "normal" member of #PxQueryHit is valid
		eUV							= (1<<3),	//!< "u" and "v" barycentric coordinates of #PxQueryHit are valid. Not applicable to sweep queries.
		eASSUME_NO_INITIAL_OVERLAP	= (1<<4),	//!< Performance hint flag for sweeps when it is known upfront there's no initial overlap.
												//!< NOTE: using this flag may cause undefined results if shapes are initially overlapping.
		eMESH_MULTIPLE				= (1<<5),	//!< Report all hits for meshes rather than just the first. Not applicable to sweep queries.
		eMESH_ANY					= (1<<6),	//!< Report any first hit for meshes. If neither eMESH_MULTIPLE nor eMESH_ANY is specified,
												//!< a single closest hit will be reported for meshes.
		eMESH_BOTH_SIDES			= (1<<7),	//!< Report hits with back faces of mesh triangles. Also report hits for raycast
												//!< originating on mesh surface and facing away from the surface normal. Not applicable to sweep queries.
												//!< Please refer to the user guide for heightfield-specific differences.
		ePRECISE_SWEEP				= (1<<8),	//!< Use more accurate but slower narrow phase sweep tests.
												//!< May provide better compatibility with PhysX 3.2 sweep behavior.
		eMTD						= (1<<9),	//!< Report the minimum translation depth, normal and contact point.
		eFACE_INDEX					= (1<<10),	//!< "face index" member of #PxQueryHit is valid

		eDEFAULT					= ePOSITION|eNORMAL|eFACE_INDEX,

		/** \brief Only this subset of flags can be modified by pre-filter. Other modifications will be discarded. */
		eMODIFIABLE_FLAGS			= eMESH_MULTIPLE|eMESH_BOTH_SIDES|eASSUME_NO_INITIAL_OVERLAP|ePRECISE_SWEEP
	};
};


/**
\brief collection of set bits defined in PxHitFlag.

@see PxHitFlag
*/
PX_FLAGS_TYPEDEF(PxHitFlag, PxU16)

/**
\brief Combines a shape pointer and the actor the shape belongs to into one memory location.

Serves as a base class for PxQueryHit.

@see PxQueryHit
*/
struct PxActorShape
{
	PX_INLINE PxActorShape() : actor(NULL), shape(NULL) {}
	PX_INLINE PxActorShape(PxRigidActor* a, PxShape* s) : actor(a), shape(s) {}

	PxRigidActor*	actor;
	PxShape*		shape;
};


/**
\brief Scene query hit information.
*/
struct PxQueryHit : public PxActorShape
{
	PX_INLINE			PxQueryHit() : faceIndex(0xFFFFffff) {}

	/**
	Face index of touched triangle, for triangle meshes, convex meshes and height fields.

	\note This index will default to 0xFFFFffff value for overlap queries.
	\note Please refer to the user guide for more details for sweep queries.
	\note This index is remapped by mesh cooking. Use #PxTriangleMesh::getTrianglesRemap() to convert to original mesh index.
	\note For convex meshes use #PxConvexMesh::getPolygonData() to retrieve touched polygon data.
	*/
	PxU32				faceIndex;
};

/**
\brief Scene query hit information for raycasts and sweeps returning hit position and normal information.

::PxHitFlag flags can be passed to scene query functions, as an optimization, to cause the SDK to
only generate specific members of this structure.
*/
struct PxLocationHit : public PxQueryHit
{
	PX_INLINE			PxLocationHit() : flags(0), position(PxVec3(0)), normal(PxVec3(0)), distance(PX_MAX_REAL)	{}

	/**
	\note For raycast hits: true for shapes overlapping with raycast origin.
	\note For sweep hits: true for shapes overlapping at zero sweep distance.

	@see PxRaycastHit PxSweepHit
	*/
	PX_INLINE bool		hadInitialOverlap() const { return (distance <= 0.0f); }

	// the following fields are set in accordance with the #PxHitFlags
	PxHitFlags			flags;		//!< Hit flags specifying which members contain valid values.
	PxVec3				position;	//!< World-space hit position (flag: #PxHitFlag::ePOSITION)
	PxVec3				normal;		//!< World-space hit normal (flag: #PxHitFlag::eNORMAL)

	/**
	\brief	Distance to hit.
	\note	If the eMTD flag is used, distance will be a negative value if shapes are overlapping indicating the penetration depth.
	\note	Otherwise, this value will be >= 0 */
	PxF32				distance;
};


/**
\brief Stores results of raycast queries.

::PxHitFlag flags can be passed to raycast function, as an optimization, to cause the SDK to only compute specified members of this
structure.

Some members like barycentric coordinates are currently only computed for triangle meshes and height fields, but next versions
might provide them in other cases. The client code should check #flags to make sure returned values are valid.

@see PxScene.raycast PxBatchQuery.raycast
*/
struct PxRaycastHit : public PxLocationHit
{
	PX_INLINE			PxRaycastHit() : u(0.0f), v(0.0f)	{}

	// the following fields are set in accordance with the #PxHitFlags

	PxReal	u, v;			//!< barycentric coordinates of hit point, for triangle mesh and height field (flag: #PxHitFlag::eUV)
#if !PX_P64_FAMILY
	PxU32	padTo16Bytes[3];
#endif
};


/**
\brief Stores results of overlap queries.

@see PxScene.overlap PxBatchQuery.overlap
*/
struct PxOverlapHit: public PxQueryHit { PxU32 padTo16Bytes; };


/**
\brief Stores results of sweep queries.

@see PxScene.sweep PxBatchQuery.sweep
*/
struct PxSweepHit : public PxLocationHit
{
	PX_INLINE			PxSweepHit() {}

	PxU32				padTo16Bytes;
};


/**
\brief Describes query behavior after returning a partial query result via a callback.

If callback returns true, traversal will continue and callback can be issued again.
If callback returns false, traversal will stop, callback will not be issued again.

@see PxHitCallback
*/
typedef bool PxAgain;


/**
\brief	This callback class facilitates reporting scene query hits (intersections) to the user.

User overrides the virtual processTouches function to receive hits in (possibly multiple) fixed size blocks.

\note	PxHitBuffer derives from this class and is used to receive touching hits in a fixed size buffer.
\note	Since the compiler doesn't look in template dependent base classes when looking for non-dependent names
\note	with some compilers it will be necessary to use "this->hasBlock" notation to access a parent variable
\note	in a child callback class.
\note	Pre-made typedef shorthands, such as ::PxRaycastCallback can be used for raycast, overlap and sweep queries.

@see PxHitBuffer PxRaycastHit PxSweepHit PxOverlapHit PxRaycastCallback PxOverlapCallback PxSweepCallback
*/
template<typename HitType>
struct PxHitCallback
{
	HitType		block;			//!< Holds the closest blocking hit result for the query. Invalid if hasBlock is false.
	bool		hasBlock;		//!< Set to true if there was a blocking hit during query.

	HitType*	touches;		//!< User specified buffer for touching hits.

	/**
	\brief	Size of the user specified touching hits buffer.
	\note	If set to 0 all hits will default to PxQueryHitType::eBLOCK, otherwise to PxQueryHitType::eTOUCH
	\note	Hit type returned from pre-filter overrides this default */
	PxU32		maxNbTouches;

	/**
	\brief	Number of touching hits returned by the query. Used with PxHitBuffer.
	\note	If true (PxAgain) is returned from the callback, nbTouches will be reset to 0. */
	PxU32		nbTouches;

	/**
	\brief	Initializes the class with user provided buffer.

	\param[in] aTouches			Optional buffer for recording PxQueryHitType::eTOUCH type hits.
	\param[in] aMaxNbTouches	Size of touch buffer.

	\note	if aTouches is NULL and aMaxNbTouches is 0, only the closest blocking hit will be recorded by the query.
	\note	If PxQueryFlag::eANY_HIT flag is used as a query parameter, hasBlock will be set to true and blockingHit will be used to receive the result.
	\note	Both eTOUCH and eBLOCK hits will be registered as hasBlock=true and stored in PxHitCallback.block when eANY_HIT flag is used.

	@see PxHitCallback.hasBlock PxHitCallback.block */
	PxHitCallback(HitType* aTouches, PxU32 aMaxNbTouches)
		: hasBlock(false), touches(aTouches), maxNbTouches(aMaxNbTouches), nbTouches(0)
	{}

	/**
	\brief virtual callback function used to communicate query results to the user.

	This callback will always be invoked with #touches as a buffer if #touches was specified as non-NULL.
	All reported touch hits are guaranteed to be closer than the closest blocking hit.

	\param[in]	buffer	Callback will report touch hits to the user in this buffer. This pointer will be the same as #touches.
	\param[in]	nbHits	Number of touch hits reported in buffer. This number will not exceed #maxNbTouches.

	\note	There is a significant performance penalty in case multiple touch callbacks are issued (up to 2x)
	\note	to avoid the penalty use a bigger buffer so that all touching hits can be reported in a single buffer.
	\note	If true (again) is returned from the callback, nbTouches will be reset to 0,
	\note	If false is returned, nbTouched will remain unchanged.
	\note	By the time processTouches is first called, the globally closest blocking hit is already determined,
	\note	values of hasBlock and block are final and all touch hits are guaranteed to be closer than the blocking hit.
	\note	touches and maxNbTouches can be modified inside of processTouches callback.

	\return	true to continue receiving callbacks in case there are more hits or false to stop.

	@see PxAgain PxRaycastHit PxSweepHit PxOverlapHit */
	virtual PxAgain processTouches(const HitType* buffer, PxU32 nbHits) = 0;

	virtual void finalizeQuery() {} //!< Query finalization callback, called after the last processTouches callback.

	virtual ~PxHitCallback() {}

	/** \brief Returns true if any blocking or touching hits were encountered during a query. */
	PX_FORCE_INLINE bool hasAnyHits() { return (hasBlock || (nbTouches > 0)); }
};


/**
\brief	Returns scene query hits (intersections) to the user in a preallocated buffer.

Will clip touch hits to maximum buffer capacity. When clipped, an arbitrary subset of touching hits will be discarded.
Overflow does not trigger warnings or errors. block and hasBlock will be valid in finalizeQuery callback and after query completion.
Touching hits are guaranteed to have closer or same distance ( <= condition) as the globally nearest blocking hit at the time any processTouches()
callback is issued.

\note	Pre-made typedef shorthands, such as ::PxRaycastBuffer can be used for raycast, overlap and sweep queries.

@see PxHitCallback
@see PxRaycastBuffer PxOverlapBuffer PxSweepBuffer PxRaycastBufferN PxOverlapBufferN PxSweepBufferN
*/
template<typename HitType>
struct PxHitBuffer : public PxHitCallback<HitType>
{
	/**
	\brief	Initializes the buffer with user memory.

	The buffer is initialized with 0 touch hits by default => query will only report a single closest blocking hit.
	Use PxQueryFlag::eANY_HIT to tell the query to abort and return any first hit encoutered as blocking.

	\param[in] aTouches			Optional buffer for recording PxQueryHitType::eTOUCH type hits.
	\param[in] aMaxNbTouches	Size of touch buffer.

	@see PxHitCallback */
	PxHitBuffer(HitType* aTouches = NULL, PxU32 aMaxNbTouches = 0) : PxHitCallback<HitType>(aTouches, aMaxNbTouches) {}

	/** \brief Computes the number of any hits in this result, blocking or touching. */
	PX_INLINE PxU32				getNbAnyHits() const				{ return getNbTouches() + PxU32(this->hasBlock); }
	/** \brief Convenience iterator used to access any hits in this result, blocking or touching. */
	PX_INLINE const HitType&	getAnyHit(const PxU32 index) const	{ PX_ASSERT(index < getNbTouches() + PxU32(this->hasBlock));
																		return index < getNbTouches() ? getTouches()[index] : this->block; }

	PX_INLINE PxU32				getNbTouches() const				{ return this->nbTouches; }
	PX_INLINE const HitType*	getTouches() const					{ return this->touches; }
	PX_INLINE const HitType&	getTouch(const PxU32 index) const	{ PX_ASSERT(index < getNbTouches()); return getTouches()[index]; }
	PX_INLINE PxU32				getMaxNbTouches() const				{ return this->maxNbTouches; }

	virtual ~PxHitBuffer() {}

protected:
	// stops after the first callback
	virtual PxAgain processTouches(const HitType* buffer, PxU32 nbHits) { PX_UNUSED(buffer); PX_UNUSED(nbHits); return false; }
};


/** \brief Raycast query callback. */
typedef PxHitCallback<PxRaycastHit> PxRaycastCallback;

/** \brief Overlap query callback. */
typedef PxHitCallback<PxOverlapHit> PxOverlapCallback;

/** \brief Sweep query callback. */
typedef PxHitCallback<PxSweepHit> PxSweepCallback;

/** \brief Raycast query buffer. */
typedef PxHitBuffer<PxRaycastHit> PxRaycastBuffer;

/** \brief Overlap query buffer. */
typedef PxHitBuffer<PxOverlapHit> PxOverlapBuffer;

/** \brief Sweep query buffer. */
typedef PxHitBuffer<PxSweepHit> PxSweepBuffer;

/** \brief	Returns touching raycast hits to the user in a fixed size array embedded in the buffer class. **/
template <int N>
struct PxRaycastBufferN : public PxHitBuffer<PxRaycastHit>
{
	PxRaycastHit hits[N];
	PxRaycastBufferN() : PxHitBuffer<PxRaycastHit>(hits, N) {}
};

/** \brief	Returns touching overlap hits to the user in a fixed size array embedded in the buffer class. **/
template <int N>
struct PxOverlapBufferN : public PxHitBuffer<PxOverlapHit>
{
	PxOverlapHit hits[N];
	PxOverlapBufferN() : PxHitBuffer<PxOverlapHit>(hits, N) {}
};

/** \brief	Returns touching sweep hits to the user in a fixed size array embedded in the buffer class. **/
template <int N>
struct PxSweepBufferN : public PxHitBuffer<PxSweepHit>
{
	PxSweepHit hits[N];
	PxSweepBufferN() : PxHitBuffer<PxSweepHit>(hits, N) {}
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
