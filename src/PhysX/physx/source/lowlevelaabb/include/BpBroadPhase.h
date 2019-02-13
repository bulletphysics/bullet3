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


#ifndef BP_BROADPHASE_H
#define BP_BROADPHASE_H

#include "foundation/PxUnionCast.h"
#include "PxBroadPhase.h"
#include "BpAABBManager.h"

namespace physx
{
class PxcScratchAllocator;

namespace Bp
{

class BroadPhaseUpdateData;
class BPMemoryAllocator;


/**
\brief Base broad phase class.  Functions only relevant to MBP.
*/
class BroadPhaseBase
{
	public:
									BroadPhaseBase()						{}
	virtual							~BroadPhaseBase()						{}

	/**
	\brief Gets broad-phase caps.

	\param[out]	caps	Broad-phase caps
	\return True if success
	*/
	virtual	bool					getCaps(PxBroadPhaseCaps& caps)			const 
	{
		caps.maxNbRegions = 0;
		caps.maxNbObjects = 0;
		caps.needsPredefinedBounds = false;
		return true;
	}

	/**
	\brief Returns number of regions currently registered in the broad-phase.

	\return Number of regions
	*/
	virtual	PxU32					getNbRegions()							const 
	{
		return 0;	
	}

	/**
	\brief Gets broad-phase regions.

	\param[out]	userBuffer	Returned broad-phase regions
	\param[in]	bufferSize	Size of userBuffer
	\param[in]	startIndex	Index of first desired region, in [0 ; getNbRegions()[
	\return Number of written out regions
	*/
	virtual	PxU32					getRegions(PxBroadPhaseRegionInfo* userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const
	{
		PX_UNUSED(userBuffer);
		PX_UNUSED(bufferSize);
		PX_UNUSED(startIndex);
		return 0;
	}

	/**
	\brief Adds a new broad-phase region.

	The bounds for the new region must be non-empty, otherwise an error occurs and the call is ignored.

	The total number of regions is limited to 256. If that number is exceeded, the call is ignored.

	The newly added region will be automatically populated with already existing SDK objects that touch it, if the
	'populateRegion' parameter is set to true. Otherwise the newly added region will be empty, and it will only be
	populated with objects when those objects are added to the simulation, or updated if they already exist.

	Using 'populateRegion=true' has a cost, so it is best to avoid it if possible. In particular it is more efficient
	to create the empty regions first (with populateRegion=false) and then add the objects afterwards (rather than
	the opposite).

	Objects automatically move from one region to another during their lifetime. The system keeps tracks of what
	regions a given object is in. It is legal for an object to be in an arbitrary number of regions. However if an
	object leaves all regions, or is created outside of all regions, several things happen:
		- collisions get disabled for this object
		- if a PxBroadPhaseCallback object is provided, an "out-of-bounds" event is generated via that callback
		- if a PxBroadPhaseCallback object is not provided, a warning/error message is sent to the error stream

	If an object goes out-of-bounds and user deletes it during the same frame, neither the out-of-bounds event nor the
	error message is generated.

	If an out-of-bounds object, whose collisions are disabled, re-enters a valid broadphase region, then collisions
	are re-enabled for that object.

	\param[in]	region			User-provided region data
	\param[in]	populateRegion	True to automatically populate the newly added region with existing objects touching it

	\return Handle for newly created region, or 0xffffffff in case of failure.
	*/
	virtual	PxU32					addRegion(const PxBroadPhaseRegion& region, bool populateRegion)
	{
		PX_UNUSED(region);
		PX_UNUSED(populateRegion);
		return 0xffffffff;	
	}

	/**
	\brief Removes a new broad-phase region.

	If the region still contains objects, and if those objects do not overlap any region any more, they are not
	automatically removed from the simulation. Instead, the PxBroadPhaseCallback::onObjectOutOfBounds notification
	is used for each object. Users are responsible for removing the objects from the simulation if this is the
	desired behavior.

	If the handle is invalid, or if a valid handle is removed twice, an error message is sent to the error stream.

	\param[in]	handle	Region's handle, as returned by PxScene::addBroadPhaseRegion.
	\return True if success
	*/
	virtual	bool					removeRegion(PxU32 handle)	
	{
		PX_UNUSED(handle);
		return false;	
	}

	/*
	\brief Return the number of objects that are not in any region.
	*/
	virtual	PxU32						getNbOutOfBoundsObjects()	const	
	{ 
		return 0;						
	}

	/*
	\brief Return an array of objects that are not in any region.
	*/
	virtual	const PxU32*				getOutOfBoundsObjects()		const	
	{ 
		return NULL;					
	}
};


/*
\brief Structure used to report created and deleted broadphase pairs
\note The indices mVolA and mVolB correspond to the bounds indices 
BroadPhaseUpdateData::mCreated used by BroadPhase::update
@see BroadPhase::getCreatedPairs, BroadPhase::getDeletedPairs
*/
struct BroadPhasePair
{
	BroadPhasePair(ShapeHandle volA, ShapeHandle volB) :
		mVolA		(PxMin(volA, volB)),
		mVolB		(PxMax(volA, volB))
	{
	}
	BroadPhasePair() :
		mVolA		(BP_INVALID_BP_HANDLE),
		mVolB		(BP_INVALID_BP_HANDLE)
	{
	}

	ShapeHandle		mVolA;		// NB: mVolA < mVolB
	ShapeHandle		mVolB;
};

class BroadPhase : public BroadPhaseBase
{
public:

	virtual	PxBroadPhaseType::Enum	getType() const = 0;

	/**
	\brief Instantiate a BroadPhase instance.
	\param[in] bpType - the bp type (either mbp or sap).  This is typically specified in PxSceneDesc. 
	\param[in] maxNbRegions is the expected maximum number of broad-phase regions.
	\param[in] maxNbBroadPhaseOverlaps is the expected maximum number of broad-phase overlaps.
	\param[in] maxNbStaticShapes is the expected maximum number of static shapes.
	\param[in] maxNbDynamicShapes is the expected maximum number of dynamic shapes.
	\param[in] contextID is the context ID parameter sent to the profiler
	\return The instantiated BroadPhase.
	\note maxNbRegions is only used if mbp is the chosen broadphase (PxBroadPhaseType::eMBP)
	\note maxNbRegions, maxNbBroadPhaseOverlaps, maxNbStaticShapes and maxNbDynamicShapes are typically specified in PxSceneLimits
	*/
	static BroadPhase* create(
		const PxBroadPhaseType::Enum bpType,
		const PxU32 maxNbRegions,
		const PxU32 maxNbBroadPhaseOverlaps,
		const PxU32 maxNbStaticShapes,
		const PxU32 maxNbDynamicShapes,
		PxU64 contextID);


	/**
	\brief Shutdown of the broadphase.
	*/
	virtual	void					destroy() = 0;

	/**
	\brief Update the broadphase and compute the lists of created/deleted pairs.

	\param[in] numCpuTasks the number of cpu tasks that can be used for the broadphase update.
	
	\param[in] scratchAllocator - a PxcScratchAllocator instance used for temporary memory allocations.
	This must be non-null.

	\param[in] updateData a description of changes to the collection of aabbs since the last broadphase update.
	The changes detail the indices of the bounds that have been added/updated/removed as well as an array of all
	bound coordinates and an array of group ids used to filter pairs with the same id.
	@see BroadPhaseUpdateData

	\param[in] continuation the task that is in the queue to be executed immediately after the broadphase has completed its update. NULL is not supported.
	\param[in] nPhaseUnlockTask this task will have its ref count decremented when it is safe to permit NP to run in parallel with BP. NULL is supported.

	\note In PX_CHECKED and PX_DEBUG build configurations illegal input data (that does not conform to the BroadPhaseUpdateData specifications) triggers 
	a special code-path that entirely bypasses the broadphase and issues a warning message to the error stream.  No guarantees can be made about the 
	correctness/consistency of broadphase behavior with illegal input data in PX_RELEASE and PX_PROFILE configs because validity checks are not active 
	in these builds.
	*/

	virtual	void					update(const PxU32 numCpuTasks, PxcScratchAllocator* scratchAllocator, const BroadPhaseUpdateData& updateData, physx::PxBaseTask* continuation, physx::PxBaseTask* nPhaseUnlockTask) = 0;

	/**
	\brief Fetch the results of any asynchronous broad phase work.
	*/
	virtual	void					fetchBroadPhaseResults(physx::PxBaseTask* nPhaseUnlockTask) = 0;

	/*
	\brief Return the number of created aabb overlap pairs computed in the execution of update() that has just completed.
	*/
	virtual	PxU32					getNbCreatedPairs()		const	= 0;

	/*
	\brief Return the array of created aabb overlap pairs computed in the execution of update() that has just completed.
	Note that each overlap pair is reported only on the frame when the overlap first occurs. The overlap persists
	until the pair appears in the list of deleted pairs or either of the bounds in the pair is removed from the broadphase.  
	A created overlap must involve at least one of the bounds of the overlap pair appearing in either the created or updated list.
	It is impossible for the same pair to appear simultaneously in the list of created and deleted overlap pairs.
	An overlap is defined as a pair of bounds that overlap on all three axes; that is when maxA > minB and maxB > minA for all three axes. 
	The rule that minima(maxima) are even(odd) (see BroadPhaseUpdateData) removes the ambiguity of touching bounds. 

	*/
	virtual	BroadPhasePair*			getCreatedPairs()					= 0;

	/**
	\brief Return the number of deleted overlap pairs computed in the execution of update() that has just completed.
	*/
	virtual	PxU32					getNbDeletedPairs()		const	= 0;

	/**
	\brief Return the number of deleted overlap pairs computed in the execution of update() that has just completed.
	Note that a deleted pair can only be reported if that pair has already appeared in the list of created pairs in an earlier update.
	A lost overlap occurs when a pair of bounds previously overlapped on all three axes but have now separated on at least one axis.
	A lost overlap must involve at least one of the bounds of the lost overlap pair appearing in the updated list.
	Lost overlaps arising from removal of bounds from the broadphase do not appear in the list of deleted pairs.
	It is impossible for the same pair to appear simultaneously in the list of created and deleted pairs.
	The test for overlap is conservative throughout, meaning that deleted pairs do not include touching pairs.
	*/
	virtual	BroadPhasePair*			getDeletedPairs()					= 0;

	/**
	\brief After the broadphase has completed its update() function and the created/deleted pairs have been queried
	with getCreatedPairs/getDeletedPairs it is desirable to free any memory that was temporarily acquired for the update but is 
	is no longer required post-update.  This can be achieved with the function freeBuffers().
	*/
	virtual	void					freeBuffers()						= 0;

	/**
	\brief Adjust internal structures after all bounds have been adjusted due to a scene origin shift.
	*/
	virtual void					shiftOrigin(const PxVec3& shift) = 0;

	/**
	\brief Test that the created/updated/removed lists obey the rules that 
	1. object ids can only feature in the created list if they have never been previously added or if they were previously removed.
	2. object ids can only be added to the updated list if they have been previously added without being removed.
	3. objects ids can only be added to the removed list if they have been previously added without being removed.
	*/
#if PX_CHECKED
	virtual bool					isValid(const BroadPhaseUpdateData& updateData) const = 0;
#endif

	virtual BroadPhasePair*			getBroadPhasePairs() const = 0;

	virtual void					deletePairs() = 0;

	// PT: for unit-testing the non-GPU versions
	virtual	void					singleThreadedUpdate(PxcScratchAllocator* /*scratchAllocator*/, const BroadPhaseUpdateData& /*updateData*/){}
};

} //namespace Bp

} //namespace physx

#endif //BP_BROADPHASE_H
