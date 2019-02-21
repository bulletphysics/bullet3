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

#ifndef SQ_EXTENDEDBUCKETPRUNER_H
#define SQ_EXTENDEDBUCKETPRUNER_H

#include "SqTypedef.h"
#include "SqBucketPruner.h"
#include "SqIncrementalAABBPrunerCore.h"
#include "SqAABBTreeUpdateMap.h"
#include "PsHashMap.h"

#define USE_INCREMENTAL_PRUNER 1

namespace physx
{
namespace Sq
{
	struct AABBPrunerMergeData;
	class AABBTreeMergeData;

#if USE_INCREMENTAL_PRUNER
	typedef IncrementalAABBPrunerCore PrunerCore;
#else
	typedef BucketPrunerCore		PrunerCore;
#endif

	// Extended bucket pruner data, if an object belongs to the tree of trees, we need to
	// remember node for the sub tree, the tree it belongs to and the main tree node
	struct ExtendedBucketPrunerData
	{		
		PxU32			mTimeStamp;				// timestamp 		
		TreeNodeIndex	mSubTreeNode;			// sub tree node index
		PxU32			mMergeIndex;			// index in bounds and merged trees array
	};

	// Merged tree structure, holds tree and its timeStamp, released when no objects is in the tree
	// or timeStamped objects are released
	struct MergedTree
	{
		AABBTree*	mTree;			// AABB tree 
		size_t		mTimeStamp;		// needs to be size_t to match PrunerPayload size
	};
	// needs to be size_t to match PrunerPayload size, pointer used for AABB tree query callbacks
	PX_COMPILE_TIME_ASSERT(sizeof(MergedTree) == sizeof(PrunerPayload));
	
	// hashing function for PrunerPaylod key
	struct ExtendedBucketPrunerHash
	{
		PX_FORCE_INLINE uint32_t operator()(const PrunerPayload& payload) const
		{
#if PX_P64_FAMILY
			//		const PxU32 h0 = Ps::hash((const void*)payload.data[0]);
			//		const PxU32 h1 = Ps::hash((const void*)payload.data[1]);
			const PxU32 h0 = PxU32(PX_MAX_U32 & payload.data[0]);
			const PxU32 h1 = PxU32(PX_MAX_U32 & payload.data[1]);
			return Ps::hash(PxU64(h0) | (PxU64(h1) << 32));
#else
			return Ps::hash(PxU64(payload.data[0]) | (PxU64(payload.data[1]) << 32));
#endif
		}
		PX_FORCE_INLINE bool equal(const PrunerPayload& k0, const PrunerPayload& k1) const
		{
			return (k0.data[0] == k1.data[0]) && (k0.data[1] == k1.data[1]);
		}
	};

	// A.B. replace, this is useless, need to be able to traverse the map and release while traversing, also eraseAt failed
	typedef Ps::HashMap<PrunerPayload, ExtendedBucketPrunerData, ExtendedBucketPrunerHash>	ExtendedBucketPrunerMap;

	// Extended bucket pruner holds single objects in a bucket pruner and AABBtrees in a tree of trees.
	// Base usage of ExtendedBucketPruner is for dynamic AABBPruner new objects, that did not make it 
	// into new tree. Single objects go directly into a bucket pruner, while merged AABBtrees 
	// go into a tree of trees.
	class ExtendedBucketPruner
	{
	public:
										ExtendedBucketPruner(const PruningPool* pool);
		virtual							~ExtendedBucketPruner();

		// release 
		void							release();

		// add single object into a bucket pruner directly
		PX_FORCE_INLINE bool			addObject(const PrunerPayload& object, const PxBounds3& worldAABB, PxU32 timeStamp, const PoolIndex poolIndex)
		{
#if USE_INCREMENTAL_PRUNER
			PX_UNUSED(worldAABB);
			PX_UNUSED(object);
			return mPrunerCore.addObject(poolIndex, timeStamp);
#else
			PX_UNUSED(poolIndex);
			return mPrunerCore.addObject(object, worldAABB, timeStamp);
#endif
		}

		// add AABB tree from pruning structure - adds new primitive into main AABB tree
		void							addTree(const AABBTreeMergeData& mergeData, PxU32 timeStamp);

		// update object
		bool							updateObject(const PxBounds3& worldAABB, const PrunerPayload& object, const PoolIndex poolIndex);

		// remove object, removed object is replaced in pruning pool by swapped object, indices needs to be updated
		bool							removeObject(const PrunerPayload& object, PxU32 objectIndex, const PrunerPayload& swapObject,
											PxU32 swapObjectIndex, PxU32& timeStamp);


		// swap object index, the object index can be in core pruner or tree of trees
		void							swapIndex(PxU32 objectIndex, const PrunerPayload& swapObject, PxU32 swapObjectIndex, bool corePrunerIncluded = true);

		// refit marked nodes in tree of trees
		void							refitMarkedNodes(const PxBounds3* boxes);

#if USE_INCREMENTAL_PRUNER
		// notify timestampChange - swap trees in incremental pruner
		void							timeStampChange()	 { mPrunerCore.timeStampChange(); }
#endif


		// look for objects marked with input timestamp everywhere in the structure, and remove them. This is the same
		// as calling 'removeObject' individually for all these objects, but much more efficient. Returns number of removed objects.
		PxU32							removeMarkedObjects(PxU32 timeStamp);

		// queries against the pruner
		PxAgain							raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, PrunerCallback&) const;
		PxAgain							overlap(const Gu::ShapeData& queryVolume, PrunerCallback&) const;
		PxAgain							sweep(const Gu::ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, PrunerCallback&) const;

		// origin shift
		void							shiftOrigin(const PxVec3& shift);

		// debug visualize
		void							visualize(Cm::RenderOutput& out, PxU32 color) const;

		PX_FORCE_INLINE	void			build()					{ mPrunerCore.build();	}

		PX_FORCE_INLINE PxU32			getNbObjects()	const	{ return mPrunerCore.getNbObjects() + mExtendedBucketPrunerMap.size(); }

	private:

		// separate call for indices invalidation, object can be either in AABBPruner or Bucket pruner, but the swapped object can be 
		// in the tree of trees
		void							invalidateObject(const ExtendedBucketPrunerData& object, PxU32 objectIndex, const PrunerPayload& swapObject,
			PxU32 swapObjectIndex);

		void							resize(PxU32 size);
		void							buildMainAABBTree();
		void							cleanTrees();

#if PX_DEBUG
		// Extended bucket pruner validity check
		bool							checkValidity();
#endif
	private:
				PrunerCore				mPrunerCore;					// pruner for single objects
				const PruningPool*		mPruningPool;					// Pruning pool from AABB pruner
				ExtendedBucketPrunerMap	mExtendedBucketPrunerMap;		// Map holding objects from tree merge - objects in tree of trees
				AABBTree*				mMainTree;						// Main tree holding merged trees
				AABBTreeUpdateMap		mMainTreeUpdateMap;				// Main tree updated map - merged trees index to nodes
				AABBTreeUpdateMap		mMergeTreeUpdateMap;			// Merged tree update map used while tree is merged
				PxBounds3*				mBounds;						// Merged trees bounds used for main tree building
				MergedTree*				mMergedTrees;					// Merged trees
				PxU32					mCurrentTreeIndex;				// Current trees index
				PxU32					mCurrentTreeCapacity;			// Current tress capacity
				bool					mTreesDirty;					// Dirty marker
	};

} // namespace Sq

}

#endif // SQ_EXTENDEDBUCKETPRUNER_H
