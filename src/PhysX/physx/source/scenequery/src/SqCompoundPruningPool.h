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


#ifndef SQ_COMPOUNDPRUNING_POOL_H
#define SQ_COMPOUNDPRUNING_POOL_H

#include "SqPrunerMergeData.h"
#include "SqIncrementalAABBTree.h"
#include "PsArray.h"

namespace physx
{
namespace Sq
{
	class PruningPool;

	///////////////////////////////////////////////////////////////////////////////////////////////

	typedef Ps::Array<IncrementalAABBTreeNode*>			UpdateMap;

	///////////////////////////////////////////////////////////////////////////////////////////////

	class CompoundTree
	{
	public:
		void	updateObjectAfterManualBoundsUpdates(PrunerHandle handle);
		void	removeObject(PrunerHandle handle);
		bool	addObject(PrunerHandle& result, const PxBounds3& bounds, const PrunerPayload userData);

	private:
		void	updateMapping(const PoolIndex poolIndex, IncrementalAABBTreeNode* node, const NodeList& changedLeaves);

	public:
		IncrementalAABBTree*	mTree;
		PruningPool*			mPruningPool;
		UpdateMap*				mUpdateMap;
		PxTransform				mGlobalPose;
		CompoundFlag::Enum		mFlags;
	};

	///////////////////////////////////////////////////////////////////////////////////////////////

	class CompoundTreePool
	{
	public:
		CompoundTreePool();
		~CompoundTreePool();

		void					preallocate(PxU32 newCapacity);

		PoolIndex				addCompound(PrunerHandle* results, const Gu::BVHStructure& bvhStructure, const PxBounds3& compoundBounds, const PxTransform& transform, CompoundFlag::Enum flags, const PrunerPayload* userData);
		PoolIndex				removeCompound(PoolIndex index);

		void					shiftOrigin(const PxVec3& shift);

		PX_FORCE_INLINE const PxBounds3*		getCurrentCompoundBounds() const { return mCompoundBounds; }
		PX_FORCE_INLINE PxBounds3*				getCurrentCompoundBounds()  { return mCompoundBounds; }

		PX_FORCE_INLINE const CompoundTree*		getCompoundTrees() const { return mCompoundTrees; }
		PX_FORCE_INLINE CompoundTree*			getCompoundTrees() { return mCompoundTrees; }

		PX_FORCE_INLINE PxU32					getNbObjects() const { return mNbObjects; }


	private:
		bool					resize(PxU32 newCapacity);

	private:
		PxU32					mNbObjects;			//!< Current number of objects
		PxU32					mMaxNbObjects;		//!< Max. number of objects (capacity for mWorldBoxes, mObjects)

		//!< these arrays are parallel
		PxBounds3*				mCompoundBounds;	//!< List of compound world boxes, stores mNbObjects, capacity=mMaxNbObjects		
		CompoundTree*			mCompoundTrees;
	};

}
}

#endif

