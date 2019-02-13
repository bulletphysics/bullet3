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


#ifndef SQ_COMPOUNDPRUNER_H
#define SQ_COMPOUNDPRUNER_H

#include "SqPrunerMergeData.h"
#include "SqCompoundPruningPool.h"
#include "SqPruningPool.h"
#include "SqIncrementalAABBTree.h"
#include "PsHashMap.h"
#include "PsArray.h"

namespace physx
{
namespace Sq
{

	///////////////////////////////////////////////////////////////////////////////////////////////

	typedef Ps::HashMap<PrunerCompoundId, PoolIndex>			ActorIdPoolIndexMap;
	typedef Ps::Array<PrunerCompoundId>							PoolIndexActorIdMap;

	///////////////////////////////////////////////////////////////////////////////////////////////

	class BVHCompoundPruner: public CompoundPruner
	{
	public:
		BVHCompoundPruner();
		~BVHCompoundPruner();

		void release();

	// CompoundPruner
		// compound level 
		virtual bool						addCompound(PrunerHandle* results, const Gu::BVHStructure& bvhStructure, PrunerCompoundId compoundId, const PxTransform& transform, CompoundFlag::Enum flags, const PrunerPayload* userData);
		virtual void						removeCompound(PrunerCompoundId compoundId);
		virtual void						updateCompound(PrunerCompoundId compoundId, const PxTransform& transform);
		// object level
		virtual void						updateObjectAfterManualBoundsUpdates(PrunerCompoundId compoundId, const PrunerHandle handle);
		virtual void						removeObject(PrunerCompoundId compoundId, const PrunerHandle handle);
		virtual bool						addObject(PrunerCompoundId compoundId, PrunerHandle& result, const PxBounds3& bounds, const PrunerPayload userData);
		//queries
		virtual	PxAgain						raycast(const PxVec3& origin, const PxVec3& unitDir, PxReal& inOutDistance, PrunerCallback&, PxQueryFlags flags) const;
		virtual	PxAgain						overlap(const Gu::ShapeData& queryVolume, PrunerCallback&, PxQueryFlags flags) const;
		virtual	PxAgain						sweep(const Gu::ShapeData& queryVolume, const PxVec3& unitDir, PxReal& inOutDistance, PrunerCallback&, PxQueryFlags flags) const;
		virtual const PrunerPayload&		getPayload(PrunerHandle handle, PrunerCompoundId compoundId) const;
		virtual const PrunerPayload&		getPayload(PrunerHandle handle, PrunerCompoundId compoundId, PxBounds3*& bounds) const;
		virtual void						shiftOrigin(const PxVec3& shift);
		virtual	void						visualize(Cm::RenderOutput&, PxU32) const;
	// ~CompoundPruner

	private:
		void updateMapping(const PoolIndex poolIndex, IncrementalAABBTreeNode* node);
		void updateMainTreeNode(PoolIndex index);

		void test();
	private:
		IncrementalAABBTree				mMainTree;
		UpdateMap						mMainTreeUpdateMap;
		
		CompoundTreePool				mCompoundTreePool;
		ActorIdPoolIndexMap				mActorPoolMap;
		PoolIndexActorIdMap				mPoolActorMap;
		NodeList						mChangedLeaves;
	};
}
}

#endif

