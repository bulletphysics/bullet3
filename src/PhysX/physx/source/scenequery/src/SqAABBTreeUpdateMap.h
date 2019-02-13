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

#ifndef SQ_PRUNERTREEMAP_H
#define SQ_PRUNERTREEMAP_H

#include "SqTypedef.h"
#include "PsArray.h"

namespace physx
{
namespace Sq
{
	static const PxU32 INVALID_NODE_ID = 0xFFffFFff;
	static const PxU32 INVALID_POOL_ID = 0xFFffFFff;

	// Maps pruning pool indices to AABB-tree indices (i.e. locates the object's box in the aabb-tree nodes pool)
	// 
	// The map spans pool indices from 0..N-1, where N is the number of pool entries when the map was created from a tree.
	//
	// It maps: 
	//		to node indices in the range 0..M-1, where M is the number of nodes in the tree the map was created from,
	//   or to INVALID_NODE_ID if the pool entry was removed or pool index is outside input domain.
	//
	// The map is the inverse of the tree mapping: (node[map[poolID]].primitive == poolID) is true at all times.

	class AABBTreeUpdateMap 
	{
	public:
												AABBTreeUpdateMap()		{}
												~AABBTreeUpdateMap()	{}

						void					release()
												{
													mMapping.reset();
												}

						// indices offset used when indices are shifted from objects (used for merged trees)
						void					initMap(PxU32 numPoolObjects, const Sq::AABBTree& tree);

						void					invalidate(PoolIndex poolIndex, PoolIndex replacementPoolIndex, Sq::AABBTree& tree);

		PX_FORCE_INLINE TreeNodeIndex operator[](PxU32 poolIndex) const
												{ 
													return poolIndex < mMapping.size() ? mMapping[poolIndex] : INVALID_NODE_ID;
												}
	private:
		// maps from prunerIndex (index in the PruningPool) to treeNode index
		// this will only map to leaf tree nodes
					Ps::Array<TreeNodeIndex>	mMapping;
	};

}
}

#endif
