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

#ifndef SQ_PRUNINGPOOL_H
#define SQ_PRUNINGPOOL_H

#include "SqPruner.h"
#include "SqTypedef.h"
#include "SqBounds.h"

namespace physx
{
namespace Sq
{
	// This class is designed to maintain a two way mapping between pair(PrunerPayload,AABB) and PrunerHandle
	// Internally there's also an index for handles (AP: can be simplified?)
	// This class effectively stores bounded pruner payloads, returns a PrunerHandle and allows O(1)
	// access to them using a PrunerHandle
	// Supported operations are add, remove, update bounds
	class PruningPool
	{
		public:
												PruningPool();
												~PruningPool();

		PX_FORCE_INLINE	const PrunerPayload&	getPayload(PrunerHandle handle) const { return mObjects[getIndex(handle)];	}

		PX_FORCE_INLINE	const PrunerPayload&	getPayload(PrunerHandle handle, PxBounds3*& bounds)	const
												{
													const PoolIndex index = getIndex(handle);
													bounds = mWorldBoxes + index;
													return mObjects[index];
												}

						void					shiftOrigin(const PxVec3& shift);

		// PT: adds 'count' objects to the pool. Needs 'count' bounds and 'count' payloads passed as input. Writes out 'count' handles
		// in 'results' array. Function returns number of successfully added objects, ideally 'count' but can be less in case we run
		// out of memory.
						PxU32					addObjects(PrunerHandle* results, const PxBounds3* bounds, const PrunerPayload* payload, PxU32 count);

		// this function will swap the last object with the hole formed by removed PrunerHandle object
		// and return the removed last object's index in the pool
						PoolIndex				removeObject(PrunerHandle h);

		// Data access
		PX_FORCE_INLINE	PoolIndex				getIndex(PrunerHandle h)const	{ return mHandleToIndex[h];	}
		PX_FORCE_INLINE	PrunerPayload*			getObjects()			const	{ return mObjects;			}
		PX_FORCE_INLINE	PxU32					getNbActiveObjects()	const	{ return mNbObjects;		}
		PX_FORCE_INLINE	const PxBounds3*		getCurrentWorldBoxes()	const	{ return mWorldBoxes;		}
		PX_FORCE_INLINE	PxBounds3*				getCurrentWorldBoxes()			{ return mWorldBoxes;		}

		PX_FORCE_INLINE	const PxBounds3&		getWorldAABB(PrunerHandle h) const
												{
													return mWorldBoxes[getIndex(h)];
												}

		PX_FORCE_INLINE	void					updateObjectsAndInflateBounds(const PrunerHandle* handles, const PxU32* indices, const PxBounds3* newBounds, PxU32 count)
												{
													for(PxU32 i=0; i<count; i++)
													{
														const PoolIndex poolIndex = getIndex(handles[i]);
														PX_ASSERT(poolIndex!=INVALID_PRUNERHANDLE);
//														if(poolIndex!=INVALID_PRUNERHANDLE)
															Sq::inflateBounds(mWorldBoxes[poolIndex], newBounds[indices[i]]);
													}
												}

						void					preallocate(PxU32 entries);
//	protected:

						PxU32					mNbObjects;			//!< Current number of objects
						PxU32					mMaxNbObjects;		//!< Max. number of objects (capacity for mWorldBoxes, mObjects)

						//!< these arrays are parallel
						PxBounds3*				mWorldBoxes;		//!< List of world boxes, stores mNbObjects, capacity=mMaxNbObjects
						PrunerPayload*			mObjects;			//!< List of objects, stores mNbObjects, capacity=mMaxNbObjects
//	private:			
						PoolIndex*				mHandleToIndex;		//!< Maps from PrunerHandle to internal index (payload index in mObjects)
						PrunerHandle*			mIndexToHandle;		//!< Inverse map from objectIndex to PrunerHandle

				// this is the head of a list of holes formed in mHandleToIndex
				// by removed handles
				// the rest of the list is stored in holes in mHandleToIndex (in place)
						PrunerHandle			mFirstRecycledHandle;

						bool					resize(PxU32 newCapacity);
	};

} // namespace Sq

}

#endif // SQ_PRUNINGPOOL_H
