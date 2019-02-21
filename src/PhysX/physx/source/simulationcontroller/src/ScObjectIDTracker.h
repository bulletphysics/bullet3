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


#ifndef PX_PHYSICS_SC_OBJECT_ID_TRACKER
#define PX_PHYSICS_SC_OBJECT_ID_TRACKER

#include "CmPhysXCommon.h"
#include "CmIDPool.h"
#include "CmBitMap.h"
#include "PsUserAllocated.h"

namespace physx
{
namespace Sc
{
	// PT: TODO: this has no direct dependency on "Sc". It should really be a "Cm" class.
	class ObjectIDTracker : public Ps::UserAllocated
	{
		PX_NOCOPY(ObjectIDTracker)
	public:
		ObjectIDTracker() : mPendingReleasedIDs(PX_DEBUG_EXP("objectIDTrackerIDs")) {}

		PX_INLINE PxU32			createID()						{ return mIDPool.getNewID();								}
		PX_INLINE void			releaseID(PxU32 id)				
		{ 
			markIDAsDeleted(id); 
			mPendingReleasedIDs.pushBack(id);	
		}
		PX_INLINE Ps::IntBool	isDeletedID(PxU32 id)	const	{ return mDeletedIDsMap.boundedTest(id);					}
		PX_FORCE_INLINE PxU32	getDeletedIDCount()		const	{ return mPendingReleasedIDs.size();						}
		PX_INLINE void			clearDeletedIDMap()				{ mDeletedIDsMap.clear();									}
		PX_INLINE void			resizeDeletedIDMap(PxU32 id, PxU32 numIds)	
		{ 
			mDeletedIDsMap.resize(id); 
			mPendingReleasedIDs.reserve(numIds);
		}
		PX_INLINE void			processPendingReleases()
		{
			for(PxU32 i=0; i < mPendingReleasedIDs.size(); i++)
			{
				 mIDPool.freeID(mPendingReleasedIDs[i]);
			}
			mPendingReleasedIDs.clear();
		}
		PX_INLINE void reset()
		{
			processPendingReleases();
			mPendingReleasedIDs.reset();

			// Don't free stuff in IDPool, we still need the list of free IDs

			// And it does not seem worth freeing the memory of the bitmap
		}

		PX_INLINE PxU32 getMaxID()
		{
			return mIDPool.getMaxID();
		}
	private:
		PX_INLINE void markIDAsDeleted(PxU32 id) { PX_ASSERT(!isDeletedID(id)); mDeletedIDsMap.growAndSet(id); }


	private:
		Cm::IDPool					mIDPool;
		Cm::BitMap					mDeletedIDsMap;
		Ps::Array<PxU32>			mPendingReleasedIDs;  // Buffer for released IDs to make sure newly created objects do not re-use these IDs immediately
	};

}
}

#endif
