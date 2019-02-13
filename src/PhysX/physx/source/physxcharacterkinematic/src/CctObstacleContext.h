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

#ifndef CCT_OBSTACLE_CONTEXT
#define CCT_OBSTACLE_CONTEXT

/* Exclude from documentation */
/** \cond */

#include "characterkinematic/PxControllerObstacles.h"
#include "PsUserAllocated.h"
#include "PsArray.h"
#include "CmPhysXCommon.h"

namespace physx
{
	struct PxRaycastHit;

namespace Cct
{
	class CharacterControllerManager;

    typedef PxU32  Handle;
	class HandleManager : public Ps::UserAllocated
	{
		public:
									HandleManager();
									~HandleManager();

						Handle		Add(void* object);
						void		Remove(Handle handle);
						void*		GetObject(Handle handle)	const;	// Returns object according to handle.
						bool		UpdateObject(Handle handle, void* object);

		PX_FORCE_INLINE	PxU32		GetMaxNbObjects()			const	{ return mMaxNbObjects;		}	//!< Returns max number of objects
		PX_FORCE_INLINE	PxU32		GetNbObjects()				const	{ return mCurrentNbObjects;	}	//!< Returns current number of objects
		PX_FORCE_INLINE	void**		GetObjects()				const	{ return mObjects;			}	//!< Gets the complete list of objects
		PX_FORCE_INLINE	void*		PickObject(Handle handle)	const	{ return mObjects[mOutToIn[PxU16(handle)]]; }

		private:
		// Physical list
						void**		mObjects;			//!< Physical list, with no holes but unsorted.
						PxU32		mCurrentNbObjects;	//!< Current number of objects in the physical list.
						PxU32		mMaxNbObjects;		//!< Maximum possible number of objects in the physical list.

		// Cross-references
						PxU16*		mOutToIn;			//!< Maps virtual indices (handles) to real ones.
						PxU16*		mInToOut;			//!< Maps real indices to virtual ones (handles).
						PxU16*      mStamps;

		// Recycled locations
						PxU32		mNbFreeIndices;		//!< Current number of free indices

		// Internal methods
						bool		SetupLists(void** objects=NULL, PxU16* oti=NULL, PxU16* ito=NULL, PxU16* stamps=NULL);
	};

	class ObstacleContext : public PxObstacleContext, public Ps::UserAllocated
	{
		public:
												ObstacleContext(CharacterControllerManager& );
		virtual									~ObstacleContext();

		// PxObstacleContext
		virtual	void							release();
		virtual PxControllerManager&			getControllerManager() const;
		virtual	ObstacleHandle					addObstacle(const PxObstacle& obstacle);
		virtual	bool							removeObstacle(ObstacleHandle handle);
		virtual	bool							updateObstacle(ObstacleHandle handle, const PxObstacle& obstacle);
		virtual	PxU32							getNbObstacles()		const;
		virtual	const PxObstacle*				getObstacle(PxU32 i)	const;
		virtual	const PxObstacle*				getObstacleByHandle(ObstacleHandle handle)	const;
		//~PxObstacleContext

				const PxObstacle*				raycastSingle(PxRaycastHit& hit, const PxVec3& origin, const PxVec3& unitDir, const PxReal distance, ObstacleHandle& obstacleHandle)	const;
				const PxObstacle*				raycastSingle(PxRaycastHit& hit, const ObstacleHandle& obstacleHandle, const PxVec3& origin, const PxVec3& unitDir, const PxReal distance)	const; // raycast just one obstacle handle

				void							onOriginShift(const PxVec3& shift);

				struct InternalBoxObstacle
				{
					InternalBoxObstacle(ObstacleHandle handle, const PxBoxObstacle& data) : mHandle(handle), mData(data)	{}

					ObstacleHandle	mHandle;
					PxBoxObstacle	mData;
				};
				Ps::Array<InternalBoxObstacle>	mBoxObstacles;

				struct InternalCapsuleObstacle
				{
					InternalCapsuleObstacle(ObstacleHandle handle, const PxCapsuleObstacle& data) : mHandle(handle), mData(data)	{}

					ObstacleHandle		mHandle;
					PxCapsuleObstacle	mData;
				};
				Ps::Array<InternalCapsuleObstacle>	mCapsuleObstacles;

	private:
				ObstacleContext&				operator=(const ObstacleContext&);
				HandleManager					mHandleManager;
				CharacterControllerManager&		mCCTManager;
	};


} // namespace Cct

}

/** \endcond */
#endif
