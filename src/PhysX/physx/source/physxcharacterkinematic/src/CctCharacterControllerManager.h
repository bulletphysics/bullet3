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

#ifndef CCT_CHARACTER_CONTROLLER_MANAGER
#define CCT_CHARACTER_CONTROLLER_MANAGER

//Exclude file from docs
/** \cond */

#include "PxControllerManager.h"
#include "PxControllerObstacles.h"
#include "PxMeshQuery.h"
#include "PxDeletionListener.h"
#include "CmRenderOutput.h"
#include "CctUtils.h"
#include "PsHashSet.h"
#include "PsHashMap.h"
#include "PsMutex.h"

namespace physx
{
namespace Cct
{
	class Controller;
	class ObstacleContext;

	struct ObservedRefCounter
	{
		ObservedRefCounter(): refCount(0)
		{
		}

		PxU32 refCount;
	};

	typedef Ps::HashMap<const PxBase*, ObservedRefCounter>			ObservedRefCountMap;

	//Implements the PxControllerManager interface, this class used to be called ControllerManager
	class CharacterControllerManager : public PxControllerManager   , public Ps::UserAllocated, public PxDeletionListener
	{		
	public:
														CharacterControllerManager(PxScene& scene, bool lockingEnabled = false);
		virtual											~CharacterControllerManager();

		// PxControllerManager
		virtual			void							release();
		virtual			PxScene&						getScene() const;
		virtual			PxU32							getNbControllers()	const;
		virtual			PxController*					getController(PxU32 index);
        virtual			PxController*					createController(const PxControllerDesc& desc);
       
		virtual			void							purgeControllers();
		virtual			PxRenderBuffer&					getRenderBuffer();
		virtual			void							setDebugRenderingFlags(PxControllerDebugRenderFlags flags);
		virtual			PxU32							getNbObstacleContexts() const;
		virtual			PxObstacleContext*				getObstacleContext(PxU32 index);
		virtual			PxObstacleContext*				createObstacleContext();
		virtual			void							computeInteractions(PxF32 elapsedTime, PxControllerFilterCallback* cctFilterCb);
		virtual			void							setTessellation(bool flag, float maxEdgeLength);
		virtual			void							setOverlapRecoveryModule(bool flag);
		virtual			void							setPreciseSweeps(bool flag);
		virtual			void							setPreventVerticalSlidingAgainstCeiling(bool flag);
		virtual			void							shiftOrigin(const PxVec3& shift);		
		//~PxControllerManager

		// PxDeletionListener
		virtual		void								onRelease(const PxBase* observed, void* userData, PxDeletionEventFlag::Enum deletionEvent);
		//~PxDeletionListener
						void							registerObservedObject(const PxBase* obj);
						void							unregisterObservedObject(const PxBase* obj);

		// ObstacleContextNotifications
						void							onObstacleRemoved(ObstacleHandle index) const;
						void							onObstacleUpdated(ObstacleHandle index, const PxObstacleContext* ) const;
						void							onObstacleAdded(ObstacleHandle index, const PxObstacleContext*) const;

						void							releaseController(PxController& controller);
						Controller**					getControllers();
						void							releaseObstacleContext(ObstacleContext& oc);
						void							resetObstaclesBuffers();

						PxScene&						mScene;

						Cm::RenderBuffer*				mRenderBuffer;
						PxControllerDebugRenderFlags	mDebugRenderingFlags;
		// Shared buffers for obstacles
						Ps::Array<const void*>			mBoxUserData;
						Ps::Array<PxExtendedBox>		mBoxes;

						Ps::Array<const void*>			mCapsuleUserData;
						Ps::Array<PxExtendedCapsule>	mCapsules;

						Ps::Array<Controller*>			mControllers;
						Ps::HashSet<PxShape*>			mCCTShapes;

						Ps::Array<ObstacleContext*>		mObstacleContexts;

						float							mMaxEdgeLength;
						bool							mTessellation;

						bool							mOverlapRecovery;
						bool							mPreciseSweeps;
						bool							mPreventVerticalSlidingAgainstCeiling;

						bool							mLockingEnabled;						

	protected:
		CharacterControllerManager &operator=(const CharacterControllerManager &);
		CharacterControllerManager(const CharacterControllerManager& );

	private:
						ObservedRefCountMap				mObservedRefCountMap;
						mutable		Ps::Mutex			mWriteLock;			// Lock used for guarding pointers in observedrefcountmap
	};

} // namespace Cct

}

/** \endcond */
#endif //CCT_CHARACTER_CONTROLLER_MANAGER
