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

#ifndef CCT_CONTROLLER
#define CCT_CONTROLLER

/* Exclude from documentation */
/** \cond */

#include "CctCharacterController.h"
#include "PsUserAllocated.h"
#include "PsMutex.h"

namespace physx
{

class PxPhysics;
class PxScene;
class PxRigidDynamic;
class PxGeometry;
class PxMaterial;

namespace Cct
{
	class CharacterControllerManager;

	class Controller : public Ps::UserAllocated
	{
		PX_NOCOPY(Controller)
	public:
														Controller(const PxControllerDesc& desc, PxScene* scene);
		virtual											~Controller();

					void								releaseInternal();
					void								getInternalState(PxControllerState& state)	const;
					void								getInternalStats(PxControllerStats& stats)	const;					

		virtual		PxF32								getHalfHeightInternal()				const	= 0;
		virtual		bool								getWorldBox(PxExtendedBounds3& box)	const	= 0;
		virtual		PxController*						getPxController()							= 0;

					void								onOriginShift(const PxVec3& shift);

					void								onRelease(const PxBase& observed);

					void								setCctManager(CharacterControllerManager* cm)
					{
						mManager = cm;
						mCctModule.setCctManager(cm);
					}

	PX_FORCE_INLINE	CharacterControllerManager*			getCctManager()				{ return mManager;									}
	PX_FORCE_INLINE	PxU64								getContextId()		const	{ return PxU64(reinterpret_cast<size_t>(mScene));	}

					PxControllerShapeType::Enum			mType;
		// User params
					CCTParams							mUserParams;
					PxUserControllerHitReport*			mReportCallback;
					PxControllerBehaviorCallback*		mBehaviorCallback;
					void*								mUserData;
		// Internal data
					SweepTest							mCctModule;			// Internal CCT object. Optim test for Ubi.
					PxRigidDynamic*						mKineActor;			// Associated kinematic actor
					PxExtendedVec3						mPosition;			// Current position
					PxVec3								mDeltaXP;
					PxVec3								mOverlapRecover;
					PxScene*							mScene;				// Handy scene owner
					PxU32								mPreviousSceneTimestamp;					
					PxF64								mGlobalTime;
					PxF64								mPreviousGlobalTime;
					PxF32								mProxyDensity;		// Density for proxy actor
					PxF32								mProxyScaleCoeff;	// Scale coeff for proxy actor
					PxControllerCollisionFlags			mCollisionFlags;	// Last known collision flags (PxControllerCollisionFlag)
					bool								mCachedStandingOnMoving;
					bool								mRegisterDeletionListener;
		mutable		Ps::Mutex							mWriteLock;			// Lock used for guarding touched pointers and cache data from overwriting 
																			// during onRelease call.
	protected:
		// Internal methods
					void								setUpDirectionInternal(const PxVec3& up);
					PxShape*							getKineShape()	const;
					bool								createProxyActor(PxPhysics& sdk, const PxGeometry& geometry, const PxMaterial& material);
					bool								setPos(const PxExtendedVec3& pos);
					void								findTouchedObject(const PxControllerFilters& filters, const PxObstacleContext* obstacleContext, const PxVec3& upDirection);
					bool								rideOnTouchedObject(SweptVolume& volume, const PxVec3& upDirection, PxVec3& disp, const PxObstacleContext* obstacleContext);
					PxControllerCollisionFlags			move(SweptVolume& volume, const PxVec3& disp, PxF32 minDist, PxF32 elapsedTime, const PxControllerFilters& filters, const PxObstacleContext* obstacles, bool constrainedClimbingMode);
					bool								filterTouchedShape(const PxControllerFilters& filters);

	PX_FORCE_INLINE	float								computeTimeCoeff()
														{
															const float elapsedTime = float(mGlobalTime - mPreviousGlobalTime);
															mPreviousGlobalTime = mGlobalTime;
															return 1.0f / elapsedTime;
														}

					CharacterControllerManager*			mManager;			// Owner manager
	};

} // namespace Cct

}

/** \endcond */
#endif
