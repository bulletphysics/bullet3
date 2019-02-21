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


#ifndef PX_PHYSICS_ARTICULATION_SIM
#define PX_PHYSICS_ARTICULATION_SIM


#include "PsUserAllocated.h"
#include "CmPhysXCommon.h"
#include "DyArticulation.h"
#include "ScArticulationCore.h" 
#include "PxsSimpleIslandManager.h"

namespace physx
{

namespace Dy
{
	class Articulation;
}

class PxsTransformCache;
class PxsSimulationController;
class PxJoint;

namespace Cm
{
	class SpatialVector;
	template <class Allocator> class BitMapBase;
	typedef BitMapBase<Ps::NonTrackingAllocator> BitMap;
}

namespace Bp
{
	class BoundsArray;
}

namespace Sc
{

	class BodySim;
	class ArticulationJointSim;
	class ArticulationCore;
	class Scene;
	class ConstraintSim;


	class ArticulationSim : public Ps::UserAllocated 
	{
	public:
											ArticulationSim(ArticulationCore& core, 
												Scene& scene,
												BodyCore& root);

											~ArticulationSim();

		PX_INLINE	Dy::ArticulationV*		getLowLevelArticulation() const { return mLLArticulation; }
		PX_INLINE	ArticulationCore&		getCore() const { return mCore; }

								void		addBody(BodySim& body, 
													BodySim* parent, 
													ArticulationJointSim* joint);
								void		removeBody(BodySim &sim);
								
								Dy::ArticulationLinkHandle	getLinkHandle(BodySim& body) const;
								
								void		checkResize() const;						// resize LL memory if necessary
								
								void		debugCheckWakeCounterOfLinks(PxReal wakeCounter) const;
								void		debugCheckSleepStateOfLinks(bool isSleeping) const;

								bool		isSleeping() const;
								void		internalWakeUp(PxReal wakeCounter);	// called when sim sets sleep timer
								void		sleepCheck(PxReal dt);
								PxU32		getCCDLinks(BodySim** sims);
								void		updateCached(Cm::BitMapPinned* shapehapeChangedMap);
								void		markShapesUpdated(Cm::BitMapPinned* shapeChangedMap);
								void		updateContactDistance(PxReal* contactDistance, const PxReal dt, Bp::BoundsArray& boundsArray);

								void		setActive(const bool b, const PxU32 infoFlag=0);

								void		updateForces(PxReal dt, bool simUsesAdaptiveForce);
								void		saveLastCCDTransform();

	// drive cache implementation
	//
						ArticulationDriveCache*		
											createDriveCache(PxReal compliance,
															 PxU32 driveIterations) const;

						void				updateDriveCache(ArticulationDriveCache& cache,
															 PxReal compliance,
															 PxU32 driveIterations) const;

						void				releaseDriveCache(ArticulationDriveCache& cache) const;
						
						void				applyImpulse(BodyCore& link,
														 const ArticulationDriveCache& driveCache,
														 const PxVec3& force,
														 const PxVec3& torque);

						void				computeImpulseResponse(BodyCore& link,
																   PxVec3& linearResponse,
																   PxVec3& angularResponse,
																   const ArticulationDriveCache& driveCache,
																   const PxVec3& force,
																   const PxVec3& torque) const;
					//external reduced coordinate implementation
					PxU32					getDofs() const;

					//This function return the dof of the inbound joint, which belong to a link with corresponding linkID
					PxU32					getDof(const PxU32 linkID) const;

					PxArticulationCache*	createCache() const;

					PxU32					getCacheDataSize() const;

					PxU32					getCacheConstantDataSize() const;

					PxU32					getScratchMemorySize() const;

					void					zeroCache(PxArticulationCache&) const;

					void					applyCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag) const;

					void					copyInternalStateToCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag) const;

					void					releaseCache(PxArticulationCache&) const;

					void					packJointData(const PxReal* maximum, PxReal* reduced) const;

					void					unpackJointData(const PxReal* reduced, PxReal* maximum) const;

					void					commonInit();

					void					computeGeneralizedGravityForce(PxArticulationCache& cache);

					void					computeCoriolisAndCentrifugalForce(PxArticulationCache& cache);

					void					computeGeneralizedExternalForce(PxArticulationCache& cache);

					void					computeJointAcceleration(PxArticulationCache& cache);

					void					computeJointForce(PxArticulationCache& cache);

					void					computeKinematicJacobian(const PxU32 linkID, PxArticulationCache& cache);

					void					computeCoefficentMatrix(PxArticulationCache& cache);

					bool					computeLambda(PxArticulationCache& cache, PxArticulationCache& rollBackCache, const PxReal* jointTorque, const PxVec3 gravity, const PxU32 maxIter);

					void					computeGeneralizedMassMatrix(PxArticulationCache& cache);

					PxU32					getCoefficentMatrixSize() const;
					//internal method implementation
					PX_FORCE_INLINE IG::NodeIndex		getIslandNodeIndex() const { return mIslandNodeIndex; }

					void					setGlobalPose();

					void					setDirty(const bool dirty);
					PxU32					findBodyIndex(BodySim &body) const;

					void					addLoopConstraint(ConstraintSim* constraint);
					void					removeLoopConstraint(ConstraintSim* constraint);

					PxU32					getMaxDepth() { return mMaxDepth; }

	private:
					ArticulationSim&		operator=(const ArticulationSim&);

					Dy::ArticulationV*								mLLArticulation;
					Scene&											mScene;
					ArticulationCore&								mCore;
					Ps::Array<Dy::ArticulationLink>					mLinks;
					Ps::Array<BodySim*>								mBodies;
					Ps::Array<ArticulationJointSim*>				mJoints;
					IG::NodeIndex									mIslandNodeIndex;
					Ps::Array <Dy::ArticulationLoopConstraint>		mLoopConstraints;
					PxU32											mMaxDepth;
	};

} // namespace Sc

}

#endif
