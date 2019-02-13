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


#ifndef PX_PHYSICS_SCP_ARTICULATION_CORE
#define PX_PHYSICS_SCP_ARTICULATION_CORE

#include "ScActorCore.h"
#include "DyArticulation.h"
#include "DyFeatherstoneArticulation.h"

namespace physx
{

class PxvArticulation;

namespace IG
{
	class NodeIndex;
}


namespace Sc
{
	typedef Dy::FsData ArticulationDriveCache;

	class ArticulationSim;
	class BodyCore;
	class BodySim;

	class ArticulationCore
	{
	//= ATTENTION! =====================================================================================
	// Changing the data layout of this class breaks the binary serialization format.  See comments for 
	// PX_BINARY_SERIAL_VERSION.  If a modification is required, please adjust the getBinaryMetaData 
	// function.  If the modification is made on a custom branch, please change PX_BINARY_SERIAL_VERSION
	// accordingly.
	//==================================================================================================

		//---------------------------------------------------------------------------------
		// Construction, destruction & initialization
		//---------------------------------------------------------------------------------

// PX_SERIALIZATION
		public:
													ArticulationCore(const PxEMPTY) : mSim(NULL)	{}
		static		void							getBinaryMetaData(PxOutputStream& stream);
//~PX_SERIALIZATION
													ArticulationCore();
													~ArticulationCore();

		//---------------------------------------------------------------------------------
		// External API
		//---------------------------------------------------------------------------------
						PxU32						getInternalDriveIterations() const; 
						void						setInternalDriveIterations(const PxU32 v);

						PxU32						getExternalDriveIterations() const; 
						void						setExternalDriveIterations(const PxU32 v);

						PxU32						getMaxProjectionIterations() const; 
						void						setMaxProjectionIterations(const PxU32 v);

						PxReal						getSeparationTolerance() const; 
						void						setSeparationTolerance(const PxReal v);

						PxReal						getSleepThreshold() const; 
						void						setSleepThreshold(const PxReal v);

						PxReal						getFreezeThreshold() const; 
						void						setFreezeThreshold(const PxReal v);

						PxReal						getWakeCounter() const; 
						void						setWakeCounter(const PxReal v);
						void						setWakeCounterInternal(const PxReal v);

						bool						isSleeping() const;
						void						wakeUp(PxReal wakeCounter);
						void						putToSleep();

						PxU16						getSolverIterationCounts()	const;
						void						setSolverIterationCounts(PxU16 c);

						PxArticulation*				getPxArticulation();
						const PxArticulation*		getPxArticulation() const;


		//---------------------------------------------------------------------------------
		// Drive Cache API
		//---------------------------------------------------------------------------------
						ArticulationDriveCache*		createDriveCache(PxReal compliance,
																	 PxU32 driveIterations) const;

						void						updateDriveCache(ArticulationDriveCache& cache,
																	 PxReal compliance,
																	 PxU32 driveIterations) const;

						void						releaseDriveCache(ArticulationDriveCache& cache) const;

						PxU32						getCacheLinkCount(const ArticulationDriveCache& cache) const;

						void						applyImpulse(BodyCore& link,
																 const ArticulationDriveCache& driveCache,
																 const PxVec3& force,
																 const PxVec3& torque);

						void						computeImpulseResponse(BodyCore& link,
																		   PxVec3& linearResponse, 
																		   PxVec3& angularResponse,
																		   const ArticulationDriveCache& driveCache,
																		   const PxVec3& force,
																		   const PxVec3& torque) const;

		//---------------------------------------------------------------------------------
		// external reduced coordinate API
		//---------------------------------------------------------------------------------
						void						setArticulationFlags(PxArticulationFlags flags) { mCore.flags = flags; }
						PxArticulationFlags			getArticulationFlags() const { return mCore.flags; }

						PxU32						getDofs() const;

						PxArticulationCache*		createCache() const;

						PxU32						getCacheDataSize() const;

						void						zeroCache(PxArticulationCache& cache) const;

						void						applyCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag)const;
		
						void						copyInternalStateToCache(PxArticulationCache& cache, const PxArticulationCacheFlags flag) const;

						void						releaseCache(PxArticulationCache& cache) const;

						void						packJointData(const PxReal* maximum, PxReal* reduced) const;

						void						unpackJointData(const PxReal* reduced, PxReal* maximum) const;

						void						commonInit() const;

						void						computeGeneralizedGravityForce(PxArticulationCache& cache) const;

						void						computeCoriolisAndCentrifugalForce(PxArticulationCache& cache) const;

						void						computeGeneralizedExternalForce(PxArticulationCache& cache) const;

						void						computeJointAcceleration(PxArticulationCache& cache) const;

						void						computeJointForce(PxArticulationCache& cache) const;

						void						computeKinematicJacobian(const PxU32 linkID, PxArticulationCache& cache) const;

						void						computeCoefficentMatrix(PxArticulationCache& cache) const;

						bool						computeLambda(PxArticulationCache& cache, PxArticulationCache& rollBackCache, const PxReal* const jointTorque, const PxVec3 gravity, const PxU32 maxIter) const;

						void						computeGeneralizedMassMatrix(PxArticulationCache& cache) const;

						PxU32						getCoefficentMatrixSize() const;
		//---------------------------------------------------------------------------------
		// Internal API
		//---------------------------------------------------------------------------------
	public:
		PX_FORCE_INLINE	void						setSim(ArticulationSim* sim)
													{
														PX_ASSERT((sim==0) ^ (mSim == 0));
														mSim = sim;
													}
		PX_FORCE_INLINE	ArticulationSim*			getSim()			const	{ return mSim;			}

		PX_FORCE_INLINE	const Dy::ArticulationCore&	getCore()					{ return mCore;			}

		static PX_FORCE_INLINE ArticulationCore&	getArticulationCore(ArticulationCore& core)
		{
			size_t offset = PX_OFFSET_OF(ArticulationCore, mCore);
			return *reinterpret_cast<ArticulationCore*>(reinterpret_cast<PxU8*>(&core) - offset);
		}

		PX_INLINE		PxArticulationBase::Enum		getArticulationType() const { return PxArticulationBase::Enum(mType); }
		PX_INLINE		void							setArticulationType(PxArticulationBase::Enum type) { mType = type; }


		IG::NodeIndex getIslandNodeIndex() const;

		void setGlobalPose();

		void setDirty(const bool dirty);


	private:
						ArticulationSim*				mSim;
						Dy::ArticulationCore			mCore;
						PxU32							mType;
	};



} // namespace Sc

}

#endif
