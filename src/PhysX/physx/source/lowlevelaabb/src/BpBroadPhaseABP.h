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

#ifndef BP_BROADPHASE_ABP_H
#define BP_BROADPHASE_ABP_H

#include "CmPhysXCommon.h"
#include "BpBroadPhase.h"
#include "PxPhysXConfig.h"
#include "BpBroadPhaseUpdate.h"
#include "PsUserAllocated.h"

namespace internalABP{
	class ABP;
}

namespace physx
{
namespace Bp
{
	class BroadPhaseABP : public BroadPhase, public Ps::UserAllocated
	{
											PX_NOCOPY(BroadPhaseABP)
		public:
											BroadPhaseABP(	PxU32 maxNbBroadPhaseOverlaps,
															PxU32 maxNbStaticShapes,
															PxU32 maxNbDynamicShapes,
															PxU64 contextID);
		virtual								~BroadPhaseABP();

	// BroadPhase
		virtual	PxBroadPhaseType::Enum		getType()					const	{ return PxBroadPhaseType::eABP;	}
		virtual	void						destroy()							{ delete this;						}
		virtual	void						update(const PxU32 numCpuTasks, PxcScratchAllocator* scratchAllocator, const BroadPhaseUpdateData& updateData, physx::PxBaseTask* continuation, physx::PxBaseTask* narrowPhaseUnblockTask);
		virtual void						fetchBroadPhaseResults(physx::PxBaseTask*) {}
		virtual	PxU32						getNbCreatedPairs()		const;
		virtual BroadPhasePair*				getCreatedPairs();
		virtual PxU32						getNbDeletedPairs()		const;
		virtual BroadPhasePair*				getDeletedPairs();
		virtual void						freeBuffers();
		virtual void						shiftOrigin(const PxVec3& shift);
#if PX_CHECKED
		virtual bool						isValid(const BroadPhaseUpdateData& updateData)	const;
#endif
		virtual BroadPhasePair*				getBroadPhasePairs() const  {return NULL;}  //KS - TODO - implement this!!!
		virtual void						deletePairs(){}								//KS - TODO - implement this!!!
		virtual	void						singleThreadedUpdate(PxcScratchAllocator* scratchAllocator, const BroadPhaseUpdateData& updateData);
	//~BroadPhase

		internalABP::ABP*					mABP;		// PT: TODO: aggregate

				Ps::Array<BroadPhasePair>	mCreated;
				Ps::Array<BroadPhasePair>	mDeleted;

				const Bp::FilterGroup::Enum*mGroups;
#ifdef BP_FILTERING_USES_TYPE_IN_GROUP
				const bool*					mLUT;
#endif
				void						setUpdateData(const BroadPhaseUpdateData& updateData);
				void						addObjects(const BroadPhaseUpdateData& updateData);
				void						removeObjects(const BroadPhaseUpdateData& updateData);
				void						updateObjects(const BroadPhaseUpdateData& updateData);

				void						update();
				void						postUpdate();

				PxU32						getCurrentNbPairs()	const;
				void						setScratchAllocator(PxcScratchAllocator* scratchAllocator);
	};

} //namespace Bp

} //namespace physx

#endif // BP_BROADPHASE_ABP_H
