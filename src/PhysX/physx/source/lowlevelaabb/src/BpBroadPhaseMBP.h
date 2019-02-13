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

#ifndef BP_BROADPHASE_MBP_H
#define BP_BROADPHASE_MBP_H

#include "CmPhysXCommon.h"
#include "BpBroadPhase.h"
#include "BpBroadPhaseMBPCommon.h"
#include "BpMBPTasks.h"

	class MBP;
	
namespace physx
{
namespace Bp
{
	class BroadPhaseMBP : public BroadPhase, public Ps::UserAllocated
	{
											PX_NOCOPY(BroadPhaseMBP)
		public:
											BroadPhaseMBP(	PxU32 maxNbRegions,
															PxU32 maxNbBroadPhaseOverlaps,
															PxU32 maxNbStaticShapes,
															PxU32 maxNbDynamicShapes,
															PxU64 contextID);
		virtual								~BroadPhaseMBP();

	// BroadPhaseBase
		virtual	bool						getCaps(PxBroadPhaseCaps& caps)														const;
		virtual	PxU32						getNbRegions()																		const;
		virtual	PxU32						getRegions(PxBroadPhaseRegionInfo* userBuffer, PxU32 bufferSize, PxU32 startIndex=0) const;
		virtual	PxU32						addRegion(const PxBroadPhaseRegion& region, bool populateRegion);
		virtual	bool						removeRegion(PxU32 handle);
		virtual	PxU32						getNbOutOfBoundsObjects()	const;
		virtual	const PxU32*				getOutOfBoundsObjects()		const;
	//~BroadPhaseBase

	// BroadPhase
		virtual	PxBroadPhaseType::Enum		getType()					const	{ return PxBroadPhaseType::eMBP;	}
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

				MBPUpdateWorkTask			mMBPUpdateWorkTask;
				MBPPostUpdateWorkTask		mMBPPostUpdateWorkTask;

				MBP*						mMBP;		// PT: TODO: aggregate

				MBP_Handle*					mMapping;
				PxU32						mCapacity;
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
				void						allocateMappingArray(PxU32 newCapacity);

				PxU32						getCurrentNbPairs()	const;
	};

} //namespace Bp

} //namespace physx

#endif // BP_BROADPHASE_MBP_H
