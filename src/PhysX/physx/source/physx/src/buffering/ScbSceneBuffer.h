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


#ifndef PX_PHYSICS_SCB_SCENE_BUFFER
#define PX_PHYSICS_SCB_SCENE_BUFFER

#include "CmPhysXCommon.h"

#include "ScScene.h"

namespace physx
{
namespace Scb
{

struct SceneBuffer
{
public:
	static const PxU32 sMaxNbDominanceGroups = 32;

	PX_INLINE SceneBuffer();

	PX_INLINE void clearDominanceBuffer();
	PX_INLINE void setDominancePair(PxU32 group1, PxU32 group2, const PxDominanceGroupPair& dominance);
	PX_INLINE bool getDominancePair(PxU32 group1, PxU32 group2, PxDominanceGroupPair& dominance) const;
	PX_INLINE void syncDominancePairs(Sc::Scene& scene);

	PX_INLINE void clearVisualizationParams();

	PxReal								mVisualizationParam[PxVisualizationParameter::eNUM_VALUES];
	PxU8								mVisualizationParamChanged[PxVisualizationParameter::eNUM_VALUES];
	PxBounds3							mVisualizationCullingBox;
private:
	PxU32								mDominancePairFlag[sMaxNbDominanceGroups - 1];
	PxU32								mDominancePairValues[sMaxNbDominanceGroups];
public:
	PxVec3								mGravity;
	PxReal								mBounceThresholdVelocity;
	PxSceneFlags						mFlags;
	PxU32								mSolverBatchSize;
	PxU32								mNumClientsCreated;
};

PX_INLINE SceneBuffer::SceneBuffer() :
	mNumClientsCreated	(0)
{
	clearDominanceBuffer();
	clearVisualizationParams();
}

PX_FORCE_INLINE void SceneBuffer::clearDominanceBuffer()
{
	PxMemZero(&mDominancePairFlag, (sMaxNbDominanceGroups - 1) * sizeof(PxU32));
}

PX_FORCE_INLINE void SceneBuffer::clearVisualizationParams()
{
	PxMemZero(mVisualizationParamChanged, PxVisualizationParameter::eNUM_VALUES * sizeof(PxU8));
}

PX_INLINE void SceneBuffer::setDominancePair(PxU32 group1, PxU32 group2, const PxDominanceGroupPair& dominance)
{
	PX_ASSERT(group1 != group2);
	PX_ASSERT(group1 < sMaxNbDominanceGroups);
	PX_ASSERT(group2 < sMaxNbDominanceGroups);

	if(group1 < group2)
		mDominancePairFlag[group1] |= (1 << group2);
	else
		mDominancePairFlag[group2] |= (1 << group1);

	if(dominance.dominance0 != 0.0f)
		mDominancePairValues[group1] |= (1 << group2);
	else
		mDominancePairValues[group1] &= ~(1 << group2);

	if(dominance.dominance1 != 0.0f)
		mDominancePairValues[group2] |= (1 << group1);
	else
		mDominancePairValues[group2] &= ~(1 << group1);
}

PX_INLINE bool SceneBuffer::getDominancePair(PxU32 group1, PxU32 group2, PxDominanceGroupPair& dominance) const
{
	PX_ASSERT(group1 != group2);
	PX_ASSERT(group1 < sMaxNbDominanceGroups);
	PX_ASSERT(group2 < sMaxNbDominanceGroups);

	PxU32 isBuffered;
	if(group1 < group2)
		isBuffered = mDominancePairFlag[group1] & (1 << group2);
	else
		isBuffered = mDominancePairFlag[group2] & (1 << group1);

	if(!isBuffered)
		return false;

	dominance.dominance0 = PxU8((mDominancePairValues[group1] & (1 << group2)) >> group2);
	dominance.dominance1 = PxU8((mDominancePairValues[group2] & (1 << group1)) >> group1);
	return true;
}

PX_INLINE void SceneBuffer::syncDominancePairs(Sc::Scene& scene)
{
	for(PxU32 i=0; i<(sMaxNbDominanceGroups - 1); i++)
	{
		if(mDominancePairFlag[i])
		{
			for(PxU32 j=(i+1); j<sMaxNbDominanceGroups; j++)
			{
				PxDominanceGroupPair dominance(0, 0);
				if(getDominancePair(i, j, dominance))
					scene.setDominanceGroupPair(PxDominanceGroup(i), PxDominanceGroup(j), dominance);
			}
		}
	}

	clearDominanceBuffer();
}


}  // namespace Scb

}

#endif
