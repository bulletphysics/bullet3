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


#ifndef PXS_NPHASE_IMPLEMENTATION_CONTEXT_H
#define PXS_NPHASE_IMPLEMENTATION_CONTEXT_H

#include "PxvNphaseImplementationContext.h" 
#include "PxsContactManagerState.h"
#include "PxcNpCache.h"

namespace physx
{

struct PxsContactManagers : PxsContactManagerBase
{
	Ps::Array<PxsContactManagerOutput>			mOutputContactManagers;
	Ps::Array<PxsContactManager*>				mContactManagerMapping;
	Ps::Array<Gu::Cache>						mCaches;


	PxsContactManagers(const PxU32 bucketId) : PxsContactManagerBase(bucketId),
		mOutputContactManagers(PX_DEBUG_EXP("mOutputContactManagers")),
		mContactManagerMapping(PX_DEBUG_EXP("mContactManagerMapping")),
		mCaches(PX_DEBUG_EXP("mCaches"))
	{
	}
		
	void clear()
	{
		mOutputContactManagers.forceSize_Unsafe(0);
		mContactManagerMapping.forceSize_Unsafe(0);
		mCaches.forceSize_Unsafe(0);
		
	}
private:
	PX_NOCOPY(PxsContactManagers)
};


class PxsNphaseImplementationContext: public PxvNphaseImplementationContextUsableAsFallback
{
public:
	static PxsNphaseImplementationContext*	create(PxsContext& context, IG::IslandSim* islandSim);

	PxsNphaseImplementationContext(PxsContext& context, IG::IslandSim* islandSim, PxU32 index = 0): PxvNphaseImplementationContextUsableAsFallback(context), mNarrowPhasePairs(index), mNewNarrowPhasePairs(index),
										mModifyCallback(NULL), mIslandSim(islandSim) {}
	virtual void				destroy();
	virtual void				updateContactManager(PxReal dt, bool hasBoundsArrayChanged, bool hasContactDistanceChanged, PxBaseTask* continuation, PxBaseTask* firstPassContinuation);
	virtual void				postBroadPhaseUpdateContactManager() {}
	virtual void				secondPassUpdateContactManager(PxReal dt, PxBaseTask* continuation);

	virtual void				registerContactManager(PxsContactManager* cm, PxI32 touching, PxU32 numPatches);
	virtual void				registerContactManagers(PxsContactManager** cm, PxU32 nbContactManagers, PxU32 maxContactManagerId);
	virtual void				unregisterContactManager(PxsContactManager* cm);
	virtual void				unregisterContactManagerFallback(PxsContactManager* cm, PxsContactManagerOutput* cmOutputs);

	
	virtual void				refreshContactManager(PxsContactManager* cm);
	virtual void				refreshContactManagerFallback(PxsContactManager* cm, PxsContactManagerOutput* cmOutputs);

	virtual void				registerShape(const PxsShapeCore& shapeCore);

	virtual void				updateShapeMaterial(const PxsShapeCore& shapeCore);
	virtual void				updateShapeContactOffset(const PxsShapeCore& shapeCore);

	virtual void				unregisterShape(const PxsShapeCore& shapeCore);

	virtual void				registerMaterial(const PxsMaterialCore& materialCore);
	virtual void				updateMaterial(const PxsMaterialCore& materialCore);
	virtual void				unregisterMaterial(const PxsMaterialCore& materialCore);

	virtual void				appendContactManagers();
	virtual void				appendContactManagersFallback(PxsContactManagerOutput* cmOutputs);

	virtual void				removeContactManagersFallback(PxsContactManagerOutput* cmOutputs);

	virtual void				setContactModifyCallback(PxContactModifyCallback* callback) { mModifyCallback = callback; }

	virtual PxsContactManagerOutputIterator getContactManagerOutputs();

	virtual PxsContactManagerOutput& getNewContactManagerOutput(PxU32 npIndex);

	virtual PxsContactManagerOutput*	getGPUContactManagerOutputBase() { return NULL; }

	virtual void							acquireContext(){}
	virtual void							releaseContext(){}
	virtual void				preallocateNewBuffers(PxU32 /*nbNewPairs*/, PxU32 /*maxIndex*/) { /*TODO - implement if it's useful to do so*/}

	void						processContactManager(PxReal dt, PxsContactManagerOutput* cmOutputs, PxBaseTask* continuation);
	void						processContactManagerSecondPass(PxReal dt, PxBaseTask* continuation);
	void						fetchUpdateContactManager() {}

	

	void						startNarrowPhaseTasks() {}

	

	Ps::Array<PxU32>			mRemovedContactManagers;
	PxsContactManagers			mNarrowPhasePairs;
	PxsContactManagers			mNewNarrowPhasePairs;

	PxContactModifyCallback*	mModifyCallback;

	IG::IslandSim*				mIslandSim;

private:

	void						unregisterContactManagerInternal(PxU32 npIndex, PxsContactManagers& managers, PxsContactManagerOutput* cmOutputs);

	PX_NOCOPY(PxsNphaseImplementationContext)
};

}

#endif
