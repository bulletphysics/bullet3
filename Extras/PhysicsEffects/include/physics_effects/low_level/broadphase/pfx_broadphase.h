/*
Physics Effects Copyright(C) 2010 Sony Computer Entertainment Inc.
All rights reserved.

Physics Effects is open software; you can redistribute it and/or
modify it under the terms of the BSD License.

Physics Effects is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the BSD License for more details.

A copy of the BSD License is distributed with
Physics Effects under the filename: physics_effects_license.txt
*/

#ifndef _SCE_PFX_BROADPHASE_H_
#define _SCE_PFX_BROADPHASE_H_

#include "../../base_level/broadphase/pfx_broadphase_pair.h"
#include "../../base_level/broadphase/pfx_broadphase_proxy.h"
#include "../../base_level/rigidbody/pfx_rigid_state.h"
#include "../../base_level/collision/pfx_collidable.h"
#include "../task/pfx_task_manager.h"

namespace sce {
namespace PhysicsEffects {

///////////////////////////////////////////////////////////////////////////////
// Update Broadphase Proxies

#define SCE_PFX_OUT_OF_WORLD_BEHAVIOR_FIX_MOTION   0x01
#define SCE_PFX_OUT_OF_WORLD_BEHAVIOR_REMOVE_PROXY 0x02

struct PfxUpdateBroadphaseProxiesParam {
	void *workBuff;
	PfxUInt32 workBytes;
	PfxBroadphaseProxy *proxiesX;
	PfxBroadphaseProxy *proxiesY;
	PfxBroadphaseProxy *proxiesZ;
	PfxBroadphaseProxy *proxiesXb;
	PfxBroadphaseProxy *proxiesYb;
	PfxBroadphaseProxy *proxiesZb;
	PfxRigidState *offsetRigidStates;
	PfxCollidable *offsetCollidables;
	PfxUInt32 numRigidBodies;
	PfxUInt32 outOfWorldBehavior;
	PfxVector3 worldCenter;
	PfxVector3 worldExtent;
	
	PfxUpdateBroadphaseProxiesParam() : outOfWorldBehavior(0) {}
};

struct PfxUpdateBroadphaseProxiesResult {
	PfxInt32 numOutOfWorldProxies;
};

PfxUInt32 pfxGetWorkBytesOfUpdateBroadphaseProxies(PfxUInt32 numRigidBodies);

PfxUInt32 pfxGetWorkBytesOfUpdateBroadphaseProxies(PfxUInt32 numRigidBodies,PfxUInt32 maxTasks);

PfxInt32 pfxUpdateBroadphaseProxies(PfxUpdateBroadphaseProxiesParam &param,PfxUpdateBroadphaseProxiesResult &result);

PfxInt32 pfxUpdateBroadphaseProxies(PfxUpdateBroadphaseProxiesParam &param,PfxUpdateBroadphaseProxiesResult &result,PfxTaskManager *taskManager);

///////////////////////////////////////////////////////////////////////////////
// Find Pairs

struct PfxFindPairsParam {
	void *workBuff;
	PfxUInt32 workBytes;
	void *pairBuff;
	PfxUInt32 pairBytes;
	PfxBroadphaseProxy *proxies;
	PfxUInt32 numProxies;
	PfxUInt32 maxPairs;
	int axis;
};

struct PfxFindPairsResult {
	PfxBroadphasePair *pairs;
	PfxUInt32 numPairs;
};

PfxUInt32 pfxGetWorkBytesOfFindPairs(PfxUInt32 maxPairs,PfxUInt32 maxTasks=1);
PfxUInt32 pfxGetPairBytesOfFindPairs(PfxUInt32 maxPairs);

PfxInt32 pfxFindPairs(PfxFindPairsParam &param,PfxFindPairsResult &result);

PfxInt32 pfxFindPairs(PfxFindPairsParam &param,PfxFindPairsResult &result,PfxTaskManager *taskManager);

///////////////////////////////////////////////////////////////////////////////
// Decompose Pairs

struct PfxDecomposePairsParam {
	void *workBuff;
	PfxUInt32 workBytes;
	void *pairBuff;
	PfxUInt32 pairBytes;
	PfxBroadphasePair *previousPairs;
	PfxUInt32 numPreviousPairs;
	PfxBroadphasePair *currentPairs;
	PfxUInt32 numCurrentPairs;
};

struct PfxDecomposePairsResult {
	PfxBroadphasePair *outNewPairs;
	PfxUInt32 numOutNewPairs;
	PfxBroadphasePair *outKeepPairs;
	PfxUInt32 numOutKeepPairs;
	PfxBroadphasePair *outRemovePairs;
	PfxUInt32 numOutRemovePairs;
};

PfxUInt32 pfxGetWorkBytesOfDecomposePairs(PfxUInt32 numPreviousPairs,PfxUInt32 numCurrentPairs,int maxTasks=1);
PfxUInt32 pfxGetPairBytesOfDecomposePairs(PfxUInt32 numPreviousPairs,PfxUInt32 numCurrentPairs);

PfxInt32 pfxDecomposePairs(PfxDecomposePairsParam &param,PfxDecomposePairsResult &result);

PfxInt32 pfxDecomposePairs(PfxDecomposePairsParam &param,PfxDecomposePairsResult &result,PfxTaskManager *taskManager);

} //namespace PhysicsEffects
} //namespace sce
#endif /* _SCE_PFX_BROADPHASE_H_ */
