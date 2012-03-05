/*
Applied Research Associates Inc. (c)2011

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

#include "../../../include/physics_effects/low_level/collision/pfx_batched_ray_cast.h"
#include "../../../include/physics_effects/base_level/base/pfx_perf_counter.h"

namespace sce {
namespace PhysicsEffects {

///////////////////////////////////////////////////////////////////////////////
// MULTIPLE THREADS

//----------------------------------------------------------------------------
//  pfxCastRaysStartTaskEntry
//
/// The thread PfxTaskEntry function used to cast rays in parallel.
//----------------------------------------------------------------------------
void pfxCastRaysStartTaskEntry(PfxTaskArg *arg)
{
	PfxRayCastParam &param = *((PfxRayCastParam*)arg->io);

	PfxRayInput *rayInputs = (PfxRayInput*)arg->data[0];
	PfxRayOutput *rayOutputs = (PfxRayOutput*)arg->data[1];
	PfxUInt32 iFirstRay = arg->data[2];
	PfxUInt32 iEndRay = arg->data[3];

	for(PfxUInt32 i = iFirstRay; i < iEndRay; i++)
	{
		pfxCastSingleRay(rayInputs[i], rayOutputs[i], param);
	}
}

//----------------------------------------------------------------------------
//  pfxCastRays
//
/// Perform cast rays in parallel using a task manager.
///
/// @param rayInputs    [in] Array of rays to cast
/// @param rayOutputs   [out] On return contains output of ray casts
/// @param param        Information about ray cast
/// @param taskManager  Pointer to the thread task manager to use
//----------------------------------------------------------------------------
void pfxCastRays(PfxRayInput *rayInputs,PfxRayOutput *rayOutputs,int numRays,
	PfxRayCastParam &param,PfxTaskManager *taskManager)
{
	SCE_PFX_ALWAYS_ASSERT(SCE_PFX_PTR_IS_ALIGNED16(param.proxiesX));
	SCE_PFX_ALWAYS_ASSERT(SCE_PFX_PTR_IS_ALIGNED16(param.proxiesY));
	SCE_PFX_ALWAYS_ASSERT(SCE_PFX_PTR_IS_ALIGNED16(param.proxiesZ));
	SCE_PFX_ALWAYS_ASSERT(SCE_PFX_PTR_IS_ALIGNED16(param.proxiesXb));
	SCE_PFX_ALWAYS_ASSERT(SCE_PFX_PTR_IS_ALIGNED16(param.proxiesYb));
	SCE_PFX_ALWAYS_ASSERT(SCE_PFX_PTR_IS_ALIGNED16(param.proxiesZb));
	SCE_PFX_ALWAYS_ASSERT(SCE_PFX_PTR_IS_ALIGNED16(param.offsetRigidStates));
	SCE_PFX_ALWAYS_ASSERT(SCE_PFX_PTR_IS_ALIGNED16(param.offsetCollidables));

	SCE_PFX_PUSH_MARKER("pfxCastRays");
	
	PfxUInt32 maxBatchSize = numRays / (PfxUInt32)(taskManager->getNumTasks());
	PfxUInt32 iEnd = maxBatchSize, iStart = 0;
	int task = 0;
	taskManager->setTaskEntry((void*)pfxCastRaysStartTaskEntry);

	for (task = 0; task < taskManager->getNumTasks() - 1; task++, iStart += maxBatchSize, iEnd += maxBatchSize)
	{
		taskManager->startTask(task, static_cast<void*>(&param), (PfxUInt32)rayInputs, (PfxUInt32)rayOutputs, iStart, iEnd);
	}

	// send final task
	iEnd = numRays;
	taskManager->startTask(taskManager->getNumTasks() - 1, static_cast<void*>(&param), iStart, iEnd, 0, 0);

	// wait for tasks to complete
	PfxUInt32 data1, data2, data3, data4;
	for (PfxUInt32 i = 0; i < taskManager->getNumTasks(); i++)
		taskManager->waitTask(task, data1, data2, data3, data4);

	SCE_PFX_POP_MARKER();
}

} //namespace PhysicsEffects
} //namespace sce
