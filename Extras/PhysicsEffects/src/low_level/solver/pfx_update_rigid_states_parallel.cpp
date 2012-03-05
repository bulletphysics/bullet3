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

#include "../../../include/physics_effects/base_level/base/pfx_perf_counter.h"
#include "../../../include/physics_effects/base_level/solver/pfx_integrate.h"
#include "../../../include/physics_effects/low_level/solver/pfx_update_rigid_states.h"

namespace sce {
namespace PhysicsEffects {

///////////////////////////////////////////////////////////////////////////////
// This function is implemented in pfx_update_rigid_states_single.cpp
extern PfxInt32 pfxCheckParamOfUpdateRigidStates(const PfxUpdateRigidStatesParam &param);

///////////////////////////////////////////////////////////////////////////////
// MULTIPLE THREADS

//----------------------------------------------------------------------------
//  pfxUpdateRigidStatesTaskEntry
//
/// The thread PfxTaskEntry function used to update rigid body states in
/// parallel.
//----------------------------------------------------------------------------
void pfxUpdateRigidStatesTaskEntry(PfxTaskArg *arg)
{
	PfxUpdateRigidStatesParam &param = *((PfxUpdateRigidStatesParam*)arg->io);

	PfxUInt32 iFirstBody = arg->data[0];
	PfxUInt32 iEndBody = arg->data[1];

	for(PfxUInt32 i = iFirstBody; i < iEndBody; i++)
	{
		pfxIntegrate(param.states[i],param.bodies[i],param.timeStep);
	}
}

//----------------------------------------------------------------------------
//  pfxUpdateRigidStates
//
/// Perform update rigid states in parallel using a task manager.
///
/// @param param        Information about rigid bodies
/// @param taskManager  Pointer to the thread task manager to use
///
/// @return SCE_PFX_OK if successful, otherwise, returns an error code.
//----------------------------------------------------------------------------
PfxInt32 pfxUpdateRigidStates(PfxUpdateRigidStatesParam &param, PfxTaskManager *taskManager)
{
	PfxInt32 ret = pfxCheckParamOfUpdateRigidStates(param);
	if(ret != SCE_PFX_OK) return ret;

	SCE_PFX_PUSH_MARKER("pfxUpdateRigidStates");

	PfxUInt32 maxBatchSize = param.numRigidBodies / (PfxUInt32)(taskManager->getNumTasks());
	PfxUInt32 iEnd = maxBatchSize, iStart = 0;
	int task = 0;
	taskManager->setTaskEntry((void*)pfxUpdateRigidStatesTaskEntry);

	for (task = 0; task < taskManager->getNumTasks() - 1; task++, iStart += maxBatchSize, iEnd += maxBatchSize)
	{
		taskManager->startTask(task, static_cast<void*>(&param), iStart, iEnd, 0, 0);
	}

	// send final task
	iEnd = param.numRigidBodies;
	taskManager->startTask(taskManager->getNumTasks() - 1, static_cast<void*>(&param), iStart, iEnd, 0, 0);

	// wait for tasks to complete
	PfxUInt32 data1, data2, data3, data4;
	for (PfxUInt32 i = 0; i < taskManager->getNumTasks(); i++)
		taskManager->waitTask(task, data1, data2, data3, data4);

	SCE_PFX_POP_MARKER();
	
	return SCE_PFX_OK;
}

} //namespace PhysicsEffects
} //namespace sce
