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
#include "../../../include/physics_effects/base_level/sort/pfx_sort.h"
#include "../../../include/physics_effects/low_level/collision/pfx_refresh_contacts.h"

namespace sce {
namespace PhysicsEffects {

///////////////////////////////////////////////////////////////////////////////
// This function is implemented in pfx_refresh_contacts_single.cpp
extern int pfxCheckParamOfRefreshContacts(PfxRefreshContactsParam &param);

///////////////////////////////////////////////////////////////////////////////
// MULTIPLE THREADS

//----------------------------------------------------------------------------
//  pfxRefreshContactsTaskEntry
//
/// The thread PfxTaskEntry function used to perform refresh contacts in
/// parallel
//----------------------------------------------------------------------------
void pfxRefreshContactsTaskEntry(PfxTaskArg *arg)
{
	PfxRefreshContactsParam &param = *((PfxRefreshContactsParam*)arg->io);
	
	PfxUInt32 iFirstContactPair = arg->data[0];
	PfxUInt32 iEndContactPair = arg->data[1];

	PfxConstraintPair *contactPairs = param.contactPairs;
	PfxContactManifold *offsetContactManifolds = param.offsetContactManifolds;
	PfxRigidState *offsetRigidStates = param.offsetRigidStates;
	PfxUInt32 numRigidBodies = param.numRigidBodies;
	
	for(PfxUInt32 i = iFirstContactPair; i < iEndContactPair; i++)
	{
		PfxBroadphasePair &pair = contactPairs[i];
		
		PfxUInt32 iContact = pfxGetContactId(pair);
		PfxUInt32 iA = pfxGetObjectIdA(pair);
		PfxUInt32 iB = pfxGetObjectIdB(pair);

		PfxContactManifold &contact = offsetContactManifolds[iContact];

		SCE_PFX_ALWAYS_ASSERT(iA==contact.getRigidBodyIdA());
		SCE_PFX_ALWAYS_ASSERT(iB==contact.getRigidBodyIdB());

		PfxRigidState &instA = offsetRigidStates[iA];
		PfxRigidState &instB = offsetRigidStates[iB];
		
		contact.refresh(
			instA.getPosition(),instA.getOrientation(),
			instB.getPosition(),instB.getOrientation() );
	}
}

//----------------------------------------------------------------------------
//  pfxRefreshContacts
//
/// Perform refresh contacts in parallel using a task manager.
///
/// @param param        Information about contact pairs
/// @param taskManager  Pointer to the thread task manager to use
///
/// @return SCE_PFX_OK if successful, otherwise, returns an error code.
//----------------------------------------------------------------------------
PfxInt32 pfxRefreshContacts(PfxRefreshContactsParam &param, PfxTaskManager *taskManager)
{
	PfxInt32 ret = pfxCheckParamOfRefreshContacts(param);
	if(ret != SCE_PFX_OK) return ret;

	SCE_PFX_PUSH_MARKER("pfxRefreshContacts");

	PfxUInt32 maxBatchSize = param.numContactPairs / (PfxUInt32)(taskManager->getNumTasks());
	PfxUInt32 iEnd = maxBatchSize, iStart = 0;
	int task = 0;
	taskManager->setTaskEntry((void*)pfxRefreshContactsTaskEntry);

	for (task = 0; task < taskManager->getNumTasks() - 1; task++, iStart += maxBatchSize, iEnd += maxBatchSize)
	{
		taskManager->startTask(task, static_cast<void*>(&param), iStart, iEnd, 0, 0);
	}

	// send final task
	iEnd = param.numContactPairs;
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
