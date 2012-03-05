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
#include "../../../include/physics_effects/base_level/collision/pfx_shape_iterator.h"
#include "../../../include/physics_effects/low_level/collision/pfx_collision_detection.h"
#include "../../base_level/broadphase/pfx_check_collidable.h"
#include "../../base_level/collision/pfx_contact_cache.h"
#include "pfx_detect_collision_func.h"

namespace sce {
namespace PhysicsEffects {

///////////////////////////////////////////////////////////////////////////////
// This function is implemented in pfx_collision_detection_single.cpp
extern int pfxCheckParamOfDetectCollision(PfxDetectCollisionParam &param);

///////////////////////////////////////////////////////////////////////////////
// MULTIPLE THREADS

#define SCE_PFX_CONTACT_THRESHOLD 0.0f

//----------------------------------------------------------------------------
//  pfxDetectCollisionTaskEntry
//
/// The thread PfxTaskEntry function used to perform narrow phase collision
/// detection in parallel.
//----------------------------------------------------------------------------
void pfxDetectCollisionTaskEntry(PfxTaskArg *arg)
{
	PfxDetectCollisionParam &param = *((PfxDetectCollisionParam*)arg->io);

	PfxUInt32 iFirstContactPair = arg->data[0];
	PfxUInt32 iEndContactPair = arg->data[1];

	PfxConstraintPair *contactPairs = param.contactPairs;
	PfxContactManifold *offsetContactManifolds = param.offsetContactManifolds;
	PfxRigidState *offsetRigidStates = param.offsetRigidStates;
	PfxCollidable *offsetCollidables = param.offsetCollidables;
	PfxUInt32 numRigidBodies = param.numRigidBodies;

	for(PfxUInt32 i = iFirstContactPair; i < iEndContactPair; i++)
	{
		const PfxBroadphasePair &pair = contactPairs[i];
		if(!pfxCheckCollidableInCollision(pair))
			continue;

		PfxUInt32 iContact = pfxGetContactId(pair);
		PfxUInt32 iA = pfxGetObjectIdA(pair);
		PfxUInt32 iB = pfxGetObjectIdB(pair);

		PfxContactManifold &contact = offsetContactManifolds[iContact];

		SCE_PFX_ALWAYS_ASSERT(iA==contact.getRigidBodyIdA());
		SCE_PFX_ALWAYS_ASSERT(iB==contact.getRigidBodyIdB());

		PfxRigidState &stateA = offsetRigidStates[iA];
		PfxRigidState &stateB = offsetRigidStates[iB];
		PfxCollidable &collA = offsetCollidables[iA];
		PfxCollidable &collB = offsetCollidables[iB];
		PfxTransform3 tA0(stateA.getOrientation(), stateA.getPosition());
		PfxTransform3 tB0(stateB.getOrientation(), stateB.getPosition());
		
		PfxContactCache contactCache;
		
		PfxShapeIterator itrShapeA(collA);
		for(PfxUInt32 j=0;j<collA.getNumShapes();j++,++itrShapeA)
		{
			const PfxShape &shapeA = *itrShapeA;
			PfxTransform3 offsetTrA = shapeA.getOffsetTransform();
			PfxTransform3 worldTrA = tA0 * offsetTrA;

			PfxShapeIterator itrShapeB(collB);
			for(PfxUInt32 k=0;k<collB.getNumShapes();k++,++itrShapeB)
			{
				const PfxShape &shapeB = *itrShapeB;
				PfxTransform3 offsetTrB = shapeB.getOffsetTransform();
				PfxTransform3 worldTrB = tB0 * offsetTrB;

				if( (shapeA.getContactFilterSelf()&shapeB.getContactFilterTarget()) && 
				    (shapeA.getContactFilterTarget()&shapeB.getContactFilterSelf()) )
				{
					pfxGetDetectCollisionFunc(shapeA.getType(),shapeB.getType())(
						contactCache,
						shapeA,offsetTrA,worldTrA,j,
						shapeB,offsetTrB,worldTrB,k,
						SCE_PFX_CONTACT_THRESHOLD);
				}
			}
		}
		
		for(int j=0;j<contactCache.getNumContacts();j++)
		{
			const PfxCachedContactPoint &cp = contactCache.getContactPoint(j);

			contact.addContactPoint(
				cp.m_distance,
				cp.m_normal,
				cp.m_localPointA,
				cp.m_localPointB,
				cp.m_subData
				);
		}
	}
}

//----------------------------------------------------------------------------
//  pfxDetectCollision
//
/// Perform narrow phase collision detection in parallel using a task
/// manager.
///
/// @param param        Information about pairs that may be colliding
/// @param taskManager  Pointer to the thread task manager to use
///
/// @return SCE_PFX_OK if successful, otherwise, returns an error code.
//----------------------------------------------------------------------------
PfxInt32 pfxDetectCollision(PfxDetectCollisionParam &param, PfxTaskManager *taskManager)
{
	PfxInt32 ret = pfxCheckParamOfDetectCollision(param);
	if(ret != SCE_PFX_OK) 
		return ret;

	SCE_PFX_PUSH_MARKER("pfxDetectCollision");

	PfxUInt32 maxBatchSize = param.numContactPairs / (PfxUInt32)(taskManager->getNumTasks());
	PfxUInt32 iEnd = maxBatchSize, iStart = 0;
	int task = 0;
	taskManager->setTaskEntry((void*)pfxDetectCollisionTaskEntry);

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
