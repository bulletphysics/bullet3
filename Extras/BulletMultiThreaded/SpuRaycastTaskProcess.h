/*
Bullet Continuous Collision Detection and Physics Library
Copyright (c) 2003-2007 Erwin Coumans  http://bulletphysics.com

This software is provided 'as-is', without any express or implied warranty.
In no event will the authors be held liable for any damages arising from the use of this software.
Permission is granted to anyone to use this software for any purpose, 
including commercial applications, and to alter it and redistribute it freely, 
subject to the following restrictions:

1. The origin of this software must not be misrepresented; you must not claim that you wrote the original software. If you use this software in a product, an acknowledgment in the product documentation would be appreciated but is not required.
2. Altered source versions must be plainly marked as such, and must not be misrepresented as being the original software.
3. This notice may not be removed or altered from any source distribution.
*/

#ifndef SPU_RAY_TASK_PROCESS_H
#define SPU_RAY_TASK_PROCESS_H

#include <assert.h>
#include <string.h>

#include <LinearMath/btScalar.h>
#include "BulletCollision/CollisionDispatch/btCollisionObject.h"
#include <LinearMath/btAlignedAllocator.h>

#include "PlatformDefinitions.h"
#include "LinearMath/btAlignedObjectArray.h"
#include "SpuRaycastTask/SpuRaycastTask.h"

#include "btThreadSupportInterface.h"

/// SpuRaycastTaskProcess handles SPU processing of raycast requests
class SpuRaycastTaskProcess
{
	unsigned char  *m_workUnitTaskBuffers;

	// track task buffers that are being used, and total busy tasks
	btAlignedObjectArray<bool>	m_taskBusy;
	btAlignedObjectArray<SpuRaycastTaskDesc>	m_spuRaycastTaskDesc;

	btThreadSupportInterface*	m_threadInterface;

	int	m_maxNumOutstandingTasks;

	int	m_numBusyTasks;

	// the current task and the current entry to insert a new work unit
	int m_currentTask;
	int m_currentWorkUnitInTask;
	int m_numSpuCollisionObjectWrappers;
	void* m_spuCollisionObjectWrappers;
	void issueTask2();
	//void postProcess(unsigned int taskId, int outputSize);

public:
	SpuRaycastTaskProcess(btThreadSupportInterface*	threadInterface, int maxNumOutstandingTasks);
	
	~SpuRaycastTaskProcess();
	
	/// call initialize in the beginning of the frame, before addCollisionPairToTask
	void initialize2(void* spuCollisionObjectsWrappers, int numSpuCollisionObjectWrappers);

	/// batch up additional work to a current task for SPU processing. When batch is full, it issues the task.
	void addWorkToTask(struct SpuRaycastTaskWorkUnit&);

	/// call flush to submit potential outstanding work to SPUs and wait for all involved SPUs to be finished
	void flush2();
};


#endif // SPU_COLLISION_TASK_PROCESS_H

