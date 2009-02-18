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

#include "SpuRaycastTaskProcess.h"


SpuRaycastTaskProcess::SpuRaycastTaskProcess(class	btThreadSupportInterface*	threadInterface,  int	maxNumOutstandingTasks)
:m_threadInterface(threadInterface),
m_maxNumOutstandingTasks(maxNumOutstandingTasks)
{
	m_workUnitTaskBuffers = (unsigned char *)0;
	m_taskBusy.resize(m_maxNumOutstandingTasks);
	m_spuRaycastTaskDesc.resize(m_maxNumOutstandingTasks);

	for (int i = 0; i < m_maxNumOutstandingTasks; i++)
	{
		m_taskBusy[i] = false;
	}
	m_numBusyTasks = 0;
	m_currentTask = 0;
	m_currentWorkUnitInTask = 0;

	m_threadInterface->startSPU();

	//printf("sizeof vec_float4: %d\n", sizeof(vec_float4));
	//printf("sizeof SpuGatherAndProcessWorkUnitInput: %d\n", sizeof(SpuGatherAndProcessWorkUnitInput));

}

SpuRaycastTaskProcess::~SpuRaycastTaskProcess()
{
	
	if (m_workUnitTaskBuffers != 0)
	{
		btAlignedFree(m_workUnitTaskBuffers);
		m_workUnitTaskBuffers = 0;
	}
	
	m_threadInterface->stopSPU();	
}



void SpuRaycastTaskProcess::initialize2(void* spuCollisionObjectsWrappers, int numSpuCollisionObjectWrappers)
{
	m_spuCollisionObjectWrappers = spuCollisionObjectsWrappers;
	m_numSpuCollisionObjectWrappers = numSpuCollisionObjectWrappers;
	for (int i = 0; i < m_maxNumOutstandingTasks; i++)
	{
		m_taskBusy[i] = false;
	}
	m_numBusyTasks = 0;
	m_currentTask = 0;
	m_currentWorkUnitInTask = 0;

#ifdef DEBUG_SpuRaycastTaskProcess
	m_initialized = true;
#endif
}


void SpuRaycastTaskProcess::issueTask2()
{
	m_taskBusy[m_currentTask] = true;
	m_numBusyTasks++;

	SpuRaycastTaskDesc& taskDesc = m_spuRaycastTaskDesc[m_currentTask];

	taskDesc.taskId = m_currentTask;
	m_threadInterface->sendRequest(1, (ppu_address_t) &taskDesc,m_currentTask);
	//printf("send thread requested for task %d\n", m_currentTask);
	// if all tasks busy, wait for spu event to clear the task.
	if (m_numBusyTasks >= m_maxNumOutstandingTasks)
	{
		unsigned int taskId;
		unsigned int outputSize;

		for (int i=0;i<m_maxNumOutstandingTasks;i++)
	  {
		  if (m_taskBusy[i])
		  {
			  taskId = i;
			  break;
		  }
	  }
		m_threadInterface->waitForResponse(&taskId, &outputSize);

		//printf("PPU: after issue, received event: %u %d\n", taskId, outputSize);

		m_taskBusy[taskId] = false;

		m_numBusyTasks--;
	} else {
		//printf("Sent request, not enough busy tasks\n");
	}
}

void SpuRaycastTaskProcess::addWorkToTask(SpuRaycastTaskWorkUnit& workunit)
{
	m_spuRaycastTaskDesc[m_currentTask].workUnits[m_currentWorkUnitInTask] = workunit;
	m_currentWorkUnitInTask++;
	if (m_currentWorkUnitInTask == SPU_RAYCAST_WORK_UNITS_PER_TASK)
	{
		m_spuRaycastTaskDesc[m_currentTask].numWorkUnits = m_currentWorkUnitInTask;
		m_spuRaycastTaskDesc[m_currentTask].numSpuCollisionObjectWrappers = m_numSpuCollisionObjectWrappers;
		m_spuRaycastTaskDesc[m_currentTask].spuCollisionObjectsWrappers = m_spuCollisionObjectWrappers;
		//printf("Task buffer full, issuing\n");
		issueTask2 ();
		//printf("Returned from issueTask2()\n");
		m_currentWorkUnitInTask = 0;

		// find new task buffer
		for (int i = 0; i < m_maxNumOutstandingTasks; i++)
		{
			if (!m_taskBusy[i])
			{
				m_currentTask = i;
				//init the task data
				break;
			}
		}
		//printf("next task = %d\n", m_currentTask);
	}
}


void 
SpuRaycastTaskProcess::flush2()
{
#ifdef DEBUG_SPU_TASK_SCHEDULING
	printf("\nSpuRaycastTaskProcess::flush()\n");
#endif //DEBUG_SPU_TASK_SCHEDULING
	
	// if there's a partially filled task buffer, submit that task
	//printf("Flushing... %d remaining\n", m_currentWorkUnitInTask);
	if (m_currentWorkUnitInTask > 0)
	{
		m_spuRaycastTaskDesc[m_currentTask].numWorkUnits = m_currentWorkUnitInTask;
		m_spuRaycastTaskDesc[m_currentTask].numSpuCollisionObjectWrappers = m_numSpuCollisionObjectWrappers;
		m_spuRaycastTaskDesc[m_currentTask].spuCollisionObjectsWrappers = m_spuCollisionObjectWrappers;
		issueTask2();
		m_currentWorkUnitInTask = 0;
	}


	// all tasks are issued, wait for all tasks to be complete
	while(m_numBusyTasks > 0)
	{
	  // Consolidating SPU code
	  unsigned int taskId;
	  unsigned int outputSize;
	  
	  for (int i=0;i<m_maxNumOutstandingTasks;i++)
	  {
		  if (m_taskBusy[i])
		  {
			  taskId = i;
			  break;
		  }
	  }

	  //printf("Busy tasks... %d\n", m_numBusyTasks);

	  {
			// SPURS support.
			m_threadInterface->waitForResponse(&taskId, &outputSize);
		}

		//printf("PPU: flushing, received event: %u %d\n", taskId, outputSize);

		//postProcess(taskId, outputSize);

		m_taskBusy[taskId] = false;

		m_numBusyTasks--;
	}
}
