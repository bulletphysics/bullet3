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

//#define __CELLOS_LV2__ 1
#ifdef USE_SAMPLE_PROCESS



#include "SpuLibspe2Support.h"
#include "Win32ThreadSupport.h"

//#include "SPUAssert.h"
#include <string.h>


#include "SpuSampleTaskProcess.h"


#include <stdio.h>


void	SampleThreadFunc(void* userPtr,void* lsMemory)
{
	//do nothing
	printf("hello world\n");
}

void*	SamplelsMemoryFunc()
{
	//don't create local store memory, just return 0
	return 0;
}

#ifdef USE_IBM_CELL_SDK
//SpuLibspe2Support gSampleSPU(SPU_ELF_SAMPLE,SAMPLE_NUM_WORKUNIT_TASKS);
#elif defined(WIN32)
Win32ThreadSupport gSampleSPU(Win32ThreadSupport::Win32ThreadConstructionInfo("sample",
							  		SampleThreadFunc,
									SamplelsMemoryFunc,
									SAMPLE_NUM_WORKUNIT_TASKS));
#elif defined(__CELLOS_LV2__)

#include "CellSPURSSupport.ppu.h"
CellSPURSSupport gSampleSPU(SPU_ELF_SAMPLE);

#endif



extern "C" {
	extern char SPU_SAMPLE_ELF_SYMBOL[];
};





//SpuSampleTaskDesc g_spuSampleTaskDesc[SAMPLE_NUM_WORKUNIT_TASKS];



SpuSampleTaskProcess::SpuSampleTaskProcess()
{

	for (int i = 0; i < SAMPLE_NUM_WORKUNIT_TASKS; i++)
	{
		m_taskBusy[i] = false;
	}
	m_numBusyTasks = 0;
	m_currentTask = 0;

	m_initialized = false;

	m_threadInterface->startSPU();

#ifdef WIN32
	Win32ThreadSupport::Win32ThreadConstructionInfo threadConstructionInfo(
	"sample",SampleThreadFunc,SamplelsMemoryFunc);

	gSampleSPU.startSPU(threadConstructionInfo);
#else
	gSampleSPU.startSPU();
#endif

}

SpuSampleTaskProcess::~SpuSampleTaskProcess()
{
	gSampleSPU.stopSPU();
	
}



void	SpuSampleTaskProcess::initialize()
{
#ifdef DEBUG_SPU_TASK_SCHEDULING
	printf("SpuSampleTaskProcess::initialize()\n");
#endif //DEBUG_SPU_TASK_SCHEDULING
	
	for (int i = 0; i < SAMPLE_NUM_WORKUNIT_TASKS; i++)
	{
		m_taskBusy[i] = false;
	}
	m_numBusyTasks = 0;
	m_currentTask = 0;
	m_initialized = true;

}


void SpuSampleTaskProcess::issueTask(void* sampleMainMemPtr,int sampleValue)
{

#ifdef DEBUG_SPU_TASK_SCHEDULING
	printf("SpuSampleTaskProcess::issueTask (m_currentTask= %d\)n", m_currentTask);
#endif //DEBUG_SPU_TASK_SCHEDULING

	m_taskBusy[m_currentTask] = true;
	m_numBusyTasks++;

	SpuSampleTaskDesc& taskDesc = g_spuSampleTaskDesc[m_currentTask];
	{
		// send task description in event message
		// no error checking here...
		// but, currently, event queue can be no larger than NUM_WORKUNIT_TASKS.
	
		taskDesc.m_mainMemoryPtr = reinterpret_cast<uint64_t>(sampleMainMemPtr);
		taskDesc.m_sampleValue = sampleValue;

		//some bookkeeping to recognize finished tasks
		taskDesc.m_taskId = m_currentTask;
	}


	gSampleSPU.sendRequest(CMD_SAMPLE_TASK_COMMAND, (uint32_t) &taskDesc, m_currentTask);

	// if all tasks busy, wait for spu event to clear the task.
	
	if (m_numBusyTasks >= SAMPLE_NUM_WORKUNIT_TASKS)
	{
		unsigned int taskId;
		unsigned int outputSize;

		gSampleSPU.waitForResponse(&taskId, &outputSize);

		//printf("PPU: after issue, received event: %u %d\n", taskId, outputSize);

		postProcess(taskId, outputSize);

		m_taskBusy[taskId] = false;

		m_numBusyTasks--;
	}

	// find new task buffer
	for (unsigned int i = 0; i < SAMPLE_NUM_WORKUNIT_TASKS; i++)
	{
		if (!m_taskBusy[i])
		{
			m_currentTask = i;
			break;
		}
	}
}


///Optional PPU-size post processing for each task
void SpuSampleTaskProcess::postProcess(int taskId, int outputSize)
{

}


void SpuSampleTaskProcess::flush()
{
#ifdef DEBUG_SPU_TASK_SCHEDULING
	printf("\nSpuCollisionTaskProcess::flush()\n");
#endif //DEBUG_SPU_TASK_SCHEDULING
	

	// all tasks are issued, wait for all tasks to be complete
	while(m_numBusyTasks > 0)
	{
// Consolidating SPU code
	  unsigned int taskId;
	  unsigned int outputSize;
	  
	  {
			
		  gSampleSPU.waitForResponse(&taskId, &outputSize);
	  }

		//printf("PPU: flushing, received event: %u %d\n", taskId, outputSize);

		postProcess(taskId, outputSize);

		m_taskBusy[taskId] = false;

		m_numBusyTasks--;
	}


}
#endif //USE_SAMPLE_PROCESS
