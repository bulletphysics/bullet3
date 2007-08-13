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
#ifdef USE_LIBSPE2

#include "SpuLibspe2Support.h"

#include "SpuCollisionTaskProcess.h"
#include "SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h"

#include <Windows.h>

///SpuLibspe2Support helps to initialize/shutdown libspe2, start/stop SPU tasks and communication
///Setup and initialize SPU/CELL/Libspe2
SpuLibspe2Support::SpuLibspe2Support(spe_program_handle_t *speprog,int numThreads)
{
	program = speprog
	startSPUs(numThreads);
	N = numThreads;
}

///cleanup/shutdown Libspe2
SpuLibspe2Support::~SpuLibspe2Support()
{
	stopSPUs();
}



#include <stdio.h>

#ifdef WIN32
DWORD WINAPI Thread_no_1( LPVOID lpParam ) 
{

	btSpuStatus* status = (btSpuStatus*)lpParam;

	
	while (1)
	{
		WaitForSingleObject(status->m_eventStartHandle,INFINITE);
		btAssert(status->m_status);

		SpuGatherAndProcessPairsTaskDesc* taskDesc = status->m_taskDesc;

		if (taskDesc)
		{
			processCollisionTask(*taskDesc);
			SetEvent(status->m_eventCompletetHandle);
		} else
		{
			//exit Thread
			break;
		}
		
	}

	return 0;

}
#endif
///send messages to SPUs
void SpuLibspe2Support::sendRequest(uint32_t uiCommand, uint32_t uiArgument0, uint32_t uiArgument1)
{
	///	gMidphaseSPU.sendRequest(CMD_GATHER_AND_PROCESS_PAIRLIST, (uint32_t) &taskDesc);
	
	///we should spawn an SPU task here, and in 'waitForResponse' it should wait for response of the (one of) the first tasks that finished
	


	switch (uiCommand)
	{
	case 	CMD_GATHER_AND_PROCESS_PAIRLIST:
		{

			SpuGatherAndProcessPairsTaskDesc* taskDesc = (SpuGatherAndProcessPairsTaskDesc*) uiArgument0 ;

//#define SINGLE_THREADED 1
#ifdef SINGLE_THREADED

			btSpuStatus&	spuStatus = m_activeSpuStatus[0];
			taskDesc->m_lsMemory = (CollisionTask_LocalStoreMemory*)spuStatus.m_lsMemory;
			processCollisionTask(*taskDesc);
			HANDLE handle =0;
#else

			btAssert(taskDesc->taskId>=0);
			btAssert(taskDesc->taskId<m_activeSpuStatus.size());

			btSpuStatus&	spuStatus = m_activeSpuStatus[taskDesc->taskId];

			spuStatus.m_commandId = uiCommand;
			spuStatus.m_status = 1;
			spuStatus.m_taskDesc = taskDesc;
			taskDesc->m_lsMemory = (CollisionTask_LocalStoreMemory*)spuStatus.m_lsMemory;
			///fire event to start new task
			SetEvent(spuStatus.m_eventStartHandle);

#endif //CollisionTask_LocalStoreMemory

			

			break;
		}
	default:
		{
			///not implemented
			btAssert(0);
		}

	};


}

///check for messages from SPUs
void SpuLibspe2Support::waitForResponse(unsigned int *puiArgument0, unsigned int *puiArgument1)
{
	///We should wait for (one of) the first tasks to finish (or other SPU messages), and report its response
	
	///A possible response can be 'yes, SPU handled it', or 'no, please do a PPU fallback'


	btAssert(m_activeSpuStatus.size());

	int last = -1;
	
	//find an active spu/thread
	for (int i=0;i<m_activeSpuStatus.size();i++)
	{
		if (m_activeSpuStatus[i].m_status)
		{
			last = i;
			break;
		}
	}


#ifndef SINGLE_THREADED
	btAssert(spuStatus.m_threadHandle);
	btAssert(spuStatus.m_eventCompletetHandle);
	btSpuStatus& spuStatus = m_activeSpuStatus[last];
	WaitForSingleObject(spuStatus.m_eventCompletetHandle, INFINITE);
	spuStatus.m_status = 0;

	///need to find an active spu
	btAssert(last>=0);

#else
	last=0;
	btSpuStatus& spuStatus = m_activeSpuStatus[last];
#endif //SINGLE_THREADED

	

	*puiArgument0 = spuStatus.m_taskId;
	*puiArgument1 = spuStatus.m_status;


}


///start the spus (can be called at the beginning of each frame, to make sure that the right SPU program is loaded)
void SpuLibspe2Support::startSPUs(int numThreads)
{
#ifdef WIN32
	m_activeSpuStatus.resize(numThreads);
#endif

	for (int i=0;i<numThreads;i++)
	{
		printf("starting thread %d\n",i);

		data[i].context = spe_context_create(0, NULL);
		spe_program_load(data[i].context, program);
		data[i].entry = SPE_DEFAULT_ENTRY;
		data[i].flags = 0;
		data[i].argp = NULL;
		data[i].envp = NULL;
		pthread_create(&data[i].pthread, NULL, &ppu_pthread_function, &data[i]);
		printf("started thread %d with threadHandle %d\n",i,handle);
		
	}

}

///tell the task scheduler we are done with the SPU tasks
void SpuLibspe2Support::stopSPUs()
{
	// wait for all threads to finish 
	for ( i=0; i<N; i++ ) { 
		pthread_join (data[i].pthread, NULL); 
	} 
	// close SPE program 
	spe_image_close(program); 
	// destroy SPE contexts 
	for ( i=0; i<N; i++ ) { 
	 spe_context_destroy (data[i].context); 
	} 

	
}

#endif// USE_LIBSPE2

