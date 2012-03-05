/*
 Applied Research Associates Inc. (c)2011

 Redistribution and use in source and binary forms,
   with or without modification, are permitted provided that the
   following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Applied Research Associates Inc nor the names
      of its contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
   AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
   IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
   ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
   LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
   CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
   SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
   INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
   CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
   ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
   POSSIBILITY OF SUCH DAMAGE.
*/

#ifdef __ANDROID__

#include "../../../include/physics_effects/base_level/base/pfx_common.h"
#include "../../../include/physics_effects/low_level/task/pfx_pthreads.h"
#include "../../../include/physics_effects/low_level/task/pfx_task_manager.h"
#include "../../../include/physics_effects/low_level/task/pfx_sync_components.h"
#include "pfx_sync_components_pthreads.h"

namespace sce {
namespace PhysicsEffects {

//----------------------------------------------------------------------------
// Standalone functions and structs
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
//  PfxPthreadsThreadData
//
/// Struct to store information needed by worker threads
//----------------------------------------------------------------------------
struct PfxPthreadsThreadData
{
	// task runner information
	PfxTaskArg *taskargument;		///< Pointer to argument for the task entry function
	PfxTaskEntry taskEntry;			///< Pointer to current task entry function

	// pthreads synchronization and thread info
	pthread_t thread;				///< Current thread
	sem_t semaphore;				///< Semaphore used to wake the thread
	sem_t *taskmanagersemaphore;	///< Semaphore used to notify parent thread
};

//----------------------------------------------------------------------------
//  PfxPthreadsThreadFunction
//
/// The thread function used for threads created and managed using a
/// PfxPthreadsThreadPool
//----------------------------------------------------------------------------
void *PfxPthreadsThreadFunction(void *argument) 
{
	PfxPthreadsThreadData *threaddata = (PfxPthreadsThreadData*)argument;

	while (1)
	{
		// wait until a task is available
		SCE_PFX_CHECK_PTHREADS_OUTCOME(sem_wait(&threaddata->semaphore));

		// do work
		if (threaddata->taskEntry)
			threaddata->taskEntry(threaddata->taskargument);

		// notify threadpool that task is done
		SCE_PFX_CHECK_PTHREADS_OUTCOME(sem_post(threaddata->taskmanagersemaphore));

		// If no task, then exit
		if (!threaddata->taskEntry)
			pthread_exit(0);
	}

	return 0;
}

//----------------------------------------------------------------------------
// Class definitions
//----------------------------------------------------------------------------
class PfxPthreadsTaskManager : public PfxTaskManager
{
	public:
		PfxPthreadsTaskManager(PfxUInt32 numTasks, PfxUInt32 maxTasks, void *workBuff, PfxUInt32 workBytes) :
									PfxTaskManager(numTasks, maxTasks, workBuff, workBytes),
									m_threads(NULL) {}

		// from PfxTaskManager
		virtual PfxUInt32 getSharedParam(int i);
		virtual void setSharedParam(int i, PfxUInt32 p);
		virtual void startTask(int taskId, void *io, PfxUInt32 data1, PfxUInt32 data2,
								PfxUInt32 data3, PfxUInt32 data4);
		virtual void waitTask(int &taskId, PfxUInt32 &data1, PfxUInt32 &data2,
								PfxUInt32 &data3, PfxUInt32 &data4);

		virtual void initialize();
		virtual void finalize();

	protected:
		PfxPthreadsTaskManager() : PfxTaskManager(), m_threads(NULL) {}

	private:
		PfxPthreadsThreadData *m_threads;	///< Pointer to array of running threads (count is m_numThreads);
		PfxPthreadsBarrier m_barrier;		///< Barrier used to sync task groups
		PfxPthreadsCriticalSection m_cs;	///< Critical section used to manage shared parameters
		sem_t m_taskmanagersemaphore;		///< Synchronization semaphore for task manager
};

//----------------------------------------------------------------------------
//  PfxPthreadsTaskManager
//
/// Implementation of a task manager using pthreads.
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
//  PfxPthreadsTaskManager::initialize
//
/// Initialize the task manager
//----------------------------------------------------------------------------
void PfxPthreadsTaskManager::initialize()
{
	if (0 == m_maxTasks)
		return;

	if (m_threads) // already started
	{
		SCE_PFX_PRINTF("PfxPthreadsThreadPool attempt to start threads when they are already started, line %i, file %s\n", __LINE__, __FILE__);
		return;
	}

	m_threads = (PfxPthreadsThreadData*)m_pool.allocate(sizeof(PfxPthreadsThreadData)*m_maxTasks);
	if (!m_threads)
	{
		SCE_PFX_PRINTF("PfxPthreadsThreadPool unable to allocate threads at line %i in file %s\n", __LINE__, __FILE__);
		return;
	}

	m_barrier.setMaxCount(m_maxTasks);

	// Initialize sync semaphore
	SCE_PFX_CHECK_PTHREADS_OUTCOME(sem_init(&m_taskmanagersemaphore, 0, 0));

	// Allocate and start the threads
	for (unsigned int i = 0; i < m_maxTasks; i++)
	{
		// Prepare argument data structure for task entry functions. Mostly, this is
		// setting parameters that are fixed until the simulation ends
		m_taskArg[i].taskId = i;
		m_taskArg[i].maxTasks = m_maxTasks;
		m_taskArg[i].barrier = &m_barrier;
		m_taskArg[i].criticalSection = &m_cs;
		m_taskArg[i].io = NULL;

		// Prepare other per-thread data
		m_threads[i].taskEntry = NULL;
		m_threads[i].taskargument = &m_taskArg[i];
		m_threads[i].taskmanagersemaphore = &m_taskmanagersemaphore;

		// Now create the thread's semaphore, then start the thread
		SCE_PFX_CHECK_PTHREADS_OUTCOME(sem_init(&m_threads[i].semaphore, 0, 0));
		SCE_PFX_CHECK_PTHREADS_OUTCOME(pthread_create(&m_threads[i].thread, NULL, PfxPthreadsThreadFunction, (void*)&m_threads[i]));
	}
}

//----------------------------------------------------------------------------
//  PfxPthreadsTaskManager::finalize
//
/// Finalize the task manager
//----------------------------------------------------------------------------
void PfxPthreadsTaskManager::finalize()
{
	// stop the threads
	for (unsigned int i = 0; i < m_maxTasks; i++)
	{
		m_threads[i].taskEntry = NULL;	// NULL task tells thread to exit
		SCE_PFX_CHECK_PTHREADS_OUTCOME(sem_post(&m_threads[i].semaphore));
	}

	// wait for them all to exit
	for (unsigned int i = 0; i < m_maxTasks; i++)
		SCE_PFX_CHECK_PTHREADS_OUTCOME(sem_wait(&m_taskmanagersemaphore));

	// destroy per-thread semaphores
	for (unsigned int i = 0; i < m_maxTasks; i++)
		SCE_PFX_CHECK_PTHREADS_OUTCOME(sem_destroy(&m_threads[i].semaphore));

	// delete thread pool
	delete [] m_threads;
	m_threads = NULL;

	// destroy task manager semaphore and reset barrier
	SCE_PFX_CHECK_PTHREADS_OUTCOME(sem_destroy(&m_taskmanagersemaphore));

	m_barrier.setMaxCount(0);
}

//----------------------------------------------------------------------------
//  PfxPthreadsTaskManager::startTask
//
/// Start a task
///
/// @param taskId       [in] task thread identifier/index
/// @param io           [in, out] task input and output buffer
/// @param data1        [in] first of four user parameter data values for the task
/// @param data2        [in] second of four user parameter data values for the task
/// @param data3        [in] third of four user parameter data values for the task
/// @param data4        [in] fourth of four user parameter data values for the task
//----------------------------------------------------------------------------
void PfxPthreadsTaskManager::startTask(int taskId,void *io,PfxUInt32 data1,
	PfxUInt32 data2,PfxUInt32 data3,PfxUInt32 data4)
{
	m_threads[taskId].taskEntry = m_taskEntry;
	m_taskArg[taskId].io = io;
	m_taskArg[taskId].data[0] = data1;
	m_taskArg[taskId].data[1] = data2;
	m_taskArg[taskId].data[2] = data3;
	m_taskArg[taskId].data[3] = data4;
	SCE_PFX_CHECK_PTHREADS_OUTCOME(sem_post(&m_threads[taskId].semaphore));
}

//----------------------------------------------------------------------------
//  PfxPthreadsTaskManager::waitTask
//
/// Wait for a task to finish
///
/// @param taskId       [out] task thread identifier/index
/// @param data1        [out] first of four data values from the task
/// @param data2        [out] second of four data values from the task
/// @param data3        [out] third of four data values from the task
/// @param data4        [out] fourth of four data values from the task
//----------------------------------------------------------------------------
void PfxPthreadsTaskManager::waitTask(int &taskId,PfxUInt32 &data1,PfxUInt32 &data2,
	PfxUInt32 &data3,PfxUInt32 &data4)
{
	SCE_PFX_CHECK_PTHREADS_OUTCOME(sem_wait(&m_taskmanagersemaphore));
}

//----------------------------------------------------------------------------
//  PfxPthreadsTaskManager::getSharedParam
//
/// Get the value of a shared parameter
///
/// @param i     index of shared parameter to retrieve. Must have value
///              between 0 and 31
///
/// @return Shared parameter value
//----------------------------------------------------------------------------
PfxUInt32 PfxPthreadsTaskManager::getSharedParam(int i)
{
	m_cs.lock();
	PfxUInt32 paramval = m_cs.getSharedParam(i);
	m_cs.unlock();
	return(paramval);
}

//----------------------------------------------------------------------------
//  PfxPthreadsTaskManager::setSharedParam
//
/// Set the value of a shared parameter
///
/// @param i     index of shared parameter to set. Must have value
///              between 0 and 31
/// @param p     Value to assign to shared parameter
//----------------------------------------------------------------------------
void PfxPthreadsTaskManager::setSharedParam(int i, PfxUInt32 p)
{
	m_cs.lock();
	PfxUInt32 paramval = m_cs.getSharedParam(i);
	m_cs.setSharedParam(i, p);
	m_cs.unlock();
}

//----------------------------------------------------------------------------
// Factory functions
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
//  PfxCreateTaskManagerPthreads
//
/// Factory function to create a pthreads-based task manager
///
/// @param numTasks       number of tasks
/// @param maxTasks       max number of tasks
/// @param workBuff       work buffer
/// @param workBytes      size of work buffer, in bytes
//----------------------------------------------------------------------------
PfxTaskManager *PfxCreateTaskManagerPthreads(PfxUInt32 numTasks,PfxUInt32 maxTasks,
	void *workBuff,PfxUInt32 workBytes)
{
	PfxTaskManager *taskmanager = new PfxPthreadsTaskManager(numTasks, maxTasks,
																workBuff, workBytes);
	return(taskmanager);
}

} //namespace PhysicsEffects
} //namespace sce

#endif //__ANDROID__
