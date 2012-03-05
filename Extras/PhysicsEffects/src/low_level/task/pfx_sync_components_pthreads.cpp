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
#include "../../../include/physics_effects/low_level/task/pfx_sync_components.h"
#include "pfx_sync_components_pthreads.h"

namespace sce {
namespace PhysicsEffects {

//----------------------------------------------------------------------------
//  PfxPthreadsBarrier::PfxPthreadsBarrier
//
/// Default constructor
//----------------------------------------------------------------------------
PfxPthreadsBarrier::PfxPthreadsBarrier() :
	m_maxThreads(0), m_called(0)
{
}

//----------------------------------------------------------------------------
//  PfxPthreadsBarrier::~PfxPthreadsBarrier
//
/// Destructor
//----------------------------------------------------------------------------
PfxPthreadsBarrier::~PfxPthreadsBarrier() 
{
	if (m_maxThreads > 0)
	{
		SCE_PFX_CHECK_PTHREADS_OUTCOME(pthread_mutex_destroy(&m_mutex));
		SCE_PFX_CHECK_PTHREADS_OUTCOME(pthread_cond_destroy(&m_cond));
	}
}
	
//----------------------------------------------------------------------------
//  PfxPthreadsBarrier::sync
//
/// This function is used to sync m_numThreads worker threads. Each worker
/// should call sync() when it finishes a task. All workers will block until
/// the last worker also calls sync().
//----------------------------------------------------------------------------
void PfxPthreadsBarrier::sync()
{		
	SCE_PFX_CHECK_PTHREADS_OUTCOME(pthread_mutex_lock(&m_mutex));

	m_called++;

	if (m_called == m_maxThreads)
	{
		// last thread to join broadcasts a condition that will release
		// all the threads waiting at the barrier. The barrier is reset
		// to be ready for the next sync point.
		m_called = 0;
		SCE_PFX_CHECK_PTHREADS_OUTCOME(pthread_cond_broadcast(&m_cond));
	}
	else
	{
		// First m_numThreads - 1 worker threads block on the condition.
		SCE_PFX_CHECK_PTHREADS_OUTCOME(pthread_cond_wait(&m_cond, &m_mutex));
	}

	SCE_PFX_CHECK_PTHREADS_OUTCOME(pthread_mutex_unlock(&m_mutex));
}

//----------------------------------------------------------------------------
//  PfxPthreadsBarrier::setMaxCount
//
/// Set the number of threads that the barrier should wait for. This also
/// initializes a mutex and condition variable that are used to implement
/// the barrier.
///
/// @param n       Number of threads that should wait at the barrier.
//----------------------------------------------------------------------------
void PfxPthreadsBarrier::setMaxCount(int n)
{
	if (m_maxThreads > 0)
	{
		SCE_PFX_CHECK_PTHREADS_OUTCOME(pthread_mutex_destroy(&m_mutex));
		SCE_PFX_CHECK_PTHREADS_OUTCOME(pthread_cond_destroy(&m_cond));
	}

	m_called = 0;

	if (0 < n)
	{
		SCE_PFX_CHECK_PTHREADS_OUTCOME(pthread_mutex_init(&m_mutex,NULL));
		SCE_PFX_CHECK_PTHREADS_OUTCOME(pthread_cond_init(&m_cond,NULL));
	}

	m_maxThreads = n;
}

//----------------------------------------------------------------------------
//  PfxPthreadsBarrier::getMaxCount
//
/// Get the number of threads that the barrier will wait for.
///
/// @return  The number of threads the barrier waits for
//----------------------------------------------------------------------------
int PfxPthreadsBarrier::getMaxCount()
{
	return m_maxThreads;
}

//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
//  PfxPthreadsCriticalSection::PfxPthreadsCriticalSection
//
/// Default constructor
//----------------------------------------------------------------------------
PfxPthreadsCriticalSection::PfxPthreadsCriticalSection()
{
	SCE_PFX_CHECK_PTHREADS_OUTCOME(pthread_mutex_init(&m_mutex,NULL));
}

//----------------------------------------------------------------------------
//  PfxPthreadsCriticalSection::~PfxPthreadsCriticalSection
//
/// Destructor
//----------------------------------------------------------------------------
PfxPthreadsCriticalSection::~PfxPthreadsCriticalSection()
{
	SCE_PFX_CHECK_PTHREADS_OUTCOME(pthread_mutex_destroy(&m_mutex));
}

//----------------------------------------------------------------------------
//  PfxPthreadsCriticalSection::getSharedParam
//
/// Get the value of a shared parameter. Note that user must lock the
/// critical section before performing this action.
///
/// @param i     index of shared parameter to retrieve. Must have value
///              between 0 and 31
///
/// @return Shared parameter value
//----------------------------------------------------------------------------
PfxUInt32 PfxPthreadsCriticalSection::getSharedParam(int i)
{
	return(m_commonBuff[i]);
}

//----------------------------------------------------------------------------
//  PfxPthreadsCriticalSection::setSharedParam
//
/// Set the value of a shared parameter. Note that user must lock the
/// critical section before performing this action.
///
/// @param i     index of shared parameter to set. Must have value
///              between 0 and 31
/// @param p     Value to assign to shared parameter
//----------------------------------------------------------------------------
void PfxPthreadsCriticalSection::setSharedParam(int i,PfxUInt32 p)
{
	m_commonBuff[i] = p;
}

//----------------------------------------------------------------------------
//  PfxPthreadsCriticalSection::lock
//
/// Lock the critical section
//----------------------------------------------------------------------------
void PfxPthreadsCriticalSection::lock()
{
	SCE_PFX_CHECK_PTHREADS_OUTCOME(pthread_mutex_lock(&m_mutex));
}

//----------------------------------------------------------------------------
//  PfxPthreadsCriticalSection::lock
//
/// Unlock the critical section
//----------------------------------------------------------------------------
void PfxPthreadsCriticalSection::unlock()
{
	SCE_PFX_CHECK_PTHREADS_OUTCOME(pthread_mutex_unlock(&m_mutex));
}

//----------------------------------------------------------------------------
// Factory functions
//----------------------------------------------------------------------------

//----------------------------------------------------------------------------
//  PfxCreateBarrierPthreads
//
/// Factory function to create a pthreads-based barrier
///
/// @param n       Max number of tasks
//----------------------------------------------------------------------------
PfxBarrier *PfxCreateBarrierPthreads(int n)
{
	PfxPthreadsBarrier *barrier = new PfxPthreadsBarrier;
	barrier->setMaxCount(n);
	return(barrier);
}

//----------------------------------------------------------------------------
//  PfxCreateCriticalSectionPthreads
//
/// Factory function to create a pthreads-based critical section
//----------------------------------------------------------------------------
PfxCriticalSection *PfxCreateCriticalSectionPthreads()
{
	PfxPthreadsCriticalSection *cs = new PfxPthreadsCriticalSection;
	return(cs);
}

} //namespace PhysicsEffects
} //namespace sce

#endif //__ANDROID__
