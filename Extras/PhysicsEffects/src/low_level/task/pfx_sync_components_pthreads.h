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

#ifndef _SCE_PFX_SYNC_COMPONENTS_PTHREADS_H
#define _SCE_PFX_SYNC_COMPONENTS_PTHREADS_H

#include "../../../include/physics_effects/base_level/base/pfx_common.h"
#include "../../../include/physics_effects/low_level/task/pfx_pthreads.h"
#include "../../../include/physics_effects/low_level/task/pfx_sync_components.h"
#include <pthread.h>
#include <semaphore.h>

namespace sce {
namespace PhysicsEffects {

//----------------------------------------------------------------------------
//  PfxPthreadsBarrier
//
/// Implementation of a barrier using pthreads. This version uses a mutex
/// and condition variable rather than using native pthreads barrier, which
/// enables it to be used on platforms that don't support pthreads barriers
/// (such as Android 2.3x)
//----------------------------------------------------------------------------
class PfxPthreadsBarrier : public PfxBarrier
{
	public:
		PfxPthreadsBarrier();
		virtual ~PfxPthreadsBarrier();

		// from PfxBarrier
		virtual void sync();
		virtual void setMaxCount(int n);
		virtual int  getMaxCount();

	private:
		pthread_mutex_t m_mutex;	///< Mutex used to block worker threads
		pthread_cond_t m_cond;		///< Condition variable
	
		int m_maxThreads;			///< Maximum number of worker threads
		int	m_called;				///< Number of worker threads waiting at barrier
};

//----------------------------------------------------------------------------
//  PfxPthreadsBarrier
//
/// Implementation of a critical section using pthreads.
//----------------------------------------------------------------------------
class PfxPthreadsCriticalSection : public PfxCriticalSection
{
	public:
		PfxPthreadsCriticalSection();
		virtual ~PfxPthreadsCriticalSection();

		// from PfxCriticalSection
		virtual PfxUInt32 getSharedParam(int i);
		virtual void setSharedParam(int i,PfxUInt32 p);

		virtual void lock();
		virtual void unlock();

	private:
		pthread_mutex_t m_mutex;	///< Mutex used to implement lock
};

} //namespace PhysicsEffects
} //namespace sce

#endif // _SCE_PFX_SYNC_COMPONENTS_PTHREADS_H
