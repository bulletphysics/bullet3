//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above copyright
//    notice, this list of conditions and the following disclaimer in the
//    documentation and/or other materials provided with the distribution.
//  * Neither the name of NVIDIA CORPORATION nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS ``AS IS'' AND ANY
// EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
// PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE COPYRIGHT OWNER OR
// CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
// EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
// PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR
// PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY
// OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// Copyright (c) 2008-2019 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef PX_PHYSICS_EXTENSIONS_NP_DEFAULT_CPU_DISPATCHER_H
#define PX_PHYSICS_EXTENSIONS_NP_DEFAULT_CPU_DISPATCHER_H

#include "CmPhysXCommon.h"
#include "PsUserAllocated.h"
#include "PsSync.h"
#include "PsSList.h"
#include "PxDefaultCpuDispatcher.h"
#include "ExtSharedQueueEntryPool.h"
#include "task/PxTask.h"
#include "common/PxProfileZone.h"

namespace physx
{
	
namespace Ext
{
	class CpuWorkerThread;

#if PX_VC
#pragma warning(push)
#pragma warning(disable:4324)	// Padding was added at the end of a structure because of a __declspec(align) value.
#endif							// Because of the SList member I assume

	class DefaultCpuDispatcher : public PxDefaultCpuDispatcher, public Ps::UserAllocated
	{
		friend class TaskQueueHelper;

	private:
												DefaultCpuDispatcher() : mQueueEntryPool(0) {}
												~DefaultCpuDispatcher();
	public:
												DefaultCpuDispatcher(PxU32 numThreads, PxU32* affinityMasks);

		//---------------------------------------------------------------------------------
		// PxCpuDispatcher implementation
		//---------------------------------------------------------------------------------
		virtual			void					submitTask(PxBaseTask& task);
		virtual			PxU32					getWorkerCount()	const	{ return mNumThreads;	}

		//---------------------------------------------------------------------------------
		// PxDefaultCpuDispatcher implementation
		//---------------------------------------------------------------------------------
		virtual			void					release();

		virtual			void					setRunProfiled(bool runProfiled) { mRunProfiled = runProfiled; }

		virtual			bool					getRunProfiled() const { return mRunProfiled; }

		//---------------------------------------------------------------------------------
		// DefaultCpuDispatcher
		//---------------------------------------------------------------------------------
						PxBaseTask*				getJob();
						PxBaseTask*				stealJob();
						PxBaseTask*				fetchNextTask();

		PX_FORCE_INLINE	void					runTask(PxBaseTask& task)
												{
#if PX_SUPPORT_PXTASK_PROFILING
													if(mRunProfiled)
													{
														PX_PROFILE_ZONE(task.getName(), task.getContextId());
														task.run();
													}
													else
#endif
														task.run();
												}

    					void					waitForWork() { mWorkReady.wait(); }
						void					resetWakeSignal();

		static			void					getAffinityMasks(PxU32* affinityMasks, PxU32 threadCount);

	protected:
						CpuWorkerThread*		mWorkerThreads;
						SharedQueueEntryPool<>	mQueueEntryPool;
						Ps::SList				mJobList;
						Ps::Sync				mWorkReady;
						PxU8*					mThreadNames;
						PxU32					mNumThreads;
						bool					mShuttingDown;
						bool					mRunProfiled;
	};

#if PX_VC
#pragma warning(pop)
#endif

} // namespace Ext
}

#endif
