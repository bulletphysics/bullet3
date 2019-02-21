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

#ifndef BP_MBP_TASKS_H
#define BP_MBP_TASKS_H

#include "PsUserAllocated.h"
#include "CmTask.h"

namespace physx
{
	class PxcScratchAllocator;

	namespace Bp
	{
		class BroadPhaseMBP;
	}

#define MBP_USE_SCRATCHPAD

	class MBPTask : public Cm::Task, public shdfnd::UserAllocated
	{
		public:
												MBPTask(PxU64 contextId) : Cm::Task(contextId), mMBP(NULL), mNumCpuTasks(0)		{}

		PX_FORCE_INLINE	void					set(Bp::BroadPhaseMBP* mbp, PxcScratchAllocator* sa, PxU32 numCpuTasks)
												{
													mMBP = mbp;
													mScratchAllocator = sa;
													mNumCpuTasks = numCpuTasks;
												}
		protected:
						Bp::BroadPhaseMBP*		mMBP;
						PxU32					mNumCpuTasks;
						PxcScratchAllocator*	mScratchAllocator;
		private:
		MBPTask& operator=(const MBPTask&);
	};

	// PT: this is the main 'update' task doing the actual box pruning work.
	class MBPUpdateWorkTask : public MBPTask
	{
	public:							
								MBPUpdateWorkTask(PxU64 contextId) : MBPTask(contextId)	{}
								~MBPUpdateWorkTask()									{}
		// PxBaseTask
		virtual const char*		getName() const { return "BpMBP.updateWork"; }
		//~PxBaseTask

		// Cm::Task
		virtual void			runInternal();
		//~Cm::Task

	private:
		MBPUpdateWorkTask& operator=(const MBPUpdateWorkTask&);
	};

	// PT: this task runs after MBPUpdateWorkTask. This is where MBP_PairManager::removeMarkedPairs is called, to finalize
	// the work and come up with created/removed lists. This is single-threaded.
	class MBPPostUpdateWorkTask : public MBPTask
	{
	public:
								MBPPostUpdateWorkTask(PxU64 contextId) : MBPTask(contextId)	{}
								~MBPPostUpdateWorkTask()									{}
		// PxBaseTask
		virtual const char*		getName() const { return "BpMBP.postUpdateWork"; }
		//~PxBaseTask

		// Cm::Task
		virtual void			runInternal();
		//~Cm::Task

	private:
		MBPPostUpdateWorkTask& operator=(const MBPPostUpdateWorkTask&);
	};

} //namespace physx

#endif // BP_MBP_TASKS_H
