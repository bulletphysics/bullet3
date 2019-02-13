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
// Copyright (c) 2008-2018 NVIDIA Corporation. All rights reserved.
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.  

#ifndef BP_AABB_MANAGER_TASKS_H
#define BP_AABB_MANAGER_TASKS_H

#include "PsUserAllocated.h"
#include "CmTask.h"

namespace physx
{
	class PxcScratchAllocator;
namespace Bp
{
	class AABBManager;
	class Aggregate;

	class AggregateBoundsComputationTask : public Cm::Task, public shdfnd::UserAllocated
	{
		public:
										AggregateBoundsComputationTask(PxU64 contextId) :
											Cm::Task	(contextId),
											mManager	(NULL),
											mStart		(0),
											mNbToGo		(0),
											mAggregates	(NULL)
										{}
										~AggregateBoundsComputationTask()	{}

		virtual const char*				getName() const { return "AggregateBoundsComputationTask"; }
		virtual void					runInternal();

				void					Init(AABBManager* manager, PxU32 start, PxU32 nb, Aggregate** aggregates)
										{
											mManager	= manager;
											mStart		= start;
											mNbToGo		= nb;
											mAggregates	= aggregates;
										}
		private:
				AABBManager*			mManager;
				PxU32					mStart;
				PxU32					mNbToGo;
				Aggregate**				mAggregates;

		AggregateBoundsComputationTask& operator=(const AggregateBoundsComputationTask&);
	};

	class FinalizeUpdateTask : public Cm::Task, public shdfnd::UserAllocated
	{
		public:
										FinalizeUpdateTask(PxU64 contextId) :
											Cm::Task				(contextId),
											mManager				(NULL),
											mNumCpuTasks			(0),
											mScratchAllocator		(NULL),
											mNarrowPhaseUnlockTask	(NULL)
										{}
										~FinalizeUpdateTask()	{}

		virtual const char*				getName() const { return "FinalizeUpdateTask"; }
		virtual void					runInternal();

				void					Init(AABBManager* manager, PxU32 numCpuTasks, PxcScratchAllocator* scratchAllocator, PxBaseTask* narrowPhaseUnlockTask)
										{
											mManager				= manager;
											mNumCpuTasks			= numCpuTasks;
											mScratchAllocator		= scratchAllocator;
											mNarrowPhaseUnlockTask	= narrowPhaseUnlockTask;
										}
		private:
				AABBManager*			mManager;
				PxU32					mNumCpuTasks;
				PxcScratchAllocator*	mScratchAllocator;
				PxBaseTask*				mNarrowPhaseUnlockTask;

		FinalizeUpdateTask& operator=(const FinalizeUpdateTask&);
	};

}
} //namespace physx

#endif // BP_AABB_MANAGER_TASKS_H
