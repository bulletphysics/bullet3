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


#ifndef PX_PHYSICS_COMMON_TASK
#define PX_PHYSICS_COMMON_TASK

#include "task/PxTask.h"
#include "CmPhysXCommon.h"
#include "PsUserAllocated.h"
#include "PsAtomic.h"
#include "PsMutex.h"
#include "PsInlineArray.h"
#include "PsFPU.h"

namespace physx
{
namespace Cm
{
	// wrapper around the public PxLightCpuTask
	// internal SDK tasks should be inherited from
	// this and override the runInternal() method
	// to ensure that the correct floating point 
	// state is set / reset during execution
	class Task : public physx::PxLightCpuTask
	{
	public:
		Task(PxU64 contextId)
		{
			mContextID = contextId;
		}

		virtual void run()
		{
#if PX_SWITCH  // special case because default rounding mode is not nearest
			PX_FPU_GUARD;
#else
			PX_SIMD_GUARD;
#endif
			runInternal();
		}

		virtual void runInternal()=0;
	};

	// same as Cm::Task but inheriting from physx::PxBaseTask
	// instead of PxLightCpuTask
	class BaseTask : public physx::PxBaseTask
	{
	public:

		virtual void run()
		{
#if PX_SWITCH  // special case because default rounding mode is not nearest
			PX_FPU_GUARD;
#else
			PX_SIMD_GUARD;
#endif
			runInternal();
		}

		virtual void runInternal()=0;
	};

	template <class T, void (T::*Fn)(physx::PxBaseTask*) >
	class DelegateTask : public Cm::Task, public shdfnd::UserAllocated
	{
	public:

		DelegateTask(PxU64 contextID, T* obj, const char* name) : Cm::Task(contextID), mObj(obj), mName(name) {}

		virtual void runInternal()
		{
			(mObj->*Fn)(mCont);
		}

		virtual const char* getName() const
		{
			return mName;
		}

		void setObject(T* obj) { mObj = obj; }

	private:
		T* mObj;
		const char* mName;
	};


	/**
	\brief A task that maintains a list of dependent tasks.
	
	This task maintains a list of dependent tasks that have their reference counts 
	reduced on completion of the task.

	The refcount is incremented every time a dependent task is added.
	*/
	class FanoutTask : public Cm::BaseTask
	{
		PX_NOCOPY(FanoutTask)
	public:
		FanoutTask(PxU64 contextID, const char* name) : Cm::BaseTask(), mRefCount(0), mName(name), mNotifySubmission(false) { mContextID = contextID; }

		virtual void runInternal() {}

		virtual const char* getName() const { return mName; }

		/**
		Swap mDependents with mReferencesToRemove when refcount goes to 0.
		*/
		virtual void removeReference()
		{
			shdfnd::Mutex::ScopedLock lock(mMutex);
			if (!physx::shdfnd::atomicDecrement(&mRefCount))
			{
				// prevents access to mReferencesToRemove until release
				physx::shdfnd::atomicIncrement(&mRefCount);
				mNotifySubmission = false;
				PX_ASSERT(mReferencesToRemove.empty());
				for (PxU32 i = 0; i < mDependents.size(); i++)
					mReferencesToRemove.pushBack(mDependents[i]);
				mDependents.clear();
				mTm->getCpuDispatcher()->submitTask(*this);
			}
		}

		/** 
		\brief Increases reference count
		*/
		virtual void addReference()
		{
			shdfnd::Mutex::ScopedLock lock(mMutex);
			physx::shdfnd::atomicIncrement(&mRefCount);
			mNotifySubmission = true;
		}

		/** 
		\brief Return the ref-count for this task 
		*/
		PX_INLINE PxI32 getReference() const
		{
			return mRefCount;
		}

		/**
		Sets the task manager. Doesn't increase the reference count.
		*/
		PX_INLINE void setTaskManager(physx::PxTaskManager& tm)
		{
			mTm = &tm;
		}

		/**
		Adds a dependent task. It also sets the task manager querying it from the dependent task.  
		The refcount is incremented every time a dependent task is added.
		*/
		PX_INLINE void addDependent(physx::PxBaseTask& dependent)
		{
			shdfnd::Mutex::ScopedLock lock(mMutex);
			physx::shdfnd::atomicIncrement(&mRefCount);
			mTm = dependent.getTaskManager();
			mDependents.pushBack(&dependent);
			dependent.addReference();
			mNotifySubmission = true;
		}

		/**
		Reduces reference counts of the continuation task and the dependent tasks, also 
		clearing the copy of continuation and dependents task list.
		*/
		virtual void release()
		{
			Ps::InlineArray<physx::PxBaseTask*, 10> referencesToRemove;

			{
				shdfnd::Mutex::ScopedLock lock(mMutex);

				const PxU32 contCount = mReferencesToRemove.size(); 
				referencesToRemove.reserve(contCount);
				for (PxU32 i=0; i < contCount; ++i)
					referencesToRemove.pushBack(mReferencesToRemove[i]);
				
				mReferencesToRemove.clear();
				// allow access to mReferencesToRemove again
				if (mNotifySubmission)
				{
					removeReference();
				}
				else
				{
					physx::shdfnd::atomicDecrement(&mRefCount);
				}

				// the scoped lock needs to get freed before the continuation tasks get (potentially) submitted because
				// those continuation tasks might trigger events that delete this task and corrupt the memory of the
				// mutex (for example, assume this task is a member of the scene then the submitted tasks cause the simulation 
				// to finish and then the scene gets released which in turn will delete this task. When this task then finally
				// continues the heap memory will be corrupted.
			}

			for (PxU32 i=0; i < referencesToRemove.size(); ++i)
				referencesToRemove[i]->removeReference();
		}

	protected:
		volatile PxI32 mRefCount;
		const char* mName;
		Ps::InlineArray<physx::PxBaseTask*, 4> mDependents;
		Ps::InlineArray<physx::PxBaseTask*, 4> mReferencesToRemove;
		bool mNotifySubmission;
		Ps::Mutex mMutex; // guarding mDependents and mNotifySubmission
	};


	/**
	\brief Specialization of FanoutTask class in order to provide the delegation mechanism.
	*/
	template <class T, void (T::*Fn)(physx::PxBaseTask*) >
	class DelegateFanoutTask : public FanoutTask, public shdfnd::UserAllocated
	{
	public:
		DelegateFanoutTask(PxU64 contextID, T* obj, const char* name) : 
		  FanoutTask(contextID, name), mObj(obj) { }

		  virtual void runInternal()
		  {
			  physx::PxBaseTask* continuation = mReferencesToRemove.empty() ? NULL : mReferencesToRemove[0];
			  (mObj->*Fn)(continuation);
		  }

		  void setObject(T* obj) { mObj = obj; }

	private:
		T* mObj;
	};

} // namespace Cm

}

#endif
