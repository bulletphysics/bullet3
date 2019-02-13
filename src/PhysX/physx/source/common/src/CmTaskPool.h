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


#ifndef PX_PHYSICS_COMMON_TASKPOOL
#define PX_PHYSICS_COMMON_TASKPOOL

#include "foundation/Px.h"
#include "PsMutex.h"
#include "PsSList.h"
#include "PsAllocator.h"
#include "PsArray.h"

class PxTask;

/*
Implimentation of a thread safe task pool. (PxTask derived classes).

T is the actual type of the task(currently NphaseTask or GroupSolveTask).
*/

namespace Cm
{
	template<class T> class TaskPool : public Ps::AlignedAllocator<16>
	{
		const static PxU32 TaskPoolSlabSize=64;

	public:

		typedef Ps::SListEntry TaskPoolItem;

		PX_INLINE TaskPool() : slabArray(PX_DEBUG_EXP("taskPoolSlabArray"))
		{
			//we have to ensure that the list header is 16byte aligned for win64.
			freeTasks = (Ps::SList*)allocate(sizeof(Ps::SList), __FILE__, __LINE__);
			PX_PLACEMENT_NEW(freeTasks, Ps::SList)();

			slabArray.reserve(16);
		}

		~TaskPool()
		{
			Ps::Mutex::ScopedLock lock(slabAllocMutex);

			freeTasks->flush();

			for(PxU32 i=0;i<slabArray.size();i++)
			{
				// call destructors
				for(PxU32 j=0; j<TaskPoolSlabSize; j++)
					slabArray[i][j].~T();

				deallocate(slabArray[i]);
			}

			slabArray.clear();

			if(freeTasks!=NULL)
			{
				freeTasks->~SList();
				deallocate(freeTasks);
				freeTasks = NULL;
			}
		}

		T *allocTask()
		{
			T *rv = static_cast<T *>(freeTasks->pop());
			if(rv == NULL)
				return static_cast<T *>(allocateSlab());
			else
				return rv;
		}
		void freeTask(T *task)
		{
			freeTasks->push(*task);
		}

	private:

		T *allocateSlab()
		{
			//ack, convoluted memory macros.

			//T *newSlab=new T[TaskPoolSlabSize];

			// we must align this memory.
			T *newSlab=(T *)allocate(sizeof(T)*TaskPoolSlabSize, __FILE__, __LINE__);

			new (newSlab) T();

			//we keep one for the caller
			// and build a list of tasks and insert in the free list
			for(PxU32 i=1;i<TaskPoolSlabSize;i++)
			{
				new (&(newSlab[i])) T();
				freeTasks->push(newSlab[i]);
			}

			Ps::Mutex::ScopedLock lock(slabAllocMutex);
			slabArray.pushBack(newSlab);

			return newSlab;
		}

		Ps::Mutex slabAllocMutex;
		Ps::Array<T *> slabArray;

		Ps::SList *freeTasks;

	};


} // namespace Cm


#endif
