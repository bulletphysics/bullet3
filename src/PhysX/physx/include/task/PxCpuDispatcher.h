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

#ifndef PXTASK_PXCPUDISPATCHER_H
#define PXTASK_PXCPUDISPATCHER_H

#include "task/PxTaskDefine.h"
#include "foundation/PxSimpleTypes.h"

namespace physx
{

class PxBaseTask;

/** 
 \brief A CpuDispatcher is responsible for scheduling the execution of tasks passed to it by the SDK.

 A typical implementation would for example use a thread pool with the dispatcher
 pushing tasks onto worker thread queues or a global queue.

 @see PxBaseTask
 @see PxTask
 @see PxTaskManager
*/
class PxCpuDispatcher
{
public:
	/**
	\brief Called by the TaskManager when a task is to be queued for execution.
	
	Upon receiving a task, the dispatcher should schedule the task
	to run when resource is available.  After the task has been run,
	it should call the release() method and discard it's pointer.

	\param[in] task The task to be run.

	@see PxBaseTask
	*/
    virtual void submitTask( PxBaseTask& task ) = 0;

	/**
	\brief Returns the number of available worker threads for this dispatcher.
	
	The SDK will use this count to control how many tasks are submitted. By
	matching the number of tasks with the number of execution units task
	overhead can be reduced.
	*/
	virtual uint32_t getWorkerCount() const = 0;

	virtual ~PxCpuDispatcher() {}
};

} // end physx namespace

#endif // PXTASK_PXCPUDISPATCHER_H
