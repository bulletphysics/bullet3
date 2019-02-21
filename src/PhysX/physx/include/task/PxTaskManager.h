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

#ifndef PXTASK_PXTASKMANAGER_H
#define PXTASK_PXTASKMANAGER_H

#include "task/PxTaskDefine.h"
#include "foundation/PxSimpleTypes.h"
#include "foundation/PxErrorCallback.h"

namespace physx
{
PX_PUSH_PACK_DEFAULT

class PxBaseTask;
class PxTask;
class PxLightCpuTask;
typedef unsigned int PxTaskID;

/**
\brief Identifies the type of each heavyweight PxTask object

\note This enum type is only used by PxTask and GpuTask objects, LightCpuTasks do not use this enum.

@see PxTask
@see PxLightCpuTask
*/
struct PxTaskType
{
	/**
	 * \brief Identifies the type of each heavyweight PxTask object
	 */
	enum Enum
	{
		TT_CPU,				//!< PxTask will be run on the CPU
		TT_GPU,				//!< PxTask will be run on the GPU
		TT_NOT_PRESENT,		//!< Return code when attempting to find a task that does not exist
		TT_COMPLETED		//!< PxTask execution has been completed
	};
};

class PxCpuDispatcher;
class PxGpuDispatcher;

/** 
 \brief The PxTaskManager interface
 
 A PxTaskManager instance holds references to user-provided dispatcher objects, when tasks are
 submitted the PxTaskManager routes them to the appropriate dispatcher and handles task profiling if enabled. 
 Users should not implement the PxTaskManager interface, the SDK creates its own concrete PxTaskManager object
 per-scene which users can configure by passing dispatcher objects into the PxSceneDesc.

 @see CpuDispatcher
 @see PxGpuDispatcher
 
*/
class PxTaskManager
{
public:

	/**
	\brief Set the user-provided dispatcher object for CPU tasks

	\param[in] ref The dispatcher object.

	@see CpuDispatcher
	*/
	virtual void     setCpuDispatcher(PxCpuDispatcher& ref) = 0;

	/**
	\brief Set the user-provided dispatcher object for GPU tasks

	\param[in] ref The dispatcher object.

	@see PxGpuDispatcher
	*/
	virtual void     setGpuDispatcher(PxGpuDispatcher& ref) = 0;
	
	/**
	\brief Get the user-provided dispatcher object for CPU tasks

	\return The CPU dispatcher object.

	@see CpuDispatcher
	*/
	virtual PxCpuDispatcher*			getCpuDispatcher() const = 0;

	/**
	\brief Get the user-provided dispatcher object for GPU tasks

	\return The GPU dispatcher object.

	@see PxGpuDispatcher
	*/
	virtual PxGpuDispatcher*			getGpuDispatcher() const = 0;

	/**
	\brief Reset any dependencies between Tasks

	\note Will be called at the start of every frame before tasks are submitted.

	@see PxTask
	*/
	virtual void	resetDependencies() = 0;
	
	/**
	\brief Called by the owning scene to start the task graph.

	\note All tasks with with ref count of 1 will be dispatched.

	@see PxTask
	*/
	virtual void	startSimulation() = 0;

	/**
	\brief Called by the owning scene at the end of a simulation step to synchronize the PxGpuDispatcher

	@see PxGpuDispatcher
	*/
	virtual void	stopSimulation() = 0;

	/**
	\brief Called by the worker threads to inform the PxTaskManager that a task has completed processing

	\param[in] task The task which has been completed
	*/
	virtual void	taskCompleted(PxTask& task) = 0;

	/**
	\brief Retrieve a task by name

	\param[in] name The unique name of a task
	\return The ID of the task with that name, or TT_NOT_PRESENT if not found
	*/
	virtual PxTaskID  getNamedTask(const char* name) = 0;

	/**
	\brief Submit a task with a unique name.

	\param[in] task The task to be executed
	\param[in] name The unique name of a task
	\param[in] type The type of the task (default TT_CPU)
	\return The ID of the task with that name, or TT_NOT_PRESENT if not found
	*/
	virtual PxTaskID  submitNamedTask(PxTask* task, const char* name, PxTaskType::Enum type = PxTaskType::TT_CPU) = 0;

	/**
	\brief Submit an unnamed task.

	\param[in] task The task to be executed
	\param[in] type The type of the task (default TT_CPU)

	\return The ID of the task with that name, or TT_NOT_PRESENT if not found
	*/
	virtual PxTaskID  submitUnnamedTask(PxTask& task, PxTaskType::Enum type = PxTaskType::TT_CPU) = 0;

	/**
	\brief Retrieve a task given a task ID

	\param[in] id The ID of the task to return, a valid ID must be passed or results are undefined

	\return The task associated with the ID
	*/
	virtual PxTask*   getTaskFromID(PxTaskID id) = 0;

	/**
	\brief Release the PxTaskManager object, referenced dispatchers will not be released
	*/
	virtual void        release() = 0;

	/**
	\brief Construct a new PxTaskManager instance with the given [optional] dispatchers
	*/
	static PxTaskManager* createTaskManager(PxErrorCallback& errorCallback, PxCpuDispatcher* = 0, PxGpuDispatcher* = 0);
	
protected:
	virtual ~PxTaskManager() {}

	/*! \cond PRIVATE */

	virtual void finishBefore(PxTask& task, PxTaskID taskID) = 0;
	virtual void startAfter(PxTask& task, PxTaskID taskID) = 0;

	virtual void addReference(PxTaskID taskID) = 0;
	virtual void decrReference(PxTaskID taskID) = 0;
	virtual int32_t getReference(PxTaskID taskID) const = 0;

	virtual void decrReference(PxLightCpuTask&) = 0;
	virtual void addReference(PxLightCpuTask&) = 0;

	/*! \endcond */

	friend class PxBaseTask;
	friend class PxTask;
	friend class PxLightCpuTask;
	friend class PxGpuWorkerThread;
};

PX_POP_PACK

} // end physx namespace


#endif // PXTASK_PXTASKMANAGER_H
