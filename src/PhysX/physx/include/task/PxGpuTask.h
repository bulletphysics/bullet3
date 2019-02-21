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


#ifndef PXTASK_PXGPUTASK_H
#define PXTASK_PXGPUTASK_H

#include "task/PxTaskDefine.h"
#include "task/PxTask.h"
#include "task/PxGpuDispatcher.h"

namespace physx
{

PX_PUSH_PACK_DEFAULT

/** \brief Define the 'flavor' of a PxGpuTask
 *
 * Each PxGpuTask should have a specific function; either copying data to the
 * device, running kernels on that data, or copying data from the device.
 *
 * For optimal performance, the dispatcher should run all available HtoD tasks
 * before running all Kernel tasks, and all Kernel tasks before running any DtoH
 * tasks.  This provides maximal kernel overlap and the least number of CUDA
 * flushes.
 */
struct PxGpuTaskHint
{
	/// \brief Enums for the type of GPU task
	enum Enum
	{
		HostToDevice,
		Kernel,
		DeviceToHost,

		NUM_GPU_TASK_HINTS
	};
};

/**
 * \brief PxTask implementation for launching CUDA work
 */
class PxGpuTask : public PxTask
{
public:
	PxGpuTask() : mComp(NULL) {}

	/**
	 * \brief iterative "run" function for a PxGpuTask
	 *
	 * The GpuDispatcher acquires the CUDA context for the duration of this
	 * function call, and it is highly recommended that the PxGpuTask use the
	 * provided CUstream for all kernels.
	 *
	 * kernelIndex will be 0 for the initial call and incremented before each
	 * subsequent call.  Once launchInstance() returns false, its PxGpuTask is
	 * considered completed and is released.
	 */
	virtual bool    launchInstance(CUstream stream, int kernelIndex) = 0;

	/**
	 * \brief Returns a hint indicating the function of this task
	 */
	virtual PxGpuTaskHint::Enum getTaskHint() const = 0;

	/**
	 * \brief Specify a task that will have its reference count decremented
	 * when this task is released
	 */
	void setCompletionTask(PxBaseTask& task)
	{
		mComp = &task;
	}

	void release()
	{
		if (mComp)
		{
			mComp->removeReference();
			mComp = NULL;
		}
		PxTask::release();
	}

protected:
	/// \brief A pointer to the completion task
	PxBaseTask* mComp;
};

PX_POP_PACK

} // end physx namespace

#endif // PXTASK_PXGPUTASK_H
