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


#ifndef PX_PHYSICS_EXTENSIONS_DEFAULT_CPU_DISPATCHER_H
#define PX_PHYSICS_EXTENSIONS_DEFAULT_CPU_DISPATCHER_H
/** \addtogroup extensions
  @{
*/

#include "common/PxPhysXCommonConfig.h"
#include "task/PxCpuDispatcher.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief A default implementation for a CPU task dispatcher.

@see PxDefaultCpuDispatcherCreate() PxCpuDispatcher
*/
class PxDefaultCpuDispatcher: public PxCpuDispatcher
{
public:
	/**
	\brief Deletes the dispatcher.
	
	Do not keep a reference to the deleted instance.

	@see PxDefaultCpuDispatcherCreate()
	*/
	virtual void release() = 0;

	/**
	\brief Enables profiling at task level.

	\note By default enabled only in profiling builds.
	
	\param[in] runProfiled True if tasks should be profiled.
	*/
	virtual void setRunProfiled(bool runProfiled) = 0;

	/**
	\brief Checks if profiling is enabled at task level.

	\return True if tasks should be profiled.
	*/
	virtual bool getRunProfiled() const = 0;
};


/**
\brief Create default dispatcher, extensions SDK needs to be initialized first.

\param[in] numThreads Number of worker threads the dispatcher should use.
\param[in] affinityMasks Array with affinity mask for each thread. If not defined, default masks will be used.

\note numThreads may be zero in which case no worker thread are initialized and
simulation tasks will be executed on the thread that calls PxScene::simulate()

@see PxDefaultCpuDispatcher
*/
PxDefaultCpuDispatcher* PxDefaultCpuDispatcherCreate(PxU32 numThreads, PxU32* affinityMasks = NULL);

#if !PX_DOXYGEN
} // namespace physx
#endif

/** @} */
#endif
