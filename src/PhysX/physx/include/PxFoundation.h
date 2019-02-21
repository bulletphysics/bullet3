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

#ifndef PX_FOUNDATION_PX_FOUNDATION_H
#define PX_FOUNDATION_PX_FOUNDATION_H

/** \addtogroup foundation
  @{
*/

#include "foundation/Px.h"
#include "foundation/PxErrors.h"
#include "foundation/PxFoundationConfig.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

/**
\brief Foundation SDK singleton class.

You need to have an instance of this class to instance the higher level SDKs.
*/
class PX_FOUNDATION_API PxFoundation
{
  public:
	/**
	\brief Destroys the instance it is called on.

	The operation will fail, if there are still modules referencing the foundation object. Release all dependent modules
	prior
	to calling this method.

	@see PxCreateFoundation()
	*/
	virtual void release() = 0;

	/**
	retrieves error callback
	*/
	virtual PxErrorCallback& getErrorCallback() = 0;

	/**
	Sets mask of errors to report.
	*/
	virtual void setErrorLevel(PxErrorCode::Enum mask = PxErrorCode::eMASK_ALL) = 0;

	/**
	Retrieves mask of errors to be reported.
	*/
	virtual PxErrorCode::Enum getErrorLevel() const = 0;

	/**
	Retrieves the allocator this object was created with.
	*/
	virtual PxAllocatorCallback& getAllocatorCallback() = 0;

	/**
	Retrieves if allocation names are being passed to allocator callback.
	*/
	virtual bool getReportAllocationNames() const = 0;

	/**
	Set if allocation names are being passed to allocator callback.
	\details Enabled by default in debug and checked build, disabled by default in profile and release build.
	*/
	virtual void setReportAllocationNames(bool value) = 0;

  protected:
	virtual ~PxFoundation()
	{
	}
};

#if !PX_DOXYGEN
} // namespace physx
#endif

/**
\brief Creates an instance of the foundation class

The foundation class is needed to initialize higher level SDKs. There may be only one instance per process.
Calling this method after an instance has been created already will result in an error message and NULL will be
returned.

\param version Version number we are expecting (should be #PX_PHYSICS_VERSION)
\param allocator User supplied interface for allocating memory(see #PxAllocatorCallback)
\param errorCallback User supplied interface for reporting errors and displaying messages(see #PxErrorCallback)
\return Foundation instance on success, NULL if operation failed

@see PxFoundation
*/

PX_C_EXPORT PX_FOUNDATION_API physx::PxFoundation* PX_CALL_CONV
PxCreateFoundation(physx::PxU32 version, physx::PxAllocatorCallback& allocator, physx::PxErrorCallback& errorCallback);
/**
\brief Retrieves the Foundation SDK after it has been created.

\note The behavior of this method is undefined if the foundation instance has not been created already.

@see PxCreateFoundation()
*/
#if PX_CLANG
#if PX_LINUX
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
#endif // PX_LINUX
#endif // PX_CLANG
PX_C_EXPORT PX_FOUNDATION_API physx::PxFoundation& PX_CALL_CONV PxGetFoundation();
#if PX_CLANG
#if PX_LINUX
#pragma clang diagnostic pop
#endif // PX_LINUX
#endif // PX_CLANG

namespace physx
{
class PxProfilerCallback;
}

/**
\brief Get the callback that will be used for all profiling.
*/
PX_C_EXPORT PX_FOUNDATION_API physx::PxProfilerCallback* PX_CALL_CONV PxGetProfilerCallback();

/**
\brief Set the callback that will be used for all profiling.
*/
PX_C_EXPORT PX_FOUNDATION_API void PX_CALL_CONV PxSetProfilerCallback(physx::PxProfilerCallback* profiler);

/** @} */
#endif // PX_FOUNDATION_PX_FOUNDATION_H
