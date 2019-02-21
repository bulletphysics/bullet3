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

#ifndef PX_PHYSICS_EXTENSIONS_DEFAULT_ERROR_CALLBACK_H
#define PX_PHYSICS_EXTENSIONS_DEFAULT_ERROR_CALLBACK_H

#include "foundation/PxErrorCallback.h"
#include "PxPhysXConfig.h"

#if !PX_DOXYGEN
namespace physx
{
#endif

	/**
	\brief default implementation of the error callback

	This class is provided in order to enable the SDK to be started with the minimum of user code. Typically an application
	will use its own error callback, and log the error to file or otherwise make it visible. Warnings and error messages from
	the SDK are usually indicative that changes are required in order for PhysX to function correctly, and should not be ignored.
	*/

	class PxDefaultErrorCallback : public PxErrorCallback
	{
	public:
		PxDefaultErrorCallback();
		~PxDefaultErrorCallback();

		virtual void reportError(PxErrorCode::Enum code, const char* message, const char* file, int line);
	};

#if !PX_DOXYGEN
} // namespace physx
#endif

#endif
