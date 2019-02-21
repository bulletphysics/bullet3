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

#ifndef PXCUDACONTEXTMANAGER_PXGPUCOPYDESC_H
#define PXCUDACONTEXTMANAGER_PXGPUCOPYDESC_H

#include "foundation/PxPreprocessor.h"

#if PX_SUPPORT_GPU_PHYSX

#include "task/PxTaskDefine.h"

namespace physx
{

PX_PUSH_PACK_DEFAULT

/**
 * \brief Input descriptor for the GpuDispatcher's built-in copy kernel
 *
 * All host memory involved in copy transactions must be page-locked.
 * If more than one descriptor is passed to the copy kernel in one launch,
 * the descriptors themselves must be in page-locked memory.
 */
struct PxGpuCopyDesc
{
	/**
	 * \brief Input descriptor for the GpuDispatcher's built-in copy kernel
	 */
	enum CopyType
	{
		HostToDevice,
		DeviceToHost,
		DeviceToDevice,
		DeviceMemset32
	};

	size_t		dest;	//!< the destination 
	size_t		source; //!< the source (32bit value when type == DeviceMemset)
	size_t		bytes;	//!< the size in bytes
	CopyType	type;	//!< the memory transaction type

	/** 
	 * \brief Copy is optimally performed as 64bit words, requires 64bit alignment.  But it can
	 * gracefully degrade to 32bit copies if necessary
	 */
	PX_INLINE bool isValid()
	{
		bool ok = true;
		ok &= ((dest & 0x3) == 0);
		ok &= ((type == DeviceMemset32) || (source & 0x3) == 0);
		ok &= ((bytes & 0x3) == 0);
		return ok;
	}
};

PX_POP_PACK

} // end physx namespace

#endif // PX_SUPPORT_GPU_PHYSX
#endif // PXCUDACONTEXTMANAGER_PXGPUCOPYDESC_H
