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

#ifndef PXPVDSDK_PXPROFILECONTEXTPROVIDER_H
#define PXPVDSDK_PXPROFILECONTEXTPROVIDER_H

#include "foundation/Px.h"

namespace physx { namespace profile {

	struct PxProfileEventExecutionContext
	{
		uint32_t				mThreadId;
		uint8_t					mCpuId;
		uint8_t					mThreadPriority;

		PxProfileEventExecutionContext( uint32_t inThreadId = 0, uint8_t inThreadPriority = 2 /*eThreadPriorityNormal*/, uint8_t inCpuId = 0 )
			: mThreadId( inThreadId )
			, mCpuId( inCpuId )
			, mThreadPriority( inThreadPriority )
		{
		}

		bool operator==( const PxProfileEventExecutionContext& inOther ) const
		{
			return mThreadId == inOther.mThreadId
				&& mCpuId == inOther.mCpuId
				&& mThreadPriority == inOther.mThreadPriority;
		}
	};

} }

#endif // PXPVDSDK_PXPROFILECONTEXTPROVIDER_H
