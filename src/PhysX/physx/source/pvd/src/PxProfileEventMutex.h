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


#ifndef PXPVDSDK_PXPROFILEEVENTMUTEX_H
#define PXPVDSDK_PXPROFILEEVENTMUTEX_H

#include "foundation/Px.h"

namespace physx { namespace profile {
	
	/**
	 *	Mutex interface that hides implementation around lock and unlock.
	 *	The event system locks the mutex for every interaction.
	 */
	class PxProfileEventMutex
	{
	protected:
		virtual ~PxProfileEventMutex(){}
	public:
		virtual void lock() = 0;
		virtual void unlock() = 0;
	};

	/**
	 * Take any mutex type that implements lock and unlock and make an EventMutex out of it.
	 */
	template<typename TMutexType>
	struct PxProfileEventMutexImpl : public PxProfileEventMutex
	{
		TMutexType* mMutex;
		PxProfileEventMutexImpl( TMutexType* inMtx ) : mMutex( inMtx ) {}
		virtual void lock() { mMutex->lock(); }
		virtual void unlock() { mMutex->unlock(); }
	};

} }

#endif // PXPVDSDK_PXPROFILEEVENTMUTEX_H
