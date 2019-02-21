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

#ifndef PSFOUNDATION_PSSYNC_H
#define PSFOUNDATION_PSSYNC_H

#include "PsAllocator.h"

namespace physx
{
namespace shdfnd
{
/*!
Implementation notes:
* - Calling set() on an already signaled Sync does not change its state.
* - Calling reset() on an already reset Sync does not change its state.
* - Calling set() on a reset Sync wakes all waiting threads (potential for thread contention).
* - Calling wait() on an already signaled Sync will return true immediately.
* - NOTE: be careful when pulsing an event with set() followed by reset(), because a
*   thread that is not waiting on the event will miss the signal.
*/
class PX_FOUNDATION_API SyncImpl
{
  public:
	static const uint32_t waitForever = 0xffffffff;

	SyncImpl();

	~SyncImpl();

	/** Wait on the object for at most the given number of ms. Returns
	*  true if the object is signaled. Sync::waitForever will block forever
	*  or until the object is signaled.
	*/

	bool wait(uint32_t milliseconds = waitForever);

	/** Signal the synchronization object, waking all threads waiting on it */

	void set();

	/** Reset the synchronization object */

	void reset();

	/**
	Size of this class.
	*/
	static uint32_t getSize();
};

/*!
Implementation notes:
* - Calling set() on an already signaled Sync does not change its state.
* - Calling reset() on an already reset Sync does not change its state.
* - Calling set() on a reset Sync wakes all waiting threads (potential for thread contention).
* - Calling wait() on an already signaled Sync will return true immediately.
* - NOTE: be careful when pulsing an event with set() followed by reset(), because a
*   thread that is not waiting on the event will miss the signal.
*/
template <typename Alloc = ReflectionAllocator<SyncImpl> >
class SyncT : protected Alloc
{
  public:
	static const uint32_t waitForever = SyncImpl::waitForever;

	SyncT(const Alloc& alloc = Alloc()) : Alloc(alloc)
	{
		mImpl = reinterpret_cast<SyncImpl*>(Alloc::allocate(SyncImpl::getSize(), __FILE__, __LINE__));
		PX_PLACEMENT_NEW(mImpl, SyncImpl)();
	}

	~SyncT()
	{
		mImpl->~SyncImpl();
		Alloc::deallocate(mImpl);
	}

	/** Wait on the object for at most the given number of ms. Returns
	*  true if the object is signaled. Sync::waitForever will block forever
	*  or until the object is signaled.
	*/

	bool wait(uint32_t milliseconds = SyncImpl::waitForever)
	{
		return mImpl->wait(milliseconds);
	}

	/** Signal the synchronization object, waking all threads waiting on it */

	void set()
	{
		mImpl->set();
	}

	/** Reset the synchronization object */

	void reset()
	{
		mImpl->reset();
	}

  private:
	class SyncImpl* mImpl;
};

typedef SyncT<> Sync;

} // namespace shdfnd
} // namespace physx

#endif // #ifndef PSFOUNDATION_PSSYNC_H
