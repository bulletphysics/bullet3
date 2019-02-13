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
// Copyright (c) 2004-2008 AGEIA Technologies, Inc. All rights reserved.
// Copyright (c) 2001-2004 NovodeX AG. All rights reserved.

#ifndef PSFOUNDATION_PSBROADCAST_H
#define PSFOUNDATION_PSBROADCAST_H

#include "Ps.h"
#include "PsInlineArray.h"

#include "foundation/PxSimpleTypes.h"
#include "foundation/PxErrorCallback.h"

namespace physx
{
namespace shdfnd
{

/**
\brief Abstract listener class that listens to allocation and deallocation events from the
    foundation memory system.

<b>Threading:</b> All methods of this class should be thread safe as it can be called from the user thread
or the physics processing thread(s).
*/
class AllocationListener
{
  public:
	/**
	\brief callback when memory is allocated.
	\param size Size of the allocation in bytes.
	\param typeName Type this data is being allocated for.
	\param filename File the allocation came from.
	\param line the allocation came from.
	\param allocatedMemory memory that will be returned from the allocation.
	*/
	virtual void onAllocation(size_t size, const char* typeName, const char* filename, int line,
	                          void* allocatedMemory) = 0;

	/**
	\brief callback when memory is deallocated.
	\param allocatedMemory memory just before allocation.
	*/
	virtual void onDeallocation(void* allocatedMemory) = 0;

  protected:
	virtual ~AllocationListener()
	{
	}
};

/**
\brief Broadcast class implementation, registering listeners.

<b>Threading:</b> All methods of this class should be thread safe as it can be called from the user thread
or the physics processing thread(s). There is not internal locking
*/
template <class Listener, class Base>
class Broadcast : public Base
{
  public:
	static const uint32_t MAX_NB_LISTENERS = 16;

	/**
	\brief The default constructor.
	*/
	Broadcast()
	{
	}

	/**
	\brief Register new listener.

	\note It is NOT SAFE to register and deregister listeners while allocations may be taking place.
	moreover, there is no thread safety to registration/deregistration.

	\param listener Listener to register.
	*/
	void registerListener(Listener& listener)
	{
		if(mListeners.size() < MAX_NB_LISTENERS)
			mListeners.pushBack(&listener);
	}

	/**
	\brief Deregister an existing listener.

	\note It is NOT SAFE to register and deregister listeners while allocations may be taking place.
	moreover, there is no thread safety to registration/deregistration.

	\param listener Listener to deregister.
	*/
	void deregisterListener(Listener& listener)
	{
		mListeners.findAndReplaceWithLast(&listener);
	}

	/**
	\brief Get number of registered listeners.

	\return Number of listeners.
	*/
	uint32_t getNbListeners() const
	{
		return mListeners.size();
	}

	/**
	\brief Get an existing listener from given index.

	\param index Index of the listener.
	\return Listener on given index.
	*/
	Listener& getListener(uint32_t index)
	{
		PX_ASSERT(index <= mListeners.size());
		return *mListeners[index];
	}

  protected:
	virtual ~Broadcast()
	{
	}

	physx::shdfnd::InlineArray<Listener*, MAX_NB_LISTENERS, physx::shdfnd::NonTrackingAllocator> mListeners;
};

/**
\brief Abstract base class for an application defined memory allocator that allows an external listener
to audit the memory allocations.
*/
class BroadcastingAllocator : public Broadcast<AllocationListener, PxAllocatorCallback>
{
	PX_NOCOPY(BroadcastingAllocator)

  public:
	/**
	\brief The default constructor.
	*/
	BroadcastingAllocator(PxAllocatorCallback& allocator, PxErrorCallback& error) : mAllocator(allocator), mError(error)
	{
		mListeners.clear();
	}

	/**
	\brief The default constructor.
	*/
	virtual ~BroadcastingAllocator()
	{
		mListeners.clear();
	}

	/**
	\brief Allocates size bytes of memory, which must be 16-byte aligned.

	This method should never return NULL.  If you run out of memory, then
	you should terminate the app or take some other appropriate action.

	<b>Threading:</b> This function should be thread safe as it can be called in the context of the user thread
	and physics processing thread(s).

	\param size			Number of bytes to allocate.
	\param typeName		Name of the datatype that is being allocated
	\param filename		The source file which allocated the memory
	\param line			The source line which allocated the memory
	\return				The allocated block of memory.
	*/
	void* allocate(size_t size, const char* typeName, const char* filename, int line)
	{
		void* mem = mAllocator.allocate(size, typeName, filename, line);

		if(!mem)
		{
			mError.reportError(PxErrorCode::eABORT, "User allocator returned NULL.", __FILE__, __LINE__);
			return NULL;
		}

		if((reinterpret_cast<size_t>(mem) & 15))
		{
			mError.reportError(PxErrorCode::eABORT, "Allocations must be 16-byte aligned.", __FILE__, __LINE__);
			return NULL;
		}

		for(uint32_t i = 0; i < mListeners.size(); i++)
			mListeners[i]->onAllocation(size, typeName, filename, line, mem);

		return mem;
	}

	/**
	\brief Frees memory previously allocated by allocate().

	<b>Threading:</b> This function should be thread safe as it can be called in the context of the user thread
	and physics processing thread(s).

	\param ptr Memory to free.
	*/
	void deallocate(void* ptr)
	{
		for(uint32_t i = 0; i < mListeners.size(); i++)
		{
			mListeners[i]->onDeallocation(ptr);
		}
		mAllocator.deallocate(ptr);
	}

  private:
	PxAllocatorCallback& mAllocator;
	PxErrorCallback& mError;
};

/**
\brief Abstract base class for an application defined error callback that allows an external listener
to report errors.
*/
class BroadcastingErrorCallback : public Broadcast<PxErrorCallback, PxErrorCallback>
{
	PX_NOCOPY(BroadcastingErrorCallback)
  public:
	/**
	\brief The default constructor.
	*/
	BroadcastingErrorCallback(PxErrorCallback& errorCallback)
	{
		registerListener(errorCallback);
	}

	/**
	\brief The default destructor.
	*/
	virtual ~BroadcastingErrorCallback()
	{
		mListeners.clear();
	}

	/**
	\brief Reports an error code.
	\param code Error code, see #PxErrorCode
	\param message Message to display.
	\param file File error occured in.
	\param line Line number error occured on.
	*/
	void reportError(PxErrorCode::Enum code, const char* message, const char* file, int line)
	{
		for(uint32_t i = 0; i < mListeners.size(); i++)
			mListeners[i]->reportError(code, message, file, line);
	}
};
}
} // namespace physx

#endif // PSFOUNDATION_PXBROADCAST_H
