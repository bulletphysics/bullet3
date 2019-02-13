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


#ifndef PXC_THREADCOHERENTCACHE_H
#define PXC_THREADCOHERENTCACHE_H

#include "PsMutex.h"
#include "PsAllocator.h"
#include "PsSList.h"

namespace physx
{

class PxsContext;
/*!
Controls a pool of large objects which must be thread safe. 
Tries to return the object most recently used by the thread(for better cache coherancy).
Assumes the object has a default contructor.

(Note the semantics are different to a pool because we dont want to construct/destroy each time
an object is requested, which may be expensive).

TODO: add thread coherancy.
*/
template<class T, class Params>
class PxcThreadCoherentCache : public Ps::AlignedAllocator<16, Ps::ReflectionAllocator<T> >
{
	typedef Ps::AlignedAllocator<16, Ps::ReflectionAllocator<T> > Allocator;
	PX_NOCOPY(PxcThreadCoherentCache)
public:

	typedef Ps::SListEntry EntryBase;

	PX_INLINE PxcThreadCoherentCache(Params* params, const Allocator& alloc = Allocator()) : Allocator(alloc), mParams(params)
	{
	}

	PX_INLINE ~PxcThreadCoherentCache()
	{
		T* np = static_cast<T*>(root.pop());

		while(np!=NULL)
		{
			np->~T();
			Allocator::deallocate(np);
			np = static_cast<T*>(root.pop());
		}
	}

	PX_INLINE T* get()
	{
		T* rv = static_cast<T*>(root.pop());
		if(rv==NULL)
		{
			rv = reinterpret_cast<T*>(Allocator::allocate(sizeof(T), __FILE__, __LINE__));
			new (rv) T(mParams);
		}

		return rv;
	}

	PX_INLINE void put(T* item)
	{
		root.push(*item);
	}


private:
	Ps::SList root;
	Params* mParams;

	template<class T2, class P2>
	friend class PxcThreadCoherentCacheIterator;
};

/*!
Used to iterate over all objects controlled by the cache.

Note: The iterator flushes the cache(extracts all items on construction and adds them back on
destruction so we can iterate the list in a safe manner).
*/
template<class T, class Params> 
class PxcThreadCoherentCacheIterator
{
public:
	PxcThreadCoherentCacheIterator(PxcThreadCoherentCache<T, Params>& cache) : mCache(cache)
	{
		mNext = cache.root.flush();
		mFirst = mNext;
	}
	~PxcThreadCoherentCacheIterator()
	{
		Ps::SListEntry* np = mFirst;
		while(np != NULL)
		{
			Ps::SListEntry* npNext = np->next();
			mCache.root.push(*np);
			np = npNext;
		}
	}

	PX_INLINE T* getNext()
	{
		if(mNext == NULL)
			return NULL;

		T* rv = static_cast<T*>(mNext);
		mNext = mNext->next();

		return rv;
	}
private:

	PxcThreadCoherentCacheIterator<T, Params>& operator=(const PxcThreadCoherentCacheIterator<T, Params>&);
	PxcThreadCoherentCache<T, Params> &mCache;
	Ps::SListEntry* mNext;
	Ps::SListEntry* mFirst;
	
};

}

#endif
