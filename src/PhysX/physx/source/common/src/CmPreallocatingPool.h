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


#ifndef PX_PHYSICS_COMMON_PREALLOCATINGPOOL
#define PX_PHYSICS_COMMON_PREALLOCATINGPOOL

#include "foundation/Px.h"
#include "PsUserAllocated.h"
#include "CmPhysXCommon.h"
#include "PsSort.h"
#include "PsArray.h"

/*
Pool used to allocate variable sized tasks. It's intended to be cleared after a short period (time step).
*/

namespace physx
{
namespace Cm
{

class PreallocatingRegion
{
public:
	PX_FORCE_INLINE		PreallocatingRegion() : mMemory(NULL), mFirstFree(NULL), mNbElements(0)	{}

	void		init(PxU32 maxElements, PxU32 elementSize, const char* typeName)
	{	
		mFirstFree	= NULL;
		mNbElements	= 0;
		PX_ASSERT(typeName);
		PX_UNUSED(typeName);
		mMemory = reinterpret_cast<PxU8*>(PX_ALLOC(sizeof(PxU8)*elementSize*maxElements, typeName?typeName:"SceneSim Pool"));	// ### addActor alloc
		PX_ASSERT(elementSize*maxElements>=sizeof(void*));
	}

	void		reset()
	{
		PX_FREE_AND_RESET(mMemory);
	}

	PX_FORCE_INLINE PxU8* allocateMemory(PxU32 maxElements, PxU32 elementSize)
	{
		if(mFirstFree)
		{
			PxU8* recycled = reinterpret_cast<PxU8*>(mFirstFree);

			void** recycled32 = reinterpret_cast<void**>(recycled);
			mFirstFree = *recycled32;

			return recycled;
		}
		else
		{
			if(mNbElements==maxElements)
				return NULL;	// Out of memory

			const PxU32 freeIndex = mNbElements++;
			return mMemory + freeIndex * elementSize;
		}
	}

	void		deallocateMemory(PxU32 maxElements, PxU32 elementSize, PxU8* element)
	{
		PX_ASSERT(element);
		PX_ASSERT(element>=mMemory && element<mMemory + maxElements * elementSize);
		PX_UNUSED(elementSize);
		PX_UNUSED(maxElements);

		void** recycled32 = reinterpret_cast<void**>(element);
		*recycled32 = mFirstFree;

		mFirstFree = element;
	}

	PX_FORCE_INLINE bool operator < (const PreallocatingRegion& p) const
	{
		return mMemory < p.mMemory;
	}

	PX_FORCE_INLINE bool operator > (const PreallocatingRegion& p) const
	{
		return mMemory > p.mMemory;
	}

	PxU8*		mMemory;
	void*		mFirstFree;
	PxU32		mNbElements;
};



class PreallocatingRegionManager
{
	public:
						PreallocatingRegionManager(PxU32 maxElements, PxU32 elementSize, const char* typeName)
						: mMaxElements		(maxElements)
						, mElementSize		(elementSize)
						, mActivePoolIndex	(0)
						, mPools(PX_DEBUG_EXP("MyPoolManagerPools"))
						, mNeedsSorting		(true)
						, mTypeName			(typeName)
						{
							PreallocatingRegion tmp;
							tmp.init(maxElements, elementSize, mTypeName);
							mPools.pushBack(tmp);
						}

						~PreallocatingRegionManager()
						{
							const PxU32 nbPools = mPools.size();
							for(PxU32 i=0;i<nbPools;i++)
								mPools[i].reset();
						}

	void				preAllocate(PxU32 n)
	{
		if(!n)
			return;

		const PxU32 nbPools = mPools.size();
		const PxU32 maxElements = mMaxElements;
		const PxU32 elementSize = mElementSize;
		PxU32 availableSpace = nbPools * maxElements;

		while(n>availableSpace)
		{
			PreallocatingRegion tmp;
			tmp.init(maxElements, elementSize, mTypeName);
			mPools.pushBack(tmp);

			availableSpace += maxElements;
		}
	}

	PX_FORCE_INLINE PxU8* allocateMemory()
	{
		PX_ASSERT(mActivePoolIndex<mPools.size());
		PxU8* memory = mPools[mActivePoolIndex].allocateMemory(mMaxElements, mElementSize);
		return memory ? memory : searchForMemory();
	}

	void				deallocateMemory(PxU8* element)
	{
		if(!element)
			return;

		if(mNeedsSorting)
			Ps::sort(mPools.begin(), mPools.size());

		const PxU32 maxElements = mMaxElements;
		const PxU32 elementSize = mElementSize;
		const PxU32 slabSize = maxElements * elementSize;
		const PxU32 nbPools = mPools.size();

		// O(log n) search
		int first = 0;
		int last = int(nbPools-1);

		while(first<=last)
		{
			const int mid = (first+last)>>1;

			PreallocatingRegion& candidate = mPools[PxU32(mid)];
			if(contains(candidate.mMemory, slabSize, element))
			{
				candidate.deallocateMemory(maxElements, elementSize, element);
				
				// when we sorted earlier we trashed the active index, but at least this region has a free element
				if(mNeedsSorting)	
					mActivePoolIndex = PxU32(mid);

				mNeedsSorting = false;
				return;
			}

			if(candidate.mMemory<element)
				first = mid+1;
			else
				last = mid-1;
		}

		PX_ASSERT(0);
	}


private:

	PreallocatingRegionManager& operator=(const PreallocatingRegionManager&);
	PxU8*				searchForMemory()
	{
		const PxU32 nbPools = mPools.size();
		const PxU32 activePoolIndex = mActivePoolIndex;
		const PxU32 maxElements = mMaxElements;
		const PxU32 elementSize = mElementSize;


		for(PxU32 i=0;i<nbPools;i++)
		{
			if(i==activePoolIndex)
				continue;

			PxU8* memory = mPools[i].allocateMemory(maxElements, elementSize);
			if(memory)
			{
				mActivePoolIndex = i;
				return memory;
			}
		}

		mActivePoolIndex = nbPools;
		mNeedsSorting = true;

		PreallocatingRegion tmp;
		tmp.init(maxElements, elementSize, mTypeName);

		PreallocatingRegion& newPool = mPools.pushBack(tmp);		// ### addActor alloc (StaticSim, ShapeSim, SceneQueryShapeData)
		return newPool.allocateMemory(maxElements, elementSize);
	}



	PX_FORCE_INLINE	bool contains(PxU8* memory, const PxU32 slabSize, PxU8* element)
	{
		return element>=memory && element<memory+slabSize;
	}



	const PxU32			mMaxElements;
	const PxU32			mElementSize;
	PxU32				mActivePoolIndex;

	Ps::Array<PreallocatingRegion>	mPools;
	bool				mNeedsSorting;
	const char*			mTypeName;
};

template<class T>
class PreallocatingPool : public Ps::UserAllocated
{
	PreallocatingPool<T>& operator=(const PreallocatingPool<T>&);

public:
	PreallocatingPool(PxU32 maxElements, const char* typeName) : mPool(maxElements, sizeof(T), typeName)
	{
	}

	~PreallocatingPool()
	{
	}

	PX_FORCE_INLINE	void	preAllocate(PxU32 n)
	{
		mPool.preAllocate(n);
	}

	PX_INLINE T* allocate()
	{
		return reinterpret_cast<T*>(mPool.allocateMemory());
	}

	PX_FORCE_INLINE T* allocateAndPrefetch()
	{
		T* t = reinterpret_cast<T*>(mPool.allocateMemory());
		Ps::prefetch(t, sizeof(T));
		return t;
	}

	PX_INLINE T* construct()
	{
		T* t = reinterpret_cast<T*>(mPool.allocateMemory());
		return t ? new (t) T() : 0;
	}

	template<class A1>
	PX_INLINE T* construct(A1& a)
	{
		T* t = reinterpret_cast<T*>(mPool.allocateMemory());
		return t ? new (t) T(a) : 0;
	}

	template<class A1, class A2>
	PX_INLINE T* construct(A1& a, A2& b)
	{
		T* t = reinterpret_cast<T*>(mPool.allocateMemory());
		return t ? new (t) T(a,b) : 0;
	}

	template<class A1, class A2, class A3>
	PX_INLINE T* construct(A1& a, A2& b, A3& c)
	{
		T* t = reinterpret_cast<T*>(mPool.allocateMemory());
		return t ? new (t) T(a,b,c) : 0;
	}

	template<class A1, class A2, class A3, class A4>
	PX_INLINE T* construct(A1& a, A2& b, A3& c, A4& d)
	{
		T* t = reinterpret_cast<T*>(mPool.allocateMemory());
		return t ? new (t) T(a,b,c,d) : 0;
	}

	template<class A1, class A2, class A3, class A4, class A5>
	PX_INLINE T* construct(A1& a, A2& b, A3& c, A4& d, A5& e)
	{
		T* t = reinterpret_cast<T*>(mPool.allocateMemory());
		return t ? new (t) T(a,b,c,d,e) : 0;
	}

	////

	PX_INLINE T* construct(T* t)
	{
		PX_ASSERT(t);
		return new (t) T();
	}

	template<class A1>
	PX_INLINE T* construct(T* t, A1& a)
	{
		PX_ASSERT(t);
		return new (t) T(a);
	}

	template<class A1, class A2>
	PX_INLINE T* construct(T* t, A1& a, A2& b)
	{
		PX_ASSERT(t);
		return new (t) T(a,b);
	}

	template<class A1, class A2, class A3>
	PX_INLINE T* construct(T* t, A1& a, A2& b, A3& c)
	{
		PX_ASSERT(t);
		return new (t) T(a,b,c);
	}

	template<class A1, class A2, class A3, class A4>
	PX_INLINE T* construct(T* t, A1& a, A2& b, A3& c, A4& d)
	{
		PX_ASSERT(t);
		return new (t) T(a,b,c,d);
	}

	template<class A1, class A2, class A3, class A4, class A5>
	PX_INLINE T* construct(T* t, A1& a, A2& b, A3& c, A4& d, A5& e)
	{
		PX_ASSERT(t);
		return new (t) T(a,b,c,d,e);
	}

	PX_INLINE void destroy(T* const p)
	{
		if(p)
		{
			p->~T();
			mPool.deallocateMemory(reinterpret_cast<PxU8*>(p));
		}
	}

	PX_INLINE void releasePreallocated(T* const p)
	{
		if(p)
			mPool.deallocateMemory(reinterpret_cast<PxU8*>(p));
	}
protected:
	PreallocatingRegionManager	mPool;
};

template<class T>
class BufferedPreallocatingPool : public PreallocatingPool<T>
{
	Ps::Array<T*> mDeletedElems;
	PX_NOCOPY(BufferedPreallocatingPool<T>)
public:
	BufferedPreallocatingPool(PxU32 maxElements, const char* typeName) : PreallocatingPool<T>(maxElements, typeName)
	{
	}

	PX_INLINE void destroy(T* const p)
	{
		if (p)
		{
			p->~T();
			mDeletedElems.pushBack(p);
		}
	}

	void processPendingDeletedElems()
	{
		for (PxU32 i = 0; i < mDeletedElems.size(); ++i)
			this->mPool.deallocateMemory(reinterpret_cast<PxU8*>(mDeletedElems[i]));
		mDeletedElems.clear();
	}


};



	
} // namespace Cm

}

#endif
