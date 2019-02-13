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

#ifndef PSFOUNDATION_PSHASHINTERNALS_H
#define PSFOUNDATION_PSHASHINTERNALS_H

#include "PsBasicTemplates.h"
#include "PsArray.h"
#include "PsBitUtils.h"
#include "PsHash.h"
#include "foundation/PxIntrinsics.h"

#if PX_VC
#pragma warning(push)
#pragma warning(disable : 4127) // conditional expression is constant
#endif
namespace physx
{
namespace shdfnd
{
namespace internal
{
template <class Entry, class Key, class HashFn, class GetKey, class Allocator, bool compacting>
class HashBase : private Allocator
{
	void init(uint32_t initialTableSize, float loadFactor)
	{
		mBuffer = NULL;
		mEntries = NULL;
		mEntriesNext = NULL;
		mHash = NULL;
		mEntriesCapacity = 0;
		mHashSize = 0;
		mLoadFactor = loadFactor;
		mFreeList = uint32_t(EOL);
		mTimestamp = 0;
		mEntriesCount = 0;

		if(initialTableSize)
			reserveInternal(initialTableSize);
	}

  public:
	typedef Entry EntryType;

	HashBase(uint32_t initialTableSize = 64, float loadFactor = 0.75f) : Allocator(PX_DEBUG_EXP("hashBase"))
	{
		init(initialTableSize, loadFactor);
	}

	HashBase(uint32_t initialTableSize, float loadFactor, const Allocator& alloc) : Allocator(alloc)
	{
		init(initialTableSize, loadFactor);
	}

	HashBase(const Allocator& alloc) : Allocator(alloc)
	{
		init(64, 0.75f);
	}

	~HashBase()
	{
		destroy(); // No need to clear()

		if(mBuffer)
			Allocator::deallocate(mBuffer);
	}

	static const uint32_t EOL = 0xffffffff;

	PX_INLINE Entry* create(const Key& k, bool& exists)
	{
		uint32_t h = 0;
		if(mHashSize)
		{
			h = hash(k);
			uint32_t index = mHash[h];
			while(index != EOL && !HashFn().equal(GetKey()(mEntries[index]), k))
				index = mEntriesNext[index];
			exists = index != EOL;
			if(exists)
				return mEntries + index;
		}
		else
			exists = false;

		if(freeListEmpty())
		{
			grow();
			h = hash(k);
		}

		uint32_t entryIndex = freeListGetNext();

		mEntriesNext[entryIndex] = mHash[h];
		mHash[h] = entryIndex;

		mEntriesCount++;
		mTimestamp++;

		return mEntries + entryIndex;
	}

	PX_INLINE const Entry* find(const Key& k) const
	{
		if(!mEntriesCount)
			return NULL;

		const uint32_t h = hash(k);
		uint32_t index = mHash[h];
		while(index != EOL && !HashFn().equal(GetKey()(mEntries[index]), k))
			index = mEntriesNext[index];
		return index != EOL ? mEntries + index : NULL;
	}

	PX_INLINE bool erase(const Key& k, Entry& e)
	{
		if(!mEntriesCount)
			return false;

		const uint32_t h = hash(k);
		uint32_t* ptr = mHash + h;
		while(*ptr != EOL && !HashFn().equal(GetKey()(mEntries[*ptr]), k))
			ptr = mEntriesNext + *ptr;

		if(*ptr == EOL)
			return false;

		PX_PLACEMENT_NEW(&e, Entry)(mEntries[*ptr]);		

		return eraseInternal(ptr);
	}

	PX_INLINE bool erase(const Key& k)
	{
		if(!mEntriesCount)
			return false;

		const uint32_t h = hash(k);
		uint32_t* ptr = mHash + h;
		while(*ptr != EOL && !HashFn().equal(GetKey()(mEntries[*ptr]), k))
			ptr = mEntriesNext + *ptr;

		if(*ptr == EOL)
			return false;		

		return eraseInternal(ptr);
	}

	PX_INLINE uint32_t size() const
	{
		return mEntriesCount;
	}

	PX_INLINE uint32_t capacity() const
	{
		return mHashSize;
	}

	void clear()
	{
		if(!mHashSize || mEntriesCount == 0)
			return;

		destroy();

		intrinsics::memSet(mHash, EOL, mHashSize * sizeof(uint32_t));

		const uint32_t sizeMinus1 = mEntriesCapacity - 1;
		for(uint32_t i = 0; i < sizeMinus1; i++)
		{
			prefetchLine(mEntriesNext + i, 128);
			mEntriesNext[i] = i + 1;
		}
		mEntriesNext[mEntriesCapacity - 1] = uint32_t(EOL);
		mFreeList = 0;
		mEntriesCount = 0;
	}

	void reserve(uint32_t size)
	{
		if(size > mHashSize)
			reserveInternal(size);
	}

	PX_INLINE const Entry* getEntries() const
	{
		return mEntries;
	}

	PX_INLINE Entry* insertUnique(const Key& k)
	{
		PX_ASSERT(find(k) == NULL);
		uint32_t h = hash(k);

		uint32_t entryIndex = freeListGetNext();

		mEntriesNext[entryIndex] = mHash[h];
		mHash[h] = entryIndex;

		mEntriesCount++;
		mTimestamp++;

		return mEntries + entryIndex;
	}

  private:
	void destroy()
	{
		for(uint32_t i = 0; i < mHashSize; i++)
		{
			for(uint32_t j = mHash[i]; j != EOL; j = mEntriesNext[j])
				mEntries[j].~Entry();
		}
	}

	template <typename HK, typename GK, class A, bool comp>
	PX_NOINLINE void copy(const HashBase<Entry, Key, HK, GK, A, comp>& other);

	// free list management - if we're coalescing, then we use mFreeList to hold
	// the top of the free list and it should always be equal to size(). Otherwise,
	// we build a free list in the next() pointers.

	PX_INLINE void freeListAdd(uint32_t index)
	{
		if(compacting)
		{
			mFreeList--;
			PX_ASSERT(mFreeList == mEntriesCount);
		}
		else
		{
			mEntriesNext[index] = mFreeList;
			mFreeList = index;
		}
	}

	PX_INLINE void freeListAdd(uint32_t start, uint32_t end)
	{
		if(!compacting)
		{
			for(uint32_t i = start; i < end - 1; i++) // add the new entries to the free list
				mEntriesNext[i] = i + 1;

			// link in old free list
			mEntriesNext[end - 1] = mFreeList;
			PX_ASSERT(mFreeList != end - 1);
			mFreeList = start;
		}
		else if(mFreeList == EOL) // don't reset the free ptr for the compacting hash unless it's empty
			mFreeList = start;
	}

	PX_INLINE uint32_t freeListGetNext()
	{
		PX_ASSERT(!freeListEmpty());
		if(compacting)
		{
			PX_ASSERT(mFreeList == mEntriesCount);
			return mFreeList++;
		}
		else
		{
			uint32_t entryIndex = mFreeList;
			mFreeList = mEntriesNext[mFreeList];
			return entryIndex;
		}
	}

	PX_INLINE bool freeListEmpty() const
	{
		if(compacting)
			return mEntriesCount == mEntriesCapacity;
		else
			return mFreeList == EOL;
	}

	PX_INLINE void replaceWithLast(uint32_t index)
	{
		PX_PLACEMENT_NEW(mEntries + index, Entry)(mEntries[mEntriesCount]);
		mEntries[mEntriesCount].~Entry();
		mEntriesNext[index] = mEntriesNext[mEntriesCount];

		uint32_t h = hash(GetKey()(mEntries[index]));
		uint32_t* ptr;
		for(ptr = mHash + h; *ptr != mEntriesCount; ptr = mEntriesNext + *ptr)
			PX_ASSERT(*ptr != EOL);
		*ptr = index;
	}

	PX_INLINE uint32_t hash(const Key& k, uint32_t hashSize) const
	{
		return HashFn()(k) & (hashSize - 1);
	}

	PX_INLINE uint32_t hash(const Key& k) const
	{
		return hash(k, mHashSize);
	}

	PX_INLINE bool eraseInternal(uint32_t* ptr)
	{
		const uint32_t index = *ptr;

		*ptr = mEntriesNext[index];

		mEntries[index].~Entry();

		mEntriesCount--;
		mTimestamp++;

		if (compacting && index != mEntriesCount)
			replaceWithLast(index);

		freeListAdd(index);
		return true;
	}

	void reserveInternal(uint32_t size)
	{
		if(!isPowerOfTwo(size))
			size = nextPowerOfTwo(size);

		PX_ASSERT(!(size & (size - 1)));

		// decide whether iteration can be done on the entries directly
		bool resizeCompact = compacting || freeListEmpty();

		// define new table sizes
		uint32_t oldEntriesCapacity = mEntriesCapacity;
		uint32_t newEntriesCapacity = uint32_t(float(size) * mLoadFactor);
		uint32_t newHashSize = size;

		// allocate new common buffer and setup pointers to new tables
		uint8_t* newBuffer;
		uint32_t* newHash;
		uint32_t* newEntriesNext;
		Entry* newEntries;
		{
			uint32_t newHashByteOffset = 0;
			uint32_t newEntriesNextBytesOffset = newHashByteOffset + newHashSize * sizeof(uint32_t);
			uint32_t newEntriesByteOffset = newEntriesNextBytesOffset + newEntriesCapacity * sizeof(uint32_t);
			newEntriesByteOffset += (16 - (newEntriesByteOffset & 15)) & 15;
			uint32_t newBufferByteSize = newEntriesByteOffset + newEntriesCapacity * sizeof(Entry);

			newBuffer = reinterpret_cast<uint8_t*>(Allocator::allocate(newBufferByteSize, __FILE__, __LINE__));
			PX_ASSERT(newBuffer);

			newHash = reinterpret_cast<uint32_t*>(newBuffer + newHashByteOffset);
			newEntriesNext = reinterpret_cast<uint32_t*>(newBuffer + newEntriesNextBytesOffset);
			newEntries = reinterpret_cast<Entry*>(newBuffer + newEntriesByteOffset);
		}

		// initialize new hash table
		intrinsics::memSet(newHash, uint32_t(EOL), newHashSize * sizeof(uint32_t));

		// iterate over old entries, re-hash and create new entries
		if(resizeCompact)
		{
			// check that old free list is empty - we don't need to copy the next entries
			PX_ASSERT(compacting || mFreeList == EOL);

			for(uint32_t index = 0; index < mEntriesCount; ++index)
			{
				uint32_t h = hash(GetKey()(mEntries[index]), newHashSize);
				newEntriesNext[index] = newHash[h];
				newHash[h] = index;

				PX_PLACEMENT_NEW(newEntries + index, Entry)(mEntries[index]);
				mEntries[index].~Entry();
			}
		}
		else
		{
			// copy old free list, only required for non compact resizing
			intrinsics::memCopy(newEntriesNext, mEntriesNext, mEntriesCapacity * sizeof(uint32_t));

			for(uint32_t bucket = 0; bucket < mHashSize; bucket++)
			{
				uint32_t index = mHash[bucket];
				while(index != EOL)
				{
					uint32_t h = hash(GetKey()(mEntries[index]), newHashSize);
					newEntriesNext[index] = newHash[h];
					PX_ASSERT(index != newHash[h]);

					newHash[h] = index;

					PX_PLACEMENT_NEW(newEntries + index, Entry)(mEntries[index]);
					mEntries[index].~Entry();

					index = mEntriesNext[index];
				}
			}
		}

		// swap buffer and pointers
		Allocator::deallocate(mBuffer);
		mBuffer = newBuffer;
		mHash = newHash;
		mHashSize = newHashSize;
		mEntriesNext = newEntriesNext;
		mEntries = newEntries;
		mEntriesCapacity = newEntriesCapacity;

		freeListAdd(oldEntriesCapacity, newEntriesCapacity);
	}

	void grow()
	{
		PX_ASSERT((mFreeList == EOL) || (compacting && (mEntriesCount == mEntriesCapacity)));

		uint32_t size = mHashSize == 0 ? 16 : mHashSize * 2;
		reserve(size);
	}

	uint8_t* mBuffer;
	Entry* mEntries;
	uint32_t* mEntriesNext; // same size as mEntries
	uint32_t* mHash;
	uint32_t mEntriesCapacity;
	uint32_t mHashSize;
	float mLoadFactor;
	uint32_t mFreeList;
	uint32_t mTimestamp;
	uint32_t mEntriesCount; // number of entries

  public:
	class Iter
	{
	  public:
		PX_INLINE Iter(HashBase& b) : mBucket(0), mEntry(uint32_t(b.EOL)), mTimestamp(b.mTimestamp), mBase(b)
		{
			if(mBase.mEntriesCapacity > 0)
			{
				mEntry = mBase.mHash[0];
				skip();
			}
		}

		PX_INLINE void check() const
		{
			PX_ASSERT(mTimestamp == mBase.mTimestamp);
		}
		PX_INLINE const Entry& operator*() const
		{
			check();
			return mBase.mEntries[mEntry];
		}
		PX_INLINE Entry& operator*()
		{
			check();
			return mBase.mEntries[mEntry];
		}
		PX_INLINE const Entry* operator->() const
		{
			check();
			return mBase.mEntries + mEntry;
		}
		PX_INLINE Entry* operator->()
		{
			check();
			return mBase.mEntries + mEntry;
		}
		PX_INLINE Iter operator++()
		{
			check();
			advance();
			return *this;
		}
		PX_INLINE Iter operator++(int)
		{
			check();
			Iter i = *this;
			advance();
			return i;
		}
		PX_INLINE bool done() const
		{
			check();
			return mEntry == mBase.EOL;
		}

	  private:
		PX_INLINE void advance()
		{
			mEntry = mBase.mEntriesNext[mEntry];
			skip();
		}
		PX_INLINE void skip()
		{
			while(mEntry == mBase.EOL)
			{
				if(++mBucket == mBase.mHashSize)
					break;
				mEntry = mBase.mHash[mBucket];
			}
		}

		Iter& operator=(const Iter&);

		uint32_t mBucket;
		uint32_t mEntry;
		uint32_t mTimestamp;
		HashBase& mBase;
	};

	/*!
	Iterate over entries in a hash base and allow entry erase while iterating
	*/
	class EraseIterator
	{
	public:
		PX_INLINE EraseIterator(HashBase& b): mBase(b)
		{
			reset();
		}

		PX_INLINE Entry* eraseCurrentGetNext(bool eraseCurrent)
		{
			if(eraseCurrent && mCurrentEntryIndexPtr)
			{
				mBase.eraseInternal(mCurrentEntryIndexPtr);
				// if next was valid return the same ptr, if next was EOL search new hash entry
				if(*mCurrentEntryIndexPtr != mBase.EOL)
					return mBase.mEntries + *mCurrentEntryIndexPtr;
				else
					return traverseHashEntries();
			}

			// traverse mHash to find next entry
			if(mCurrentEntryIndexPtr == NULL)
				return traverseHashEntries();
			
			const uint32_t index = *mCurrentEntryIndexPtr;			
			if(mBase.mEntriesNext[index] == mBase.EOL)
			{
				return traverseHashEntries();
			}
			else
			{
				mCurrentEntryIndexPtr = mBase.mEntriesNext + index;
				return mBase.mEntries + *mCurrentEntryIndexPtr;
			}
		}

		PX_INLINE void reset()
		{
			mCurrentHashIndex = 0;
			mCurrentEntryIndexPtr = NULL;			
		}

	private:
		PX_INLINE Entry* traverseHashEntries()
		{
			mCurrentEntryIndexPtr = NULL;			
			while (mCurrentEntryIndexPtr == NULL && mCurrentHashIndex < mBase.mHashSize)
			{
				if (mBase.mHash[mCurrentHashIndex] != mBase.EOL)
				{
					mCurrentEntryIndexPtr = mBase.mHash + mCurrentHashIndex;
					mCurrentHashIndex++;
					return mBase.mEntries + *mCurrentEntryIndexPtr;
				}
				else
				{
					mCurrentHashIndex++;
				}
			}
			return NULL;
		}

		EraseIterator& operator=(const EraseIterator&);
	private:
		uint32_t*	mCurrentEntryIndexPtr;
		uint32_t	mCurrentHashIndex;		
		HashBase&	mBase;
	};
};

template <class Entry, class Key, class HashFn, class GetKey, class Allocator, bool compacting>
template <typename HK, typename GK, class A, bool comp>
PX_NOINLINE void
HashBase<Entry, Key, HashFn, GetKey, Allocator, compacting>::copy(const HashBase<Entry, Key, HK, GK, A, comp>& other)
{
	reserve(other.mEntriesCount);

	for(uint32_t i = 0; i < other.mEntriesCount; i++)
	{
		for(uint32_t j = other.mHash[i]; j != EOL; j = other.mEntriesNext[j])
		{
			const Entry& otherEntry = other.mEntries[j];

			bool exists;
			Entry* newEntry = create(GK()(otherEntry), exists);
			PX_ASSERT(!exists);

			PX_PLACEMENT_NEW(newEntry, Entry)(otherEntry);
		}
	}
}

template <class Key, class HashFn, class Allocator = typename AllocatorTraits<Key>::Type, bool Coalesced = false>
class HashSetBase
{
	PX_NOCOPY(HashSetBase)
  public:
	struct GetKey
	{
		PX_INLINE const Key& operator()(const Key& e)
		{
			return e;
		}
	};

	typedef HashBase<Key, Key, HashFn, GetKey, Allocator, Coalesced> BaseMap;
	typedef typename BaseMap::Iter Iterator;

	HashSetBase(uint32_t initialTableSize, float loadFactor, const Allocator& alloc)
	: mBase(initialTableSize, loadFactor, alloc)
	{
	}

	HashSetBase(const Allocator& alloc) : mBase(64, 0.75f, alloc)
	{
	}

	HashSetBase(uint32_t initialTableSize = 64, float loadFactor = 0.75f) : mBase(initialTableSize, loadFactor)
	{
	}

	bool insert(const Key& k)
	{
		bool exists;
		Key* e = mBase.create(k, exists);
		if(!exists)
			PX_PLACEMENT_NEW(e, Key)(k);
		return !exists;
	}

	PX_INLINE bool contains(const Key& k) const
	{
		return mBase.find(k) != 0;
	}
	PX_INLINE bool erase(const Key& k)
	{
		return mBase.erase(k);
	}
	PX_INLINE uint32_t size() const
	{
		return mBase.size();
	}
	PX_INLINE uint32_t capacity() const
	{
		return mBase.capacity();
	}
	PX_INLINE void reserve(uint32_t size)
	{
		mBase.reserve(size);
	}
	PX_INLINE void clear()
	{
		mBase.clear();
	}

  protected:
	BaseMap mBase;
};

template <class Key, class Value, class HashFn, class Allocator = typename AllocatorTraits<Pair<const Key, Value> >::Type>
class HashMapBase
{
	PX_NOCOPY(HashMapBase)
  public:
	typedef Pair<const Key, Value> Entry;

	struct GetKey
	{
		PX_INLINE const Key& operator()(const Entry& e)
		{
			return e.first;
		}
	};

	typedef HashBase<Entry, Key, HashFn, GetKey, Allocator, true> BaseMap;
	typedef typename BaseMap::Iter Iterator;
	typedef typename BaseMap::EraseIterator EraseIterator;

	HashMapBase(uint32_t initialTableSize, float loadFactor, const Allocator& alloc)
	: mBase(initialTableSize, loadFactor, alloc)
	{
	}

	HashMapBase(const Allocator& alloc) : mBase(64, 0.75f, alloc)
	{
	}

	HashMapBase(uint32_t initialTableSize = 64, float loadFactor = 0.75f) : mBase(initialTableSize, loadFactor)
	{
	}

	bool insert(const Key /*&*/ k, const Value /*&*/ v)
	{
		bool exists;
		Entry* e = mBase.create(k, exists);
		if(!exists)
			PX_PLACEMENT_NEW(e, Entry)(k, v);
		return !exists;
	}

	Value& operator[](const Key& k)
	{
		bool exists;
		Entry* e = mBase.create(k, exists);
		if(!exists)
			PX_PLACEMENT_NEW(e, Entry)(k, Value());

		return e->second;
	}

	PX_INLINE const Entry* find(const Key& k) const
	{
		return mBase.find(k);
	}
	PX_INLINE bool erase(const Key& k)
	{
		return mBase.erase(k);
	}
	PX_INLINE bool erase(const Key& k, Entry& e)
	{		
		return mBase.erase(k, e);
	}
	PX_INLINE uint32_t size() const
	{
		return mBase.size();
	}
	PX_INLINE uint32_t capacity() const
	{
		return mBase.capacity();
	}
	PX_INLINE Iterator getIterator()
	{
		return Iterator(mBase);
	}
	PX_INLINE EraseIterator getEraseIterator()
	{
		return EraseIterator(mBase);
	}
	PX_INLINE void reserve(uint32_t size)
	{
		mBase.reserve(size);
	}
	PX_INLINE void clear()
	{
		mBase.clear();
	}

  protected:
	BaseMap mBase;
};
}

} // namespace shdfnd
} // namespace physx

#if PX_VC
#pragma warning(pop)
#endif
#endif // #ifndef PSFOUNDATION_PSHASHINTERNALS_H
