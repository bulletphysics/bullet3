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

#ifndef PSFOUNDATION_PSHASHMAP_H
#define PSFOUNDATION_PSHASHMAP_H

#include "PsHashInternals.h"

// TODO: make this doxy-format
//
// This header defines two hash maps. Hash maps
// * support custom initial table sizes (rounded up internally to power-of-2)
// * support custom static allocator objects
// * auto-resize, based on a load factor (i.e. a 64-entry .75 load factor hash will resize
//                                        when the 49th element is inserted)
// * are based on open hashing
// * have O(1) contains, erase
//
// Maps have STL-like copying semantics, and properly initialize and destruct copies of objects
//
// There are two forms of map: coalesced and uncoalesced. Coalesced maps keep the entries in the
// initial segment of an array, so are fast to iterate over; however deletion is approximately
// twice as expensive.
//
// HashMap<T>:
//		bool			insert(const Key& k, const Value& v)	O(1) amortized (exponential resize policy)
//		Value &			operator[](const Key& k)				O(1) for existing objects, else O(1) amortized
//		const Entry *	find(const Key& k);						O(1)
//		bool			erase(const T& k);						O(1)
//		uint32_t			size();									constant
//		void			reserve(uint32_t size);					O(MAX(currentOccupancy,size))
//		void			clear();								O(currentOccupancy) (with zero constant for objects
// without
// destructors)
//      Iterator		getIterator();
//
// operator[] creates an entry if one does not exist, initializing with the default constructor.
// CoalescedHashMap<T> does not support getIterator, but instead supports
// 		const Key *getEntries();
//
// Use of iterators:
//
// for(HashMap::Iterator iter = test.getIterator(); !iter.done(); ++iter)
//			myFunction(iter->first, iter->second);

namespace physx
{
namespace shdfnd
{
template <class Key, class Value, class HashFn = Hash<Key>, class Allocator = NonTrackingAllocator>
class HashMap : public internal::HashMapBase<Key, Value, HashFn, Allocator>
{
  public:
	typedef internal::HashMapBase<Key, Value, HashFn, Allocator> HashMapBase;
	typedef typename HashMapBase::Iterator Iterator;

	HashMap(uint32_t initialTableSize = 64, float loadFactor = 0.75f) : HashMapBase(initialTableSize, loadFactor)
	{
	}
	HashMap(uint32_t initialTableSize, float loadFactor, const Allocator& alloc)
	: HashMapBase(initialTableSize, loadFactor, alloc)
	{
	}
	HashMap(const Allocator& alloc) : HashMapBase(64, 0.75f, alloc)
	{
	}
	Iterator getIterator()
	{
		return Iterator(HashMapBase::mBase);
	}
};

template <class Key, class Value, class HashFn = Hash<Key>, class Allocator = NonTrackingAllocator>
class CoalescedHashMap : public internal::HashMapBase<Key, Value, HashFn, Allocator>
{
  public:
	typedef internal::HashMapBase<Key, Value, HashFn, Allocator> HashMapBase;

	CoalescedHashMap(uint32_t initialTableSize = 64, float loadFactor = 0.75f)
	: HashMapBase(initialTableSize, loadFactor)
	{
	}
	const Pair<const Key, Value>* getEntries() const
	{
		return HashMapBase::mBase.getEntries();
	}
};

} // namespace shdfnd
} // namespace physx

#endif // #ifndef PSFOUNDATION_PSHASHMAP_H
