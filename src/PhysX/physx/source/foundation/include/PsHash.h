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

#ifndef PSFOUNDATION_PSHASH_H
#define PSFOUNDATION_PSHASH_H

#include "Ps.h"
#include "PsBasicTemplates.h"

#if PX_VC
#pragma warning(push)
#pragma warning(disable : 4302)
#endif

#if PX_LINUX
#include "foundation/PxSimpleTypes.h"
#endif

/*!
Central definition of hash functions
*/

namespace physx
{
namespace shdfnd
{
// Hash functions

// Thomas Wang's 32 bit mix
// http://www.cris.com/~Ttwang/tech/inthash.htm
PX_FORCE_INLINE uint32_t hash(const uint32_t key)
{
	uint32_t k = key;
	k += ~(k << 15);
	k ^= (k >> 10);
	k += (k << 3);
	k ^= (k >> 6);
	k += ~(k << 11);
	k ^= (k >> 16);
	return uint32_t(k);
}

PX_FORCE_INLINE uint32_t hash(const int32_t key)
{
	return hash(uint32_t(key));
}

// Thomas Wang's 64 bit mix
// http://www.cris.com/~Ttwang/tech/inthash.htm
PX_FORCE_INLINE uint32_t hash(const uint64_t key)
{
	uint64_t k = key;
	k += ~(k << 32);
	k ^= (k >> 22);
	k += ~(k << 13);
	k ^= (k >> 8);
	k += (k << 3);
	k ^= (k >> 15);
	k += ~(k << 27);
	k ^= (k >> 31);
	return uint32_t(UINT32_MAX & k);
}

#if PX_APPLE_FAMILY
// hash for size_t, to make gcc happy
PX_INLINE uint32_t hash(const size_t key)
{
#if PX_P64_FAMILY
	return hash(uint64_t(key));
#else
	return hash(uint32_t(key));
#endif
}
#endif

// Hash function for pointers
PX_INLINE uint32_t hash(const void* ptr)
{
#if PX_P64_FAMILY
	return hash(uint64_t(ptr));
#else
	return hash(uint32_t(UINT32_MAX & size_t(ptr)));
#endif
}

// Hash function for pairs
template <typename F, typename S>
PX_INLINE uint32_t hash(const Pair<F, S>& p)
{
	uint32_t seed = 0x876543;
	uint32_t m = 1000007;
	return hash(p.second) ^ (m * (hash(p.first) ^ (m * seed)));
}

// hash object for hash map template parameter
template <class Key>
struct Hash
{
	uint32_t operator()(const Key& k) const
	{
		return hash(k);
	}
	bool equal(const Key& k0, const Key& k1) const
	{
		return k0 == k1;
	}
};

// specialization for strings
template <>
struct Hash<const char*>
{
  public:
	uint32_t operator()(const char* _string) const
	{
		// "DJB" string hash
		const uint8_t* string = reinterpret_cast<const uint8_t*>(_string);
		uint32_t h = 5381;
		for(const uint8_t* ptr = string; *ptr; ptr++)
			h = ((h << 5) + h) ^ uint32_t(*ptr);
		return h;
	}
	bool equal(const char* string0, const char* string1) const
	{
		return !strcmp(string0, string1);
	}
};

} // namespace shdfnd
} // namespace physx

#if PX_VC
#pragma warning(pop)
#endif

#endif // #ifndef PSFOUNDATION_PSHASH_H
