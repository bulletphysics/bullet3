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

#ifndef GU_EDGECACHE_H
#define GU_EDGECACHE_H

#include "foundation/PxMemory.h"
#include "CmPhysXCommon.h"
#include "PsHash.h"

namespace physx
{
namespace Gu
{
	class EdgeCache
	{
#define NUM_EDGES_IN_CACHE 64		//must be power of 2.	32 lines result in 10% extra work (due to cache misses), 64 lines in 6% extra work, 128 lines in 4%.
	public:
		EdgeCache()
		{
			PxMemZero(cacheLines, NUM_EDGES_IN_CACHE*sizeof(CacheLine));
		}

		PxU32 hash(PxU32 key)	const
		{
			return (NUM_EDGES_IN_CACHE - 1) & Ps::hash(key);		//Only a 16 bit hash would be needed here.
		}

		bool isInCache(PxU8 vertex0, PxU8 vertex1)
		{
			PX_ASSERT(vertex1 >= vertex0);
			PxU16 key = PxU16((vertex0 << 8) | vertex1);
			PxU32 h = hash(key);
			CacheLine& cl = cacheLines[h];
			if (cl.fullKey == key)
			{
				return true;
			}
			else	//cache the line now as it's about to be processed
			{
				cl.fullKey = key;
				return false;
			}
		}

	private:
		struct CacheLine
		{
			PxU16 fullKey;
		};
		CacheLine cacheLines[NUM_EDGES_IN_CACHE];
#undef NUM_EDGES_IN_CACHE
	};
}

}

#endif

