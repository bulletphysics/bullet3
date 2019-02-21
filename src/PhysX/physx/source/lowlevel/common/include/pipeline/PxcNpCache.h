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

#ifndef PXC_NPCACHE_H
#define PXC_NPCACHE_H

#include "foundation/PxMemory.h"

#include "PsIntrinsics.h"
#include "PxcNpCacheStreamPair.h"

#include "PsPool.h"
#include "PsFoundation.h"
#include "GuContactMethodImpl.h"
#include "PsUtilities.h"

namespace physx
{

template <typename T>
void PxcNpCacheWrite(PxcNpCacheStreamPair& streams,
					 Gu::Cache& cache,
					 const T& payload,
					 PxU32 bytes, 
					 const PxU8* data)
{
	const PxU32 payloadSize = (sizeof(payload)+3)&~3;
	cache.mCachedSize = Ps::to16((payloadSize + 4 + bytes + 0xF)&~0xF);

	PxU8* ls = streams.reserve(cache.mCachedSize);
	cache.mCachedData = ls;
	if(ls==NULL || (reinterpret_cast<PxU8*>(-1))==ls)
	{
		if(ls==NULL)
		{
			PX_WARN_ONCE(
				"Reached limit set by PxSceneDesc::maxNbContactDataBlocks - ran out of buffer space for narrow phase. "
				"Either accept dropped contacts or increase buffer size allocated for narrow phase by increasing PxSceneDesc::maxNbContactDataBlocks.");
			return;
		}
		else
		{
			PX_WARN_ONCE(
				"Attempting to allocate more than 16K of contact data for a single contact pair in narrowphase. "
				"Either accept dropped contacts or simplify collision geometry.");
			cache.mCachedData = NULL;
			ls = NULL;
			return;
		}
	}

	*reinterpret_cast<T*>(ls) = payload;
	*reinterpret_cast<PxU32*>(ls+payloadSize) = bytes;
	if(data)
		PxMemCopy(ls+payloadSize+sizeof(PxU32), data, bytes);
}


template <typename T>
PxU8* PxcNpCacheWriteInitiate(PxcNpCacheStreamPair& streams, Gu::Cache& cache, const T& payload, PxU32 bytes)
{
	PX_UNUSED(payload);

	const PxU32 payloadSize = (sizeof(payload)+3)&~3;
	cache.mCachedSize = Ps::to16((payloadSize + 4 + bytes + 0xF)&~0xF);

	PxU8* ls = streams.reserve(cache.mCachedSize);
	cache.mCachedData = ls;
	if(NULL==ls || reinterpret_cast<PxU8*>(-1)==ls)
	{
		if(NULL==ls)
		{
			PX_WARN_ONCE(
				"Reached limit set by PxSceneDesc::maxNbContactDataBlocks - ran out of buffer space for narrow phase. "
				"Either accept dropped contacts or increase buffer size allocated for narrow phase by increasing PxSceneDesc::maxNbContactDataBlocks.");
		}
		else
		{
			PX_WARN_ONCE(
				"Attempting to allocate more than 16K of contact data for a single contact pair in narrowphase. "
				"Either accept dropped contacts or simplify collision geometry.");
			cache.mCachedData = NULL;
			ls = NULL;
		}
	}
	return ls;
}

template <typename T>
PX_FORCE_INLINE void PxcNpCacheWriteFinalize(PxU8* ls, const T& payload, PxU32 bytes, const PxU8* data)
{
	const PxU32 payloadSize = (sizeof(payload)+3)&~3;
	*reinterpret_cast<T*>(ls) = payload;
	*reinterpret_cast<PxU32*>(ls+payloadSize) = bytes;
	if(data)
		PxMemCopy(ls+payloadSize+sizeof(PxU32), data, bytes);
}


template <typename T>
PX_FORCE_INLINE PxU8* PxcNpCacheRead(Gu::Cache& cache, T*& payload)
{
	PxU8* ls = cache.mCachedData;
	payload = reinterpret_cast<T*>(ls);
	const PxU32 payloadSize = (sizeof(T)+3)&~3;
	return reinterpret_cast<PxU8*>(ls+payloadSize+sizeof(PxU32));
}

template <typename T>
const PxU8* PxcNpCacheRead2(Gu::Cache& cache, T& payload, PxU32& bytes)
{
	const PxU8* ls = cache.mCachedData;
	if(ls==NULL)
	{
		bytes = 0;
		return NULL;
	}

	const PxU32 payloadSize = (sizeof(payload)+3)&~3;
	payload = *reinterpret_cast<const T*>(ls);
	bytes = *reinterpret_cast<const PxU32*>(ls+payloadSize);
	PX_ASSERT(cache.mCachedSize == ((payloadSize + 4 + bytes+0xF)&~0xF));
	return reinterpret_cast<const PxU8*>(ls+payloadSize+sizeof(PxU32));
}

}

#endif // #ifndef PXC_NPCACHE_H
