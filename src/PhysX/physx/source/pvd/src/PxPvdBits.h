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

#ifndef PXPVDSDK_PXPVDBITS_H
#define PXPVDSDK_PXPVDBITS_H

#include "PxPvdObjectModelBaseTypes.h"

namespace physx
{
namespace pvdsdk
{

// Marshallers cannot assume src is aligned, but they can assume dest is aligned.
typedef void (*TSingleMarshaller)(const uint8_t* src, uint8_t* dest);
typedef void (*TBlockMarshaller)(const uint8_t* src, uint8_t* dest, uint32_t numItems);

template <uint8_t ByteCount>
static inline void doSwapBytes(uint8_t* __restrict inData)
{
	for(uint32_t idx = 0; idx < ByteCount / 2; ++idx)
	{
		uint32_t endIdx = ByteCount - idx - 1;
		uint8_t theTemp = inData[idx];
		inData[idx] = inData[endIdx];
		inData[endIdx] = theTemp;
	}
}

template <uint8_t ByteCount>
static inline void doSwapBytes(uint8_t* __restrict inData, uint32_t itemCount)
{
	uint8_t* end = inData + itemCount * ByteCount;
	for(; inData < end; inData += ByteCount)
		doSwapBytes<ByteCount>(inData);
}

static inline void swapBytes(uint8_t* __restrict dataPtr, uint32_t numBytes, uint32_t itemWidth)
{
	uint32_t numItems = numBytes / itemWidth;
	switch(itemWidth)
	{
	case 1:
		break;
	case 2:
		doSwapBytes<2>(dataPtr, numItems);
		break;
	case 4:
		doSwapBytes<4>(dataPtr, numItems);
		break;
	case 8:
		doSwapBytes<8>(dataPtr, numItems);
		break;
	case 16:
		doSwapBytes<16>(dataPtr, numItems);
		break;
	default:
		PX_ASSERT(false);
		break;
	}
}

static inline void swapBytes(uint8_t&)
{
}
static inline void swapBytes(int8_t&)
{
}
static inline void swapBytes(uint16_t& inData)
{
	doSwapBytes<2>(reinterpret_cast<uint8_t*>(&inData));
}
static inline void swapBytes(int16_t& inData)
{
	doSwapBytes<2>(reinterpret_cast<uint8_t*>(&inData));
}
static inline void swapBytes(uint32_t& inData)
{
	doSwapBytes<4>(reinterpret_cast<uint8_t*>(&inData));
}
static inline void swapBytes(int32_t& inData)
{
	doSwapBytes<4>(reinterpret_cast<uint8_t*>(&inData));
}
static inline void swapBytes(float& inData)
{
	doSwapBytes<4>(reinterpret_cast<uint8_t*>(&inData));
}
static inline void swapBytes(uint64_t& inData)
{
	doSwapBytes<8>(reinterpret_cast<uint8_t*>(&inData));
}
static inline void swapBytes(int64_t& inData)
{
	doSwapBytes<8>(reinterpret_cast<uint8_t*>(&inData));
}
static inline void swapBytes(double& inData)
{
	doSwapBytes<8>(reinterpret_cast<uint8_t*>(&inData));
}

static inline bool checkLength(const uint8_t* inStart, const uint8_t* inStop, uint32_t inLength)
{
	return static_cast<uint32_t>(inStop - inStart) >= inLength;
}
}
}
#endif // PXPVDSDK_PXPVDBITS_H
