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

#ifndef PXPVDSDK_PXPVDINTERNALBYTESTREAMS_H
#define PXPVDSDK_PXPVDINTERNALBYTESTREAMS_H

#include "PxPvdByteStreams.h"
#include "PxPvdFoundation.h"

namespace physx
{
namespace pvdsdk
{
struct MemPvdInputStream : public PvdInputStream
{
	const uint8_t* mBegin;
	const uint8_t* mEnd;
	bool mGood;

	MemPvdInputStream(const uint8_t* beg = NULL, const uint8_t* end = NULL)
	{
		mBegin = beg;
		mEnd = end;
		mGood = true;
	}

	uint32_t size() const
	{
		return mGood ? static_cast<uint32_t>(mEnd - mBegin) : 0;
	}
	bool isGood() const
	{
		return mGood;
	}

	void setup(uint8_t* start, uint8_t* stop)
	{
		mBegin = start;
		mEnd = stop;
	}

	void nocopyRead(uint8_t*& buffer, uint32_t& len)
	{
		if(len == 0 || mGood == false)
		{
			len = 0;
			buffer = NULL;
			return;
		}
		uint32_t original = len;
		len = PxMin(len, size());
		if(mGood && len != original)
			mGood = false;
		buffer = const_cast<uint8_t*>(mBegin);
		mBegin += len;
	}

	virtual bool read(uint8_t* buffer, uint32_t& len)
	{
		if(len == 0)
			return true;
		uint32_t original = len;
		len = PxMin(len, size());

		physx::intrinsics::memCopy(buffer, mBegin, len);
		mBegin += len;
		if(len < original)
			physx::intrinsics::memZero(buffer + len, original - len);
		mGood = mGood && len == original;
		return mGood;
	}
};
}
}
#endif // PXPVDSDK_PXPVDINTERNALBYTESTREAMS_H
