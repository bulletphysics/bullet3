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

#ifndef PXPVDSDK_PXPVDBYTESTREAMS_H
#define PXPVDSDK_PXPVDBYTESTREAMS_H
#include "PxPvdObjectModelBaseTypes.h"

namespace physx
{
namespace pvdsdk
{

static inline uint32_t strLen(const char* inStr)
{
	uint32_t len = 0;
	if(inStr)
	{
		while(*inStr)
		{
			++len;
			++inStr;
		}
	}
	return len;
}

class PvdInputStream
{
  protected:
	virtual ~PvdInputStream()
	{
	}

  public:
	// Return false if you can't write the number of bytes requested
	// But make an absolute best effort to read the data...
	virtual bool read(uint8_t* buffer, uint32_t& len) = 0;

	template <typename TDataType>
	bool read(TDataType* buffer, uint32_t numItems)
	{
		uint32_t expected = numItems;
		uint32_t amountToRead = numItems * sizeof(TDataType);
		read(reinterpret_cast<uint8_t*>(buffer), amountToRead);
		numItems = amountToRead / sizeof(TDataType);
		PX_ASSERT(numItems == expected);
		return expected == numItems;
	}

	template <typename TDataType>
	PvdInputStream& operator>>(TDataType& data)
	{
		uint32_t dataSize = static_cast<uint32_t>(sizeof(TDataType));
		bool success = read(reinterpret_cast<uint8_t*>(&data), dataSize);
		// PX_ASSERT( success );
		// PX_ASSERT( dataSize == sizeof( data ) );
		(void)success;
		return *this;
	}
};

class PvdOutputStream
{
  protected:
	virtual ~PvdOutputStream()
	{
	}

  public:
	// Return false if you can't write the number of bytes requested
	// But make an absolute best effort to write the data...
	virtual bool write(const uint8_t* buffer, uint32_t len) = 0;
	virtual bool directCopy(PvdInputStream& inStream, uint32_t len) = 0;

	template <typename TDataType>
	bool write(const TDataType* buffer, uint32_t numItems)
	{
		return write(reinterpret_cast<const uint8_t*>(buffer), numItems * sizeof(TDataType));
	}

	template <typename TDataType>
	PvdOutputStream& operator<<(const TDataType& data)
	{
		bool success = write(reinterpret_cast<const uint8_t*>(&data), sizeof(data));
		PX_ASSERT(success);
		(void)success;
		return *this;
	}

	PvdOutputStream& operator<<(const char* inString)
	{
		if(inString && *inString)
		{
			uint32_t len(strLen(inString));
			write(inString, len);
		}
		return *this;
	}
};
}
}
#endif // PXPVDSDK_PXPVDBYTESTREAMS_H
