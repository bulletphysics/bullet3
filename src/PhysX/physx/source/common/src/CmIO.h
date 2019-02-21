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


#ifndef PX_PHYSICS_COMMON_IO
#define PX_PHYSICS_COMMON_IO

#include "foundation/PxIO.h"
#include "foundation/PxAssert.h"
#include "foundation/PxMemory.h"
#include "CmPhysXCommon.h"

namespace physx
{

	// wrappers for IO classes so that we can add extra functionality (byte counting, buffering etc)

namespace Cm
{

class InputStreamReader
{
public:

	InputStreamReader(PxInputStream& stream) : mStream(stream) {	}
	PxU32	read(void* dest, PxU32 count)	
	{	
		PxU32 readLength = mStream.read(dest, count);

		// zero the buffer if we didn't get all the data
		if(readLength<count)
			PxMemZero(reinterpret_cast<PxU8*>(dest)+readLength, count-readLength);
	
		return readLength;
	}

	template <typename T> T get()			
	{		
		T val;	
		PxU32 length = mStream.read(&val, sizeof(T));
		PX_ASSERT(length == sizeof(T));
		PX_UNUSED(length);
		return val; 
	}


protected:
	PxInputStream &mStream;
private:
	InputStreamReader& operator=(const InputStreamReader&);
};


class InputDataReader : public InputStreamReader
{
public:
	InputDataReader(PxInputData& data) : InputStreamReader(data) {}
	InputDataReader &operator=(const InputDataReader &);

	PxU32	length() const					{		return getData().getLength();		}
	void	seek(PxU32 offset)				{		getData().seek(offset);				}
	PxU32	tell()							{		return getData().tell();			}

private:
	PxInputData& getData()					{		return static_cast<PxInputData&>(mStream); }
	const PxInputData& getData() const		{		return static_cast<const PxInputData&>(mStream); }
};


class OutputStreamWriter
{
public:

	PX_INLINE OutputStreamWriter(PxOutputStream& stream) 
	:	mStream(stream)
	,	mCount(0)
	{}

	PX_INLINE	PxU32	write(const void* src, PxU32 offset)		
	{		
		PxU32 count = mStream.write(src, offset);
		mCount += count;
		return count;
	}

	PX_INLINE	PxU32	getStoredSize()
	{
		return mCount;
	}

	template<typename T> void put(const T& val)	
	{		
		PxU32 length = write(&val, sizeof(T));		
		PX_ASSERT(length == sizeof(T));
		PX_UNUSED(length);
	}

private:

	OutputStreamWriter& operator=(const OutputStreamWriter&);
	PxOutputStream& mStream;
	PxU32 mCount;
};



}
}

#endif
