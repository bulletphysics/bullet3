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

#include "foundation/PxPreprocessor.h"
#include "foundation/PxAssert.h"
#include "foundation/PxAllocatorCallback.h"
#include "foundation/PxMemory.h"

#include <errno.h>
#include "PsFoundation.h"
#include "PxDefaultStreams.h"
#include "SnFile.h"
#include "CmPhysXCommon.h"
#include "PsUtilities.h"
#include "PsBitUtils.h"

using namespace physx;

PxDefaultMemoryOutputStream::PxDefaultMemoryOutputStream(PxAllocatorCallback &allocator) 
:	mAllocator	(allocator)	
,	mData		(NULL)
,	mSize		(0)
,	mCapacity	(0)
{
}

PxDefaultMemoryOutputStream::~PxDefaultMemoryOutputStream()
{
	if(mData)
		mAllocator.deallocate(mData);
}

PxU32 PxDefaultMemoryOutputStream::write(const void* src, PxU32 size)
{
	PxU32 expectedSize = mSize + size;
	if(expectedSize > mCapacity)
	{
		mCapacity = PxMax(Ps::nextPowerOfTwo(expectedSize), 4096u);

		PxU8* newData = reinterpret_cast<PxU8*>(mAllocator.allocate(mCapacity,"PxDefaultMemoryOutputStream",__FILE__,__LINE__));
		PX_ASSERT(newData!=NULL);

		PxMemCopy(newData, mData, mSize);
		if(mData)
			mAllocator.deallocate(mData);

		mData = newData;
	}
	PxMemCopy(mData+mSize, src, size);
	mSize += size;
	return size;
}

///////////////////////////////////////////////////////////////////////////////

PxDefaultMemoryInputData::PxDefaultMemoryInputData(PxU8* data, PxU32 length) :
	mSize	(length),
	mData	(data),
	mPos	(0)
{
}

PxU32 PxDefaultMemoryInputData::read(void* dest, PxU32 count)
{
	PxU32 length = PxMin<PxU32>(count, mSize-mPos);
	PxMemCopy(dest, mData+mPos, length);
	mPos += length;
	return length;
}

PxU32 PxDefaultMemoryInputData::getLength() const
{
	return mSize;
}

void PxDefaultMemoryInputData::seek(PxU32 offset)
{
	mPos = PxMin<PxU32>(mSize, offset);
}

PxU32 PxDefaultMemoryInputData::tell() const
{
	return mPos;
}

PxDefaultFileOutputStream::PxDefaultFileOutputStream(const char* filename)
{
	mFile = NULL;
	sn::fopen_s(&mFile, filename, "wb");
	// PT: when this fails, check that:
	// - the path is correct
	// - the file does not already exist. If it does, check that it is not write protected.
	if(NULL == mFile)
	{
		Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, 
			"Unable to open file %s, errno 0x%x\n",filename,errno);
	}
	PX_ASSERT(mFile);
}

PxDefaultFileOutputStream::~PxDefaultFileOutputStream()
{
	if(mFile)
		fclose(mFile);
}

PxU32 PxDefaultFileOutputStream::write(const void* src, PxU32 count)
{
	return mFile ? PxU32(fwrite(src, 1, count, mFile)) : 0;
}

bool PxDefaultFileOutputStream::isValid()
{
	return mFile != NULL;
}

///////////////////////////////////////////////////////////////////////////////

PxDefaultFileInputData::PxDefaultFileInputData(const char* filename)
{
	mFile = NULL;
	sn::fopen_s(&mFile, filename, "rb");

	if(mFile)
	{
		fseek(mFile, 0, SEEK_END);
		mLength = PxU32(ftell(mFile));
		fseek(mFile, 0, SEEK_SET);
	}
	else
	{
		mLength = 0;
	}
}

PxDefaultFileInputData::~PxDefaultFileInputData()
{
	if(mFile)
		fclose(mFile);
}

PxU32 PxDefaultFileInputData::read(void* dest, PxU32 count)
{
	PX_ASSERT(mFile);
	const size_t size = fread(dest, 1, count, mFile);
	// there should be no assert here since by spec of PxInputStream we can read fewer bytes than expected
	return PxU32(size);
}

PxU32 PxDefaultFileInputData::getLength() const
{
	return mLength;
}

void PxDefaultFileInputData::seek(PxU32 pos)
{
	fseek(mFile, long(pos), SEEK_SET);
}

PxU32 PxDefaultFileInputData::tell() const
{
	return PxU32(ftell(mFile));
}

bool PxDefaultFileInputData::isValid() const
{
	return mFile != NULL;
}
