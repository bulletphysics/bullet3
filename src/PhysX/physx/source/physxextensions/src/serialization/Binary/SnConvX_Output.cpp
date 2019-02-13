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

#include "foundation/PxIO.h"
#include "foundation/PxErrorCallback.h"
#include "SnConvX.h"

#if PX_VC
#pragma warning(disable:4389)	// signed/unsigned mismatch
#endif

using namespace physx;

void Sn::ConvX::setNullPtr(bool flag)
{
	mNullPtr = flag;
}

void Sn::ConvX::setNoOutput(bool flag)
{
	mNoOutput = flag;
}

bool Sn::ConvX::initOutput(PxOutputStream& targetStream)
{
	mOutStream = &targetStream;

	mOutputSize = 0;
	mNullPtr = false;
	mNoOutput = false;

	const MetaData* srcMetaData = getBinaryMetaData(META_DATA_SRC);
	PX_ASSERT(srcMetaData);
	const MetaData* dstMetaData = getBinaryMetaData(META_DATA_DST);
	PX_ASSERT(dstMetaData);

	mSrcPtrSize = srcMetaData->getPtrSize();
	mDstPtrSize = dstMetaData->getPtrSize();

	PX_ASSERT(!srcMetaData->getFlip());
	mMustFlip = dstMetaData->getFlip();
	return true;
}

void Sn::ConvX::closeOutput()
{
	mOutStream = NULL;
}

int Sn::ConvX::getCurrentOutputSize()
{
	return mOutputSize;
}

void Sn::ConvX::output(short value)
{
	if(mNoOutput)
		return;

	if(mMustFlip)
		flip(value);

	PX_ASSERT(mOutStream);
	const size_t size = mOutStream->write(&value, 2);
	PX_ASSERT(size==2);
	mOutputSize += int(size);
}

void Sn::ConvX::output(int value)
{
	if(mNoOutput)
		return;

	if(mMustFlip)
		flip(value);

	PX_ASSERT(mOutStream);
	const size_t size = mOutStream->write(&value, 4);
	PX_ASSERT(size==4);
	mOutputSize += int(size);
}

//ntohll is a macro on apple yosemite
static PxU64 ntohll_internal(const PxU64 value)
{
	union
	{
		PxU64 ull;
		PxU8  c[8];
	} x;

	x.ull = value;

	PxU8 c = 0;
	c = x.c[0]; x.c[0] = x.c[7]; x.c[7] = c;
	c = x.c[1]; x.c[1] = x.c[6]; x.c[6] = c;
	c = x.c[2]; x.c[2] = x.c[5]; x.c[5] = c;
	c = x.c[3]; x.c[3] = x.c[4]; x.c[4] = c;

	return x.ull;
}

void Sn::ConvX::output(PxU64 value)
{
	if(mNoOutput)
		return;

	if(mMustFlip)
//		flip(value);
		value = ntohll_internal(value);

	PX_ASSERT(mOutStream);
	const size_t size = mOutStream->write(&value, 8);
	PX_ASSERT(size==8);
	mOutputSize += int(size);
}

void Sn::ConvX::output(const char* buffer, int nbBytes)
{
	if(mNoOutput)
		return;

	if(!nbBytes)
		return;

	PX_ASSERT(mOutStream);
	const PxU32 size = mOutStream->write(buffer, PxU32(nbBytes));
	PX_ASSERT(size== PxU32(nbBytes));
	mOutputSize += int(size);
}

void Sn::ConvX::convert8(const char* src, const PxMetaDataEntry& entry, const PxMetaDataEntry& dstEntry)
{
	(void)dstEntry;
	if(mNoOutput)
		return;

	PX_ASSERT(entry.mSize==1*entry.mCount);
	PX_ASSERT(mOutStream);
	PX_ASSERT(entry.mSize==dstEntry.mSize);

	const PxU32 size = mOutStream->write(src, PxU32(entry.mSize));
	PX_ASSERT(size== PxU32(entry.mSize));
	mOutputSize += int(size);
}

// This is called to convert auto-generated "padding bytes" (or so we think).
// We use a special converter to check the input bytes and issue warnings when it doesn't look like padding
void Sn::ConvX::convertPad8(const char* src, const PxMetaDataEntry& entry, const PxMetaDataEntry& dstEntry)
{
	(void)dstEntry;
	(void)src;
	if(mNoOutput)
		return;

	PX_ASSERT(entry.mSize);
	PX_ASSERT(entry.mSize==1*entry.mCount);
	PX_ASSERT(mOutStream);
	PX_ASSERT(entry.mSize==dstEntry.mSize);

	// PT: we don't output the source data on purpose, to catch missing meta-data
	// sschirm: changed that to 0xcd, so we can mark the output as "having marked pads" 
	const unsigned char b = 0xcd;
	for(int i=0;i<entry.mSize;i++)
	{
		const size_t size = mOutStream->write(&b, 1);
		(void)size;
	}
	mOutputSize += entry.mSize;
}

void Sn::ConvX::convert16(const char* src, const PxMetaDataEntry& entry, const PxMetaDataEntry& dstEntry)
{
	(void)dstEntry;
	if(mNoOutput)
		return;

	PX_ASSERT(entry.mSize==int(sizeof(short)*entry.mCount));
	PX_ASSERT(mOutStream);
	PX_ASSERT(entry.mSize==dstEntry.mSize);

	const short* data = reinterpret_cast<const short*>(src);
	for(int i=0;i<entry.mCount;i++)
	{
		short value = *data++;
		if(mMustFlip)
			flip(value);

		const size_t size = mOutStream->write(&value, sizeof(short));
		PX_ASSERT(size==sizeof(short));
		mOutputSize += int(size);
	}
}

void Sn::ConvX::convert32(const char* src, const PxMetaDataEntry& entry, const PxMetaDataEntry& dstEntry)
{
	(void)dstEntry;
	if(mNoOutput)
		return;

	PX_ASSERT(entry.mSize==int(sizeof(int)*entry.mCount));
	PX_ASSERT(mOutStream);
	PX_ASSERT(entry.mSize==dstEntry.mSize);

	const int* data = reinterpret_cast<const int*>(src);
	for(int i=0;i<entry.mCount;i++)
	{
		int value = *data++;
		if(mMustFlip)
			flip(value);

		const size_t size = mOutStream->write(&value, sizeof(int));
		PX_ASSERT(size==sizeof(int));
		mOutputSize += int(size);
	}
}

void Sn::ConvX::convert64(const char* src, const PxMetaDataEntry& entry, const PxMetaDataEntry& dstEntry)
{
	(void)dstEntry;
	if(mNoOutput)
		return;

	PX_ASSERT(entry.mSize==int(sizeof(PxU64)*entry.mCount));
	PX_ASSERT(mOutStream);
	PX_ASSERT(entry.mSize==dstEntry.mSize);

	const PxU64* data = reinterpret_cast<const PxU64*>(src);
	for(int i=0;i<entry.mCount;i++)
	{
		PxU64 value = *data++;
		if(mMustFlip)
			value = ntohll_internal(value);

		const size_t size = mOutStream->write(&value, sizeof(PxU64));
		PX_ASSERT(size==sizeof(PxU64));
		mOutputSize += int(size);
	}
}

void Sn::ConvX::convertFloat(const char* src, const PxMetaDataEntry& entry, const PxMetaDataEntry& dstEntry)
{
	(void)dstEntry;
	if(mNoOutput)
		return;

	PX_ASSERT(entry.mSize==int(sizeof(float)*entry.mCount));
	PX_ASSERT(mOutStream);
	PX_ASSERT(entry.mSize==dstEntry.mSize);

	const float* data = reinterpret_cast<const float*>(src);
	for(int i=0;i<entry.mCount;i++)
	{
		float value = *data++;
		if(mMustFlip)
			flip(value);

		const size_t size = mOutStream->write(&value, sizeof(float));
		PX_ASSERT(size==sizeof(float));
		mOutputSize += int(size);
	}
}

void Sn::ConvX::convertPtr(const char* src, const PxMetaDataEntry& entry, const PxMetaDataEntry& dstEntry)
{
	(void)dstEntry;
	if(mNoOutput)
		return;

	PX_ASSERT(entry.mSize==mSrcPtrSize*entry.mCount);
	PX_ASSERT(mOutStream);

	char buffer[16];
	for(int i=0;i<entry.mCount;i++)
	{
		PxU64 testValue=0;
		// Src pointer can be 4 or 8 bytes so we can't use "void*" here
		if(mSrcPtrSize==4)
		{
			PX_ASSERT(sizeof(PxU32)==4);
			const PxU32* data = reinterpret_cast<const PxU32*>(src);
			PxU32 value = *data++;
			src = reinterpret_cast<const char*>(data);

			if(mActiveRemap)
			{
				PxU32 ref;
				if(mActiveRemap->getObjectRef(value, ref))
				{
					value = ref;
				}
				else if(value)
				{
	//				value = 0;
					//We use the pointer of mName for its length
					// PT: on serialization mName is transformed to an index by the name manager, so we should not modify its value.
					if(!entry.mName || strcmp(entry.mName, "mName"))
						value=0x12345678;
				}
			}
			else
			{
				//we should only get here during convertReferenceTables to build up the pointer map
				PxU32 ref;
				if (mRemap.getObjectRef(value, ref))
				{
					value = ref;
				}
				else if(value)
				{
					const PxU32 remappedRef = 0x80000000 | (mPointerRemapCounter++ +1);
					mRemap.setObjectRef(value, remappedRef);
					value = remappedRef;
				}
			}

			if(mMustFlip)
				flip(value);

			if(mNullPtr)
				value = 0;

			*reinterpret_cast<PxU32*>(buffer) = value;
		}
		else
		{
			PX_ASSERT(mSrcPtrSize==8);
			PX_ASSERT(sizeof(PxU64)==8);
			const PxU64* data = reinterpret_cast<const PxU64*>(src);
			PxU64 value = *data++;
			src = reinterpret_cast<const char*>(data);

			if(mActiveRemap)
			{
				PxU32 ref;
				if(mActiveRemap->getObjectRef(value, ref))
				{
					value = ref;
				}
				else if(value)
				{
	//				value = 0;
					//We use the pointer of mName for its length
					// PT: on serialization mName is transformed to an index by the name manager, so we should not modify its value.
					if(!entry.mName || strcmp(entry.mName, "mName"))
						value=0x12345678;
				}
			}
			else
			{
				//we should only get here during convertReferenceTables to build up the pointer map
				PxU32 ref;
				if (mRemap.getObjectRef(value, ref))
				{
					value = ref;
				}
				else if(value)
				{
					const PxU32 remappedRef = 0x80000000 | (mPointerRemapCounter++ +1);
					mRemap.setObjectRef(value, remappedRef);
					value = remappedRef;
				}
			}

//			PX_ASSERT(!mMustFlip);
//			if(mMustFlip)
//				flip(value);

			if(mNullPtr)
				value = 0;

			testValue = value;

			*reinterpret_cast<PxU64*>(buffer) = value;
		}

		if(mSrcPtrSize==mDstPtrSize)
		{
			const size_t size = mOutStream->write(buffer, PxU32(mSrcPtrSize));
			PX_ASSERT(size==PxU32(mSrcPtrSize));
			mOutputSize += int(size);
		}
		else
		{
			if(mDstPtrSize>mSrcPtrSize)
			{
				// 32bit to 64bit
				PX_ASSERT(mDstPtrSize==8);
				PX_ASSERT(mSrcPtrSize==4);

				// We need to output the lower 32bits first for PC. Might be different on a 64bit console....

				// Output src ptr for the lower 32bits
				const size_t size = mOutStream->write(buffer, PxU32(mSrcPtrSize));
				PX_ASSERT(size==PxU32(mSrcPtrSize));
				mOutputSize += int(size);

				// Output zeros for the higher 32bits
				const int zero = 0;
				const size_t size0 = mOutStream->write(&zero, 4);
				PX_ASSERT(size0==4);
				mOutputSize += int(size0);
			}
			else
			{
				// 64bit to 32bit
				PX_ASSERT(mSrcPtrSize==8);
				PX_ASSERT(mDstPtrSize==4);

				// Not sure how we can safely convert 64bit ptrs to 32bit... just drop the high 32 bits?!?

				PxU32 ptr32 = *reinterpret_cast<PxU32*>(buffer);
				(void)ptr32;
				PxU32 ptr32b = PxU32(testValue);
				(void)ptr32b;

				if(mMustFlip)
					flip(ptr32b);

				// Output src ptr for the lower 32bits
				const size_t size = mOutStream->write(&ptr32b, 4);
				PX_ASSERT(size==4);
				mOutputSize += int(size);
			}
		}
	}
}
