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

#ifndef PSFILEBUFFER_PSMEMORYBUFFER_H
#define PSFILEBUFFER_PSMEMORYBUFFER_H

#include "Ps.h"
#include "PsUserAllocated.h"
#include "PsAlignedMalloc.h"
#include "filebuf/PxFileBuf.h"
#include "foundation/PxAssert.h"

namespace physx
{
namespace general_PxIOStream2
{
	using namespace shdfnd;

	const uint32_t BUFFER_SIZE_DEFAULT = 4096;

//Use this class if you want to use your own allocator
template<class Allocator>
class PxMemoryBufferBase : public PxFileBuf, public Allocator
{
	PX_NOCOPY(PxMemoryBufferBase)
	void init(const void *readMem, uint32_t readLen)
	{
		mAllocator = this;

		mReadBuffer = mReadLoc = static_cast<const uint8_t *>(readMem);
		mReadStop   = &mReadLoc[readLen];

		mWriteBuffer = mWriteLoc = mWriteStop = NULL;
		mWriteBufferSize = 0;
		mDefaultWriteBufferSize = BUFFER_SIZE_DEFAULT;

		mOpenMode = OPEN_READ_ONLY;
		mSeekType = SEEKABLE_READ;
	}

	void init(uint32_t defaultWriteBufferSize)
	{
		mAllocator = this;

		mReadBuffer = mReadLoc = mReadStop = NULL;

		mWriteBuffer = mWriteLoc = mWriteStop = NULL;
		mWriteBufferSize = 0;
		mDefaultWriteBufferSize = defaultWriteBufferSize;

		mOpenMode = OPEN_READ_WRITE_NEW;
		mSeekType = SEEKABLE_READWRITE;
	}

public:
	PxMemoryBufferBase(const void *readMem,uint32_t readLen)
	{
		init(readMem, readLen);
    }

	PxMemoryBufferBase(const void *readMem,uint32_t readLen, const Allocator &alloc): Allocator(alloc)
	{
		init(readMem, readLen);
    }

	PxMemoryBufferBase(uint32_t defaultWriteBufferSize = BUFFER_SIZE_DEFAULT)
    {
		init(defaultWriteBufferSize);
	}

	PxMemoryBufferBase(uint32_t defaultWriteBufferSize, const Allocator &alloc): Allocator(alloc)
    {
		init(defaultWriteBufferSize);
	}

	virtual ~PxMemoryBufferBase(void)
	{
		reset();
	}

	void setAllocator(Allocator *allocator)
	{
		mAllocator = allocator;
	}

	void initWriteBuffer(uint32_t size)
	{
		if ( mWriteBuffer == NULL )
		{
			if ( size < mDefaultWriteBufferSize ) size = mDefaultWriteBufferSize;
			mWriteBuffer = static_cast<uint8_t *>(mAllocator->allocate(size));
			PX_ASSERT( mWriteBuffer );
    		mWriteLoc    = mWriteBuffer;
    		mWriteStop	= &mWriteBuffer[size];
    		mWriteBufferSize = size;
    		mReadBuffer = mWriteBuffer;
    		mReadStop	= &mWriteBuffer[size];
    		mReadLoc    = mWriteBuffer;
		}
    }

	void reset(void)
	{
		mAllocator->deallocate(mWriteBuffer);
		mWriteBuffer = NULL;
		mWriteBufferSize = 0;
		mWriteLoc = NULL;
		mWriteStop = NULL;
		mReadBuffer = NULL;
		mReadStop = NULL;
		mReadLoc = NULL;
    }

	virtual OpenMode	getOpenMode(void) const
	{
		return mOpenMode;
	}


	SeekType isSeekable(void) const
	{
		return mSeekType;
	}

	virtual		uint32_t			read(void* buffer, uint32_t size)
	{
		if ( (mReadLoc+size) > mReadStop )
		{
			size = uint32_t(mReadStop - mReadLoc);
		}
		if ( size != 0 )
		{
			memmove(buffer,mReadLoc,size);
			mReadLoc+=size;
		}
		return size;
	}

	virtual		uint32_t			peek(void* buffer, uint32_t size)
	{
		if ( (mReadLoc+size) > mReadStop )
		{
			size = uint32_t(mReadStop - mReadLoc);
		}
		if ( size != 0 )
		{
			memmove(buffer,mReadLoc,size);
		}
		return size;
	}

	virtual		uint32_t		write(const void* buffer, uint32_t size)
	{
		PX_ASSERT( mOpenMode ==	OPEN_READ_WRITE_NEW );
		if ( mOpenMode == OPEN_READ_WRITE_NEW )
		{
    		if ( (mWriteLoc+size) > mWriteStop )
    		    growWriteBuffer(size);
    		memmove(mWriteLoc,buffer,size);
    		mWriteLoc+=size;
    		mReadStop = mWriteLoc;
    	}
    	else
    	{
    		size = 0;
    	}
		return size;
	}

	PX_INLINE const uint8_t * getReadLoc(void) const { return mReadLoc; }
	PX_INLINE void advanceReadLoc(uint32_t len)
	{
		PX_ASSERT(mReadBuffer);
		if ( mReadBuffer )
		{
			mReadLoc+=len;
			if ( mReadLoc >= mReadStop )
			{
				mReadLoc = mReadStop;
			}
		}
	}

	virtual uint32_t tellRead(void) const
	{
		uint32_t ret=0;

		if ( mReadBuffer )
		{
			ret = uint32_t(mReadLoc-mReadBuffer);
		}
		return ret;
	}

	virtual uint32_t tellWrite(void) const
	{
		return uint32_t(mWriteLoc-mWriteBuffer);
	}

	virtual uint32_t seekRead(uint32_t loc)
	{
		uint32_t ret = 0;
		PX_ASSERT(mReadBuffer);
		if ( mReadBuffer )
		{
			mReadLoc = &mReadBuffer[loc];
			if ( mReadLoc >= mReadStop )
			{
				mReadLoc = mReadStop;
			}
			ret = uint32_t(mReadLoc-mReadBuffer);
		}
		return ret;
	}

	virtual uint32_t seekWrite(uint32_t loc)
	{
		uint32_t ret = 0;
		PX_ASSERT( mOpenMode ==	OPEN_READ_WRITE_NEW );
		if ( mWriteBuffer )
		{
    		if ( loc > mWriteBufferSize )
			{
				mWriteLoc = mWriteStop;
    		    growWriteBuffer(loc - mWriteBufferSize);
			}
    		mWriteLoc = &mWriteBuffer[loc];
			ret = uint32_t(mWriteLoc-mWriteBuffer);
		}
		return ret;
	}

	virtual void flush(void)
	{

	}

	virtual uint32_t getFileLength(void) const
	{
		uint32_t ret = 0;
		if ( mReadBuffer )
		{
			ret = uint32_t(mReadStop-mReadBuffer);
		}
		else if ( mWriteBuffer )
		{
			ret = uint32_t(mWriteLoc-mWriteBuffer);
		}
		return ret;
	}

	uint32_t	getWriteBufferSize(void) const
	{
		return uint32_t(mWriteLoc-mWriteBuffer);
	}

	void setWriteLoc(uint8_t *writeLoc)
	{
		PX_ASSERT(writeLoc >= mWriteBuffer && writeLoc < mWriteStop );
		mWriteLoc = writeLoc;
		mReadStop = mWriteLoc;
	}

	const uint8_t * getWriteBuffer(void) const
	{
		return mWriteBuffer;
	}

	/**
	 * Attention: if you use aligned allocator you cannot free memory with PX_FREE macros instead use deallocate method from base
	 */
	uint8_t * getWriteBufferOwnership(uint32_t &dataLen) // return the write buffer, and zero it out, the caller is taking ownership of the memory
	{
		uint8_t *ret = mWriteBuffer;
		dataLen = uint32_t(mWriteLoc-mWriteBuffer);
		mWriteBuffer = NULL;
		mWriteLoc = NULL;
		mWriteStop = NULL;
		mWriteBufferSize = 0;
		return ret;
	}


	void alignRead(uint32_t a)
	{
		uint32_t loc = tellRead();
		uint32_t aloc = ((loc+(a-1))/a)*a;
		if ( aloc != loc )
		{
			seekRead(aloc);
		}
	}

	void alignWrite(uint32_t a)
	{
		uint32_t loc = tellWrite();
		uint32_t aloc = ((loc+(a-1))/a)*a;
		if ( aloc != loc )
		{
			seekWrite(aloc);
		}
	}

private:


	// double the size of the write buffer or at least as large as the 'size' value passed in.
	void growWriteBuffer(uint32_t size)
	{
		if ( mWriteBuffer == NULL )
		{
			if ( size < mDefaultWriteBufferSize ) size = mDefaultWriteBufferSize;
			initWriteBuffer(size);
		}
		else
		{
			uint32_t oldWriteIndex = uint32_t(mWriteLoc - mWriteBuffer);
			uint32_t newSize =	mWriteBufferSize*2;
			uint32_t avail = newSize-oldWriteIndex;
			if ( size >= avail ) newSize = newSize+size;
			uint8_t *writeBuffer = static_cast<uint8_t *>(mAllocator->allocate(newSize));
			PX_ASSERT( writeBuffer );
			memmove(writeBuffer,mWriteBuffer,mWriteBufferSize);
			mAllocator->deallocate(mWriteBuffer);
			mWriteBuffer = writeBuffer;
			mWriteBufferSize = newSize;
			mWriteLoc = &mWriteBuffer[oldWriteIndex];
			mWriteStop = &mWriteBuffer[mWriteBufferSize];
			uint32_t oldReadLoc = uint32_t(mReadLoc-mReadBuffer);
			mReadBuffer = mWriteBuffer;
			mReadStop   = mWriteLoc;
			mReadLoc = &mReadBuffer[oldReadLoc];
		}
	}

	const	uint8_t	*mReadBuffer;
	const	uint8_t	*mReadLoc;
	const	uint8_t	*mReadStop;

			uint8_t	*mWriteBuffer;
			uint8_t	*mWriteLoc;
			uint8_t	*mWriteStop;

			uint32_t	mWriteBufferSize;
			uint32_t	mDefaultWriteBufferSize;
			Allocator	*mAllocator;
			OpenMode	mOpenMode;
			SeekType	mSeekType;

};

class PxMemoryBufferAllocator
{
public:
	PxMemoryBufferAllocator(uint32_t a = 0) : alignment(a) {}

	virtual void * allocate(uint32_t size)
	{
		switch(alignment)
		{
		case 0:
			return PX_ALLOC(size, PX_DEBUG_EXP("PxMemoryBufferAllocator"));			
		case 16 :
			return physx::AlignedAllocator<16>().allocate(size, __FILE__, __LINE__);			
		case 32 :
			return physx::AlignedAllocator<32>().allocate(size, __FILE__, __LINE__);			
		case 64 :
			return physx::AlignedAllocator<64>().allocate(size, __FILE__, __LINE__);			
		case 128 :
			return physx::AlignedAllocator<128>().allocate(size, __FILE__, __LINE__);			
		default :
			PX_ASSERT(0);
		}
		return NULL;
	}
	virtual void deallocate(void *mem)
	{
		switch(alignment)
		{
		case 0:
			PX_FREE(mem);
			break;
		case 16 :
			physx::AlignedAllocator<16>().deallocate(mem);			
			break;
		case 32 :
			physx::AlignedAllocator<32>().deallocate(mem);
			break;
		case 64 :
			physx::AlignedAllocator<64>().deallocate(mem);
			break;
		case 128 :
			physx::AlignedAllocator<128>().deallocate(mem);
			break;
		default :
			PX_ASSERT(0);
		}
	}
	virtual ~PxMemoryBufferAllocator(void) {}
private:
	PxMemoryBufferAllocator& operator=(const PxMemoryBufferAllocator&);

	const uint32_t alignment;
};

//Use this class if you want to use PhysX memory allocator
class PsMemoryBuffer: public PxMemoryBufferBase<PxMemoryBufferAllocator>, public UserAllocated
{
	PX_NOCOPY(PsMemoryBuffer)
	typedef PxMemoryBufferBase<PxMemoryBufferAllocator> BaseClass;

public:
	PsMemoryBuffer(const void *readMem,uint32_t readLen): BaseClass(readMem, readLen) {}	
	PsMemoryBuffer(const void *readMem,uint32_t readLen, uint32_t alignment): BaseClass(readMem, readLen, PxMemoryBufferAllocator(alignment)) {}

	PsMemoryBuffer(uint32_t defaultWriteBufferSize=BUFFER_SIZE_DEFAULT): BaseClass(defaultWriteBufferSize) {}
	PsMemoryBuffer(uint32_t defaultWriteBufferSize,uint32_t alignment): BaseClass(defaultWriteBufferSize, PxMemoryBufferAllocator(alignment)) {}
};

}
using namespace general_PxIOStream2;
}

#endif // PSFILEBUFFER_PSMEMORYBUFFER_H

