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

#ifndef PSFILEBUFFER_PSFILEBUFFER_H
#define PSFILEBUFFER_PSFILEBUFFER_H

#include "filebuf/PxFileBuf.h"

#include "Ps.h"
#include "PsUserAllocated.h"
#include <stdio.h>

namespace physx
{
namespace general_PxIOStream2
{
	using namespace shdfnd;

//Use this class if you want to use your own allocator
class PxFileBufferBase : public PxFileBuf
{
public:
	PxFileBufferBase(const char *fileName,OpenMode mode)
	{
		mOpenMode = mode;
		mFph = NULL;
		mFileLength = 0;
		mSeekRead   = 0;
		mSeekWrite  = 0;
		mSeekCurrent = 0;
		switch ( mode )
		{
			case OPEN_READ_ONLY:
				mFph = fopen(fileName,"rb");
				break;
			case OPEN_WRITE_ONLY:
				mFph = fopen(fileName,"wb");
				break;
			case OPEN_READ_WRITE_NEW:
				mFph = fopen(fileName,"wb+");
				break;
			case OPEN_READ_WRITE_EXISTING:
				mFph = fopen(fileName,"rb+");
				break;
			case OPEN_FILE_NOT_FOUND:
				break;
		}
		if ( mFph )
		{
			fseek(mFph,0L,SEEK_END);
			mFileLength = static_cast<uint32_t>(ftell(mFph));
			fseek(mFph,0L,SEEK_SET);
		}
		else
		{
			mOpenMode = OPEN_FILE_NOT_FOUND;
		}
    }

	virtual						~PxFileBufferBase()
	{
		close();
	}

	virtual void close()
	{
		if( mFph )
		{
			fclose(mFph);
			mFph = 0;
		}
	}

	virtual SeekType isSeekable(void) const
	{
		return mSeekType;
	}

	virtual		uint32_t			read(void* buffer, uint32_t size)	
	{
		uint32_t ret = 0;
		if ( mFph )
		{
			setSeekRead();
			ret = static_cast<uint32_t>(::fread(buffer,1,size,mFph));
			mSeekRead+=ret;
			mSeekCurrent+=ret;
		}
		return ret;
	}

	virtual		uint32_t			peek(void* buffer, uint32_t size)
	{
		uint32_t ret = 0;
		if ( mFph )
		{
			uint32_t loc = tellRead();
			setSeekRead();
			ret = static_cast<uint32_t>(::fread(buffer,1,size,mFph));
			mSeekCurrent+=ret;
			seekRead(loc);
		}
		return ret;
	}

	virtual		uint32_t		write(const void* buffer, uint32_t size)
	{
		uint32_t ret = 0;
		if ( mFph )
		{
			setSeekWrite();
			ret = static_cast<uint32_t>(::fwrite(buffer,1,size,mFph));
			mSeekWrite+=ret;
			mSeekCurrent+=ret;
			if ( mSeekWrite > mFileLength )
			{
				mFileLength = mSeekWrite;
			}
		}
		return ret;
	}

	virtual uint32_t tellRead(void) const
	{
		return mSeekRead;
	}

	virtual uint32_t tellWrite(void) const
	{
		return mSeekWrite;
	}

	virtual uint32_t seekRead(uint32_t loc) 
	{
		mSeekRead = loc;
		if ( mSeekRead > mFileLength )
		{
			mSeekRead = mFileLength;
		}
		return mSeekRead;
	}

	virtual uint32_t seekWrite(uint32_t loc)
	{
		mSeekWrite = loc;
		if ( mSeekWrite > mFileLength )
		{
			mSeekWrite = mFileLength;
		}
		return mSeekWrite;
	}

	virtual void flush(void)
	{
		if ( mFph )
		{
			::fflush(mFph);
		}
	}

	virtual OpenMode	getOpenMode(void) const
	{
		return mOpenMode;
	}

	virtual uint32_t getFileLength(void) const
	{
		return mFileLength;
	}

private:
	// Moves the actual file pointer to the current read location
	void setSeekRead(void) 
	{
		if ( mSeekRead != mSeekCurrent && mFph )
		{
			if ( mSeekRead >= mFileLength )
			{
				fseek(mFph,0L,SEEK_END);
			}
			else
			{
				fseek(mFph,static_cast<long>(mSeekRead),SEEK_SET);
			}
			mSeekCurrent = mSeekRead = static_cast<uint32_t>(ftell(mFph));
		}
	}
	// Moves the actual file pointer to the current write location
	void setSeekWrite(void)
	{
		if ( mSeekWrite != mSeekCurrent && mFph )
		{
			if ( mSeekWrite >= mFileLength )
			{
				fseek(mFph,0L,SEEK_END);
			}
			else
			{
				fseek(mFph,static_cast<long>(mSeekWrite),SEEK_SET);
			}
			mSeekCurrent = mSeekWrite = static_cast<uint32_t>(ftell(mFph));
		}
	}


	FILE		*mFph;
	uint32_t		mSeekRead;
	uint32_t		mSeekWrite;
	uint32_t		mSeekCurrent;
	uint32_t		mFileLength;
	SeekType	mSeekType;
	OpenMode	mOpenMode;
};

//Use this class if you want to use PhysX memory allocator
class PsFileBuffer: public PxFileBufferBase, public UserAllocated
{
public:
	PsFileBuffer(const char *fileName,OpenMode mode): PxFileBufferBase(fileName, mode) {}
};

}
using namespace general_PxIOStream2;
}

#endif // PSFILEBUFFER_PSFILEBUFFER_H
