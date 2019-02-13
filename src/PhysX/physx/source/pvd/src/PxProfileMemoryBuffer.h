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


#ifndef PXPVDSDK_PXPROFILEMEMORYBUFFER_H
#define PXPVDSDK_PXPROFILEMEMORYBUFFER_H

#include "PsAllocator.h"
#include "foundation/PxMemory.h"

namespace physx { namespace profile {

	template<typename TAllocator = typename shdfnd::AllocatorTraits<uint8_t>::Type >
	class MemoryBuffer : public TAllocator
	{
		uint8_t* mBegin;
		uint8_t* mEnd;
		uint8_t* mCapacityEnd;

	public:
		MemoryBuffer( const TAllocator& inAlloc = TAllocator() ) : TAllocator( inAlloc ), mBegin( 0 ), mEnd( 0 ), mCapacityEnd( 0 ) {}
		~MemoryBuffer()
		{
			if ( mBegin ) TAllocator::deallocate( mBegin );
		}
		uint32_t size() const { return static_cast<uint32_t>( mEnd - mBegin ); }
		uint32_t capacity() const { return static_cast<uint32_t>( mCapacityEnd - mBegin ); }
		uint8_t* begin() { return mBegin; }
		uint8_t* end() { return mEnd; }
		void setEnd(uint8_t* nEnd) { mEnd = nEnd; }
		const uint8_t* begin() const { return mBegin; }
		const uint8_t* end() const { return mEnd; }
		void clear() { mEnd = mBegin; }
		uint32_t write( uint8_t inValue )
		{
			growBuf( 1 );
			*mEnd = inValue;
			++mEnd;
			return 1;
		}

		template<typename TDataType>
		uint32_t write( const TDataType& inValue )
		{
			uint32_t writtenSize = sizeof(TDataType);
			growBuf(writtenSize);
			const uint8_t* __restrict readPtr = reinterpret_cast< const uint8_t* >( &inValue );
			uint8_t* __restrict writePtr = mEnd;
			for ( uint32_t idx = 0; idx < sizeof(TDataType); ++idx ) writePtr[idx] = readPtr[idx];
			mEnd += writtenSize;
			return writtenSize;
		}
		
		template<typename TDataType>
		uint32_t write( const TDataType* inValue, uint32_t inLength )
		{
			if ( inValue && inLength )
			{
				uint32_t writeSize = inLength * sizeof( TDataType );
				growBuf( writeSize );
				PxMemCopy( mBegin + size(), inValue, writeSize );
				mEnd += writeSize;
				return writeSize;
			}
			return 0;
		}

		// used by atomic write. Store the data and write the end afterwards
		// we dont check the buffer size, it should not resize on the fly
		template<typename TDataType>
		uint32_t write(const TDataType* inValue, uint32_t inLength, int32_t index)
		{
			if (inValue && inLength)
			{
				uint32_t writeSize = inLength * sizeof(TDataType);
				PX_ASSERT(mBegin + index + writeSize < mCapacityEnd);
				PxMemCopy(mBegin + index, inValue, writeSize);				
				return writeSize;
			}
			return 0;
		}
		
		void growBuf( uint32_t inAmount )
		{
			uint32_t newSize = size() + inAmount;
			reserve( newSize );
		}
		void resize( uint32_t inAmount )
		{
			reserve( inAmount );
			mEnd = mBegin + inAmount;
		}
		void reserve( uint32_t newSize )
		{
			uint32_t currentSize = size();
			if ( newSize >= capacity() )
			{
				const uint32_t allocSize = mBegin ? newSize * 2 : newSize;

				uint8_t* newData = static_cast<uint8_t*>(TAllocator::allocate(allocSize, __FILE__, __LINE__));
				memset(newData, 0xf,allocSize);
				if ( mBegin )
				{
					PxMemCopy( newData, mBegin, currentSize );
					TAllocator::deallocate( mBegin );
				}
				mBegin = newData;
				mEnd = mBegin + currentSize;
				mCapacityEnd = mBegin + allocSize;
			}
		}
	};

	
	class TempMemoryBuffer
	{
		uint8_t* mBegin;
		uint8_t* mEnd;
		uint8_t* mCapacityEnd;

	public:
		TempMemoryBuffer(uint8_t* data, int32_t size) : mBegin(data), mEnd(data), mCapacityEnd(data + size) {}
		~TempMemoryBuffer()
		{			
		}
		uint32_t size() const { return static_cast<uint32_t>(mEnd - mBegin); }
		uint32_t capacity() const { return static_cast<uint32_t>(mCapacityEnd - mBegin); }
		const uint8_t* begin() { return mBegin; }
		uint8_t* end() { return mEnd; }
		const uint8_t* begin() const { return mBegin; }
		const uint8_t* end() const { return mEnd; }		
		uint32_t write(uint8_t inValue)
		{			
			*mEnd = inValue;
			++mEnd;
			return 1;
		}

		template<typename TDataType>
		uint32_t write(const TDataType& inValue)
		{
			uint32_t writtenSize = sizeof(TDataType);			
			const uint8_t* __restrict readPtr = reinterpret_cast<const uint8_t*>(&inValue);
			uint8_t* __restrict writePtr = mEnd;
			for (uint32_t idx = 0; idx < sizeof(TDataType); ++idx) writePtr[idx] = readPtr[idx];
			mEnd += writtenSize;
			return writtenSize;
		}

		template<typename TDataType>
		uint32_t write(const TDataType* inValue, uint32_t inLength)
		{
			if (inValue && inLength)
			{
				uint32_t writeSize = inLength * sizeof(TDataType);
				PxMemCopy(mBegin + size(), inValue, writeSize);
				mEnd += writeSize;
				return writeSize;
			}
			return 0;
		}
	};

}}

#endif // PXPVDSDK_PXPROFILEMEMORYBUFFER_H
