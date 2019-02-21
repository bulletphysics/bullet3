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
#ifndef PX_XML_MEMORY_POOL_STREAMS_H
#define PX_XML_MEMORY_POOL_STREAMS_H

#include "foundation/PxTransform.h"
#include "foundation/PxIO.h"
#include "SnXmlMemoryPool.h"
#include "CmPhysXCommon.h"

namespace physx {

	template<typename TDataType>
	struct XmlDefaultValue
	{
		bool force_compile_error;
	};


#define XML_DEFINE_DEFAULT_VALUE(type, defVal )		\
	template<>											\
	struct XmlDefaultValue<type>						\
	{													\
		type getDefaultValue() { return type(defVal); }	\
	};

	XML_DEFINE_DEFAULT_VALUE(PxU8, 0)
	XML_DEFINE_DEFAULT_VALUE(PxI8, 0)
	XML_DEFINE_DEFAULT_VALUE(PxU16, 0)
	XML_DEFINE_DEFAULT_VALUE(PxI16, 0)
	XML_DEFINE_DEFAULT_VALUE(PxU32, 0)
	XML_DEFINE_DEFAULT_VALUE(PxI32, 0)
	XML_DEFINE_DEFAULT_VALUE(PxU64, 0)
	XML_DEFINE_DEFAULT_VALUE(PxI64, 0)
	XML_DEFINE_DEFAULT_VALUE(PxF32, 0)
	XML_DEFINE_DEFAULT_VALUE(PxF64, 0)

#undef XML_DEFINE_DEFAULT_VALUE

	template<>											
	struct XmlDefaultValue<PxVec3>						
	{													
		PxVec3 getDefaultValue() { return PxVec3( 0,0,0 ); }	
	};
	
	template<>											
	struct XmlDefaultValue<PxTransform>						
	{													
		PxTransform getDefaultValue() { return PxTransform(PxIdentity); }	
	};

	template<>											
	struct XmlDefaultValue<PxQuat>	
	{													
		PxQuat getDefaultValue() { return PxQuat(PxIdentity); }	
	};

/** 
 *	Mapping of PxOutputStream to a memory pool manager.
 *	Allows write-then-read semantics of a set of
 *	data.  Can safely write up to 4GB of data; then you
 *	will silently fail...
 */

template<typename TAllocatorType>
struct MemoryBufferBase : public PxOutputStream, public PxInputStream
{
	TAllocatorType* mManager;
	mutable PxU32	mWriteOffset;
	mutable PxU32	mReadOffset;
	PxU8*	mBuffer;
	PxU32	mCapacity;


	MemoryBufferBase( TAllocatorType* inManager )
		: mManager( inManager )
		, mWriteOffset( 0 )
		, mReadOffset( 0 )
		, mBuffer( NULL )
		, mCapacity( 0 )
	{
	}
	virtual						~MemoryBufferBase()
	{
		mManager->deallocate( mBuffer );
	}
	PxU8* releaseBuffer()
	{
		clear();
		mCapacity = 0;
		PxU8* retval(mBuffer);
		mBuffer = NULL;
		return retval;
	}
	void clear()
	{
		mWriteOffset = mReadOffset = 0;
	}

	virtual PxU32 read(void* dest, PxU32 count)
	{
		bool fits = ( mReadOffset + count ) <= mWriteOffset;
		PX_ASSERT( fits );
		if ( fits )
		{
			PxMemCopy( dest, mBuffer + mReadOffset, count );
			mReadOffset += count;
			return count;
		}
		return 0;
	}

	inline void checkCapacity( PxU32 inNewCapacity )
	{
		if ( mCapacity < inNewCapacity )
		{
			PxU32 newCapacity = 32;
			while( newCapacity < inNewCapacity )
				newCapacity = newCapacity << 1;

			PxU8* newData( mManager->allocate( newCapacity ) );
			if ( mWriteOffset )
				PxMemCopy( newData, mBuffer, mWriteOffset );
			mManager->deallocate( mBuffer );
			mBuffer = newData;
			mCapacity = newCapacity;
		}
	}

	virtual PxU32 write(const void* src, PxU32 count)
	{
		checkCapacity( mWriteOffset + count );
		PxMemCopy( mBuffer + mWriteOffset, src, count );
		mWriteOffset += count;
		return count;
	}
};

class MemoryBuffer : public MemoryBufferBase<CMemoryPoolManager >
{
public:
	MemoryBuffer( CMemoryPoolManager* inManager ) : MemoryBufferBase<CMemoryPoolManager >( inManager ) {}
};

}

#endif
