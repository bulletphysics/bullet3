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
#ifndef PX_XML_MEMORY_ALLOCATOR_H
#define PX_XML_MEMORY_ALLOCATOR_H

#include "foundation/PxSimpleTypes.h"

namespace physx { 

	class XmlMemoryAllocator
	{
	protected:
		virtual ~XmlMemoryAllocator(){}
	public:
		virtual PxU8* allocate(PxU32 inSize) = 0;
		virtual void deallocate( PxU8* inMem ) = 0;
		virtual PxAllocatorCallback& getAllocator() = 0;
		template<typename TObjectType>
		TObjectType* allocate()
		{
			TObjectType* retval = reinterpret_cast< TObjectType* >( allocate( sizeof( TObjectType ) ) );
			new (retval) TObjectType();
			return retval;
		}

		template<typename TObjectType, typename TArgType>
		TObjectType* allocate(const TArgType &arg)
		{
			TObjectType* retval = reinterpret_cast< TObjectType* >( allocate( sizeof( TObjectType ) ) );
			new (retval) TObjectType(arg);
			return retval;
		}

		template<typename TObjectType>
		void deallocate( TObjectType* inObject )
		{
			deallocate( reinterpret_cast<PxU8*>( inObject ) );
		}
		template<typename TObjectType>
		inline TObjectType* batchAllocate(PxU32 inCount )
		{
			TObjectType* retval = reinterpret_cast<TObjectType*>( allocate( sizeof(TObjectType) * inCount ) );
			for ( PxU32 idx = 0; idx < inCount; ++idx )
			{
				new (retval + idx) TObjectType();
			}
			return retval;
		}

		template<typename TObjectType, typename TArgType>
		inline TObjectType* batchAllocate(PxU32 inCount, const TArgType &arg)
		{
			TObjectType* retval = reinterpret_cast<TObjectType*>( allocate( sizeof(TObjectType) * inCount ) );
			for ( PxU32 idx = 0; idx < inCount; ++idx )
			{
				new (retval + idx) TObjectType(arg);
			}
			return retval;
		}


		//Duplicate function definition for gcc.
		template<typename TObjectType>
		inline TObjectType* batchAllocate(TObjectType*, PxU32 inCount )
		{
			TObjectType* retval = reinterpret_cast<TObjectType*>( allocate( sizeof(TObjectType) * inCount ) );
			for ( PxU32 idx = 0; idx < inCount; ++idx )
			{
				new (retval + idx) TObjectType();
			}
			return retval;
		}
	};
	
	struct XmlMemoryAllocatorImpl : public XmlMemoryAllocator
	{
		Sn::TMemoryPoolManager mManager;

		XmlMemoryAllocatorImpl( PxAllocatorCallback& inAllocator )
			: mManager( inAllocator )
		{
		}
		XmlMemoryAllocatorImpl &operator=(const XmlMemoryAllocatorImpl &);
		virtual PxAllocatorCallback& getAllocator()
		{
			return mManager.getWrapper().getAllocator();
		}
		
		virtual PxU8* allocate(PxU32 inSize )
		{
			if ( !inSize )
				return NULL;

			return mManager.allocate( inSize );
		}
		virtual void deallocate( PxU8* inMem )
		{
			if ( inMem )
				mManager.deallocate( inMem );
		}
	};
}
#endif
