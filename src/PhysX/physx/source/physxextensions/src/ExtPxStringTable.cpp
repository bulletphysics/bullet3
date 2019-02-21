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


#include "foundation/PxAllocatorCallback.h"
#include "PxStringTableExt.h"
#include "PxProfileAllocatorWrapper.h" //tools for using a custom allocator
#include "PsString.h"
#include "PsUserAllocated.h"
#include "CmPhysXCommon.h"

namespace physx
{
	using namespace physx::profile;

	class PxStringTableImpl : public PxStringTable, public Ps::UserAllocated
	{
		typedef PxProfileHashMap<const char*, PxU32> THashMapType;
		PxProfileAllocatorWrapper mWrapper;
		THashMapType mHashMap;
	public:

		PxStringTableImpl( PxAllocatorCallback& inAllocator )
			: mWrapper ( inAllocator )
			, mHashMap ( mWrapper )
		{
		}

		virtual ~PxStringTableImpl()
		{
			for ( THashMapType::Iterator iter = mHashMap.getIterator();
				iter.done() == false;
				++iter )
				PX_PROFILE_DELETE( mWrapper, const_cast<char*>( iter->first ) );
			mHashMap.clear();
		}


		virtual const char* allocateStr( const char* inSrc )
		{
			if ( inSrc == NULL )
				inSrc = "";
			const THashMapType::Entry* existing( mHashMap.find( inSrc ) );
			if ( existing == NULL )
			{
				size_t len( strlen( inSrc ) );
				len += 1;
				char* newMem = reinterpret_cast<char*>(mWrapper.getAllocator().allocate( len, "PxStringTableImpl: const char*", __FILE__, __LINE__ ));
				physx::shdfnd::strlcpy( newMem, len, inSrc );
				mHashMap.insert( newMem, 1 );
				return newMem;
			}
			else
			{
				++const_cast<THashMapType::Entry*>(existing)->second;
				return existing->first;
			}
		}

		/**
		 *	Release the string table and all the strings associated with it.
		 */
		virtual void release()
		{
			PX_PROFILE_DELETE( mWrapper.getAllocator(), this );
		}
	};

	PxStringTable& PxStringTableExt::createStringTable( PxAllocatorCallback& inAllocator )
	{
		return *PX_PROFILE_NEW( inAllocator, PxStringTableImpl )( inAllocator );
	}
}
