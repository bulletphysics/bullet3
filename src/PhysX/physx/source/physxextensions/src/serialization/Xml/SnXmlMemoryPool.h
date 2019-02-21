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
#ifndef PX_XML_MEMORYPOOL_H
#define PX_XML_MEMORYPOOL_H

#include "foundation/PxAssert.h"
#include "PsArray.h"
#include "PxProfileAllocatorWrapper.h"

namespace physx { 

	using namespace physx::profile;

	/** 
	 *	Linked list used to store next node ptr.
	 */
	struct SMemPoolNode
	{
		SMemPoolNode* mNextNode;
	};

	/**
	 *	Template arguments are powers of two.
	 *	A very fast memory pool that is not memory efficient.  It contains a vector of pointers
	 *	to blocks of memory along with a linked list of free sections.  All sections are
	 *	of the same size so allocating memory is very fast, there isn't a linear search
	 *	through blocks of indeterminate size.  It also means there is memory wasted
	 *	when objects aren't sized to powers of two.
	 */
	template<PxU8 TItemSize
		, PxU8 TItemCount >
	class CMemoryPool
	{
		typedef PxProfileArray<PxU8*> TPxU8PtrList;

		PxProfileAllocatorWrapper& mWrapper;
		TPxU8PtrList mAllMemory;
		SMemPoolNode* mFirstFreeNode;
	public:
		CMemoryPool(PxProfileAllocatorWrapper& inWrapper) 
			: mWrapper( inWrapper )
			, mAllMemory( inWrapper )
			, mFirstFreeNode( NULL )
		{}
		~CMemoryPool()
		{
			TPxU8PtrList::ConstIterator theEnd = mAllMemory.end();
			for ( TPxU8PtrList::ConstIterator theIter = mAllMemory.begin();
				theIter != theEnd; 
				++theIter )
			{
				PxU8* thePtr = *theIter;
				mWrapper.getAllocator().deallocate( thePtr );
			}
			mAllMemory.clear();
			mFirstFreeNode = NULL;
		}
		//Using deallocated memory to hold the pointers to the next amount of memory.
		PxU8* allocate()
		{
			if ( mFirstFreeNode )
			{
				PxU8* retval = reinterpret_cast<PxU8*>(mFirstFreeNode);
				mFirstFreeNode = mFirstFreeNode->mNextNode;
				return retval;
			}
			PxU32 itemSize = GetItemSize();
			PxU32 itemCount = 1 << TItemCount;
			//No free nodes, make some more.
			PxU8* retval = reinterpret_cast<PxU8*>(mWrapper.getAllocator().allocate( itemCount * itemSize, "RepX fixed-size memory pool", __FILE__, __LINE__ ));
			PxU8* dataPtr = retval + itemSize;
			//Free extra chunks
			for( PxU32 idx = 1; idx < itemCount; ++idx, dataPtr += itemSize )
				deallocate( dataPtr );
			mAllMemory.pushBack(retval);
			return retval;
		}
		void deallocate( PxU8* inData )
		{
			SMemPoolNode* nodePtr = reinterpret_cast<SMemPoolNode*>(inData);
			nodePtr->mNextNode = mFirstFreeNode;
			mFirstFreeNode = nodePtr;
		}
		//We have to have at least a pointer's worth of memory
		inline PxU32 GetItemSize() { return sizeof(SMemPoolNode) << TItemSize; }
	};

	typedef PxU32 TMemAllocSizeType;

	struct SVariableMemPoolNode : SMemPoolNode
	{
		TMemAllocSizeType mSize;
		SVariableMemPoolNode* NextNode() { return static_cast< SVariableMemPoolNode* >( mNextNode ); }
	};

	/**
	 *	Manages variable sized allocations.  
	 *	Keeps track of freed allocations in a insertion sorted
	 *	list.  Allocating new memory traverses the list linearly.
	 *	This object will split nodes if the node is more than
	 *	twice as large as the request memory allocation.
	 */
	class CVariableMemoryPool
	{
		typedef PxProfileHashMap<TMemAllocSizeType, SVariableMemPoolNode*> TFreeNodeMap;
		typedef PxProfileArray<PxU8*> TPxU8PtrList;
		PxProfileAllocatorWrapper& mWrapper;
		TPxU8PtrList mAllMemory;
		TFreeNodeMap mFreeNodeMap;
		PxU32 mMinAllocationSize;
		
		CVariableMemoryPool &operator=(const CVariableMemoryPool &);

	public:
		CVariableMemoryPool(PxProfileAllocatorWrapper& inWrapper, PxU32 inMinAllocationSize = 0x20 ) 
			: mWrapper( inWrapper )
			, mAllMemory( inWrapper )
			, mFreeNodeMap( inWrapper)
			, mMinAllocationSize( inMinAllocationSize )
		{}

		~CVariableMemoryPool()
		{
			TPxU8PtrList::ConstIterator theEnd = mAllMemory.end();
			for ( TPxU8PtrList::ConstIterator theIter = mAllMemory.begin();
				theIter != theEnd; 
				++theIter )
			{
				PxU8* thePtr = *theIter;
				mWrapper.getAllocator().deallocate( thePtr );
			}
			mAllMemory.clear();
			mFreeNodeMap.clear();
		}
		PxU8* MarkMem( PxU8* inMem, TMemAllocSizeType inSize )
		{
			PX_ASSERT( inSize >= sizeof( SVariableMemPoolNode ) );
			SVariableMemPoolNode* theMem = reinterpret_cast<SVariableMemPoolNode*>( inMem );
			theMem->mSize = inSize;
			return reinterpret_cast< PxU8* >( theMem + 1 );
		}
		//Using deallocated memory to hold the pointers to the next amount of memory.
		PxU8* allocate( PxU32 size )
		{
			//Ensure we can place the size of the memory at the start
			//of the memory block.
			//Kai: to reduce the size of hash map, the requested size is aligned to 128 bytes
			PxU32 theRequestedSize = (size + sizeof(SVariableMemPoolNode) + 127) & ~127;

			TFreeNodeMap::Entry* entry = const_cast<TFreeNodeMap::Entry*>( mFreeNodeMap.find( theRequestedSize ) );
			if ( NULL != entry )
			{
				SVariableMemPoolNode* theNode = entry->second;
				PX_ASSERT( NULL != theNode );
				PX_ASSERT( theNode->mSize == theRequestedSize );
				entry->second = theNode->NextNode();
				if (entry->second == NULL)
					mFreeNodeMap.erase( theRequestedSize );

				return reinterpret_cast< PxU8* >( theNode + 1 );
			}

			if ( theRequestedSize < mMinAllocationSize )
				theRequestedSize = mMinAllocationSize;

			//No large enough free nodes, make some more.
			PxU8* retval = reinterpret_cast<PxU8*>(mWrapper.getAllocator().allocate( size_t(theRequestedSize), "RepX variable sized memory pool", __FILE__, __LINE__ ));
			//If we allocated it, we free it.
			mAllMemory.pushBack( retval );
			return MarkMem( retval, theRequestedSize );
		}

		//The size is stored at the beginning of the memory block.
		void deallocate( PxU8* inData )
		{
			SVariableMemPoolNode* theData = reinterpret_cast< SVariableMemPoolNode* >( inData ) - 1;
			TMemAllocSizeType theSize = theData->mSize;
			AddFreeMem( reinterpret_cast< PxU8* >( theData ), theSize );
		}

		void CheckFreeListInvariant( SVariableMemPoolNode* inNode )
		{
			if ( inNode && inNode->mNextNode )
			{
				PX_ASSERT( inNode->mSize <= inNode->NextNode()->mSize );
			}
		}

		void AddFreeMem( PxU8* inMemory, TMemAllocSizeType inSize )
		{
			PX_ASSERT( inSize >= sizeof( SVariableMemPoolNode ) );
			SVariableMemPoolNode* theNewNode = reinterpret_cast< SVariableMemPoolNode* >( inMemory );
			theNewNode->mNextNode = NULL;
			theNewNode->mSize = inSize;
			TFreeNodeMap::Entry* entry = const_cast<TFreeNodeMap::Entry*>( mFreeNodeMap.find( inSize ) );
			if (NULL != entry)
			{
				theNewNode->mNextNode = entry->second;
				entry->second = theNewNode;
			}
			else
			{
				mFreeNodeMap.insert( inSize, theNewNode );
			}
		}
	};

	/**
	 *	The manager keeps a list of memory pools for different sizes of allocations.
	 *	Anything too large simply gets allocated using the new operator.
	 *	This doesn't mark the memory with the size of the allocated memory thus
	 *	allowing much more efficient allocation of small items.  For large enough
	 *	allocations, it does mark the size.
	 *	
	 *	When using as a general memory manager, you need to wrap this class with
	 *	something that actually does mark the returned allocation with the size
	 *	of the allocation.
	 */
	class CMemoryPoolManager
	{
		CMemoryPoolManager &operator=(const CMemoryPoolManager &);

	public:
		PxProfileAllocatorWrapper mWrapper;

		//CMemoryPool<0,8> m0ItemPool;
		//CMemoryPool<1,8> m1ItemPool;
		//CMemoryPool<2,8> m2ItemPool;
		//CMemoryPool<3,8> m3ItemPool;
		//CMemoryPool<4,8> m4ItemPool;
		//CMemoryPool<5,8> m5ItemPool;
		//CMemoryPool<6,8> m6ItemPool;
		//CMemoryPool<7,8> m7ItemPool;
		//CMemoryPool<8,8> m8ItemPool;
		CVariableMemoryPool		mVariablePool;
		CMemoryPoolManager( PxAllocatorCallback& inAllocator )
			: mWrapper( inAllocator )
			//, m0ItemPool( mWrapper )
			//, m1ItemPool( mWrapper )
			//, m2ItemPool( mWrapper )
			//, m3ItemPool( mWrapper )
			//, m4ItemPool( mWrapper )
			//, m5ItemPool( mWrapper )
			//, m6ItemPool( mWrapper )
			//, m7ItemPool( mWrapper )
			//, m8ItemPool( mWrapper )
			, mVariablePool( mWrapper )
		{
		}
		PxProfileAllocatorWrapper& getWrapper() { return mWrapper; }
		inline PxU8* allocate( PxU32 inSize )
		{
			/*
			if ( inSize <= m0ItemPool.GetItemSize() )
				return m0ItemPool.allocate();
			if ( inSize <= m1ItemPool.GetItemSize() )
				return m1ItemPool.allocate();
			if ( inSize <= m2ItemPool.GetItemSize() )
				return m2ItemPool.allocate();
			if ( inSize <= m3ItemPool.GetItemSize() )
				return m3ItemPool.allocate();
			if ( inSize <= m4ItemPool.GetItemSize() )
				return m4ItemPool.allocate();
			if ( inSize <= m5ItemPool.GetItemSize() )
				return m5ItemPool.allocate();
			if ( inSize <= m6ItemPool.GetItemSize() )
				return m6ItemPool.allocate();
			if ( inSize <= m7ItemPool.GetItemSize() )
				return m7ItemPool.allocate();
			if ( inSize <= m8ItemPool.GetItemSize() )
				return m8ItemPool.allocate();
				*/
			return mVariablePool.allocate( inSize );
		}
		inline void deallocate( PxU8* inMemory )
		{
			if ( inMemory == NULL )
				return;
			/*
			if ( inSize <= m0ItemPool.GetItemSize() )
				m0ItemPool.deallocate(inMemory);
			else if ( inSize <= m1ItemPool.GetItemSize() )
				m1ItemPool.deallocate(inMemory);
			else if ( inSize <= m2ItemPool.GetItemSize() )
				m2ItemPool.deallocate(inMemory);
			else if ( inSize <= m3ItemPool.GetItemSize() )
				m3ItemPool.deallocate(inMemory);
			else if ( inSize <= m4ItemPool.GetItemSize() )
				m4ItemPool.deallocate(inMemory);
			else if ( inSize <= m5ItemPool.GetItemSize() )
				m5ItemPool.deallocate(inMemory);
			else if ( inSize <= m6ItemPool.GetItemSize() )
				m6ItemPool.deallocate(inMemory);
			else if ( inSize <= m7ItemPool.GetItemSize() )
				m7ItemPool.deallocate(inMemory);
			else if ( inSize <= m8ItemPool.GetItemSize() )
				m8ItemPool.deallocate(inMemory);
			else
			*/
				mVariablePool.deallocate(inMemory);
		}
		/**
		 *	allocate an object.  Calls constructor on the new memory.
		 */
		template<typename TObjectType>
		inline TObjectType* allocate()
		{
			TObjectType* retval = reinterpret_cast<TObjectType*>( allocate( sizeof(TObjectType) ) );
			new (retval)TObjectType();
			return retval;
		}

		/**
		 *	deallocate an object calling the destructor on the object.  
		 *	This *must* be the concrete type, it cannot be a generic type.
		 */
		template<typename TObjectType>
		inline void deallocate( TObjectType* inObject )
		{
			inObject->~TObjectType();
			deallocate( reinterpret_cast<PxU8*>(inObject) );
		}
		
		/**
		 *	allocate an object.  Calls constructor on the new memory.
		 */
		template<typename TObjectType>
		inline TObjectType* BatchAllocate(PxU32 inCount )
		{
			TObjectType* retval = reinterpret_cast<TObjectType*>( allocate( sizeof(TObjectType) * inCount ) );
			return retval;
		}

		/**
		 *	deallocate an object calling the destructor on the object.  
		 *	This *must* be the concrete type, it cannot be a generic type.
		 */
		template<typename TObjectType>
		inline void BatchDeallocate( TObjectType* inObject, PxU32 inCount )
		{
			PX_UNUSED(inCount);
			deallocate( reinterpret_cast<PxU8*>(inObject) );
		}
	};
}

#endif
