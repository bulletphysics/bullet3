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
#ifndef PX_XML_IMPL_H
#define PX_XML_IMPL_H

#include "SnXmlMemoryPool.h"
#include "PsString.h"
#include "foundation/PxMemory.h"

namespace physx { namespace Sn {

typedef CMemoryPoolManager TMemoryPoolManager;

namespace snXmlImpl {

	inline PxU32 strLen( const char* inStr )
	{
		PxU32 len = 0;
		if ( inStr )
		{
			while ( *inStr )
			{
				++len;
				++inStr;
			}
		}
		return len;
	}
}
	inline const char* copyStr( PxAllocatorCallback& inAllocator, const char* inStr )
	{
		if ( inStr && *inStr )
		{
			PxU32 theLen = snXmlImpl::strLen( inStr );
			//The memory will never be released by repx.  If you want it released, you need to pass in a custom allocator
			//that tracks all allocations and releases unreleased allocations yourself.
			char* dest = reinterpret_cast<char* >( inAllocator.allocate( theLen + 1, "Repx::const char*", __FILE__, __LINE__ ) );
			PxMemCopy( dest, inStr, theLen );
			dest[theLen] = 0;
			return dest;
		}
		return "";
	}

	template<typename TManagerType>
	inline const char* copyStr( TManagerType* inMgr, const char* inStr )
	{
		if ( inStr && *inStr )
		{
			PxU32 theLen = snXmlImpl::strLen( inStr );
			char* dest = reinterpret_cast<char* >( inMgr->allocate( theLen + 1 ) );
			PxMemCopy( dest, inStr, theLen );
			dest[theLen] = 0;
			return dest;
		}
		return "";
	}

	inline void releaseStr( TMemoryPoolManager* inMgr, const char* inStr, PxU32 )
	{
		if ( inStr && *inStr )
		{
			inMgr->deallocate( reinterpret_cast< PxU8* >( const_cast<char*>( inStr ) ) ); 
		}
	}

	inline void releaseStr( TMemoryPoolManager* inMgr, const char* inStr )
	{
		if ( inStr && *inStr )
		{
			PxU32 theLen = snXmlImpl::strLen( inStr );
			releaseStr( inMgr, inStr, theLen );
		}
	}

	struct XmlNode
	{
		const char* mName; //Never released until all collections are released
		const char* mData; //Never released until all collections are released
		
		XmlNode* mNextSibling;
		XmlNode* mPreviousSibling;
		XmlNode* mFirstChild;
		XmlNode* mParent;
		XmlNode( const XmlNode& );
		XmlNode& operator=( const XmlNode& );

		PX_INLINE void initPtrs()
		{
			mNextSibling = NULL;
			mPreviousSibling = NULL;
			mFirstChild = NULL;
			mParent = NULL;
		}

		PX_INLINE XmlNode( const char* inName = "", const char* inData = "" ) 
			: mName( inName )
			, mData( inData ) 
		{ initPtrs(); }

		void addChild( XmlNode* inItem )
		{
			inItem->mParent = this;
			if ( mFirstChild == NULL )
				mFirstChild = inItem;
			else
			{
				XmlNode* theNode = mFirstChild;
				//Follow the chain till the end.
				while( theNode->mNextSibling != NULL )
					theNode = theNode->mNextSibling;
				theNode->mNextSibling = inItem;
				inItem->mPreviousSibling = theNode;
			}
		}

		PX_INLINE XmlNode* findChildByName( const char* inName )
		{
			for ( XmlNode* theNode = mFirstChild; theNode; theNode = theNode->mNextSibling )
			{
				XmlNode* theRepXNode = theNode;
				if ( physx::shdfnd::stricmp( theRepXNode->mName, inName ) == 0 )
					return theNode;
			}
			return NULL;
		}
		
		PX_INLINE void orphan()
		{
			if ( mParent )
			{
				if ( mParent->mFirstChild == this )
					mParent->mFirstChild = mNextSibling;
			}
			if ( mPreviousSibling )
				mPreviousSibling->mNextSibling = mNextSibling;
			if ( mNextSibling )
				mNextSibling->mPreviousSibling = mPreviousSibling;
			if ( mFirstChild )
				mFirstChild->mParent = NULL;
			initPtrs();
		}
	};

	inline XmlNode* allocateRepXNode( TMemoryPoolManager* inManager, const char* inName, const char* inData )
	{
		XmlNode* retval = inManager->allocate<XmlNode>();
		retval->mName = copyStr( inManager, inName );
		retval->mData = copyStr( inManager, inData );
		return retval;
	}

	inline void release( TMemoryPoolManager* inManager, XmlNode* inNode )
	{
		//We *don't* release the strings associated with the node
		//because they could be shared.  Instead, we just let them 'leak'
		//in some sense, at least until the memory manager itself is deleted.
		//DO NOT UNCOMMENT THE LINES BELOW!!
		//releaseStr( inManager, inNode->mName );
		//releaseStr( inManager, inNode->mData );
		inManager->deallocate( inNode );
	}

	static PX_INLINE void releaseNodeAndChildren( TMemoryPoolManager* inManager, XmlNode* inNode )
	{
		if ( inNode->mFirstChild )
		{
			XmlNode* childNode( inNode->mFirstChild );
			while( childNode )
			{
				XmlNode* _node( childNode );
				childNode = _node->mNextSibling;
				releaseNodeAndChildren( inManager, _node );
			}
		}
		inNode->orphan();
		release( inManager, inNode );
	}

	static XmlNode* copyRepXNodeAndSiblings( TMemoryPoolManager* inManager, const XmlNode* inNode, XmlNode* inParent );

	static XmlNode* copyRepXNode( TMemoryPoolManager* inManager, const XmlNode* inNode, XmlNode* inParent = NULL )
	{
		XmlNode* newNode( allocateRepXNode( inManager, NULL, NULL ) );
		newNode->mName = inNode->mName; //Some light structural sharing
		newNode->mData = inNode->mData; //Some light structural sharing
		newNode->mParent = inParent;
		if ( inNode->mFirstChild )
			newNode->mFirstChild = copyRepXNodeAndSiblings( inManager, inNode->mFirstChild, newNode );
		return newNode;
	}
	
	static XmlNode* copyRepXNodeAndSiblings( TMemoryPoolManager* inManager, const XmlNode* inNode, XmlNode* inParent )
	{
		XmlNode* sibling = inNode->mNextSibling;
		if ( sibling ) sibling = copyRepXNodeAndSiblings( inManager, sibling, inParent );
		XmlNode* newNode = copyRepXNode( inManager, inNode, inParent );
		newNode->mNextSibling = sibling;
		if ( sibling ) sibling->mPreviousSibling = newNode;
		return newNode;
	}

	inline bool isBigEndian() { int i = 1; return *(reinterpret_cast<char*>(&i))==0; }
	

	struct NameStackEntry
	{
		const char* mName;
		bool		mOpen;
		NameStackEntry( const char* nm ) : mName( nm ), mOpen( false ) {}
	};

	typedef PxProfileArray<NameStackEntry> TNameStack;
}  }


#endif
