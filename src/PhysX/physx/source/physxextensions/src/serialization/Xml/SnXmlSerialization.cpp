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
#include "SnXmlImpl.h"
#include "PsHash.h"
#include "PsHashMap.h"
#include "SnSimpleXmlWriter.h"
#include "PsSort.h"
#include "PsFastXml.h"
#include "PsString.h"
#include "SnXmlMemoryPool.h"
#include "PxExtensionMetaDataObjects.h"
#include "SnXmlVisitorWriter.h"
#include "SnXmlVisitorReader.h"
#include "SnXmlMemoryAllocator.h"
#include "SnXmlStringToType.h"
#include "PsString.h"
#include "SnRepXCollection.h"
#include "SnRepXUpgrader.h"
#include "../SnSerializationRegistry.h"
#include "PsFoundation.h"
#include "CmCollection.h"

using namespace physx;
using namespace Sn;

using namespace physx::profile; //for the foundation wrapper system.

namespace physx { namespace Sn {	

	class XmlNodeWriter : public SimpleXmlWriter
	{
		XmlMemoryAllocatorImpl&		mParseAllocator;
		XmlNode*					mCurrentNode;
		XmlNode*					mTopNode;
		PxU32						mTabCount;

	public:
		XmlNodeWriter( XmlMemoryAllocatorImpl& inAllocator, PxU32 inTabCount = 0 ) 
			: mParseAllocator( inAllocator ) 
			, mCurrentNode( NULL )
			, mTopNode( NULL )
			, mTabCount( inTabCount )
		{}
		XmlNodeWriter& operator=(const XmlNodeWriter&);
		virtual ~XmlNodeWriter(){}
		void onNewNode( XmlNode* newNode )
		{
			if ( mCurrentNode != NULL )
				mCurrentNode->addChild( newNode );
			if ( mTopNode == NULL )
				mTopNode = newNode;
			mCurrentNode = newNode;
			++mTabCount;
		}

		XmlNode* getTopNode() const { return mTopNode; }

		virtual void beginTag( const char* inTagname )
		{
			onNewNode( allocateRepXNode( &mParseAllocator.mManager, inTagname, NULL ) );
		}
		virtual void endTag()
		{
			if ( mCurrentNode )
				mCurrentNode = mCurrentNode->mParent;
			if ( mTabCount )
				--mTabCount;
		}
		virtual void addAttribute( const char*, const char* )
		{
			PX_ASSERT( false );
		}
		virtual void writeContentTag( const char* inTag, const char* inContent )
		{
			onNewNode( allocateRepXNode( &mParseAllocator.mManager, inTag, inContent ) );
			endTag();
		}
		virtual void addContent( const char* inContent )
		{
			if ( mCurrentNode->mData )
				releaseStr( &mParseAllocator.mManager, mCurrentNode->mData );
			mCurrentNode->mData = copyStr( &mParseAllocator.mManager, inContent );
		}
		virtual PxU32 tabCount() { return mTabCount; }
	};

	struct XmlWriterImpl : public XmlWriter
	{
		PxU32				mTagDepth;
		SimpleXmlWriter*	mWriter;
		MemoryBuffer*		mMemBuffer;

		XmlWriterImpl( SimpleXmlWriter* inWriter, MemoryBuffer* inMemBuffer )
			: mTagDepth( 0 )
			, mWriter( inWriter )
			, mMemBuffer( inMemBuffer )
		{
		}
		~XmlWriterImpl()
		{
			while( mTagDepth )
			{
				--mTagDepth;
				mWriter->endTag();
			}
		}
		virtual void write( const char* inName, const char* inData )
		{
			mWriter->writeContentTag( inName, inData );
		}
		virtual void write( const char* inName, const PxRepXObject& inLiveObject )
		{
			(*mMemBuffer) << inLiveObject.id;
			writeProperty( *mWriter, *mMemBuffer, inName );
		}
		virtual void addAndGotoChild( const char* inName )
		{
			mWriter->beginTag( inName );
			mTagDepth++;
		}
		virtual void leaveChild()
		{
			if ( mTagDepth )
			{
				mWriter->endTag();
				--mTagDepth;
			}
		}
	};

	struct XmlParseArgs
	{
		XmlMemoryAllocatorImpl*					mAllocator;
		PxProfileArray<RepXCollectionItem>*		mCollection;
		
		XmlParseArgs( XmlMemoryAllocatorImpl* inAllocator
			, PxProfileArray<RepXCollectionItem>* inCollection)
			: mAllocator( inAllocator )
			, mCollection( inCollection )
		{
		}
	};

	struct XmlNodeReader : public XmlReaderWriter
	{
		PxProfileAllocatorWrapper mWrapper;
		CMemoryPoolManager& mManager;
		XmlNode* mCurrentNode;
		XmlNode* mTopNode;
		PxProfileArray<XmlNode*> mContext;
		XmlNodeReader( XmlNode* inCurrentNode, PxAllocatorCallback& inAllocator, CMemoryPoolManager& nodePoolManager )
			: mWrapper( inAllocator )
			, mManager( nodePoolManager )
			, mCurrentNode( inCurrentNode )
			, mTopNode( inCurrentNode )
			, mContext( mWrapper )
		{
		}

		//Does this node exist as data in the format.
		virtual bool read( const char* inName, const char*& outData )
		{
			XmlNode* theChild( mCurrentNode->findChildByName( inName ) );
			if ( theChild )
			{
				outData = theChild->mData;
				return outData && *outData;
			}
			return false;
		}

		virtual bool read( const char* inName, PxSerialObjectId& outId )
		{
			XmlNode* theChild( mCurrentNode->findChildByName( inName ) );
			if ( theChild )
			{
				const char* theValue( theChild->mData );
				strto( outId, theValue );
				return true;
			}
			return false;
		}

		virtual bool gotoChild( const char* inName )
		{
			XmlNode* theChild( mCurrentNode->findChildByName( inName ) );
			if ( theChild )
			{
				mCurrentNode =theChild;
				return true;
			}
			return false;
		}
		virtual bool gotoFirstChild()
		{
			if ( mCurrentNode->mFirstChild )
			{
				mCurrentNode = mCurrentNode->mFirstChild;
				return true;
			}
			return false;
		}
		virtual bool gotoNextSibling()
		{
			if ( mCurrentNode->mNextSibling )
			{
				mCurrentNode = mCurrentNode->mNextSibling;
				return true;
			}
			return false;
		}
		virtual PxU32 countChildren()
		{
			PxU32 retval=  0;
			for ( XmlNode* theChild = mCurrentNode->mFirstChild; theChild != NULL; theChild = theChild->mNextSibling )
				++retval;
			return retval;
		}
		virtual const char* getCurrentItemName()
		{
			return mCurrentNode->mName;
		}
		virtual const char* getCurrentItemValue()
		{
			return mCurrentNode->mData;
		}

		virtual bool leaveChild()
		{
			if ( mCurrentNode != mTopNode && mCurrentNode->mParent )
			{
				mCurrentNode = mCurrentNode->mParent;
				return true;
			}
			return false;
		}

		virtual void pushCurrentContext()
		{
			mContext.pushBack( mCurrentNode );
		}
		virtual void popCurrentContext()
		{
			if ( mContext.size() )
			{
				mCurrentNode = mContext.back();
				mContext.popBack();
			}
		}

		virtual void setNode( XmlNode& inNode )
		{
			mContext.clear();
			mCurrentNode = &inNode;
			mTopNode = mCurrentNode;
		}

		virtual XmlReader* getParentReader()
		{
			XmlReader* retval = PX_PLACEMENT_NEW((mWrapper.getAllocator().allocate(sizeof(XmlNodeReader), "createNodeEditor",  __FILE__, __LINE__ )), XmlNodeReader) 
				( mTopNode, mWrapper.getAllocator(), mManager );
			return retval;
		}

		virtual void addOrGotoChild( const char* inName )
		{
			if ( gotoChild( inName )== false )
			{
				XmlNode* newNode = allocateRepXNode( &mManager, inName, NULL );
				mCurrentNode->addChild( newNode );
				mCurrentNode = newNode;
			}
		}
		virtual void setCurrentItemValue( const char* inValue )
		{
			mCurrentNode->mData = copyStr( &mManager, inValue ); 
		}
		virtual bool removeChild( const char* name )
		{
			XmlNode* theChild( mCurrentNode->findChildByName( name ) );
			if ( theChild )
			{
				releaseNodeAndChildren( &mManager, theChild );
				return true;
			}
			return false;
		}
		virtual void release() { this->~XmlNodeReader(); mWrapper.getAllocator().deallocate(this); }

	private:
		XmlNodeReader& operator=(const XmlNodeReader&);
	};

	PX_INLINE void  freeNodeAndChildren( XmlNode* tempNode, TMemoryPoolManager& inManager )
	{
		for( XmlNode* theNode = tempNode->mFirstChild; theNode != NULL; theNode = theNode->mNextSibling )
			freeNodeAndChildren( theNode, inManager );
		tempNode->orphan();
		release( &inManager, tempNode );
	}

	class XmlParser : public Ps::FastXml::Callback
	{
		XmlParseArgs			mParseArgs;
		//For parse time only allocations
		XmlMemoryAllocatorImpl& mParseAllocator;
		XmlNode* mCurrentNode;
		XmlNode* mTopNode;

	public:
		XmlParser( XmlParseArgs inArgs, XmlMemoryAllocatorImpl& inParseAllocator )
			: mParseArgs( inArgs )
			, mParseAllocator( inParseAllocator )
			, mCurrentNode( NULL )
			, mTopNode( NULL )
		{
		}

		virtual ~XmlParser(){}

		virtual bool processComment(const char* /*comment*/) { return true; }
		// 'element' is the name of the element that is being closed.
		// depth is the recursion depth of this element.
		// Return true to continue processing the XML file.
		// Return false to stop processing the XML file; leaves the read pointer of the stream right after this close tag.
		// The bool 'isError' indicates whether processing was stopped due to an error, or intentionally canceled early.
		virtual bool processClose(const char* /*element*/,physx::PxU32 /*depth*/,bool& /*isError*/)
		{
			mCurrentNode = mCurrentNode->mParent;
			return true;
		}

		// return true to continue processing the XML document, false to skip.
		virtual bool processElement(
			const char *elementName,   // name of the element		
			const char  *elementData,  // element data, null if none
			const Ps::FastXml::AttributePairs& attr,      // attributes
			PxI32 /*lineno*/)
		{
			XmlNode* newNode = allocateRepXNode( &mParseAllocator.mManager, elementName, elementData );
			if ( mCurrentNode )
				mCurrentNode->addChild( newNode );
			mCurrentNode = newNode;
			//Add the elements as children.
			for( PxI32 item = 0; item < attr.getNbAttr(); item ++ )
			{
				XmlNode* node = allocateRepXNode( &mParseAllocator.mManager, attr.getKey(PxU32(item)), attr.getValue(PxU32(item)) );
				mCurrentNode->addChild( node );
			}
			if ( mTopNode == NULL ) mTopNode = newNode;
			return true;
		}

		XmlNode* getTopNode() { return mTopNode; }

		virtual void *  allocate(PxU32 size)
		{ 
			if ( size )
				return mParseAllocator.allocate(size);
			return NULL; 
		}
		virtual void	deallocate(void *mem)
		{ 
			if ( mem )
				mParseAllocator.deallocate(reinterpret_cast<PxU8*>(mem)); 
		}

	private:
		XmlParser& operator=(const XmlParser&);
	};

	struct RepXCollectionSharedData
	{
		PxProfileAllocatorWrapper		mWrapper;
		XmlMemoryAllocatorImpl			mAllocator;
		PxU32							mRefCount;

		RepXCollectionSharedData( PxAllocatorCallback& inAllocator )
			: mWrapper( inAllocator )
			, mAllocator( inAllocator )
			, mRefCount( 0 )
		{
		}
		~RepXCollectionSharedData() {}
		
		void addRef() { ++mRefCount;}
		void release()
		{
			if ( mRefCount ) --mRefCount;
			if ( !mRefCount ) { this->~RepXCollectionSharedData(); mWrapper.getAllocator().deallocate(this);} 
		}
	};

	struct SharedDataPtr
	{
		RepXCollectionSharedData* mData;
		SharedDataPtr( RepXCollectionSharedData* inData )
			: mData( inData )
		{
			mData->addRef();
		}
		SharedDataPtr( const SharedDataPtr& inOther )
			: mData( inOther.mData )
		{
			mData->addRef();
		}
		SharedDataPtr& operator=( const SharedDataPtr& inOther );
		~SharedDataPtr()
		{
			mData->release();
			mData = NULL;
		}
		RepXCollectionSharedData* operator->() { return mData; }
		const RepXCollectionSharedData* operator->() const { return mData; }
	};
	
	class RepXCollectionImpl : public RepXCollection, public Ps::UserAllocated
	{
		SharedDataPtr							mSharedData;

		XmlMemoryAllocatorImpl&					mAllocator;
		PxSerializationRegistry&				mSerializationRegistry;
		PxProfileArray<RepXCollectionItem>		mCollection;
		TMemoryPoolManager						mSerializationManager;
		MemoryBuffer							mPropertyBuffer;
		PxTolerancesScale						mScale;
		PxVec3									mUpVector;
		const char*								mVersionStr;
		PxCollection*							mPxCollection;
		
	public:
		RepXCollectionImpl( PxSerializationRegistry& inRegistry, PxAllocatorCallback& inAllocator, PxCollection& inPxCollection )
			: mSharedData( &PX_NEW_REPX_SERIALIZER( RepXCollectionSharedData ))
			, mAllocator( mSharedData->mAllocator )
			, mSerializationRegistry( inRegistry )
			, mCollection( mSharedData->mWrapper )
			, mSerializationManager( inAllocator )
			, mPropertyBuffer( &mSerializationManager )
			, mUpVector( 0,0,0 )
			, mVersionStr( getLatestVersion() )
			, mPxCollection( &inPxCollection )
		{
			memset( &mScale, 0, sizeof( PxTolerancesScale ) );
			PX_ASSERT( mScale.isValid() == false );
		}

		RepXCollectionImpl( PxSerializationRegistry& inRegistry, const RepXCollectionImpl& inSrc, const char* inNewVersion )
			: mSharedData( inSrc.mSharedData )
			, mAllocator( mSharedData->mAllocator )
			, mSerializationRegistry( inRegistry )
			, mCollection( mSharedData->mWrapper )
			, mSerializationManager( mSharedData->mWrapper.getAllocator() )
			, mPropertyBuffer( &mSerializationManager )
			, mScale( inSrc.mScale )
			, mUpVector( inSrc.mUpVector )
			, mVersionStr( inNewVersion )
			, mPxCollection( NULL )
		{
		}

		virtual ~RepXCollectionImpl()
		{
			PxU32 numItems = mCollection.size();
			for ( PxU32 idx = 0; idx < numItems; ++idx )
			{
				XmlNode* theNode = mCollection[idx].descriptor;
				releaseNodeAndChildren( &mAllocator.mManager, theNode );
			}
		}
		RepXCollectionImpl& operator=(const RepXCollectionImpl&);

		virtual void destroy() 
		{ 
			PxProfileAllocatorWrapper tempWrapper( mSharedData->mWrapper.getAllocator() );
			this->~RepXCollectionImpl();
			tempWrapper.getAllocator().deallocate(this); 
		}
		
		virtual void setTolerancesScale(const PxTolerancesScale& inScale) {  mScale = inScale; }
		virtual PxTolerancesScale getTolerancesScale() const { return mScale; }
		virtual void setUpVector( const PxVec3& inUpVector ) { mUpVector = inUpVector; }
		virtual PxVec3 getUpVector() const { return mUpVector; }

	
		PX_INLINE RepXCollectionItem findItemBySceneItem( const PxRepXObject& inObject ) const
		{
			//See if the object is in the collection
			for ( PxU32 idx =0; idx < mCollection.size(); ++idx )
				if ( mCollection[idx].liveObject.serializable == inObject.serializable )
					return mCollection[idx];
			return RepXCollectionItem();
		}

		virtual RepXAddToCollectionResult addRepXObjectToCollection( const PxRepXObject& inObject, PxCollection* inCollection, PxRepXInstantiationArgs& inArgs )
		{
			PX_ASSERT( inObject.serializable );
			PX_ASSERT( inObject.id );
			if ( inObject.serializable == NULL || inObject.id == 0 )
				return RepXAddToCollectionResult( RepXAddToCollectionResult::InvalidParameters );
		
			PxRepXSerializer* theSerializer = mSerializationRegistry.getRepXSerializer( inObject.typeName );
			if ( theSerializer == NULL )
				return RepXAddToCollectionResult( RepXAddToCollectionResult::SerializerNotFound );

			RepXCollectionItem existing = findItemBySceneItem( inObject );
			if ( existing.liveObject.serializable )
				return RepXAddToCollectionResult( RepXAddToCollectionResult::AlreadyInCollection, existing.liveObject.id );
			
			XmlNodeWriter theXmlWriter( mAllocator, 1 );
			XmlWriterImpl theRepXWriter( &theXmlWriter, &mPropertyBuffer );
			{
				SimpleXmlWriter::STagWatcher theWatcher( theXmlWriter, inObject.typeName );
				writeProperty( theXmlWriter, mPropertyBuffer, "Id", inObject.id  );
				theSerializer->objectToFile( inObject, inCollection, theRepXWriter, mPropertyBuffer,inArgs );
			}
			mCollection.pushBack( RepXCollectionItem( inObject, theXmlWriter.getTopNode() ) );
			return RepXAddToCollectionResult( RepXAddToCollectionResult::Success, inObject.id );
		}

		virtual bool instantiateCollection( PxRepXInstantiationArgs& inArgs, PxCollection& inCollection )
		{
			for ( PxU32 idx =0; idx < mCollection.size(); ++idx )
			{
				RepXCollectionItem theItem( mCollection[idx] );
				PxRepXSerializer* theSerializer = mSerializationRegistry.getRepXSerializer( theItem.liveObject.typeName );
				if (theSerializer )				
				{
					XmlNodeReader theReader( theItem.descriptor, mAllocator.getAllocator(), mAllocator.mManager );
					XmlMemoryAllocatorImpl instantiationAllocator( mAllocator.getAllocator() );
					PxRepXObject theLiveObject = theSerializer->fileToObject( theReader, instantiationAllocator, inArgs, &inCollection );
					if (theLiveObject.isValid())
					{
						const PxBase* s =  reinterpret_cast<const PxBase*>( theLiveObject.serializable ) ;
                        inCollection.add( *const_cast<PxBase*>(s), PxSerialObjectId( theItem.liveObject.id ));
                    }
					else
						return false;
				}
				else
				{
					Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__, 
						"PxSerialization::createCollectionFromXml: "
						"PxRepXSerializer missing for type %s", theItem.liveObject.typeName);
					return false;					
				}				
			}

			return true;
		}

		void saveXmlNode( XmlNode* inNode, SimpleXmlWriter& inWriter )
		{
			XmlNode* theNode( inNode );
			if ( theNode->mData && *theNode->mData && theNode->mFirstChild == NULL )
				inWriter.writeContentTag( theNode->mName, theNode->mData );
			else
			{
				inWriter.beginTag( theNode->mName );
				if ( theNode->mData && *theNode->mData )
					inWriter.addContent( theNode->mData );
				for ( XmlNode* theChild = theNode->mFirstChild; 
						theChild != NULL;
						theChild = theChild->mNextSibling )
					saveXmlNode( theChild, inWriter );
				inWriter.endTag();
			}
		}

		virtual void save( PxOutputStream& inStream )
		{
			SimpleXmlWriterImpl<PxOutputStream> theWriter( inStream, mAllocator.getAllocator() );
			theWriter.beginTag( "PhysXCollection" );
			theWriter.addAttribute( "version", mVersionStr );
			{
				XmlWriterImpl theRepXWriter( &theWriter, &mPropertyBuffer );
				writeProperty( theWriter, mPropertyBuffer, "UpVector", mUpVector );
				theRepXWriter.addAndGotoChild( "Scale" );
				writeAllProperties( &mScale, theRepXWriter, mPropertyBuffer, *mPxCollection);
				theRepXWriter.leaveChild();
			}
			for ( PxU32 idx =0; idx < mCollection.size(); ++idx )
			{
				RepXCollectionItem theItem( mCollection[idx] );
				XmlNode* theNode( theItem.descriptor );
				saveXmlNode( theNode, theWriter );
			}
		}

		void load( PxInputData& inFileBuf, SerializationRegistry& s )
		{
			inFileBuf.seek(0);
			XmlParser theParser( XmlParseArgs( &mAllocator, &mCollection ), mAllocator );
			Ps::FastXml* theFastXml = Ps::createFastXml( &theParser );
			theFastXml->processXml( inFileBuf );
			XmlNode* theTopNode = theParser.getTopNode();
			if ( theTopNode != NULL )
			{
				{
					
					XmlMemoryAllocatorImpl instantiationAllocator( mAllocator.getAllocator() );
					XmlNodeReader theReader( theTopNode, mAllocator.getAllocator(), mAllocator.mManager );
					readProperty( theReader, "UpVector", mUpVector );
					if ( theReader.gotoChild( "Scale" ) )
					{
						readAllProperties( PxRepXInstantiationArgs( s.getPhysics() ), theReader, &mScale, instantiationAllocator, *mPxCollection);
						theReader.leaveChild();
					}
					const char* verStr = NULL;
					if ( theReader.read( "version", verStr ) )
						mVersionStr = verStr;
				}
				for ( XmlNode* theChild = theTopNode->mFirstChild; 
						theChild != NULL;
						theChild = theChild->mNextSibling )
				{
					if ( physx::shdfnd::stricmp( theChild->mName, "scale" ) == 0 
						|| physx::shdfnd::stricmp( theChild->mName, "version" ) == 0 
						|| physx::shdfnd::stricmp( theChild->mName, "upvector" ) == 0 )
						continue;
					XmlNodeReader theReader( theChild, mAllocator.getAllocator(), mAllocator.mManager );
					PxRepXObject theObject;
					theObject.typeName = theChild->mName;
					theObject.serializable = NULL;
					PxSerialObjectId theId = 0;
					theReader.read( "Id", theId );
					theObject.id = theId;
					mCollection.pushBack( RepXCollectionItem( theObject, theChild ) );
				}
			}
			else
			{
				Ps::getFoundation().error(PxErrorCode::eDEBUG_WARNING, __FILE__, __LINE__, 
				"Cannot parse any object from the input buffer, please check the input repx data.");
			}
			theFastXml->release();
		}
		
		virtual const char* getVersion() { return mVersionStr; }
		
		virtual const RepXCollectionItem* begin() const
		{
			return mCollection.begin();
		}
		virtual const RepXCollectionItem* end() const
		{
			return mCollection.end();
		}
		
		virtual RepXCollection& createCollection( const char* inVersionStr )
		{
			PxAllocatorCallback& allocator = mSharedData->mWrapper.getAllocator(); 
			RepXCollectionImpl* retval = PX_PLACEMENT_NEW((allocator.allocate(sizeof(RepXCollectionImpl), "createCollection",  __FILE__, __LINE__ )), RepXCollectionImpl) ( mSerializationRegistry, *this, inVersionStr );
		
			return *retval;
		}
		
		//Performs a deep copy of the repx node.
		virtual XmlNode* copyRepXNode( const XmlNode* srcNode ) 
		{
			return physx::Sn::copyRepXNode( &mAllocator.mManager, srcNode );
		}

		virtual void addCollectionItem( RepXCollectionItem inItem ) 
		{
			mCollection.pushBack( inItem );
		}
		
		virtual PxAllocatorCallback& getAllocator() { return mSharedData->mAllocator.getAllocator(); }
		//Create a new repx node with this name.  Its value is unset.
		virtual XmlNode& createRepXNode( const char* name )
		{
			XmlNode* newNode = allocateRepXNode( &mSharedData->mAllocator.mManager, name, NULL );
			return *newNode;
		}

		//Release this when finished.
		virtual XmlReaderWriter& createNodeEditor()
		{
			PxAllocatorCallback& allocator = mSharedData->mWrapper.getAllocator(); 
			XmlReaderWriter* retval = PX_PLACEMENT_NEW((allocator.allocate(sizeof(XmlNodeReader), "createNodeEditor",  __FILE__, __LINE__ )), XmlNodeReader) ( NULL, allocator, mAllocator.mManager );
			return *retval;
		}
	};
	
	const char* RepXCollection::getLatestVersion()
	{ 
#define TOSTR_(x)   #x
#define CONCAT_(a, b, c) TOSTR_(a.##b.##c)
#define MAKE_VERSION_STR(a,b,c)  CONCAT_(a, b, c)

		return MAKE_VERSION_STR(PX_PHYSICS_VERSION_MAJOR,PX_PHYSICS_VERSION_MINOR,PX_PHYSICS_VERSION_BUGFIX);

	}

	static RepXCollection* create(SerializationRegistry& s, PxAllocatorCallback& inAllocator, PxCollection& inCollection )
	{
		return PX_PLACEMENT_NEW((inAllocator.allocate(sizeof(RepXCollectionImpl), "RepXCollection::create",  __FILE__, __LINE__ )), RepXCollectionImpl) ( s, inAllocator, inCollection );
	}

	static RepXCollection* create(SerializationRegistry& s, PxInputData &data, PxAllocatorCallback& inAllocator, PxCollection& inCollection )
	{			
		RepXCollectionImpl* theCollection = static_cast<RepXCollectionImpl*>( create(s, inAllocator, inCollection ) );
		theCollection->load( data, s );
		return theCollection;
	}
}

	bool PxSerialization::serializeCollectionToXml( PxOutputStream& outputStream, PxCollection& collection, PxSerializationRegistry& sr, PxCooking* cooking, const PxCollection* externalRefs, PxXmlMiscParameter* inArgs )
	{
		if( !PxSerialization::isSerializable(collection, sr, const_cast<PxCollection*>(externalRefs)) )
			return false;
		
		bool bRet = true;

		SerializationRegistry& sn = static_cast<SerializationRegistry&>(sr);
		PxRepXInstantiationArgs args( sn.getPhysics(), cooking );

		PxCollection* tmpCollection = PxCreateCollection();
		PX_ASSERT(tmpCollection);

		tmpCollection->add( collection );
		if(externalRefs)
		{
			tmpCollection->add(*const_cast<PxCollection*>(externalRefs));
		}
		
		PxAllocatorCallback& allocator = PxGetFoundation().getAllocatorCallback(); 
		Sn::RepXCollection* theRepXCollection = Sn::create(sn, allocator, *tmpCollection );
				
		if(inArgs != NULL)
		{
			theRepXCollection->setTolerancesScale(inArgs->scale);
			theRepXCollection->setUpVector(inArgs->upVector);
		}		

		PxU32 nbObjects = collection.getNbObjects();
		if( nbObjects )
		{
			sortCollection( static_cast<Cm::Collection&>(collection), sn, true);

            for( PxU32 i = 0; i < nbObjects; i++ )
			{
                PxBase& s = collection.getObject(i);
				if( PxConcreteType::eSHAPE == s.getConcreteType() )
				{
					PxShape& shape = static_cast<PxShape&>(s);
					if( shape.isExclusive() )
						continue;
				}
				
				PxSerialObjectId id = collection.getId(s);
				if(id == PX_SERIAL_OBJECT_ID_INVALID)
					id = static_cast<PxSerialObjectId>( reinterpret_cast<size_t>( &s ));
				
				PxRepXObject ro = PxCreateRepXObject( &s, id );
				if ( ro.serializable == NULL || ro.id == 0 )
				{
					bRet = false;
					break;
				}
					
				theRepXCollection->addRepXObjectToCollection( ro, tmpCollection, args );				
			}
		}
		tmpCollection->release();

		theRepXCollection->save(outputStream);
		theRepXCollection->destroy();
		
		
		return bRet;
	}
	
	PxCollection* PxSerialization::createCollectionFromXml(PxInputData& inputData, PxCooking& cooking, PxSerializationRegistry& sr, const PxCollection* externalRefs, PxStringTable* stringTable, PxXmlMiscParameter* outArgs)
	{
		SerializationRegistry& sn = static_cast<SerializationRegistry&>(sr);
		PxCollection* collection = PxCreateCollection();
		PX_ASSERT(collection);
		
		if( externalRefs )
			collection->add(*const_cast<PxCollection*>(externalRefs));

		PxAllocatorCallback& allocator = PxGetFoundation().getAllocatorCallback(); 
		Sn::RepXCollection* theRepXCollection = Sn::create(sn, inputData, allocator, *collection);
		theRepXCollection = &Sn::RepXUpgrader::upgradeCollection( *theRepXCollection );
				
		PxRepXInstantiationArgs args( sn.getPhysics(), &cooking, stringTable );  
		if( !theRepXCollection->instantiateCollection(args, *collection) )
		{
			collection->release();
			theRepXCollection->destroy();
			return NULL;
		}
		
		if( externalRefs )
			collection->remove(*const_cast<PxCollection*>(externalRefs));
		
		if(outArgs != NULL)
		{
			outArgs->upVector = theRepXCollection->getUpVector();
			outArgs->scale = theRepXCollection->getTolerancesScale();
		}

		theRepXCollection->destroy();
		
		return collection;
	}
} 
