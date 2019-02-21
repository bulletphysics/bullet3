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
#ifndef PX_XML_VISITOR_WRITER_H
#define PX_XML_VISITOR_WRITER_H

#include "PsInlineArray.h"
#include "CmPhysXCommon.h"
#include "RepXMetaDataPropertyVisitor.h"
#include "SnPxStreamOperators.h"
#include "SnXmlMemoryPoolStreams.h"
#include "SnXmlWriter.h"
#include "SnXmlImpl.h"
#include "PsFoundation.h"
#include "foundation/PxStrideIterator.h"

namespace physx { namespace Sn {

	template<typename TDataType>
	inline void writeReference( XmlWriter& writer, PxCollection& inCollection, const char* inPropName, const TDataType* inDatatype )
	{
		const PxBase* s =  static_cast<const PxBase*>( inDatatype ) ;
		if( inDatatype && !inCollection.contains( *const_cast<PxBase*>(s) ))
		{
			Ps::getFoundation().error(PxErrorCode::eINTERNAL_ERROR, __FILE__, __LINE__,
				"PxSerialization::serializeCollectionToXml: Reference \"%s\" could not be resolved.", inPropName);
		}
		
		PxSerialObjectId theId = 0;
		if( s )
		{
			theId = inCollection.getId( *s );
			if( theId == 0 )
				theId = static_cast<uint64_t>(reinterpret_cast<size_t>(inDatatype));
		}	
		
		writer.write( inPropName, PxCreateRepXObject( inDatatype, theId ) );
	}

	inline void writeProperty( XmlWriter& inWriter, MemoryBuffer& inBuffer, const char* inProp )
	{
		PxU8 data = 0;
		inBuffer.write( &data, sizeof(PxU8) );
		inWriter.write( inProp, reinterpret_cast<const char*>( inBuffer.mBuffer ) );
		inBuffer.clear();
	}

	template<typename TDataType>
	inline void writeProperty( XmlWriter& inWriter, PxCollection&, MemoryBuffer& inBuffer, const char* inPropName, TDataType inValue )
	{
		inBuffer << inValue;
		writeProperty( inWriter, inBuffer, inPropName );
	}
	
	inline void writeProperty( XmlWriter& writer,  PxCollection& inCollection, MemoryBuffer& /*inBuffer*/, const char* inPropName, const PxConvexMesh* inDatatype )
	{
		writeReference( writer, inCollection, inPropName, inDatatype );
	}

	inline void writeProperty( XmlWriter& writer, PxCollection& inCollection, MemoryBuffer& /*inBuffer*/, const char* inPropName, PxConvexMesh* inDatatype )
	{
		writeReference( writer, inCollection, inPropName, inDatatype );
	}

	inline void writeProperty( XmlWriter& writer, PxCollection& inCollection, MemoryBuffer& /*inBuffer*/, const char* inPropName, const PxTriangleMesh* inDatatype )
	{
		if (inDatatype->getConcreteType() == PxConcreteType::eTRIANGLE_MESH_BVH33)
		{
			const PxBVH33TriangleMesh* dataType = inDatatype->is<PxBVH33TriangleMesh>();
			writeReference(writer, inCollection, inPropName, dataType);
		}
		else if (inDatatype->getConcreteType() == PxConcreteType::eTRIANGLE_MESH_BVH34)
		{
			const PxBVH34TriangleMesh* dataType = inDatatype->is<PxBVH34TriangleMesh>();
			writeReference(writer, inCollection, inPropName, dataType);
		}
		else
		{
			PX_ASSERT(0);
		}
	}
	
	inline void writeProperty( XmlWriter& writer, PxCollection& inCollection, MemoryBuffer& /*inBuffer*/, const char* inPropName, PxTriangleMesh* inDatatype )
	{
		if (inDatatype->getConcreteType() == PxConcreteType::eTRIANGLE_MESH_BVH33)
		{
			PxBVH33TriangleMesh* dataType = inDatatype->is<PxBVH33TriangleMesh>();
			writeReference(writer, inCollection, inPropName, dataType);
		}
		else if (inDatatype->getConcreteType() == PxConcreteType::eTRIANGLE_MESH_BVH34)
		{
			PxBVH34TriangleMesh* dataType = inDatatype->is<PxBVH34TriangleMesh>();
			writeReference(writer, inCollection, inPropName, dataType);
		}
		else
		{
			PX_ASSERT(0);
		}
	}

	inline void writeProperty( XmlWriter& writer, PxCollection& inCollection, MemoryBuffer& /*inBuffer*/, const char* inPropName, const PxBVH33TriangleMesh* inDatatype )
	{
		writeReference( writer, inCollection, inPropName, inDatatype );
	}
	
	inline void writeProperty( XmlWriter& writer, PxCollection& inCollection, MemoryBuffer& /*inBuffer*/, const char* inPropName, PxBVH33TriangleMesh* inDatatype )
	{
		writeReference( writer, inCollection, inPropName, inDatatype );
	}

	inline void writeProperty( XmlWriter& writer, PxCollection& inCollection, MemoryBuffer& /*inBuffer*/, const char* inPropName, const PxBVH34TriangleMesh* inDatatype )
	{
		writeReference( writer, inCollection, inPropName, inDatatype );
	}
	
	inline void writeProperty( XmlWriter& writer, PxCollection& inCollection, MemoryBuffer& /*inBuffer*/, const char* inPropName, PxBVH34TriangleMesh* inDatatype )
	{
		writeReference( writer, inCollection, inPropName, inDatatype );
	}

	inline void writeProperty( XmlWriter& writer, PxCollection& inCollection, MemoryBuffer& /*inBuffer*/, const char* inPropName, const PxHeightField* inDatatype )
	{
		writeReference( writer, inCollection, inPropName, inDatatype );
	}

	inline void writeProperty( XmlWriter& writer, PxCollection& inCollection, MemoryBuffer& /*inBuffer*/, const char* inPropName, PxHeightField* inDatatype )
	{
		writeReference( writer, inCollection, inPropName, inDatatype );
	}

	inline void writeProperty( XmlWriter& writer, PxCollection& inCollection, MemoryBuffer& /*inBuffer*/, const char* inPropName, const PxRigidActor* inDatatype )
	{
		writeReference( writer, inCollection, inPropName, inDatatype );
	}

	inline void writeProperty( XmlWriter& writer, PxCollection& inCollection, MemoryBuffer& /*inBuffer*/, const char* inPropName, PxArticulation* inDatatype )
	{
		writeReference( writer, inCollection, inPropName, inDatatype );
	}
	
	inline void writeProperty( XmlWriter& writer, PxCollection& inCollection, MemoryBuffer& /*inBuffer*/, const char* inPropName, PxRigidActor* inDatatype )
	{
		writeReference( writer, inCollection, inPropName, inDatatype );
	}

	inline void writeFlagsProperty( XmlWriter& inWriter, MemoryBuffer& tempBuf, const char* inPropName, PxU32 inFlags, const PxU32ToName* inTable )
	{
		if ( inTable )
		{
			PxU32 flagValue( inFlags );
			if ( flagValue )
			{
				for ( PxU32 idx =0; inTable[idx].mName != NULL; ++idx )
				{
					if ( (inTable[idx].mValue & flagValue) == inTable[idx].mValue )
					{
						if ( tempBuf.mWriteOffset != 0 )
							tempBuf << "|";
						tempBuf << inTable[idx].mName;
					}
				}
				writeProperty( inWriter, tempBuf, inPropName );
			}
		}
	}

	inline void writeFlagsBuffer( MemoryBuffer& tempBuf, PxU32 flagValue, const PxU32ToName* inTable )
	{
		PX_ASSERT(inTable);
		bool added = false;

		if ( flagValue )
		{					
			for ( PxU32 item =0; inTable[item].mName != NULL; ++item )
			{
				if ( (inTable[item].mValue & flagValue) != 0 )
				{
					if ( added )
						tempBuf << "|";
					tempBuf << inTable[item].mName;
					added = true;
				}
			}	
		}
	}

	inline void writePxVec3( PxOutputStream& inStream, const PxVec3& inVec ) { inStream << inVec; }
	

	template<typename TDataType>
	inline const TDataType& PtrAccess( const TDataType* inPtr, PxU32 inIndex )
	{
		return inPtr[inIndex];
	}
	
	template<typename TDataType>
	inline void BasicDatatypeWrite( PxOutputStream& inStream, const TDataType& item ) { inStream << item; }
	
	template<typename TObjType, typename TAccessOperator, typename TWriteOperator>
	inline void writeBuffer( XmlWriter& inWriter, MemoryBuffer& inTempBuffer
							, PxU32 inObjPerLine, const TObjType* inObjType, TAccessOperator inAccessOperator
							, PxU32 inBufSize, const char* inPropName, TWriteOperator inOperator )
	{
		if ( inBufSize && inObjType )
		{
			for ( PxU32 idx = 0; idx < inBufSize; ++idx )
			{
				if ( idx && ( idx % inObjPerLine == 0 ) )
					inTempBuffer << "\n\t\t\t";
				else
					inTempBuffer << " ";
				inOperator( inTempBuffer, inAccessOperator( inObjType, idx ) );
			}
			writeProperty( inWriter, inTempBuffer, inPropName );
		}
	}

	template<typename TDataType, typename TAccessOperator, typename TWriteOperator>
	inline void writeStrideBuffer( XmlWriter& inWriter, MemoryBuffer& inTempBuffer
							, PxU32 inObjPerLine, PxStrideIterator<const TDataType>& inData, TAccessOperator inAccessOperator
							, PxU32 inBufSize, const char* inPropName, PxU32 /*inStride*/, TWriteOperator inOperator )
	{
#if PX_SWITCH
		const auto *dat = &inData[0];
		if (inBufSize && dat != NULL)
#else
		if ( inBufSize && &inData[0])
#endif
		{
			for ( PxU32 idx = 0; idx < inBufSize; ++idx )
			{
				if ( idx && ( idx % inObjPerLine == 0 ) )
					inTempBuffer << "\n\t\t\t";
				else
					inTempBuffer << " ";
				
				inOperator( inTempBuffer, inAccessOperator( &inData[idx], 0  ) );
			}
			writeProperty( inWriter, inTempBuffer, inPropName );
		}
	}
	

	template<typename TDataType, typename TAccessOperator>
	inline void writeStrideFlags( XmlWriter& inWriter, MemoryBuffer& inTempBuffer
							, PxU32 inObjPerLine, PxStrideIterator<const TDataType>& inData, TAccessOperator /*inAccessOperator*/
							, PxU32 inBufSize, const char* inPropName, const PxU32ToName* inTable)
	{
#if PX_SWITCH
		const auto *dat = &inData[0];
		if (inBufSize && dat != NULL)
#else
		if ( inBufSize && &inData[0])
#endif
		{
			for ( PxU32 idx = 0; idx < inBufSize; ++idx )
			{	
				writeFlagsBuffer(inTempBuffer, inData[idx], inTable);

				if ( idx && ( idx % inObjPerLine == 0 ) )
					inTempBuffer << "\n\t\t\t";
				else
					inTempBuffer << " ";
			}
			writeProperty( inWriter, inTempBuffer, inPropName );
		}
	}

	template<typename TDataType, typename TWriteOperator>
	inline void writeBuffer( XmlWriter& inWriter, MemoryBuffer& inTempBuffer
							, PxU32 inObjPerLine, const TDataType* inBuffer
							, PxU32 inBufSize, const char* inPropName, TWriteOperator inOperator )
	{
		writeBuffer( inWriter, inTempBuffer, inObjPerLine, inBuffer, PtrAccess<TDataType>, inBufSize, inPropName, inOperator );
	}

	template<typename TEnumType>
	inline void writeEnumProperty( XmlWriter& inWriter, const char* inPropName, TEnumType inEnumValue, const PxU32ToName* inConversions )
	{
		PxU32 theValue = static_cast<PxU32>( inEnumValue );
		for ( const PxU32ToName* conv = inConversions; conv->mName != NULL; ++conv )
			if ( conv->mValue == theValue ) inWriter.write( inPropName, conv->mName );
	}
	
	
		
	template<typename TObjType, typename TWriterType, typename TInfoType>
	inline void handleComplexObj( TWriterType& oldVisitor, const TObjType* inObj, TInfoType& info);

	template<typename TCollectionType, typename TVisitor, typename TPropType, typename TInfoType >
	void handleComplexCollection( TVisitor& visitor, const TPropType& inProp, const char* childName, TInfoType& inInfo )
	{
		PxU32 count( inProp.size( visitor.mObj ) );
		if ( count )
		{
			Ps::InlineArray<TCollectionType*,5> theData;
			theData.resize( count );
			inProp.get( visitor.mObj, theData.begin(), count );
			for( PxU32 idx =0; idx < count; ++idx )
			{
				visitor.pushName( childName );
				handleComplexObj( visitor, theData[idx], inInfo );
				visitor.popName();
			}
		}
	}

	template<typename TCollectionType, typename TVisitor, typename TPropType, typename TInfoType >
	void handleBufferCollection( TVisitor& visitor, const TPropType& inProp, const char* childName, TInfoType& inInfo )
	{
		PxU32 count( inProp.size( visitor.mObj ) );
		if ( count )
		{
			Ps::InlineArray<TCollectionType*,5> theData;
			theData.resize( count );
			inProp.get( visitor.mObj, theData.begin());

			for( PxU32 idx =0; idx < count; ++idx )
			{
				visitor.pushName( childName );
				handleComplexObj( visitor, theData[idx], inInfo );
				visitor.popName();
			}
		}
	}

	template<typename TVisitor>
	void handleShapes( TVisitor& visitor, const PxRigidActorShapeCollection& inProp )
	{
		PxShapeGeneratedInfo theInfo;
		
		PxU32 count( inProp.size( visitor.mObj ) );
		if ( count )
		{
			Ps::InlineArray<PxShape*,5> theData;
			theData.resize( count );
			inProp.get( visitor.mObj, theData.begin(), count );
			for( PxU32 idx = 0; idx < count; ++idx )
			{
				const PxShape* shape = theData[idx];
				visitor.pushName( "PxShape" );
				
				if( !shape->isExclusive() )
				{
					writeReference( visitor.mWriter, visitor.mCollection, "PxShapeRef", shape );
				}
				else
				{
					handleComplexObj( visitor, shape, theInfo );
				}
				visitor.popName();
			}
		}
	}

	template<typename TVisitor>
	void handleShapeMaterials( TVisitor& visitor, const PxShapeMaterialsProperty& inProp )
	{
		PxU32 count( inProp.size( visitor.mObj ) );
		if ( count )
		{
			Ps::InlineArray<PxMaterial*,5> theData;
			theData.resize( count );
			inProp.get( visitor.mObj, theData.begin(), count );
			visitor.pushName( "PxMaterialRef" );
			
			for( PxU32 idx =0; idx < count; ++idx )
				writeReference( visitor.mWriter, visitor.mCollection, "PxMaterialRef", theData[idx] );
			visitor.popName();
		}
	}

	template<typename TObjType>
	struct RepXVisitorWriterBase
	{
		TNameStack&	mNameStack;
		XmlWriter&					mWriter;
		const TObjType*				mObj;
		MemoryBuffer&				mTempBuffer;
		PxCollection&				mCollection;

		RepXVisitorWriterBase( TNameStack& ns, XmlWriter& writer, const TObjType* obj, MemoryBuffer& buf, PxCollection& collection )
			: mNameStack( ns )
			, mWriter( writer )
			, mObj( obj )
			, mTempBuffer( buf )
			, mCollection( collection )
		{
		}

		RepXVisitorWriterBase( const RepXVisitorWriterBase<TObjType>& other )
			: mNameStack( other.mNameStack )
			, mWriter( other.mWriter )
			, mObj( other.mObj )
			, mTempBuffer( other.mTempBuffer )
			, mCollection( other.mCollection )
		{
		}

		RepXVisitorWriterBase& operator=( const RepXVisitorWriterBase& ){ PX_ASSERT( false ); return *this; }

		void gotoTopName()
		{
			if ( mNameStack.size() && mNameStack.back().mOpen == false ) 
			{
				mWriter.addAndGotoChild( mNameStack.back().mName );
				mNameStack.back().mOpen = true;
			}
		}

		void pushName( const char* inName ) 
		{ 
			gotoTopName();
			mNameStack.pushBack( inName ); 
		}

		void pushBracketedName( const char* inName ) { pushName( inName ); }
		void popName() 
		{ 
			if ( mNameStack.size() )
			{
				if ( mNameStack.back().mOpen )
					mWriter.leaveChild();
				mNameStack.popBack(); 
			}
		}

		const char* topName() const
		{
			if ( mNameStack.size() ) return mNameStack.back().mName;
			PX_ASSERT( false );
			return "bad__repx__name";
		}

		template<typename TAccessorType>
		void simpleProperty( PxU32 /*key*/, TAccessorType& inProp )
		{
			typedef typename TAccessorType::prop_type TPropertyType;
			TPropertyType propVal = inProp.get( mObj );
			writeProperty( mWriter, mCollection, mTempBuffer, topName(), propVal );
		}
		
		template<typename TAccessorType>
		void enumProperty( PxU32 /*key*/, TAccessorType& inProp, const PxU32ToName* inConversions )
		{
			writeEnumProperty( mWriter, topName(),  inProp.get( mObj ), inConversions );
		}

		template<typename TAccessorType>
		void flagsProperty( PxU32 /*key*/, const TAccessorType& inProp, const PxU32ToName* inConversions )
		{
			writeFlagsProperty( mWriter, mTempBuffer, topName(), inProp.get( mObj ), inConversions );
		}

		template<typename TAccessorType, typename TInfoType>
		void complexProperty( PxU32* /*key*/, const TAccessorType& inProp, TInfoType& inInfo )
		{
			typedef typename TAccessorType::prop_type TPropertyType;
			TPropertyType propVal = inProp.get( mObj );
			handleComplexObj( *this, &propVal, inInfo );
		}
		
		template<typename TAccessorType, typename TInfoType>
		void bufferCollectionProperty( PxU32* /*key*/, const TAccessorType& inProp, TInfoType& inInfo )
		{
			typedef typename TAccessorType::prop_type TPropertyType;
			
			PxU32 count( inProp.size( mObj ) );
			Ps::InlineArray<TPropertyType,5> theData;
			theData.resize( count );
	
			PxClassInfoTraits<TInfoType> theTraits;
			PX_UNUSED(theTraits);
			PxU32 numItems = inProp.get( mObj, theData.begin(), count );
			PX_ASSERT( numItems == count );
			for( PxU32 idx =0; idx < numItems; ++idx )
			{
				pushName( inProp.name() );
				handleComplexObj( *this, &theData[idx], inInfo );
				popName();
			}
		}
		
		template<typename TAccessorType, typename TInfoType>
		void extendedIndexedProperty( PxU32* /*key*/, const TAccessorType& inProp, TInfoType& /*inInfo */)
		{
			typedef typename TAccessorType::prop_type TPropertyType;
			
			PxU32 count( inProp.size( mObj ) );
			Ps::InlineArray<TPropertyType,5> theData;
			theData.resize( count );
	
			for(PxU32 i = 0; i < count; ++i)
			{
				char buffer[32] = { 0 };
				sprintf( buffer, "id_%u", i );
				pushName( buffer );
				
				TPropertyType propVal = inProp.get( mObj, i );
				TInfoType& infoType = PxClassInfoTraits<TPropertyType>().Info;
				handleComplexObj(*this, &propVal, infoType);
				popName();
			}		
		}
		
		template<typename TAccessorType, typename TInfoType>
		void PxFixedSizeLookupTableProperty( PxU32* /*key*/, TAccessorType& inProp, TInfoType& /*inInfo */)
		{
			typedef typename TAccessorType::prop_type TPropertyType;
			PxU32 count( inProp.size( mObj ) );
			
			PxU32 index = 0;
			for(PxU32 i = 0; i < count; ++i)
			{
				char buffer[32] = { 0 };
				sprintf( buffer, "id_%u", index++ );
				pushName( buffer );
				TPropertyType propVal = inProp.getX( mObj , i);
				writeProperty( mWriter, mCollection, mTempBuffer, topName(), propVal );
				popName();

				sprintf( buffer, "id_%u", index++ );
				pushName( buffer );
				propVal = inProp.getY( mObj , i);
				writeProperty( mWriter, mCollection, mTempBuffer, topName(), propVal );
				popName();
			}	
		}


		void handleShapes( const PxRigidActorShapeCollection& inProp )
		{
			physx::Sn::handleShapes( *this, inProp );
		}

		void handleShapeMaterials( const PxShapeMaterialsProperty& inProp )
		{
			physx::Sn::handleShapeMaterials( *this, inProp );
		}
	};

	template<typename TObjType>
	struct RepXVisitorWriter : RepXVisitorWriterBase<TObjType>
	{
		RepXVisitorWriter( TNameStack& ns, XmlWriter& writer, const TObjType* obj, MemoryBuffer& buf, PxCollection& collection )
			: RepXVisitorWriterBase<TObjType>( ns, writer, obj, buf, collection )
		{
		}

		RepXVisitorWriter( const RepXVisitorWriter<TObjType>& other )
			: RepXVisitorWriterBase<TObjType>( other )
		{
		}
	};

	template<>
	struct RepXVisitorWriter<PxArticulationLink> : RepXVisitorWriterBase<PxArticulationLink>
	{
		RepXVisitorWriter( TNameStack& ns, XmlWriter& writer, const PxArticulationLink* obj, MemoryBuffer& buf, PxCollection& collection )
			: RepXVisitorWriterBase<PxArticulationLink>( ns, writer, obj, buf, collection )
		{
		}

		RepXVisitorWriter( const RepXVisitorWriter<PxArticulationLink>& other )
			: RepXVisitorWriterBase<PxArticulationLink>( other )
		{
		}

		void handleIncomingJoint( const TIncomingJointPropType& prop )
		{
			const PxArticulationJointBase* theJoint( prop.get( mObj ) );
			if ( theJoint )
			{
				PxArticulationJointGeneratedInfo info;
				pushName( "Joint" );
				handleComplexObj( *this, theJoint, info );
				popName();
			}
		}
	};
	
	typedef PxProfileHashMap< const PxSerialObjectId, const PxArticulationLink* > TArticulationLinkLinkMap;
	
	template<>
	struct RepXVisitorWriter<PxArticulation> : RepXVisitorWriterBase<PxArticulation>
	{
		TArticulationLinkLinkMap& mArticulationLinkParents;

		RepXVisitorWriter( TNameStack& ns, XmlWriter& writer, const PxArticulation* inArticulation, MemoryBuffer& buf, PxCollection& collection, TArticulationLinkLinkMap* artMap = NULL )
			: RepXVisitorWriterBase<PxArticulation>( ns, writer, inArticulation, buf, collection )
			, mArticulationLinkParents( *artMap )
		{
			Ps::InlineArray<PxArticulationLink*, 64, PxProfileWrapperReflectionAllocator<PxArticulationLink*> > linkList( PxProfileWrapperReflectionAllocator<PxArticulationLink*>( buf.mManager->getWrapper() ) );
			PxU32 numLinks = inArticulation->getNbLinks();
			linkList.resize( numLinks );
			inArticulation->getLinks( linkList.begin(), numLinks );
			for ( PxU32 idx = 0; idx < numLinks; ++idx )
			{
				const PxArticulationLink* theLink( linkList[idx] );
				
				Ps::InlineArray<PxArticulationLink*, 64> theChildList;
				PxU32 numChildren = theLink->getNbChildren();
				theChildList.resize( numChildren );
				theLink->getChildren( theChildList.begin(), numChildren );
				for ( PxU32 childIdx = 0; childIdx < numChildren; ++childIdx )
					mArticulationLinkParents.insert(static_cast<uint64_t>(reinterpret_cast<size_t>(theChildList[childIdx])), theLink );
			}
		}

		RepXVisitorWriter( const RepXVisitorWriter<PxArticulation>& other )
			: RepXVisitorWriterBase<PxArticulation>( other )
			, mArticulationLinkParents( other.mArticulationLinkParents )
		{
		}
		template<typename TAccessorType, typename TInfoType>
		void complexProperty( PxU32* /*key*/, const TAccessorType& inProp, TInfoType& inInfo )
		{
			typedef typename TAccessorType::prop_type TPropertyType;
			TPropertyType propVal = inProp.get( mObj );
			handleComplexObj( *this, &propVal, inInfo );
		}

		void writeArticulationLink( const PxArticulationLink* inLink )
		{
			pushName( "PxArticulationLink" );
			gotoTopName();

			const TArticulationLinkLinkMap::Entry* theParentPtr = mArticulationLinkParents.find(static_cast<uint64_t>(reinterpret_cast<size_t>(inLink)) );
			if ( theParentPtr != NULL )
				writeProperty( mWriter, mCollection, mTempBuffer, "Parent",  theParentPtr->second );
			writeProperty( mWriter, mCollection, mTempBuffer, "Id", inLink  );

			PxArticulationLinkGeneratedInfo info;
			handleComplexObj( *this, inLink, info );
			popName();
		}

		void recurseAddLinkAndChildren( const PxArticulationLink* inLink, Ps::InlineArray<const PxArticulationLink*, 64>& ioLinks )
		{
			ioLinks.pushBack( inLink );
			Ps::InlineArray<PxArticulationLink*, 8> theChildren;
			PxU32 childCount( inLink->getNbChildren() );
			theChildren.resize( childCount );
			inLink->getChildren( theChildren.begin(), childCount );
			for ( PxU32 idx = 0; idx < childCount; ++idx )
				recurseAddLinkAndChildren( theChildren[idx], ioLinks );
		}

		void handleArticulationLinks( const PxArticulationLinkCollectionProp& inProp )
		{
			//topologically sort the links as per my discussion with Dilip because
			//links aren't guaranteed to have the parents before the children in the
			//overall link list and it is unlikely to be done by beta 1.
			PxU32 count( inProp.size( mObj ) );
			if ( count )
			{
				Ps::InlineArray<PxArticulationLink*, 64> theLinks;
				theLinks.resize( count );
				inProp.get( mObj, theLinks.begin(), count );
				
				Ps::InlineArray<const PxArticulationLink*, 64> theSortedLinks;
				for ( PxU32 idx = 0; idx < count; ++idx )
				{
					const PxArticulationLink* theLink( theLinks[idx] );
					if ( mArticulationLinkParents.find(static_cast<uint64_t>(reinterpret_cast<size_t>(theLink)) ) == NULL )
						recurseAddLinkAndChildren( theLink, theSortedLinks );
				}	
				PX_ASSERT( theSortedLinks.size() == count );
				for ( PxU32 idx = 0; idx < count; ++idx )
					writeArticulationLink( theSortedLinks[idx] );
				popName();
			}
		}
	private:
		RepXVisitorWriter<PxArticulation>& operator=(const RepXVisitorWriter<PxArticulation>&);
	};
	
	template<>
	struct RepXVisitorWriter<PxShape> : RepXVisitorWriterBase<PxShape>
	{
		RepXVisitorWriter( TNameStack& ns, XmlWriter& writer, const PxShape* obj, MemoryBuffer& buf, PxCollection& collection )
			: RepXVisitorWriterBase<PxShape>( ns, writer, obj, buf, collection )
		{
		}

		RepXVisitorWriter( const RepXVisitorWriter<PxShape>& other )
			: RepXVisitorWriterBase<PxShape>( other )
		{
		}

		template<typename GeometryType>
		inline void writeGeometryProperty( const PxShapeGeometryProperty& inProp, const char* inTypeName )
		{
			pushName( "Geometry" );
			pushName( inTypeName );
			GeometryType theType;
			inProp.getGeometry( mObj, theType );
			PxClassInfoTraits<GeometryType> theTraits;
			PxU32 count = theTraits.Info.totalPropertyCount();
			if(count)
			{				
				handleComplexObj( *this, &theType, theTraits.Info);
			}
			else
			{
				writeProperty(mWriter, mTempBuffer, inTypeName);
			}
			popName();
			popName();
		}

		void handleGeometryProperty( const PxShapeGeometryProperty& inProp )
		{
			switch( mObj->getGeometryType() )
			{
				case PxGeometryType::eSPHERE: writeGeometryProperty<PxSphereGeometry>( inProp, "PxSphereGeometry" ); break;
				case PxGeometryType::ePLANE: writeGeometryProperty<PxPlaneGeometry>( inProp, "PxPlaneGeometry" ); break;
				case PxGeometryType::eCAPSULE: writeGeometryProperty<PxCapsuleGeometry>( inProp, "PxCapsuleGeometry" ); break;
				case PxGeometryType::eBOX: writeGeometryProperty<PxBoxGeometry>( inProp, "PxBoxGeometry" ); break;
				case PxGeometryType::eCONVEXMESH: writeGeometryProperty<PxConvexMeshGeometry>( inProp, "PxConvexMeshGeometry" ); break;
				case PxGeometryType::eTRIANGLEMESH: writeGeometryProperty<PxTriangleMeshGeometry>( inProp, "PxTriangleMeshGeometry" ); break;
				case PxGeometryType::eHEIGHTFIELD: writeGeometryProperty<PxHeightFieldGeometry>( inProp, "PxHeightFieldGeometry" ); break;
				case PxGeometryType::eINVALID:
				case PxGeometryType::eGEOMETRY_COUNT:
					PX_ASSERT( false );
			}
		}
	};
	
	template<typename TObjType>
	inline void writeAllProperties( TNameStack& inNameStack, const TObjType* inObj, XmlWriter& writer, MemoryBuffer& buffer, PxCollection& collection )
	{
		RepXVisitorWriter<TObjType> newVisitor( inNameStack, writer, inObj, buffer, collection );
		RepXPropertyFilter<RepXVisitorWriter<TObjType> > theOp( newVisitor );
		PxClassInfoTraits<TObjType> info;
		info.Info.visitBaseProperties( theOp );
		info.Info.visitInstanceProperties( theOp );
	}
	
	template<typename TObjType>
	inline void writeAllProperties( TNameStack& inNameStack, TObjType* inObj, XmlWriter& writer, MemoryBuffer& buffer, PxCollection& collection )
	{
		RepXVisitorWriter<TObjType> newVisitor( inNameStack, writer, inObj, buffer, collection );
		RepXPropertyFilter<RepXVisitorWriter<TObjType> > theOp( newVisitor );
		PxClassInfoTraits<TObjType> info;
		info.Info.visitBaseProperties( theOp );
		info.Info.visitInstanceProperties( theOp );
	}
	
	template<typename TObjType>
	inline void writeAllProperties( const TObjType* inObj, XmlWriter& writer, MemoryBuffer& buffer, PxCollection& collection )
	{
		TNameStack theNames( buffer.mManager->getWrapper() );
		writeAllProperties( theNames, inObj, writer, buffer, collection );
	}
	
	template<typename TObjType, typename TWriterType, typename TInfoType>
	inline void handleComplexObj( TWriterType& oldVisitor, const TObjType* inObj, const TInfoType& /*info*/)
	{
		writeAllProperties( oldVisitor.mNameStack, inObj, oldVisitor.mWriter, oldVisitor.mTempBuffer, oldVisitor.mCollection );
	}

	template<typename TObjType, typename TWriterType, typename TInfoType>
	inline void handleComplexObj( TWriterType& oldVisitor, const TObjType* inObj, TInfoType& /*info*/)
	{
		writeAllProperties( oldVisitor.mNameStack, inObj, oldVisitor.mWriter, oldVisitor.mTempBuffer, oldVisitor.mCollection );
	}

	template<typename TObjType, typename TWriterType>
	inline void handleComplexObj( TWriterType& oldVisitor, const TObjType* inObj, const PxUnknownClassInfo& /*info*/)
	{
		writeProperty( oldVisitor.mWriter, oldVisitor.mCollection, oldVisitor.mTempBuffer, oldVisitor.topName(), *inObj );
	}
	
} }
#endif
