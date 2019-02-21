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
#ifndef PX_XML_VISITOR_READER_H
#define PX_XML_VISITOR_READER_H

#include "PsArray.h"
#include "PsUtilities.h"
#include "RepXMetaDataPropertyVisitor.h"
#include "SnPxStreamOperators.h"
#include "SnXmlMemoryPoolStreams.h"
#include "SnXmlReader.h"
#include "SnXmlImpl.h"
#include "SnXmlMemoryAllocator.h"
#include "SnXmlStringToType.h"

namespace physx { namespace Sn {
	
	
	inline PxU32 findEnumByName( const char* inName, const PxU32ToName* inTable )
	{
		for ( PxU32 idx = 0; inTable[idx].mName != NULL; ++idx )
		{
			if ( physx::shdfnd::stricmp( inTable[idx].mName, inName ) == 0 )
				return inTable[idx].mValue;
		}
		return 0;
	}

	PX_INLINE void stringToFlagsType( const char* strData, XmlMemoryAllocator& alloc, PxU32& ioType, const PxU32ToName* inTable )
	{
		if ( inTable == NULL )
			return;
		ioType = 0;
		if ( strData && *strData)
		{
			//Destructively parse the string to get out the different flags.
			char* theValue = const_cast<char*>( copyStr( &alloc, strData ) );
			char* theMarker = theValue;
			char* theNext = theValue;
			while( theNext && *theNext )
			{
				++theNext;
				if( *theNext == '|' )
				{
					*theNext = 0;
					++theNext;
					ioType |= static_cast< PxU32 > ( findEnumByName( theMarker, inTable ) );
					theMarker = theNext;
				}
			}
			if ( theMarker && *theMarker )
				ioType |= static_cast< PxU32 > ( findEnumByName( theMarker, inTable ) );
			alloc.deallocate( reinterpret_cast<PxU8*>( theValue ) );
		}
	}

	template<typename TDataType>
	PX_INLINE void stringToEnumType( const char* strData, TDataType& ioType, const PxU32ToName* inTable )
	{
		ioType = static_cast<TDataType>( findEnumByName( strData, inTable ) );
	}

	template<typename TDataType>
	PX_INLINE bool readProperty( XmlReader& inReader, const char* pname, TDataType& ioType )
	{
		const char* value;
		if ( inReader.read( pname, value ) )
		{
			stringToType( value, ioType );
			return true;
		}
		return false;
	}
	
	template<typename TObjType>
	inline TObjType* findReferencedObject( PxCollection& collection, PxSerialObjectId id)
	{
		PX_ASSERT(id > 0);
		TObjType* outObject = static_cast<TObjType*>(const_cast<PxBase*>(collection.find(id)));
		if (outObject == NULL)
		{
			Ps::getFoundation().error(PxErrorCode::eINVALID_PARAMETER, __FILE__, __LINE__, 
				"PxSerialization::createCollectionFromXml: "
				"Reference to ID %d cannot be resolved. Make sure externalRefs collection is specified if required and "
				"check Xml file for completeness.",
				id);
		}
		return outObject;
	}

	template<typename TObjType>
	inline bool readReference( XmlReader& inReader, PxCollection& collection, TObjType*& outObject )
	{
		PxSerialObjectId theId;
		const char* theValue = inReader.getCurrentItemValue();
		strto( theId, theValue );
		if( theId == 0)
		{
			// the NULL pointer is a valid pointer if the input id is 0
			outObject = NULL;
			return true;
		}
		else
		{
			outObject = findReferencedObject<TObjType>(collection, theId);
			return outObject != NULL;
		}
	}
	
	template<typename TObjType>
	inline bool readReference( XmlReader& inReader, PxCollection& inCollection, const char* pname, TObjType*& outObject )
	{
		outObject = NULL;
		PxSerialObjectId theId = 0;
		if (readProperty ( inReader, pname, theId ) && theId )
		{
			outObject = findReferencedObject<TObjType>(inCollection, theId);
		}
		// the NULL pointer is a valid pointer if the input id is 0
		return (outObject != NULL) || 0 == theId;
	}

	template<typename TEnumType, typename TStorageType>
	inline bool readFlagsProperty( XmlReader& reader, XmlMemoryAllocator& allocator, const char* pname, const PxU32ToName* inConversions, PxFlags<TEnumType,TStorageType>& outFlags )
	{
		const char* value;
		if ( reader.read( pname, value ) )
		{
			PxU32 tempValue = 0;
			stringToFlagsType( value, allocator, tempValue, inConversions );
			outFlags = PxFlags<TEnumType,TStorageType>(Ps::to16(tempValue) );
			return true;
		}
		return false;
	}

	template<typename TObjType, typename TReaderType, typename TInfoType>
	inline void readComplexObj( TReaderType& oldVisitor, TObjType* inObj, TInfoType& info);
	template<typename TObjType, typename TReaderType>
	inline void readComplexObj( TReaderType& oldVisitor, TObjType* inObj);
	
	template<typename TReaderType, typename TGeomType>
	inline PxGeometry* parseGeometry( TReaderType& reader, TGeomType& /*inGeom*/)
	{	
		PxAllocatorCallback& inAllocator = reader.mAllocator.getAllocator();
		
		TGeomType* shape = PX_PLACEMENT_NEW((inAllocator.allocate(sizeof(TGeomType), "parseGeometry",  __FILE__, __LINE__ )), TGeomType);
		PxClassInfoTraits<TGeomType> info;
		readComplexObj( reader, shape);
		return shape;
	}
	

	template<typename TReaderType>
	inline void parseShape( TReaderType& visitor, PxGeometry*& outResult, Ps::Array<PxMaterial*>& outMaterials)
	{
		XmlReader& theReader( visitor.mReader );
		PxCollection& collection = visitor.mCollection;
		visitor.pushCurrentContext();
		if ( visitor.gotoTopName() )
		{
			visitor.pushCurrentContext();
			if ( visitor.gotoChild( "Materials" ) )
			{
				for( bool matSuccess = visitor.gotoFirstChild(); matSuccess;
					matSuccess = visitor.gotoNextSibling() )
				{
					PxMaterial* material = NULL;					
					if(!readReference<PxMaterial>( theReader, collection, material ))
						visitor.mHadError = true;
					if ( material ) 
						outMaterials.pushBack( material );
				}
			}
			visitor.popCurrentContext();
			visitor.pushCurrentContext();

			PxPlaneGeometry			plane;
			PxHeightFieldGeometry	heightField;
			PxSphereGeometry		sphere;
			PxTriangleMeshGeometry	mesh;
			PxConvexMeshGeometry	convex;
			PxBoxGeometry			box;
			PxCapsuleGeometry		capsule;
			if ( visitor.gotoChild( "Geometry" ) )
			{
				if ( visitor.gotoFirstChild() )
				{
					const char* geomTypeName = visitor.getCurrentItemName();

					if ( physx::shdfnd::stricmp( geomTypeName, "PxSphereGeometry" ) == 0 ) outResult = parseGeometry(visitor, sphere);
					else if ( physx::shdfnd::stricmp( geomTypeName, "PxPlaneGeometry" ) == 0 ) outResult = parseGeometry(visitor, plane);
					else if ( physx::shdfnd::stricmp( geomTypeName, "PxCapsuleGeometry" ) == 0 ) outResult = parseGeometry(visitor, capsule);
					else if ( physx::shdfnd::stricmp( geomTypeName, "PxBoxGeometry" ) == 0 ) outResult = parseGeometry(visitor, box);
					else if ( physx::shdfnd::stricmp( geomTypeName, "PxConvexMeshGeometry" ) == 0 ) outResult = parseGeometry(visitor, convex);
					else if ( physx::shdfnd::stricmp( geomTypeName, "PxTriangleMeshGeometry" ) == 0 ) outResult = parseGeometry(visitor, mesh);
					else if ( physx::shdfnd::stricmp( geomTypeName, "PxHeightFieldGeometry" ) == 0 ) outResult = parseGeometry(visitor, heightField);
					else
						PX_ASSERT( false );
				}
			}
			visitor.popCurrentContext();
		}
		visitor.popCurrentContext();

		return;
	}

	template<typename TReaderType, typename TObjType>
	inline void readShapesProperty( TReaderType& visitor, TObjType* inObj, const PxRigidActorShapeCollection* inProp = NULL, bool isSharedShape = false )
	{
		PX_UNUSED(isSharedShape);
		PX_UNUSED(inProp);

		XmlReader& theReader( visitor.mReader );
		PxCollection& collection( visitor.mCollection );

		visitor.pushCurrentContext();
		if ( visitor.gotoTopName() )
		{
			//uggh working around the shape collection api.
			//read out materials and geometry
			for ( bool success = visitor.gotoFirstChild(); success; 
					success = visitor.gotoNextSibling() )
			{
				if( 0 == physx::shdfnd::stricmp( visitor.getCurrentItemName(), "PxShapeRef" ) )
				{
					PxShape* shape = NULL;
					if(!readReference<PxShape>( theReader, collection, shape ))
						visitor.mHadError = true;
					if(shape)
						inObj->attachShape( *shape );
				}
				else
				{
					Ps::Array<PxMaterial*> materials;
					PxGeometry* geometry = NULL;
					parseShape( visitor, geometry, materials);
					PxShape* theShape = NULL;
					if ( materials.size() )
					{
						theShape = visitor.mArgs.physics.createShape( *geometry, materials.begin(), Ps::to16(materials.size()), true );
						if ( theShape )
						{
							readComplexObj( visitor, theShape );
		          			
					        if(theShape) 
							{
								inObj->attachShape(*theShape);						
								collection.add( *theShape );
							}
						}
					}

					switch(geometry->getType())
					{
					case PxGeometryType::eSPHERE :
						static_cast<PxSphereGeometry*>(geometry)->~PxSphereGeometry();
						break;
					case PxGeometryType::ePLANE :
						static_cast<PxPlaneGeometry*>(geometry)->~PxPlaneGeometry();
						break;
					case PxGeometryType::eCAPSULE :
						static_cast<PxCapsuleGeometry*>(geometry)->~PxCapsuleGeometry();
						break;
					case PxGeometryType::eBOX :
						static_cast<PxBoxGeometry*>(geometry)->~PxBoxGeometry();
						break;
					case PxGeometryType::eCONVEXMESH :
						static_cast<PxConvexMeshGeometry*>(geometry)->~PxConvexMeshGeometry();
						break;
					case PxGeometryType::eTRIANGLEMESH :
						static_cast<PxTriangleMeshGeometry*>(geometry)->~PxTriangleMeshGeometry();
						break;
					case PxGeometryType::eHEIGHTFIELD :
						static_cast<PxHeightFieldGeometry*>(geometry)->~PxHeightFieldGeometry();
						break;

					case PxGeometryType::eGEOMETRY_COUNT:
					case PxGeometryType::eINVALID:
						PX_ASSERT(0);			
					}		
					visitor.mAllocator.getAllocator().deallocate(geometry);
				}				
			}
		}
		visitor.popCurrentContext();
	}

	struct ReaderNameStackEntry : NameStackEntry
	{
		bool		mValid;
		ReaderNameStackEntry( const char* nm, bool valid ) : NameStackEntry(nm), mValid(valid) {}
	};

	typedef PxProfileArray<ReaderNameStackEntry> TReaderNameStack;

	template<typename TObjType>
	struct RepXVisitorReaderBase
	{

	protected:
		RepXVisitorReaderBase<TObjType>& operator=(const RepXVisitorReaderBase<TObjType>&);
	public:
		TReaderNameStack&		mNames;
		PxProfileArray<PxU32>&	mContexts;
		PxRepXInstantiationArgs	mArgs;
		XmlReader&				mReader;
		TObjType*				mObj;
		XmlMemoryAllocator&		mAllocator;
		PxCollection&			mCollection;		
		bool					mValid;	
		bool&                   mHadError;

		RepXVisitorReaderBase( TReaderNameStack& names, PxProfileArray<PxU32>& contexts, const PxRepXInstantiationArgs& args, XmlReader& reader, TObjType* obj
								, XmlMemoryAllocator&	alloc, PxCollection& collection, bool& hadError )
			: mNames( names )
			, mContexts( contexts )
			, mArgs( args )
			, mReader( reader )
			, mObj( obj )
			, mAllocator( alloc )
			, mCollection( collection )
			, mValid( true )
			, mHadError(hadError)
		{
		}
		RepXVisitorReaderBase( const RepXVisitorReaderBase& other )
			: mNames( other.mNames )
			, mContexts( other.mContexts )
			, mArgs( other.mArgs )
			, mReader( other.mReader )
			, mObj( other.mObj )
			, mAllocator( other.mAllocator )
			, mCollection( other.mCollection )
			, mValid( other.mValid )
			, mHadError( other.mHadError )
		{
		}

		
		void pushName( const char* name )
		{
			gotoTopName();
			mNames.pushBack( ReaderNameStackEntry( name, mValid ) );
		}
		void pushBracketedName( const char* name ) { pushName( name ); }
		void popName()
		{
			if ( mNames.size() )
			{
				if ( mNames.back().mOpen && mNames.back().mValid )
					mReader.leaveChild();
				mNames.popBack();
			}
			mValid =true;
			if ( mNames.size() && mNames.back().mValid == false )
				mValid = false;
		}

		void pushCurrentContext()
		{
			mContexts.pushBack( static_cast<PxU32>( mNames.size() ) );
		}
		void popCurrentContext()
		{
			if ( mContexts.size() )
			{
				PxU32 depth = mContexts.back();
				PX_ASSERT( mNames.size() >= depth );
				while( mNames.size() > depth )
					popName();
				mContexts.popBack();
			}
		}

		bool updateLastEntryAfterOpen()
		{
			mNames.back().mValid = mValid;
			mNames.back().mOpen = mValid;
			return mValid;
		}

		bool gotoTopName()
		{
			if ( mNames.size() && mNames.back().mOpen == false )
			{
				if ( mValid )
					mValid = mReader.gotoChild( mNames.back().mName );
				updateLastEntryAfterOpen();
			}
			return mValid;
		}

		bool isValid() const { return mValid; }

		bool gotoChild( const char* name )
		{
			pushName( name );
			return gotoTopName();
		}

		bool gotoFirstChild()
		{
			pushName( "__child" );
			if ( mValid ) mValid = mReader.gotoFirstChild();
			return updateLastEntryAfterOpen();
		}

		bool gotoNextSibling()
		{
			bool retval = mValid;
			if ( mValid ) retval = mReader.gotoNextSibling();
			return retval;
		}

		const char* getCurrentItemName() { if (mValid ) return mReader.getCurrentItemName(); return ""; }
		
		const char* topName() const
		{
			if ( mNames.size() ) return mNames.back().mName;
			PX_ASSERT( false );
			return "bad__repx__name";
		}

		const char* getCurrentValue()
		{
			const char* value = NULL;
			if ( isValid() && mReader.read( topName(), value ) )
				return value;
			return NULL;
		}

		template<typename TDataType>
		bool readProperty(TDataType& outType)
		{
			const char* value = getCurrentValue();
			if ( value && *value )
			{
				stringToType( value, outType );
				return true;
			}
			return false;
		}

		template<typename TDataType>
		bool readExtendedIndexProperty(TDataType& outType)
		{
			const char* value = mReader.getCurrentItemValue();
			if ( value && *value )
			{
				stringToType( value, outType );
				return true;
			}
			return false;
		}


		template<typename TRefType>
		bool readReference(TRefType*& outRef)
		{
			return physx::Sn::readReference<TRefType>( mReader, mCollection, topName(), outRef );
		}
		
		inline bool readProperty(const char*& outProp )
		{
			outProp = "";
			const char* value = getCurrentValue();
			if ( value && *value && mArgs.stringTable )
			{
				outProp = mArgs.stringTable->allocateStr( value );
				return true;
			}
			return false;
		}

		inline bool readProperty(PxConvexMesh*& outProp )
		{
			return readReference<PxConvexMesh>( outProp );
		}
		
		inline bool readProperty(PxTriangleMesh*& outProp )
		{
			return readReference<PxTriangleMesh>( outProp );
		}
		inline bool readProperty(PxBVH33TriangleMesh*& outProp )
		{
			return readReference<PxBVH33TriangleMesh>( outProp );
		}
		inline bool readProperty(PxBVH34TriangleMesh*& outProp )
		{
			return readReference<PxBVH34TriangleMesh>( outProp );
		}
		
		inline bool readProperty(PxHeightField*& outProp )
		{
			return readReference<PxHeightField>( outProp );
		}

		inline bool readProperty( PxRigidActor *& outProp )
		{
			return readReference<PxRigidActor>( outProp );
		}

		template<typename TAccessorType>
		void simpleProperty( PxU32 /*key*/, TAccessorType& inProp )
		{
			typedef typename TAccessorType::prop_type TPropertyType;
			TPropertyType value;
			if ( readProperty( value ) )
				inProp.set( mObj, value );
		}
		
		template<typename TAccessorType>
		void enumProperty( PxU32 /*key*/, TAccessorType& inProp, const PxU32ToName* inConversions )
		{
			typedef typename TAccessorType::prop_type TPropertyType;
			const char* strVal = getCurrentValue();
			if ( strVal && *strVal )
			{
				TPropertyType pval;
				stringToEnumType( strVal, pval, inConversions );
				inProp.set( mObj, pval );
			}
		}

		template<typename TAccessorType>
		void flagsProperty( PxU32 /*key*/, const TAccessorType& inProp, const PxU32ToName* inConversions )
		{
			typedef typename TAccessorType::prop_type TPropertyType;
			typedef typename TPropertyType::InternalType TInternalType;

			const char* strVal = getCurrentValue();
			if ( strVal && *strVal )
			{
				PxU32 tempValue = 0;
				stringToFlagsType( strVal, mAllocator, tempValue, inConversions );
				inProp.set( mObj, TPropertyType(TInternalType( tempValue )));
			}
		}
	
		template<typename TAccessorType, typename TInfoType>
		void complexProperty( PxU32* /*key*/, const TAccessorType& inProp, TInfoType& inInfo )
		{
			typedef typename TAccessorType::prop_type TPropertyType;
			if ( gotoTopName() )
			{
				TPropertyType propVal = inProp.get( mObj );
				readComplexObj( *this, &propVal, inInfo );
				inProp.set( mObj, propVal );
			}
		}
		
		template<typename TAccessorType, typename TInfoType>
		void bufferCollectionProperty( PxU32* /*key*/, const TAccessorType& inProp, TInfoType& inInfo )
		{
			typedef typename TAccessorType::prop_type TPropertyType;
			Ps::InlineArray<TPropertyType,5> theData;
	
			this->pushCurrentContext();
			if (  this->gotoTopName() )
			{
				for ( bool success =  this->gotoFirstChild(); success; 
						success =  this->gotoNextSibling() )
				{
					TPropertyType propVal;
					readComplexObj( *this, &propVal, inInfo );
					theData.pushBack(propVal);
				}	
			}
			this->popCurrentContext();

			inProp.set( mObj, theData.begin(), theData.size() );
			
		}
		
		template<typename TAccessorType, typename TInfoType>
		void extendedIndexedProperty( PxU32* /*key*/, const TAccessorType& inProp, TInfoType& inInfo )
		{
			typedef typename TAccessorType::prop_type TPropertyType;
		
			this->pushCurrentContext();
			if (  this->gotoTopName() )
			{
				PxU32 index = 0;
				for ( bool success =  this->gotoFirstChild(); success; 
						success =  this->gotoNextSibling() )
				{
					TPropertyType propVal;
					readComplexObj( *this, &propVal, inInfo );
					inProp.set(mObj, index, propVal);
					++index;
				}	
			}
			this->popCurrentContext();
		}
		
		template<typename TAccessorType, typename TInfoType>
		void PxFixedSizeLookupTableProperty( PxU32* /*key*/, const TAccessorType& inProp, TInfoType& inInfo )
		{
			typedef typename TAccessorType::prop_type TPropertyType;
			const_cast<TAccessorType&>(inProp).clear( mObj );

			this->pushCurrentContext();
			if (  this->gotoTopName() )
			{
				for ( bool success =  this->gotoFirstChild(); success; 
						success =  this->gotoNextSibling() )
				{
					TPropertyType propXVal;
					readComplexObj( *this, &propXVal, inInfo );
					
					if(this->gotoNextSibling())
					{
						TPropertyType propYVal;
						readComplexObj( *this, &propYVal, inInfo );
						const_cast<TAccessorType&>(inProp).addPair(mObj, propXVal, propYVal);
					}					
				}	
			}
			this->popCurrentContext();
		}

		void handleShapes( const PxRigidActorShapeCollection& inProp )
		{
			physx::Sn::readShapesProperty( *this, mObj, &inProp );		
		}
	};
	
	template<typename TObjType>
	struct RepXVisitorReader : public RepXVisitorReaderBase<TObjType>
	{
		RepXVisitorReader( TReaderNameStack& names, PxProfileArray<PxU32>& contexts, const PxRepXInstantiationArgs& args, XmlReader& reader, TObjType* obj
								, XmlMemoryAllocator&	alloc, PxCollection& collection, bool& ret)
			: RepXVisitorReaderBase<TObjType>( names, contexts, args, reader, obj, alloc, collection, ret)
		{
		}
		RepXVisitorReader( const RepXVisitorReader<TObjType>& other )
			: RepXVisitorReaderBase<TObjType>( other )
		{
		}
	};

	// Specialized template to load dynamic rigid, to determine the kinematic state first
	template<>
	struct RepXVisitorReader<PxRigidDynamic> : public RepXVisitorReaderBase<PxRigidDynamic>
	{
		RepXVisitorReader( TReaderNameStack& names, PxProfileArray<PxU32>& contexts, const PxRepXInstantiationArgs& args, XmlReader& reader, PxRigidDynamic* obj
								, XmlMemoryAllocator&	alloc, PxCollection& collection, bool& ret)
			: RepXVisitorReaderBase<PxRigidDynamic>( names, contexts, args, reader, obj, alloc, collection, ret)
		{
		}
		RepXVisitorReader( const RepXVisitorReader<PxRigidDynamic>& other )
			: RepXVisitorReaderBase<PxRigidDynamic>( other )
		{
		}

		void handleShapes( const PxRigidActorShapeCollection& inProp )
		{
			// Need to read the parental actor to check if actor is kinematic
			// in that case we need to apply the kinematic flag before a shape is set
			XmlReaderWriter* parentReader = static_cast<XmlReaderWriter*>(mReader.getParentReader());
			if(mObj)
			{
				const char* value;
				if (parentReader->read( "RigidBodyFlags", value ))
				{
					if(strstr(value, "eKINEMATIC"))
					{						
						mObj->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, true);
					}
				}
			}
			physx::Sn::readShapesProperty( *this, mObj, &inProp );		
			parentReader->release();
		}


		template<typename TAccessorType>
		void simpleProperty( PxU32 /*key*/, TAccessorType& inProp )
		{
			typedef typename TAccessorType::prop_type TPropertyType;
			TPropertyType value;
			if (readProperty(value))
			{
				// If the rigid body is kinematic, we cannot set the LinearVelocity or AngularVelocity
				const bool kinematic = (mObj->getRigidBodyFlags() & PxRigidBodyFlag::eKINEMATIC);
				if(kinematic && (inProp.mProperty.mKey == PxPropertyInfoName::PxRigidBody_LinearVelocity || inProp.mProperty.mKey == PxPropertyInfoName::PxRigidBody_AngularVelocity))
					return;

				inProp.set(mObj, value );
			}
		}
	private:		
		RepXVisitorReader<PxRigidDynamic>& operator=(const  RepXVisitorReader<PxRigidDynamic>&);			
	};
	
	template<>
	struct RepXVisitorReader<PxShape> : public RepXVisitorReaderBase<PxShape>
	{
		RepXVisitorReader( TReaderNameStack& names, PxProfileArray<PxU32>& contexts, const PxRepXInstantiationArgs& args, XmlReader& reader, PxShape* obj
								, XmlMemoryAllocator&	alloc, PxCollection& collection, bool& ret )
			: RepXVisitorReaderBase<PxShape>( names, contexts, args, reader, obj, alloc, collection, ret )
		{
		}
		RepXVisitorReader( const RepXVisitorReader<PxShape>& other )
			: RepXVisitorReaderBase<PxShape>( other )
		{
		}
		void handleShapeMaterials( const PxShapeMaterialsProperty& ) //these were handled during construction.
		{
		}
		void handleGeometryProperty( const PxShapeGeometryProperty& )
		{
		}
	private:
		 RepXVisitorReader<PxShape>& operator=(const  RepXVisitorReader<PxShape>&);
	};

	template<>
	struct RepXVisitorReader<PxArticulationLink> : public RepXVisitorReaderBase<PxArticulationLink>
	{
		RepXVisitorReader( TReaderNameStack& names, PxProfileArray<PxU32>& contexts, const PxRepXInstantiationArgs& args, XmlReader& reader, PxArticulationLink* obj
								, XmlMemoryAllocator&	alloc, PxCollection& collection, bool& ret )
			: RepXVisitorReaderBase<PxArticulationLink>( names, contexts, args, reader, obj, alloc, collection, ret )
		{
		}
		RepXVisitorReader( const RepXVisitorReader<PxArticulationLink>& other )
			: RepXVisitorReaderBase<PxArticulationLink>( other )
		{
		}
		void handleIncomingJoint( const TIncomingJointPropType& prop )
		{
			pushName( "Joint" );
			if ( gotoTopName() )
			{
				PxArticulationBase::Enum type = mObj->getArticulation().getType();
				if (type == PxArticulationBase::eMaximumCoordinate)
				{
					PxArticulationJoint* theJoint = static_cast<PxArticulationJoint*>((prop.get(mObj)));

					readComplexObj(*this, theJoint);

					//Add joint to PxCollection, since PxArticulation requires PxArticulationLink and joint.
					mCollection.add(*theJoint);

				}
				else
				{
					PxArticulationJointReducedCoordinate* theJoint = static_cast<PxArticulationJointReducedCoordinate*>((prop.get(mObj)));

					readComplexObj(*this, theJoint);

					//Add joint to PxCollection, since PxArticulation requires PxArticulationLink and joint.
					mCollection.add(*theJoint);
				}


				
			}
			popName();
		}
	private:
		RepXVisitorReader<PxArticulationLink>& operator=(const RepXVisitorReader<PxArticulationLink>&);
	};
	
	inline void readProperty( RepXVisitorReaderBase<PxArticulation>& inSerializer, PxArticulation* inObj, const PxArticulationLinkCollectionProp& /*inProp*/)
	{
		PxProfileAllocatorWrapper theWrapper( inSerializer.mAllocator.getAllocator() );
		PxCollection& collection( inSerializer.mCollection );

		TArticulationLinkLinkMap linkRemapMap( theWrapper );
		inSerializer.pushCurrentContext();
		if( inSerializer.gotoTopName() )
		{
			for ( bool links = inSerializer.gotoFirstChild();
				links != false;
				links = inSerializer.gotoNextSibling() )
			{
				//Need enough information to create the link...
				PxSerialObjectId theParentPtr = 0;
				const PxArticulationLink* theParentLink = NULL;
				if ( inSerializer.mReader.read( "Parent", theParentPtr ) )
				{
					const TArticulationLinkLinkMap::Entry* theRemappedParent( linkRemapMap.find( theParentPtr ) );
					//If we have a valid at write time, we had better have a valid parent at read time.
					PX_ASSERT( theRemappedParent );
					theParentLink = theRemappedParent->second;
				}
				PxArticulationLink* newLink = inObj->createLink( const_cast<PxArticulationLink*>( theParentLink ), PxTransform(PxIdentity) );
				PxSerialObjectId theIdPtr = 0;
				inSerializer.mReader.read( "Id", theIdPtr );

				linkRemapMap.insert( theIdPtr, newLink );
				readComplexObj( inSerializer, newLink );
				
				//Add link to PxCollection, since PxArticulation requires PxArticulationLink and joint.
				collection.add( *newLink, theIdPtr );
			}
		}
		inSerializer.popCurrentContext();
	}
	
	template<>
	struct RepXVisitorReader<PxArticulation> : public RepXVisitorReaderBase<PxArticulation>
	{
		RepXVisitorReader( TReaderNameStack& names, PxProfileArray<PxU32>& contexts, const PxRepXInstantiationArgs& args, XmlReader& reader, PxArticulation* obj
								, XmlMemoryAllocator&	alloc, PxCollection& collection, bool& ret)
			: RepXVisitorReaderBase<PxArticulation>( names, contexts, args, reader, obj, alloc, collection, ret)
		{
		}
		RepXVisitorReader( const RepXVisitorReader<PxArticulation>& other )
			: RepXVisitorReaderBase<PxArticulation>( other )
		{
		}
		
		void handleArticulationLinks( const PxArticulationLinkCollectionProp& inProp )
		{
			physx::Sn::readProperty( *this, mObj, inProp );
		}
	};

	template<typename TObjType, typename TInfoType>
	inline bool readAllProperties( PxRepXInstantiationArgs args, TReaderNameStack& names, PxProfileArray<PxU32>& contexts, XmlReader& reader, TObjType* obj, XmlMemoryAllocator& alloc, PxCollection& collection, TInfoType& info )
	{
		bool hadError = false;
		RepXVisitorReader<TObjType> theReader( names, contexts, args, reader, obj, alloc, collection, hadError);
		RepXPropertyFilter<RepXVisitorReader<TObjType> > theOp( theReader );
		info.visitBaseProperties( theOp );
		info.visitInstanceProperties( theOp );
		return !hadError;
	}

	
	template<typename TObjType>
	inline bool readAllProperties( PxRepXInstantiationArgs args, XmlReader& reader, TObjType* obj, XmlMemoryAllocator& alloc, PxCollection& collection )
	{
		PxProfileAllocatorWrapper wrapper( alloc.getAllocator() );
		TReaderNameStack names( wrapper );
		PxProfileArray<PxU32> contexts( wrapper );
		PxClassInfoTraits<TObjType> info;
		return readAllProperties( args, names, contexts, reader, obj, alloc, collection, info.Info );
	}
	
	template<typename TObjType, typename TReaderType, typename TInfoType>
	inline void readComplexObj( TReaderType& oldVisitor, TObjType* inObj, TInfoType& info)
	{
		if(!readAllProperties( oldVisitor.mArgs, oldVisitor.mNames, oldVisitor.mContexts, oldVisitor.mReader, inObj, oldVisitor.mAllocator, oldVisitor.mCollection, info ))
			oldVisitor.mHadError = true;
	}

	template<typename TObjType, typename TReaderType, typename TInfoType>
	inline void readComplexObj( TReaderType& oldVisitor, TObjType* inObj, const TInfoType& info)
	{
		if(!readAllProperties( oldVisitor.mArgs, oldVisitor.mNames, oldVisitor.mContexts, oldVisitor.mReader, inObj, oldVisitor.mAllocator, oldVisitor.mCollection, info ))
			oldVisitor.mHadError = true;
	}

	template<typename TObjType, typename TReaderType>
	inline void readComplexObj( TReaderType& oldVisitor, TObjType* inObj, const PxUnknownClassInfo& /*info*/)
	{
		const char* value = oldVisitor.mReader.getCurrentItemValue();
		if ( value && *value )
		{
			stringToType( value, *inObj );
			return;
		}
		oldVisitor.mHadError = true;		
	}

	template<typename TObjType, typename TReaderType>
	inline void readComplexObj( TReaderType& oldVisitor, TObjType* inObj)
	{
		PxClassInfoTraits<TObjType> info;
		if(!readAllProperties( oldVisitor.mArgs, oldVisitor.mNames, oldVisitor.mContexts, oldVisitor.mReader, inObj, oldVisitor.mAllocator, oldVisitor.mCollection, info.Info ))
			oldVisitor.mHadError = true;
	}

} }

#endif
