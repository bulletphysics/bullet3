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

#ifndef PVD_META_DATA_DEFINE_PROPERTIES_H
#define PVD_META_DATA_DEFINE_PROPERTIES_H

#if PX_SUPPORT_PVD

#include "PvdMetaDataPropertyVisitor.h"
#include "PxPvdDataStreamHelpers.h"
#include "PxPvdDataStream.h"
#include "PxCoreUtilityTypes.h"


namespace physx
{
namespace Vd
{	
	using namespace physx::shdfnd;
	using namespace physx::pvdsdk;

	template<typename TPropType>
	struct PropertyDefinitionOp
	{
		void defineProperty( PvdPropertyDefinitionHelper& mHelper, NamespacedName mClassKey )
		{
			mHelper.createProperty( mClassKey, "", getPvdNamespacedNameForType<TPropType>(), PropertyType::Scalar );
		}
	};
	template<>
	struct PropertyDefinitionOp<const char*>
	{
		void defineProperty( PvdPropertyDefinitionHelper& mHelper, NamespacedName mClassKey )
		{
			mHelper.createProperty( mClassKey, "", getPvdNamespacedNameForType<StringHandle>(), PropertyType::Scalar );
		}
	};
#define DEFINE_PROPERTY_DEFINITION_OP_NOP( type ) \
	template<> struct PropertyDefinitionOp<type> { void defineProperty( PvdPropertyDefinitionHelper&, NamespacedName ){} };

	//NOP out these two types.
	DEFINE_PROPERTY_DEFINITION_OP_NOP( PxStridedData )
	DEFINE_PROPERTY_DEFINITION_OP_NOP( PxBoundedData )

#define DEFINE_PROPERTY_DEFINITION_OBJECT_REF( type )										\
	template<> struct PropertyDefinitionOp<type> {											\
	void defineProperty( PvdPropertyDefinitionHelper& mHelper, NamespacedName mClassKey)	\
	{																						\
		mHelper.createProperty( mClassKey, "", getPvdNamespacedNameForType<ObjectRef>(), PropertyType::Scalar ); \
	}																						\
	};

	DEFINE_PROPERTY_DEFINITION_OBJECT_REF( PxTriangleMesh* )
	DEFINE_PROPERTY_DEFINITION_OBJECT_REF( PxBVH33TriangleMesh* )
	DEFINE_PROPERTY_DEFINITION_OBJECT_REF( PxBVH34TriangleMesh* )
	DEFINE_PROPERTY_DEFINITION_OBJECT_REF( PxConvexMesh* )
	DEFINE_PROPERTY_DEFINITION_OBJECT_REF( PxHeightField* )


struct PvdClassInfoDefine
{
	PvdPropertyDefinitionHelper& mHelper;
	NamespacedName mClassKey;

	PvdClassInfoDefine( PvdPropertyDefinitionHelper& info, NamespacedName inClassName )
		: mHelper( info )
		, mClassKey( inClassName ) { }

	PvdClassInfoDefine( const PvdClassInfoDefine& other )
		: mHelper( other.mHelper )
		, mClassKey( other.mClassKey )
	{
	}

	void defineProperty( NamespacedName inDtype, const char* semantic = "", PropertyType::Enum inPType = PropertyType::Scalar )
	{
		mHelper.createProperty( mClassKey, semantic, inDtype, inPType ); 
	}

	void pushName( const char* inName )
	{
		mHelper.pushName( inName );
	}
	
	void pushBracketedName( const char* inName) 
	{
		mHelper.pushBracketedName( inName );
	}

	void popName()
	{
		mHelper.popName();
	}

	inline void defineNameValueDefs( const PxU32ToName* theConversions )
	{
		while( theConversions->mName != NULL )
		{
			mHelper.addNamedValue( theConversions->mName, theConversions->mValue );
			++theConversions;
		}
	}

	template<typename TAccessorType>
	void simpleProperty( PxU32, TAccessorType& /*inProp */)
	{
		typedef typename TAccessorType::prop_type TPropertyType;
		PropertyDefinitionOp<TPropertyType>().defineProperty( mHelper, mClassKey );
	}

	template<typename TAccessorType, typename TInfoType>
	void extendedIndexedProperty( PxU32* key, const TAccessorType& inProp, TInfoType&  )
	{
		simpleProperty(*key, inProp);
	}

	template<typename TDataType>
	static NamespacedName getNameForEnumType()
	{
		size_t s = sizeof( TDataType );
		switch(s)
		{
		case 1: return getPvdNamespacedNameForType<PxU8>();
		case 2: return getPvdNamespacedNameForType<PxU16>();
		case 4: return getPvdNamespacedNameForType<PxU32>();
		default: return getPvdNamespacedNameForType<PxU64>();
		}
	}
	
	template<typename TAccessorType>
	void enumProperty( PxU32 /*key*/, TAccessorType& /*inProp*/, const PxU32ToName* inConversions )
	{
		typedef typename TAccessorType::prop_type TPropType;
		defineNameValueDefs( inConversions );
		defineProperty( getNameForEnumType<TPropType>(), "Enumeration Value" );
	}

	template<typename TAccessorType>
	void flagsProperty( PxU32 /*key*/, const TAccessorType& /*inAccessor*/, const PxU32ToName* inConversions )
	{
		typedef typename TAccessorType::prop_type TPropType;
		defineNameValueDefs( inConversions );
		defineProperty( getNameForEnumType<TPropType>(), "Bitflag" );
	}

	template<typename TAccessorType, typename TInfoType>
	void complexProperty( PxU32* key, const TAccessorType& inAccessor, TInfoType& inInfo )
	{
		PxU32 theOffset = inAccessor.mOffset;
		inInfo.visitBaseProperties( makePvdPropertyFilter( *this, key, &theOffset ) );
		inInfo.visitInstanceProperties( makePvdPropertyFilter( *this, key, &theOffset ) );
	}
	
	template<typename TAccessorType, typename TInfoType>
	void bufferCollectionProperty( PxU32* key, const TAccessorType& inAccessor, TInfoType& inInfo )
	{
		complexProperty(key, inAccessor, inInfo);
	}

	template<PxU32 TKey, typename TObjectType, typename TPropertyType, PxU32 TEnableFlag>
	void handleBuffer( const PxBufferPropertyInfo<TKey, TObjectType, const Array< TPropertyType >&, TEnableFlag>& inProp )
	{
		mHelper.pushName( inProp.mName );
		defineProperty( getPvdNamespacedNameForType<TPropertyType>(), "", PropertyType::Array );
		mHelper.popName();
	}

	template<PxU32 TKey, typename TObjectType, typename TCollectionType>
	void handleCollection( const PxReadOnlyCollectionPropertyInfo<TKey, TObjectType, TCollectionType>& inProp )
	{
		mHelper.pushName( inProp.mName );
		defineProperty( getPvdNamespacedNameForType<TCollectionType>(), "", PropertyType::Array );
		mHelper.popName();
	}
	
	template<PxU32 TKey, typename TObjectType, typename TEnumType>
	void handleCollection( const PxReadOnlyCollectionPropertyInfo<TKey, TObjectType, TEnumType>& inProp, const PxU32ToName* inConversions )
	{
		mHelper.pushName( inProp.mName );
		defineNameValueDefs( inConversions );
		defineProperty( getNameForEnumType<TEnumType>(), "Enumeration Value", PropertyType::Array );
		mHelper.popName();
	}

private:
	PvdClassInfoDefine& operator=(const PvdClassInfoDefine&);
};

template<typename TPropType>
struct SimplePropertyValueStructOp
{
	void addPropertyMessageArg( PvdPropertyDefinitionHelper& mHelper, PxU32 inOffset )
	{
		mHelper.addPropertyMessageArg<TPropType>( inOffset );
	}
};

#define DEFINE_SIMPLE_PROPERTY_VALUE_STRUCT_OP_NOP( type ) \
template<> struct SimplePropertyValueStructOp<type> { void addPropertyMessageArg( PvdPropertyDefinitionHelper&, PxU32 ){}};

DEFINE_SIMPLE_PROPERTY_VALUE_STRUCT_OP_NOP( PxStridedData )
DEFINE_SIMPLE_PROPERTY_VALUE_STRUCT_OP_NOP( PxBoundedData )

#define DEFINE_SIMPLE_PROPERTY_VALUE_STRUCT_VOIDPTR_OP( type )						\
template<> struct SimplePropertyValueStructOp<type> {								\
void addPropertyMessageArg( PvdPropertyDefinitionHelper& mHelper, PxU32 inOffset )	\
{																					\
	mHelper.addPropertyMessageArg<VoidPtr>( inOffset );								\
}																					\
};

DEFINE_SIMPLE_PROPERTY_VALUE_STRUCT_VOIDPTR_OP( PxTriangleMesh* )
DEFINE_SIMPLE_PROPERTY_VALUE_STRUCT_VOIDPTR_OP( PxBVH33TriangleMesh* )
DEFINE_SIMPLE_PROPERTY_VALUE_STRUCT_VOIDPTR_OP( PxBVH34TriangleMesh* )
DEFINE_SIMPLE_PROPERTY_VALUE_STRUCT_VOIDPTR_OP( PxConvexMesh* )
DEFINE_SIMPLE_PROPERTY_VALUE_STRUCT_VOIDPTR_OP( PxHeightField* )


struct PvdClassInfoValueStructDefine
{
private:
	PvdClassInfoValueStructDefine& operator=(const PvdClassInfoValueStructDefine&);
public:

	PvdPropertyDefinitionHelper& mHelper;

	PvdClassInfoValueStructDefine( PvdPropertyDefinitionHelper& info )
		: mHelper( info )
	{ }

	PvdClassInfoValueStructDefine( const PvdClassInfoValueStructDefine& other )
		: mHelper( other.mHelper )
	{
	}

	void defineValueStructOffset( const ValueStructOffsetRecord& inProp, PxU32 inPropSize )
	{
		if( inProp.mHasValidOffset )
		{
			switch( inPropSize )
			{
			case 8: mHelper.addPropertyMessageArg<PxU64>( inProp.mOffset ); break;
			case 4: mHelper.addPropertyMessageArg<PxU32>( inProp.mOffset ); break;
			case 2: mHelper.addPropertyMessageArg<PxU16>( inProp.mOffset ); break;
			default: 
				PX_ASSERT(1 == inPropSize);
				mHelper.addPropertyMessageArg<PxU8>( inProp.mOffset ); break;
			}
		}
	}

	void pushName( const char* inName )
	{
		mHelper.pushName( inName );
	}
	
	void pushBracketedName( const char* inName) 
	{
		mHelper.pushBracketedName( inName );
	}

	void popName()
	{
		mHelper.popName();
	}

    template<typename TAccessorType, typename TInfoType>
	void bufferCollectionProperty( PxU32* /*key*/, const TAccessorType& /*inAccessor*/, TInfoType& /*inInfo*/ )
	{
		//complexProperty(key, inAccessor, inInfo);
	}

	template<typename TAccessorType>
	void simpleProperty( PxU32 /*key*/, TAccessorType& inProp )
	{
		typedef typename TAccessorType::prop_type TPropertyType;
		if ( inProp.mHasValidOffset )
		{
			SimplePropertyValueStructOp<TPropertyType>().addPropertyMessageArg( mHelper, inProp.mOffset );
		}
	}
	
	template<typename TAccessorType>
	void enumProperty( PxU32 /*key*/, TAccessorType& inAccessor, const PxU32ToName* /*inConversions */)
	{
		typedef typename TAccessorType::prop_type TPropType;
		defineValueStructOffset( inAccessor, sizeof( TPropType ) );
	}

	template<typename TAccessorType>
	void flagsProperty( PxU32 /*key*/, const TAccessorType& inAccessor, const PxU32ToName* /*inConversions */)
	{
		typedef typename TAccessorType::prop_type TPropType;
		defineValueStructOffset( inAccessor, sizeof( TPropType ) );
	}

	template<typename TAccessorType, typename TInfoType>
	void complexProperty( PxU32* key, const TAccessorType& inAccessor, TInfoType& inInfo )
	{
		PxU32 theOffset = inAccessor.mOffset;
		inInfo.visitBaseProperties( makePvdPropertyFilter( *this, key, &theOffset ) );
		inInfo.visitInstanceProperties( makePvdPropertyFilter( *this, key, &theOffset ) );
	}

	template<PxU32 TKey, typename TObjectType, typename TCollectionType>
	void handleCollection( const PxReadOnlyCollectionPropertyInfo<TKey, TObjectType, TCollectionType>& /*prop*/ )
	{
	}
	
	template<PxU32 TKey, typename TObjectType, typename TEnumType>
	void handleCollection( const PxReadOnlyCollectionPropertyInfo<TKey, TObjectType, TEnumType>& /*prop*/, const PxU32ToName* /*inConversions */)
	{
	}

    template<PxU32 TKey, typename TObjectType, typename TInfoType>
	void handleCollection( const PxBufferCollectionPropertyInfo<TKey, TObjectType, TInfoType>& /*prop*/,  const TInfoType& /*inInfo */)
	{
	}
};

}

}

#endif
#endif
