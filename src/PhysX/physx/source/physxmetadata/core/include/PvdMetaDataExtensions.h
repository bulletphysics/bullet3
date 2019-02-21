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

#ifndef PX_META_DATA_EXTENSIONS_H
#define PX_META_DATA_EXTENSIONS_H
#include "PxMetaDataObjects.h"

#if PX_SUPPORT_PVD
#include "PxPvdObjectModelBaseTypes.h"

namespace physx { namespace pvdsdk {

	template<> PX_INLINE NamespacedName getPvdNamespacedNameForType<physx::PxMetaDataPlane>() { return getPvdNamespacedNameForType<PxVec4>(); }
	template<> PX_INLINE NamespacedName getPvdNamespacedNameForType<physx::PxRigidActor*>() { return getPvdNamespacedNameForType<VoidPtr>(); }
	
}}
#endif

namespace physx
{
namespace Vd
{
//Additional properties that exist only in pvd land.
struct PxPvdOnlyProperties
{
	enum Enum
	{
		FirstProp = PxPropertyInfoName::LastPxPropertyInfoName,
		PxScene_Frame,
		PxScene_Contacts,
		PxScene_SimulateElapsedTime,
#define DEFINE_ENUM_RANGE( stem, count ) \
	stem##Begin, \
	stem##End = stem##Begin + count

		//I can't easily add up the number of required property entries, but it is large due to the below
		//geometry count squared properties.  Thus I punt and allocate way more than I need right now.
		DEFINE_ENUM_RANGE( PxScene_SimulationStatistics, 1000 ),
		DEFINE_ENUM_RANGE( PxSceneDesc_Limits, PxPropertyInfoName::PxSceneLimits_PropertiesStop - PxPropertyInfoName::PxSceneLimits_PropertiesStart ),
		DEFINE_ENUM_RANGE( PxSimulationStatistics_NumShapes, PxGeometryType::eGEOMETRY_COUNT ),
		DEFINE_ENUM_RANGE( PxSimulationStatistics_NumDiscreteContactPairs, PxGeometryType::eGEOMETRY_COUNT * PxGeometryType::eGEOMETRY_COUNT ),
		DEFINE_ENUM_RANGE( PxSimulationStatistics_NumModifiedContactPairs, PxGeometryType::eGEOMETRY_COUNT * PxGeometryType::eGEOMETRY_COUNT ),
		DEFINE_ENUM_RANGE( PxSimulationStatistics_NumSweptIntegrationPairs, PxGeometryType::eGEOMETRY_COUNT * PxGeometryType::eGEOMETRY_COUNT ),
		DEFINE_ENUM_RANGE( PxSimulationStatistics_NumTriggerPairs, PxGeometryType::eGEOMETRY_COUNT * PxGeometryType::eGEOMETRY_COUNT ),
		DEFINE_ENUM_RANGE( PxRigidDynamic_SolverIterationCounts, 2 ),
 		DEFINE_ENUM_RANGE( PxArticulation_SolverIterationCounts, 2 ),
		DEFINE_ENUM_RANGE( PxArticulationJoint_SwingLimit, 2 ),
		DEFINE_ENUM_RANGE( PxArticulationJoint_TwistLimit, 2 ),
		DEFINE_ENUM_RANGE( PxConvexMeshGeometry_Scale, PxPropertyInfoName::PxMeshScale_PropertiesStop - PxPropertyInfoName::PxMeshScale_PropertiesStart ),
		DEFINE_ENUM_RANGE( PxTriangleMeshGeometry_Scale, PxPropertyInfoName::PxMeshScale_PropertiesStop - PxPropertyInfoName::PxMeshScale_PropertiesStart ),

		LastPxPvdOnlyProperty
	};
};

template<PxU32 TKey, typename TObjectType, typename TPropertyType, PxU32 TEnableFlag>
struct PxBufferPropertyInfo : PxReadOnlyPropertyInfo< TKey, TObjectType, TPropertyType >
{
	typedef PxReadOnlyPropertyInfo< TKey, TObjectType, TPropertyType > TBaseType;
	typedef typename TBaseType::TGetterType TGetterType;
	PxBufferPropertyInfo( const char* inName, TGetterType inGetter )
		: TBaseType( inName, inGetter )
	{
	}
	bool isEnabled( PxU32 inFlags ) const { return (inFlags & TEnableFlag) > 0; }
};


#define DECLARE_BUFFER_PROPERTY( objectType, baseType, propType, propName, fieldName, flagName )												\
typedef PxBufferPropertyInfo< PxPvdOnlyProperties::baseType##_##propName, objectType, propType, flagName > T##objectType##propName##Base;		\
inline propType get##propName( const objectType* inData ) { return inData->fieldName; }															\
struct baseType##propName##Property : T##objectType##propName##Base																				\
{																																				\
	baseType##propName##Property()  : T##objectType##propName##Base( #propName, get##propName ){}												\
};

template<PxU32 PropertyKey, typename TEnumType >
struct IndexerToNameMap
{
	PxEnumTraits<TEnumType> Converter;
};

struct ValueStructOffsetRecord
{
	mutable bool	mHasValidOffset;
	mutable PxU32	mOffset;
	ValueStructOffsetRecord() : mHasValidOffset( false ), mOffset( 0 ) {}
	void setupValueStructOffset( PxU32 inValue ) const
	{
		mHasValidOffset = true;
		mOffset = inValue;
	}
};

template<PxU32 TKey, typename TObjectType, typename TPropertyType>
struct PxPvdReadOnlyPropertyAccessor : public ValueStructOffsetRecord
{
	typedef PxReadOnlyPropertyInfo<TKey,TObjectType,TPropertyType> TPropertyInfoType;
	typedef TPropertyType prop_type;

	const TPropertyInfoType	mProperty;
	PxPvdReadOnlyPropertyAccessor( const TPropertyInfoType& inProp )
		: mProperty( inProp )
	{
	}
	prop_type get( const TObjectType* inObj ) const { return mProperty.get( inObj ); }

private:
	PxPvdReadOnlyPropertyAccessor& operator=(const PxPvdReadOnlyPropertyAccessor&);
};

template<PxU32 TKey, typename TObjectType, typename TPropertyType>
struct PxBufferCollectionPropertyAccessor : public ValueStructOffsetRecord
{
	typedef PxBufferCollectionPropertyInfo< TKey, TObjectType, TPropertyType > TPropertyInfoType;
	typedef TPropertyType prop_type;
	const TPropertyInfoType& mProperty;
	const char* mName;
	
	PxBufferCollectionPropertyAccessor( const TPropertyInfoType& inProp, const char* inName )
		: mProperty( inProp )
		, mName( inName )
	{
	}
	
	const char* name() const { return mName; }
	PxU32 size( const TObjectType* inObj ) const { return mProperty.size( inObj ); }
	PxU32 get( const TObjectType* inObj, prop_type* buffer, PxU32 inNumItems) const { return mProperty.get( inObj, buffer, inNumItems); }
	void set( TObjectType* inObj, prop_type* inBuffer, PxU32 inNumItems ) const { mProperty.set( inObj, inBuffer, inNumItems ); }
};

template<PxU32 TKey, typename TObjectType, typename TIndexType, typename TPropertyType>
struct PxPvdIndexedPropertyAccessor : public ValueStructOffsetRecord
{
	typedef PxIndexedPropertyInfo< TKey, TObjectType, TIndexType, TPropertyType > TPropertyInfoType;
	typedef TPropertyType prop_type;
	TIndexType mIndex;
	const TPropertyInfoType& mProperty;
	PxPvdIndexedPropertyAccessor( const TPropertyInfoType& inProp, PxU32 inIndex )
		: mIndex( static_cast<TIndexType>( inIndex ) )
		, mProperty( inProp )
	{
	}
	prop_type get( const TObjectType* inObj ) const { return mProperty.get( inObj, mIndex ); }
	void set( TObjectType* inObj, prop_type val ) const { mProperty.set( inObj, mIndex, val ); }

	void operator = (PxPvdIndexedPropertyAccessor&) {}
};

template<PxU32 TKey, typename TObjectType, typename TIndexType, typename TPropertyType>
struct PxPvdExtendedIndexedPropertyAccessor : public ValueStructOffsetRecord
{
	typedef PxExtendedIndexedPropertyInfo< TKey, TObjectType, TIndexType, TPropertyType > TPropertyInfoType;
	typedef TPropertyType prop_type;
	TIndexType mIndex;
	const TPropertyInfoType& mProperty;
	PxPvdExtendedIndexedPropertyAccessor( const TPropertyInfoType& inProp, PxU32 inIndex )
		: mIndex( static_cast<TIndexType>( inIndex ) )
		, mProperty( inProp )
	{
	}

	PxU32 size( const TObjectType* inObj ) const { return mProperty.size( inObj ); }
	prop_type get( const TObjectType* inObj, TIndexType index ) const { return mProperty.get( inObj, index ); }
	void set( TObjectType* inObj, TIndexType index, prop_type val ) const { mProperty.set( inObj, index, val ); }

	void operator = (PxPvdExtendedIndexedPropertyAccessor&) {}
};

template<PxU32 TKey, typename TObjectType, typename TIndexType, typename TPropertyType>
struct PxPvdFixedSizeLookupTablePropertyAccessor : public ValueStructOffsetRecord
{
	typedef PxFixedSizeLookupTablePropertyInfo< TKey, TObjectType, TIndexType, TPropertyType > TPropertyInfoType;
	typedef TPropertyType prop_type;
	TIndexType	mIndex;
	
	const TPropertyInfoType& mProperty;
	PxPvdFixedSizeLookupTablePropertyAccessor( const TPropertyInfoType& inProp, const PxU32 inIndex3 )
		: mIndex( static_cast<TIndexType>( inIndex3 ) )
		, mProperty( inProp )
	{
	}

	PxU32 size( const TObjectType* inObj ) const { return mProperty.size( inObj ); }
	prop_type getX( const TObjectType* inObj, const TIndexType index ) const { return mProperty.getX( inObj, index ); }
	prop_type getY( const TObjectType* inObj, const TIndexType index ) const { return mProperty.getY( inObj, index ); }
	void addPair(  TObjectType* inObj, const PxReal x, const PxReal y ) { const_cast<TPropertyInfoType&>(mProperty).addPair( inObj, x, y );  }
	void clear( TObjectType* inObj ) { const_cast<TPropertyInfoType&>(mProperty).clear( inObj );  }
	void operator = (PxPvdFixedSizeLookupTablePropertyAccessor&) {}
};

template<PxU32 TKey, typename TObjectType, typename TIdx0Type, typename TIdx1Type, typename TPropertyType>
struct PxPvdDualIndexedPropertyAccessor : public ValueStructOffsetRecord
{
	typedef PxDualIndexedPropertyInfo< TKey, TObjectType, TIdx0Type, TIdx1Type, TPropertyType > TPropertyInfoType;
	typedef TPropertyType prop_type;
	TIdx0Type mIdx0;
	TIdx1Type mIdx1;
	const TPropertyInfoType& mProperty;

	PxPvdDualIndexedPropertyAccessor( const TPropertyInfoType& inProp, PxU32 idx0, PxU32 idx1 )
		: mIdx0( static_cast<TIdx0Type>( idx0 ) )
		, mIdx1( static_cast<TIdx1Type>( idx1 ) )
		, mProperty( inProp )
	{
	}
	prop_type get( const TObjectType* inObj ) const { return mProperty.get( inObj, mIdx0, mIdx1 ); }
	void set( TObjectType* inObj, prop_type val ) const { mProperty.set( inObj, mIdx0, mIdx1, val ); }

private:
	PxPvdDualIndexedPropertyAccessor& operator = (const PxPvdDualIndexedPropertyAccessor&);
};

template<PxU32 TKey, typename TObjectType, typename TIdx0Type, typename TIdx1Type, typename TPropertyType>
struct PxPvdExtendedDualIndexedPropertyAccessor : public ValueStructOffsetRecord
{
	typedef PxExtendedDualIndexedPropertyInfo< TKey, TObjectType, TIdx0Type, TIdx1Type, TPropertyType > TPropertyInfoType;
	typedef TPropertyType prop_type;
	TIdx0Type mIdx0;
	TIdx1Type mIdx1;
	const TPropertyInfoType& mProperty;
	
	PxPvdExtendedDualIndexedPropertyAccessor( const TPropertyInfoType& inProp, PxU32 idx0, PxU32 idx1 )
		: mIdx0( static_cast<TIdx0Type>( idx0 ) )
		, mIdx1( static_cast<TIdx1Type>( idx1 ) )
		, mProperty( inProp )
	{
	}
	prop_type get( const TObjectType* inObj ) const { return mProperty.get( inObj, mIdx0, mIdx1 ); }
	void set( TObjectType* inObj, prop_type val ) const { mProperty.set( inObj, mIdx0, mIdx1, val ); }

private:
	PxPvdExtendedDualIndexedPropertyAccessor& operator = (const PxPvdExtendedDualIndexedPropertyAccessor&);
};

template<PxU32 TKey, typename TObjType, typename TPropertyType>
struct PxPvdRangePropertyAccessor : public ValueStructOffsetRecord
{
	typedef PxRangePropertyInfo<TKey, TObjType, TPropertyType> TPropertyInfoType;
	typedef TPropertyType prop_type;
	bool mFirstValue;
	const TPropertyInfoType& mProperty;

	PxPvdRangePropertyAccessor( const TPropertyInfoType& inProp, bool inFirstValue )
		: mFirstValue( inFirstValue )
		, mProperty( inProp )
	{
	}

	prop_type get( const TObjType* inObj ) const {
		prop_type first,second;
		mProperty.get( inObj, first, second );
		return mFirstValue ? first : second;
	}
	void set( TObjType* inObj, prop_type val ) const 
	{ 
		prop_type first,second;
		mProperty.get( inObj, first, second );
		if ( mFirstValue ) mProperty.set( inObj, val, second ); 
		else mProperty.set( inObj, first, val );
	}

	void operator = (PxPvdRangePropertyAccessor&) {}
};


template<typename TDataType>
struct IsFlagsType
{
	bool FlagData;
};

template<typename TEnumType, typename TStorageType>
struct IsFlagsType<PxFlags<TEnumType, TStorageType> > 
{
	const PxU32ToName* FlagData;
	IsFlagsType<PxFlags<TEnumType, TStorageType> > () : FlagData( PxEnumTraits<TEnumType>().NameConversion ) {}
};



template<typename TDataType>
struct PvdClassForType
{
	bool Unknown;
};

}

}

#endif
