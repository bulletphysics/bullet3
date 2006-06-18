/*
 * Copyright 2006 Sony Computer Entertainment Inc.
 *
 * Licensed under the SCEA Shared Source License, Version 1.0 (the "License"); you may not use this 
 * file except in compliance with the License. You may obtain a copy of the License at:
 * http://research.scea.com/scea_shared_source_license.html
 *
 * Unless required by applicable law or agreed to in writing, software distributed under the License 
 * is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or 
 * implied. See the License for the specific language governing permissions and limitations under the 
 * License. 
 */

#include <dae/daeDom.h>
#include <dom/domCg_param_type.h>

daeElementRef
domCg_param_type::create(daeInt bytes)
{
	domCg_param_typeRef ref = new(bytes) domCg_param_type;
	return ref;
}


daeMetaElement *
domCg_param_type::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "cg_param_type" );
	_Meta->setStaticPointerAddress(&domCg_param_type::_Meta);
	_Meta->registerConstructor(domCg_param_type::create);

	_Meta->setIsTransparent( true );
	// Add elements: bool, bool1, bool2, bool3, bool4, bool1x1, bool1x2, bool1x3, bool1x4, bool2x1, bool2x2, bool2x3, bool2x4, bool3x1, bool3x2, bool3x3, bool3x4, bool4x1, bool4x2, bool4x3, bool4x4, float, float1, float2, float3, float4, float1x1, float1x2, float1x3, float1x4, float2x1, float2x2, float2x3, float2x4, float3x1, float3x2, float3x3, float3x4, float4x1, float4x2, float4x3, float4x4, int, int1, int2, int3, int4, int1x1, int1x2, int1x3, int1x4, int2x1, int2x2, int2x3, int2x4, int3x1, int3x2, int3x3, int3x4, int4x1, int4x2, int4x3, int4x4, half, half1, half2, half3, half4, half1x1, half1x2, half1x3, half1x4, half2x1, half2x2, half2x3, half2x4, half3x1, half3x2, half3x3, half3x4, half4x1, half4x2, half4x3, half4x4, fixed, fixed1, fixed2, fixed3, fixed4, fixed1x1, fixed1x2, fixed1x3, fixed1x4, fixed2x1, fixed2x2, fixed2x3, fixed2x4, fixed3x1, fixed3x2, fixed3x3, fixed3x4, fixed4x1, fixed4x2, fixed4x3, fixed4x4, surface, sampler1D, sampler2D, sampler3D, samplerRECT, samplerCUBE, samplerDEPTH, string, enum
    _Meta->appendElement(domCg_param_type::domBool::registerElement(),daeOffsetOf(domCg_param_type,elemBool));
    _Meta->appendElement(domCg_param_type::domBool1::registerElement(),daeOffsetOf(domCg_param_type,elemBool1));
    _Meta->appendElement(domCg_param_type::domBool2::registerElement(),daeOffsetOf(domCg_param_type,elemBool2));
    _Meta->appendElement(domCg_param_type::domBool3::registerElement(),daeOffsetOf(domCg_param_type,elemBool3));
    _Meta->appendElement(domCg_param_type::domBool4::registerElement(),daeOffsetOf(domCg_param_type,elemBool4));
    _Meta->appendElement(domCg_param_type::domBool1x1::registerElement(),daeOffsetOf(domCg_param_type,elemBool1x1));
    _Meta->appendElement(domCg_param_type::domBool1x2::registerElement(),daeOffsetOf(domCg_param_type,elemBool1x2));
    _Meta->appendElement(domCg_param_type::domBool1x3::registerElement(),daeOffsetOf(domCg_param_type,elemBool1x3));
    _Meta->appendElement(domCg_param_type::domBool1x4::registerElement(),daeOffsetOf(domCg_param_type,elemBool1x4));
    _Meta->appendElement(domCg_param_type::domBool2x1::registerElement(),daeOffsetOf(domCg_param_type,elemBool2x1));
    _Meta->appendElement(domCg_param_type::domBool2x2::registerElement(),daeOffsetOf(domCg_param_type,elemBool2x2));
    _Meta->appendElement(domCg_param_type::domBool2x3::registerElement(),daeOffsetOf(domCg_param_type,elemBool2x3));
    _Meta->appendElement(domCg_param_type::domBool2x4::registerElement(),daeOffsetOf(domCg_param_type,elemBool2x4));
    _Meta->appendElement(domCg_param_type::domBool3x1::registerElement(),daeOffsetOf(domCg_param_type,elemBool3x1));
    _Meta->appendElement(domCg_param_type::domBool3x2::registerElement(),daeOffsetOf(domCg_param_type,elemBool3x2));
    _Meta->appendElement(domCg_param_type::domBool3x3::registerElement(),daeOffsetOf(domCg_param_type,elemBool3x3));
    _Meta->appendElement(domCg_param_type::domBool3x4::registerElement(),daeOffsetOf(domCg_param_type,elemBool3x4));
    _Meta->appendElement(domCg_param_type::domBool4x1::registerElement(),daeOffsetOf(domCg_param_type,elemBool4x1));
    _Meta->appendElement(domCg_param_type::domBool4x2::registerElement(),daeOffsetOf(domCg_param_type,elemBool4x2));
    _Meta->appendElement(domCg_param_type::domBool4x3::registerElement(),daeOffsetOf(domCg_param_type,elemBool4x3));
    _Meta->appendElement(domCg_param_type::domBool4x4::registerElement(),daeOffsetOf(domCg_param_type,elemBool4x4));
    _Meta->appendElement(domCg_param_type::domFloat::registerElement(),daeOffsetOf(domCg_param_type,elemFloat));
    _Meta->appendElement(domCg_param_type::domFloat1::registerElement(),daeOffsetOf(domCg_param_type,elemFloat1));
    _Meta->appendElement(domCg_param_type::domFloat2::registerElement(),daeOffsetOf(domCg_param_type,elemFloat2));
    _Meta->appendElement(domCg_param_type::domFloat3::registerElement(),daeOffsetOf(domCg_param_type,elemFloat3));
    _Meta->appendElement(domCg_param_type::domFloat4::registerElement(),daeOffsetOf(domCg_param_type,elemFloat4));
    _Meta->appendElement(domCg_param_type::domFloat1x1::registerElement(),daeOffsetOf(domCg_param_type,elemFloat1x1));
    _Meta->appendElement(domCg_param_type::domFloat1x2::registerElement(),daeOffsetOf(domCg_param_type,elemFloat1x2));
    _Meta->appendElement(domCg_param_type::domFloat1x3::registerElement(),daeOffsetOf(domCg_param_type,elemFloat1x3));
    _Meta->appendElement(domCg_param_type::domFloat1x4::registerElement(),daeOffsetOf(domCg_param_type,elemFloat1x4));
    _Meta->appendElement(domCg_param_type::domFloat2x1::registerElement(),daeOffsetOf(domCg_param_type,elemFloat2x1));
    _Meta->appendElement(domCg_param_type::domFloat2x2::registerElement(),daeOffsetOf(domCg_param_type,elemFloat2x2));
    _Meta->appendElement(domCg_param_type::domFloat2x3::registerElement(),daeOffsetOf(domCg_param_type,elemFloat2x3));
    _Meta->appendElement(domCg_param_type::domFloat2x4::registerElement(),daeOffsetOf(domCg_param_type,elemFloat2x4));
    _Meta->appendElement(domCg_param_type::domFloat3x1::registerElement(),daeOffsetOf(domCg_param_type,elemFloat3x1));
    _Meta->appendElement(domCg_param_type::domFloat3x2::registerElement(),daeOffsetOf(domCg_param_type,elemFloat3x2));
    _Meta->appendElement(domCg_param_type::domFloat3x3::registerElement(),daeOffsetOf(domCg_param_type,elemFloat3x3));
    _Meta->appendElement(domCg_param_type::domFloat3x4::registerElement(),daeOffsetOf(domCg_param_type,elemFloat3x4));
    _Meta->appendElement(domCg_param_type::domFloat4x1::registerElement(),daeOffsetOf(domCg_param_type,elemFloat4x1));
    _Meta->appendElement(domCg_param_type::domFloat4x2::registerElement(),daeOffsetOf(domCg_param_type,elemFloat4x2));
    _Meta->appendElement(domCg_param_type::domFloat4x3::registerElement(),daeOffsetOf(domCg_param_type,elemFloat4x3));
    _Meta->appendElement(domCg_param_type::domFloat4x4::registerElement(),daeOffsetOf(domCg_param_type,elemFloat4x4));
    _Meta->appendElement(domCg_param_type::domInt::registerElement(),daeOffsetOf(domCg_param_type,elemInt));
    _Meta->appendElement(domCg_param_type::domInt1::registerElement(),daeOffsetOf(domCg_param_type,elemInt1));
    _Meta->appendElement(domCg_param_type::domInt2::registerElement(),daeOffsetOf(domCg_param_type,elemInt2));
    _Meta->appendElement(domCg_param_type::domInt3::registerElement(),daeOffsetOf(domCg_param_type,elemInt3));
    _Meta->appendElement(domCg_param_type::domInt4::registerElement(),daeOffsetOf(domCg_param_type,elemInt4));
    _Meta->appendElement(domCg_param_type::domInt1x1::registerElement(),daeOffsetOf(domCg_param_type,elemInt1x1));
    _Meta->appendElement(domCg_param_type::domInt1x2::registerElement(),daeOffsetOf(domCg_param_type,elemInt1x2));
    _Meta->appendElement(domCg_param_type::domInt1x3::registerElement(),daeOffsetOf(domCg_param_type,elemInt1x3));
    _Meta->appendElement(domCg_param_type::domInt1x4::registerElement(),daeOffsetOf(domCg_param_type,elemInt1x4));
    _Meta->appendElement(domCg_param_type::domInt2x1::registerElement(),daeOffsetOf(domCg_param_type,elemInt2x1));
    _Meta->appendElement(domCg_param_type::domInt2x2::registerElement(),daeOffsetOf(domCg_param_type,elemInt2x2));
    _Meta->appendElement(domCg_param_type::domInt2x3::registerElement(),daeOffsetOf(domCg_param_type,elemInt2x3));
    _Meta->appendElement(domCg_param_type::domInt2x4::registerElement(),daeOffsetOf(domCg_param_type,elemInt2x4));
    _Meta->appendElement(domCg_param_type::domInt3x1::registerElement(),daeOffsetOf(domCg_param_type,elemInt3x1));
    _Meta->appendElement(domCg_param_type::domInt3x2::registerElement(),daeOffsetOf(domCg_param_type,elemInt3x2));
    _Meta->appendElement(domCg_param_type::domInt3x3::registerElement(),daeOffsetOf(domCg_param_type,elemInt3x3));
    _Meta->appendElement(domCg_param_type::domInt3x4::registerElement(),daeOffsetOf(domCg_param_type,elemInt3x4));
    _Meta->appendElement(domCg_param_type::domInt4x1::registerElement(),daeOffsetOf(domCg_param_type,elemInt4x1));
    _Meta->appendElement(domCg_param_type::domInt4x2::registerElement(),daeOffsetOf(domCg_param_type,elemInt4x2));
    _Meta->appendElement(domCg_param_type::domInt4x3::registerElement(),daeOffsetOf(domCg_param_type,elemInt4x3));
    _Meta->appendElement(domCg_param_type::domInt4x4::registerElement(),daeOffsetOf(domCg_param_type,elemInt4x4));
    _Meta->appendElement(domCg_param_type::domHalf::registerElement(),daeOffsetOf(domCg_param_type,elemHalf));
    _Meta->appendElement(domCg_param_type::domHalf1::registerElement(),daeOffsetOf(domCg_param_type,elemHalf1));
    _Meta->appendElement(domCg_param_type::domHalf2::registerElement(),daeOffsetOf(domCg_param_type,elemHalf2));
    _Meta->appendElement(domCg_param_type::domHalf3::registerElement(),daeOffsetOf(domCg_param_type,elemHalf3));
    _Meta->appendElement(domCg_param_type::domHalf4::registerElement(),daeOffsetOf(domCg_param_type,elemHalf4));
    _Meta->appendElement(domCg_param_type::domHalf1x1::registerElement(),daeOffsetOf(domCg_param_type,elemHalf1x1));
    _Meta->appendElement(domCg_param_type::domHalf1x2::registerElement(),daeOffsetOf(domCg_param_type,elemHalf1x2));
    _Meta->appendElement(domCg_param_type::domHalf1x3::registerElement(),daeOffsetOf(domCg_param_type,elemHalf1x3));
    _Meta->appendElement(domCg_param_type::domHalf1x4::registerElement(),daeOffsetOf(domCg_param_type,elemHalf1x4));
    _Meta->appendElement(domCg_param_type::domHalf2x1::registerElement(),daeOffsetOf(domCg_param_type,elemHalf2x1));
    _Meta->appendElement(domCg_param_type::domHalf2x2::registerElement(),daeOffsetOf(domCg_param_type,elemHalf2x2));
    _Meta->appendElement(domCg_param_type::domHalf2x3::registerElement(),daeOffsetOf(domCg_param_type,elemHalf2x3));
    _Meta->appendElement(domCg_param_type::domHalf2x4::registerElement(),daeOffsetOf(domCg_param_type,elemHalf2x4));
    _Meta->appendElement(domCg_param_type::domHalf3x1::registerElement(),daeOffsetOf(domCg_param_type,elemHalf3x1));
    _Meta->appendElement(domCg_param_type::domHalf3x2::registerElement(),daeOffsetOf(domCg_param_type,elemHalf3x2));
    _Meta->appendElement(domCg_param_type::domHalf3x3::registerElement(),daeOffsetOf(domCg_param_type,elemHalf3x3));
    _Meta->appendElement(domCg_param_type::domHalf3x4::registerElement(),daeOffsetOf(domCg_param_type,elemHalf3x4));
    _Meta->appendElement(domCg_param_type::domHalf4x1::registerElement(),daeOffsetOf(domCg_param_type,elemHalf4x1));
    _Meta->appendElement(domCg_param_type::domHalf4x2::registerElement(),daeOffsetOf(domCg_param_type,elemHalf4x2));
    _Meta->appendElement(domCg_param_type::domHalf4x3::registerElement(),daeOffsetOf(domCg_param_type,elemHalf4x3));
    _Meta->appendElement(domCg_param_type::domHalf4x4::registerElement(),daeOffsetOf(domCg_param_type,elemHalf4x4));
    _Meta->appendElement(domCg_param_type::domFixed::registerElement(),daeOffsetOf(domCg_param_type,elemFixed));
    _Meta->appendElement(domCg_param_type::domFixed1::registerElement(),daeOffsetOf(domCg_param_type,elemFixed1));
    _Meta->appendElement(domCg_param_type::domFixed2::registerElement(),daeOffsetOf(domCg_param_type,elemFixed2));
    _Meta->appendElement(domCg_param_type::domFixed3::registerElement(),daeOffsetOf(domCg_param_type,elemFixed3));
    _Meta->appendElement(domCg_param_type::domFixed4::registerElement(),daeOffsetOf(domCg_param_type,elemFixed4));
    _Meta->appendElement(domCg_param_type::domFixed1x1::registerElement(),daeOffsetOf(domCg_param_type,elemFixed1x1));
    _Meta->appendElement(domCg_param_type::domFixed1x2::registerElement(),daeOffsetOf(domCg_param_type,elemFixed1x2));
    _Meta->appendElement(domCg_param_type::domFixed1x3::registerElement(),daeOffsetOf(domCg_param_type,elemFixed1x3));
    _Meta->appendElement(domCg_param_type::domFixed1x4::registerElement(),daeOffsetOf(domCg_param_type,elemFixed1x4));
    _Meta->appendElement(domCg_param_type::domFixed2x1::registerElement(),daeOffsetOf(domCg_param_type,elemFixed2x1));
    _Meta->appendElement(domCg_param_type::domFixed2x2::registerElement(),daeOffsetOf(domCg_param_type,elemFixed2x2));
    _Meta->appendElement(domCg_param_type::domFixed2x3::registerElement(),daeOffsetOf(domCg_param_type,elemFixed2x3));
    _Meta->appendElement(domCg_param_type::domFixed2x4::registerElement(),daeOffsetOf(domCg_param_type,elemFixed2x4));
    _Meta->appendElement(domCg_param_type::domFixed3x1::registerElement(),daeOffsetOf(domCg_param_type,elemFixed3x1));
    _Meta->appendElement(domCg_param_type::domFixed3x2::registerElement(),daeOffsetOf(domCg_param_type,elemFixed3x2));
    _Meta->appendElement(domCg_param_type::domFixed3x3::registerElement(),daeOffsetOf(domCg_param_type,elemFixed3x3));
    _Meta->appendElement(domCg_param_type::domFixed3x4::registerElement(),daeOffsetOf(domCg_param_type,elemFixed3x4));
    _Meta->appendElement(domCg_param_type::domFixed4x1::registerElement(),daeOffsetOf(domCg_param_type,elemFixed4x1));
    _Meta->appendElement(domCg_param_type::domFixed4x2::registerElement(),daeOffsetOf(domCg_param_type,elemFixed4x2));
    _Meta->appendElement(domCg_param_type::domFixed4x3::registerElement(),daeOffsetOf(domCg_param_type,elemFixed4x3));
    _Meta->appendElement(domCg_param_type::domFixed4x4::registerElement(),daeOffsetOf(domCg_param_type,elemFixed4x4));
    _Meta->appendElement(domCg_surface_type::registerElement(),daeOffsetOf(domCg_param_type,elemSurface),"surface"); 
    _Meta->appendElement(domCg_sampler1D::registerElement(),daeOffsetOf(domCg_param_type,elemSampler1D),"sampler1D"); 
    _Meta->appendElement(domCg_sampler2D::registerElement(),daeOffsetOf(domCg_param_type,elemSampler2D),"sampler2D"); 
    _Meta->appendElement(domCg_sampler3D::registerElement(),daeOffsetOf(domCg_param_type,elemSampler3D),"sampler3D"); 
    _Meta->appendElement(domCg_samplerRECT::registerElement(),daeOffsetOf(domCg_param_type,elemSamplerRECT),"samplerRECT"); 
    _Meta->appendElement(domCg_samplerCUBE::registerElement(),daeOffsetOf(domCg_param_type,elemSamplerCUBE),"samplerCUBE"); 
    _Meta->appendElement(domCg_samplerDEPTH::registerElement(),daeOffsetOf(domCg_param_type,elemSamplerDEPTH),"samplerDEPTH"); 
    _Meta->appendElement(domCg_param_type::domString::registerElement(),daeOffsetOf(domCg_param_type,elemString));
    _Meta->appendElement(domCg_param_type::domEnum::registerElement(),daeOffsetOf(domCg_param_type,elemEnum));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domCg_param_type,_contents));

	
	
	_Meta->setElementSize(sizeof(domCg_param_type));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domBool::create(daeInt bytes)
{
	domCg_param_type::domBoolRef ref = new(bytes) domCg_param_type::domBool;
	return ref;
}


daeMetaElement *
domCg_param_type::domBool::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domBool::_Meta);
	_Meta->registerConstructor(domCg_param_type::domBool::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_bool"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domBool , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domBool));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domBool1::create(daeInt bytes)
{
	domCg_param_type::domBool1Ref ref = new(bytes) domCg_param_type::domBool1;
	return ref;
}


daeMetaElement *
domCg_param_type::domBool1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domBool1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domBool1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_bool1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domBool1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domBool1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domBool2::create(daeInt bytes)
{
	domCg_param_type::domBool2Ref ref = new(bytes) domCg_param_type::domBool2;
	return ref;
}


daeMetaElement *
domCg_param_type::domBool2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domBool2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domBool2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_bool2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domBool2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domBool2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domBool3::create(daeInt bytes)
{
	domCg_param_type::domBool3Ref ref = new(bytes) domCg_param_type::domBool3;
	return ref;
}


daeMetaElement *
domCg_param_type::domBool3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domBool3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domBool3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_bool3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domBool3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domBool3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domBool4::create(daeInt bytes)
{
	domCg_param_type::domBool4Ref ref = new(bytes) domCg_param_type::domBool4;
	return ref;
}


daeMetaElement *
domCg_param_type::domBool4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domBool4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domBool4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_bool4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domBool4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domBool4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domBool1x1::create(daeInt bytes)
{
	domCg_param_type::domBool1x1Ref ref = new(bytes) domCg_param_type::domBool1x1;
	return ref;
}


daeMetaElement *
domCg_param_type::domBool1x1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool1x1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domBool1x1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domBool1x1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_bool1x1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domBool1x1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domBool1x1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domBool1x2::create(daeInt bytes)
{
	domCg_param_type::domBool1x2Ref ref = new(bytes) domCg_param_type::domBool1x2;
	return ref;
}


daeMetaElement *
domCg_param_type::domBool1x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool1x2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domBool1x2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domBool1x2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_bool1x2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domBool1x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domBool1x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domBool1x3::create(daeInt bytes)
{
	domCg_param_type::domBool1x3Ref ref = new(bytes) domCg_param_type::domBool1x3;
	return ref;
}


daeMetaElement *
domCg_param_type::domBool1x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool1x3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domBool1x3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domBool1x3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_bool1x3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domBool1x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domBool1x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domBool1x4::create(daeInt bytes)
{
	domCg_param_type::domBool1x4Ref ref = new(bytes) domCg_param_type::domBool1x4;
	return ref;
}


daeMetaElement *
domCg_param_type::domBool1x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool1x4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domBool1x4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domBool1x4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_bool1x4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domBool1x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domBool1x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domBool2x1::create(daeInt bytes)
{
	domCg_param_type::domBool2x1Ref ref = new(bytes) domCg_param_type::domBool2x1;
	return ref;
}


daeMetaElement *
domCg_param_type::domBool2x1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool2x1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domBool2x1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domBool2x1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_bool2x1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domBool2x1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domBool2x1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domBool2x2::create(daeInt bytes)
{
	domCg_param_type::domBool2x2Ref ref = new(bytes) domCg_param_type::domBool2x2;
	return ref;
}


daeMetaElement *
domCg_param_type::domBool2x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool2x2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domBool2x2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domBool2x2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_bool2x2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domBool2x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domBool2x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domBool2x3::create(daeInt bytes)
{
	domCg_param_type::domBool2x3Ref ref = new(bytes) domCg_param_type::domBool2x3;
	return ref;
}


daeMetaElement *
domCg_param_type::domBool2x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool2x3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domBool2x3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domBool2x3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_bool2x3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domBool2x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domBool2x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domBool2x4::create(daeInt bytes)
{
	domCg_param_type::domBool2x4Ref ref = new(bytes) domCg_param_type::domBool2x4;
	return ref;
}


daeMetaElement *
domCg_param_type::domBool2x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool2x4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domBool2x4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domBool2x4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_bool2x4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domBool2x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domBool2x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domBool3x1::create(daeInt bytes)
{
	domCg_param_type::domBool3x1Ref ref = new(bytes) domCg_param_type::domBool3x1;
	return ref;
}


daeMetaElement *
domCg_param_type::domBool3x1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool3x1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domBool3x1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domBool3x1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_bool3x1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domBool3x1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domBool3x1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domBool3x2::create(daeInt bytes)
{
	domCg_param_type::domBool3x2Ref ref = new(bytes) domCg_param_type::domBool3x2;
	return ref;
}


daeMetaElement *
domCg_param_type::domBool3x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool3x2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domBool3x2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domBool3x2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_bool3x2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domBool3x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domBool3x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domBool3x3::create(daeInt bytes)
{
	domCg_param_type::domBool3x3Ref ref = new(bytes) domCg_param_type::domBool3x3;
	return ref;
}


daeMetaElement *
domCg_param_type::domBool3x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool3x3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domBool3x3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domBool3x3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_bool3x3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domBool3x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domBool3x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domBool3x4::create(daeInt bytes)
{
	domCg_param_type::domBool3x4Ref ref = new(bytes) domCg_param_type::domBool3x4;
	return ref;
}


daeMetaElement *
domCg_param_type::domBool3x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool3x4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domBool3x4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domBool3x4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_bool3x4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domBool3x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domBool3x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domBool4x1::create(daeInt bytes)
{
	domCg_param_type::domBool4x1Ref ref = new(bytes) domCg_param_type::domBool4x1;
	return ref;
}


daeMetaElement *
domCg_param_type::domBool4x1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool4x1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domBool4x1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domBool4x1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_bool4x1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domBool4x1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domBool4x1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domBool4x2::create(daeInt bytes)
{
	domCg_param_type::domBool4x2Ref ref = new(bytes) domCg_param_type::domBool4x2;
	return ref;
}


daeMetaElement *
domCg_param_type::domBool4x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool4x2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domBool4x2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domBool4x2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_bool4x2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domBool4x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domBool4x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domBool4x3::create(daeInt bytes)
{
	domCg_param_type::domBool4x3Ref ref = new(bytes) domCg_param_type::domBool4x3;
	return ref;
}


daeMetaElement *
domCg_param_type::domBool4x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool4x3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domBool4x3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domBool4x3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_bool4x3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domBool4x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domBool4x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domBool4x4::create(daeInt bytes)
{
	domCg_param_type::domBool4x4Ref ref = new(bytes) domCg_param_type::domBool4x4;
	return ref;
}


daeMetaElement *
domCg_param_type::domBool4x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool4x4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domBool4x4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domBool4x4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_bool4x4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domBool4x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domBool4x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFloat::create(daeInt bytes)
{
	domCg_param_type::domFloatRef ref = new(bytes) domCg_param_type::domFloat;
	return ref;
}


daeMetaElement *
domCg_param_type::domFloat::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFloat::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFloat::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_float"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFloat , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFloat));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFloat1::create(daeInt bytes)
{
	domCg_param_type::domFloat1Ref ref = new(bytes) domCg_param_type::domFloat1;
	return ref;
}


daeMetaElement *
domCg_param_type::domFloat1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFloat1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFloat1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_float1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFloat1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFloat1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFloat2::create(daeInt bytes)
{
	domCg_param_type::domFloat2Ref ref = new(bytes) domCg_param_type::domFloat2;
	return ref;
}


daeMetaElement *
domCg_param_type::domFloat2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFloat2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFloat2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_float2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFloat2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFloat2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFloat3::create(daeInt bytes)
{
	domCg_param_type::domFloat3Ref ref = new(bytes) domCg_param_type::domFloat3;
	return ref;
}


daeMetaElement *
domCg_param_type::domFloat3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFloat3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFloat3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_float3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFloat3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFloat3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFloat4::create(daeInt bytes)
{
	domCg_param_type::domFloat4Ref ref = new(bytes) domCg_param_type::domFloat4;
	return ref;
}


daeMetaElement *
domCg_param_type::domFloat4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFloat4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFloat4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_float4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFloat4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFloat4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFloat1x1::create(daeInt bytes)
{
	domCg_param_type::domFloat1x1Ref ref = new(bytes) domCg_param_type::domFloat1x1;
	return ref;
}


daeMetaElement *
domCg_param_type::domFloat1x1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float1x1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFloat1x1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFloat1x1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_float1x1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFloat1x1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFloat1x1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFloat1x2::create(daeInt bytes)
{
	domCg_param_type::domFloat1x2Ref ref = new(bytes) domCg_param_type::domFloat1x2;
	return ref;
}


daeMetaElement *
domCg_param_type::domFloat1x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float1x2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFloat1x2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFloat1x2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_float1x2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFloat1x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFloat1x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFloat1x3::create(daeInt bytes)
{
	domCg_param_type::domFloat1x3Ref ref = new(bytes) domCg_param_type::domFloat1x3;
	return ref;
}


daeMetaElement *
domCg_param_type::domFloat1x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float1x3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFloat1x3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFloat1x3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_float1x3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFloat1x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFloat1x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFloat1x4::create(daeInt bytes)
{
	domCg_param_type::domFloat1x4Ref ref = new(bytes) domCg_param_type::domFloat1x4;
	return ref;
}


daeMetaElement *
domCg_param_type::domFloat1x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float1x4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFloat1x4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFloat1x4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_float1x4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFloat1x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFloat1x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFloat2x1::create(daeInt bytes)
{
	domCg_param_type::domFloat2x1Ref ref = new(bytes) domCg_param_type::domFloat2x1;
	return ref;
}


daeMetaElement *
domCg_param_type::domFloat2x1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float2x1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFloat2x1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFloat2x1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_float2x1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFloat2x1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFloat2x1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFloat2x2::create(daeInt bytes)
{
	domCg_param_type::domFloat2x2Ref ref = new(bytes) domCg_param_type::domFloat2x2;
	return ref;
}


daeMetaElement *
domCg_param_type::domFloat2x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float2x2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFloat2x2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFloat2x2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_float2x2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFloat2x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFloat2x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFloat2x3::create(daeInt bytes)
{
	domCg_param_type::domFloat2x3Ref ref = new(bytes) domCg_param_type::domFloat2x3;
	return ref;
}


daeMetaElement *
domCg_param_type::domFloat2x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float2x3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFloat2x3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFloat2x3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_float2x3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFloat2x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFloat2x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFloat2x4::create(daeInt bytes)
{
	domCg_param_type::domFloat2x4Ref ref = new(bytes) domCg_param_type::domFloat2x4;
	return ref;
}


daeMetaElement *
domCg_param_type::domFloat2x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float2x4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFloat2x4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFloat2x4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_float2x4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFloat2x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFloat2x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFloat3x1::create(daeInt bytes)
{
	domCg_param_type::domFloat3x1Ref ref = new(bytes) domCg_param_type::domFloat3x1;
	return ref;
}


daeMetaElement *
domCg_param_type::domFloat3x1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float3x1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFloat3x1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFloat3x1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_float3x1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFloat3x1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFloat3x1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFloat3x2::create(daeInt bytes)
{
	domCg_param_type::domFloat3x2Ref ref = new(bytes) domCg_param_type::domFloat3x2;
	return ref;
}


daeMetaElement *
domCg_param_type::domFloat3x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float3x2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFloat3x2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFloat3x2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_float3x2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFloat3x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFloat3x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFloat3x3::create(daeInt bytes)
{
	domCg_param_type::domFloat3x3Ref ref = new(bytes) domCg_param_type::domFloat3x3;
	return ref;
}


daeMetaElement *
domCg_param_type::domFloat3x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float3x3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFloat3x3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFloat3x3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_float3x3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFloat3x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFloat3x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFloat3x4::create(daeInt bytes)
{
	domCg_param_type::domFloat3x4Ref ref = new(bytes) domCg_param_type::domFloat3x4;
	return ref;
}


daeMetaElement *
domCg_param_type::domFloat3x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float3x4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFloat3x4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFloat3x4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_float3x4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFloat3x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFloat3x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFloat4x1::create(daeInt bytes)
{
	domCg_param_type::domFloat4x1Ref ref = new(bytes) domCg_param_type::domFloat4x1;
	return ref;
}


daeMetaElement *
domCg_param_type::domFloat4x1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float4x1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFloat4x1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFloat4x1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_float4x1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFloat4x1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFloat4x1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFloat4x2::create(daeInt bytes)
{
	domCg_param_type::domFloat4x2Ref ref = new(bytes) domCg_param_type::domFloat4x2;
	return ref;
}


daeMetaElement *
domCg_param_type::domFloat4x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float4x2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFloat4x2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFloat4x2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_float4x2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFloat4x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFloat4x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFloat4x3::create(daeInt bytes)
{
	domCg_param_type::domFloat4x3Ref ref = new(bytes) domCg_param_type::domFloat4x3;
	return ref;
}


daeMetaElement *
domCg_param_type::domFloat4x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float4x3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFloat4x3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFloat4x3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_float4x3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFloat4x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFloat4x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFloat4x4::create(daeInt bytes)
{
	domCg_param_type::domFloat4x4Ref ref = new(bytes) domCg_param_type::domFloat4x4;
	return ref;
}


daeMetaElement *
domCg_param_type::domFloat4x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float4x4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFloat4x4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFloat4x4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_float4x4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFloat4x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFloat4x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domInt::create(daeInt bytes)
{
	domCg_param_type::domIntRef ref = new(bytes) domCg_param_type::domInt;
	return ref;
}


daeMetaElement *
domCg_param_type::domInt::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domInt::_Meta);
	_Meta->registerConstructor(domCg_param_type::domInt::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_int"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domInt , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domInt));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domInt1::create(daeInt bytes)
{
	domCg_param_type::domInt1Ref ref = new(bytes) domCg_param_type::domInt1;
	return ref;
}


daeMetaElement *
domCg_param_type::domInt1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domInt1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domInt1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_int1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domInt1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domInt1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domInt2::create(daeInt bytes)
{
	domCg_param_type::domInt2Ref ref = new(bytes) domCg_param_type::domInt2;
	return ref;
}


daeMetaElement *
domCg_param_type::domInt2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domInt2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domInt2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_int2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domInt2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domInt2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domInt3::create(daeInt bytes)
{
	domCg_param_type::domInt3Ref ref = new(bytes) domCg_param_type::domInt3;
	return ref;
}


daeMetaElement *
domCg_param_type::domInt3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domInt3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domInt3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_int3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domInt3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domInt3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domInt4::create(daeInt bytes)
{
	domCg_param_type::domInt4Ref ref = new(bytes) domCg_param_type::domInt4;
	return ref;
}


daeMetaElement *
domCg_param_type::domInt4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domInt4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domInt4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_int4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domInt4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domInt4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domInt1x1::create(daeInt bytes)
{
	domCg_param_type::domInt1x1Ref ref = new(bytes) domCg_param_type::domInt1x1;
	return ref;
}


daeMetaElement *
domCg_param_type::domInt1x1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int1x1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domInt1x1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domInt1x1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_int1x1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domInt1x1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domInt1x1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domInt1x2::create(daeInt bytes)
{
	domCg_param_type::domInt1x2Ref ref = new(bytes) domCg_param_type::domInt1x2;
	return ref;
}


daeMetaElement *
domCg_param_type::domInt1x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int1x2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domInt1x2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domInt1x2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_int1x2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domInt1x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domInt1x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domInt1x3::create(daeInt bytes)
{
	domCg_param_type::domInt1x3Ref ref = new(bytes) domCg_param_type::domInt1x3;
	return ref;
}


daeMetaElement *
domCg_param_type::domInt1x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int1x3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domInt1x3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domInt1x3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_int1x3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domInt1x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domInt1x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domInt1x4::create(daeInt bytes)
{
	domCg_param_type::domInt1x4Ref ref = new(bytes) domCg_param_type::domInt1x4;
	return ref;
}


daeMetaElement *
domCg_param_type::domInt1x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int1x4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domInt1x4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domInt1x4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_int1x4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domInt1x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domInt1x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domInt2x1::create(daeInt bytes)
{
	domCg_param_type::domInt2x1Ref ref = new(bytes) domCg_param_type::domInt2x1;
	return ref;
}


daeMetaElement *
domCg_param_type::domInt2x1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int2x1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domInt2x1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domInt2x1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_int2x1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domInt2x1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domInt2x1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domInt2x2::create(daeInt bytes)
{
	domCg_param_type::domInt2x2Ref ref = new(bytes) domCg_param_type::domInt2x2;
	return ref;
}


daeMetaElement *
domCg_param_type::domInt2x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int2x2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domInt2x2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domInt2x2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_int2x2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domInt2x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domInt2x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domInt2x3::create(daeInt bytes)
{
	domCg_param_type::domInt2x3Ref ref = new(bytes) domCg_param_type::domInt2x3;
	return ref;
}


daeMetaElement *
domCg_param_type::domInt2x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int2x3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domInt2x3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domInt2x3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_int2x3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domInt2x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domInt2x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domInt2x4::create(daeInt bytes)
{
	domCg_param_type::domInt2x4Ref ref = new(bytes) domCg_param_type::domInt2x4;
	return ref;
}


daeMetaElement *
domCg_param_type::domInt2x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int2x4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domInt2x4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domInt2x4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_int2x4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domInt2x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domInt2x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domInt3x1::create(daeInt bytes)
{
	domCg_param_type::domInt3x1Ref ref = new(bytes) domCg_param_type::domInt3x1;
	return ref;
}


daeMetaElement *
domCg_param_type::domInt3x1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int3x1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domInt3x1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domInt3x1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_int3x1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domInt3x1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domInt3x1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domInt3x2::create(daeInt bytes)
{
	domCg_param_type::domInt3x2Ref ref = new(bytes) domCg_param_type::domInt3x2;
	return ref;
}


daeMetaElement *
domCg_param_type::domInt3x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int3x2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domInt3x2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domInt3x2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_int3x2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domInt3x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domInt3x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domInt3x3::create(daeInt bytes)
{
	domCg_param_type::domInt3x3Ref ref = new(bytes) domCg_param_type::domInt3x3;
	return ref;
}


daeMetaElement *
domCg_param_type::domInt3x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int3x3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domInt3x3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domInt3x3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_int3x3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domInt3x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domInt3x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domInt3x4::create(daeInt bytes)
{
	domCg_param_type::domInt3x4Ref ref = new(bytes) domCg_param_type::domInt3x4;
	return ref;
}


daeMetaElement *
domCg_param_type::domInt3x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int3x4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domInt3x4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domInt3x4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_int3x4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domInt3x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domInt3x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domInt4x1::create(daeInt bytes)
{
	domCg_param_type::domInt4x1Ref ref = new(bytes) domCg_param_type::domInt4x1;
	return ref;
}


daeMetaElement *
domCg_param_type::domInt4x1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int4x1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domInt4x1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domInt4x1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_int4x1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domInt4x1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domInt4x1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domInt4x2::create(daeInt bytes)
{
	domCg_param_type::domInt4x2Ref ref = new(bytes) domCg_param_type::domInt4x2;
	return ref;
}


daeMetaElement *
domCg_param_type::domInt4x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int4x2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domInt4x2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domInt4x2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_int4x2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domInt4x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domInt4x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domInt4x3::create(daeInt bytes)
{
	domCg_param_type::domInt4x3Ref ref = new(bytes) domCg_param_type::domInt4x3;
	return ref;
}


daeMetaElement *
domCg_param_type::domInt4x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int4x3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domInt4x3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domInt4x3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_int4x3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domInt4x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domInt4x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domInt4x4::create(daeInt bytes)
{
	domCg_param_type::domInt4x4Ref ref = new(bytes) domCg_param_type::domInt4x4;
	return ref;
}


daeMetaElement *
domCg_param_type::domInt4x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int4x4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domInt4x4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domInt4x4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_int4x4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domInt4x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domInt4x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domHalf::create(daeInt bytes)
{
	domCg_param_type::domHalfRef ref = new(bytes) domCg_param_type::domHalf;
	return ref;
}


daeMetaElement *
domCg_param_type::domHalf::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "half" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domHalf::_Meta);
	_Meta->registerConstructor(domCg_param_type::domHalf::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_half"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domHalf , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domHalf));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domHalf1::create(daeInt bytes)
{
	domCg_param_type::domHalf1Ref ref = new(bytes) domCg_param_type::domHalf1;
	return ref;
}


daeMetaElement *
domCg_param_type::domHalf1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "half1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domHalf1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domHalf1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_half1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domHalf1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domHalf1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domHalf2::create(daeInt bytes)
{
	domCg_param_type::domHalf2Ref ref = new(bytes) domCg_param_type::domHalf2;
	return ref;
}


daeMetaElement *
domCg_param_type::domHalf2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "half2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domHalf2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domHalf2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_half2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domHalf2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domHalf2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domHalf3::create(daeInt bytes)
{
	domCg_param_type::domHalf3Ref ref = new(bytes) domCg_param_type::domHalf3;
	return ref;
}


daeMetaElement *
domCg_param_type::domHalf3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "half3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domHalf3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domHalf3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_half3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domHalf3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domHalf3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domHalf4::create(daeInt bytes)
{
	domCg_param_type::domHalf4Ref ref = new(bytes) domCg_param_type::domHalf4;
	return ref;
}


daeMetaElement *
domCg_param_type::domHalf4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "half4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domHalf4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domHalf4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_half4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domHalf4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domHalf4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domHalf1x1::create(daeInt bytes)
{
	domCg_param_type::domHalf1x1Ref ref = new(bytes) domCg_param_type::domHalf1x1;
	return ref;
}


daeMetaElement *
domCg_param_type::domHalf1x1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "half1x1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domHalf1x1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domHalf1x1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_half1x1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domHalf1x1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domHalf1x1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domHalf1x2::create(daeInt bytes)
{
	domCg_param_type::domHalf1x2Ref ref = new(bytes) domCg_param_type::domHalf1x2;
	return ref;
}


daeMetaElement *
domCg_param_type::domHalf1x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "half1x2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domHalf1x2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domHalf1x2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_half1x2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domHalf1x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domHalf1x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domHalf1x3::create(daeInt bytes)
{
	domCg_param_type::domHalf1x3Ref ref = new(bytes) domCg_param_type::domHalf1x3;
	return ref;
}


daeMetaElement *
domCg_param_type::domHalf1x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "half1x3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domHalf1x3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domHalf1x3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_half1x3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domHalf1x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domHalf1x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domHalf1x4::create(daeInt bytes)
{
	domCg_param_type::domHalf1x4Ref ref = new(bytes) domCg_param_type::domHalf1x4;
	return ref;
}


daeMetaElement *
domCg_param_type::domHalf1x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "half1x4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domHalf1x4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domHalf1x4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_half1x4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domHalf1x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domHalf1x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domHalf2x1::create(daeInt bytes)
{
	domCg_param_type::domHalf2x1Ref ref = new(bytes) domCg_param_type::domHalf2x1;
	return ref;
}


daeMetaElement *
domCg_param_type::domHalf2x1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "half2x1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domHalf2x1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domHalf2x1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_half2x1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domHalf2x1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domHalf2x1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domHalf2x2::create(daeInt bytes)
{
	domCg_param_type::domHalf2x2Ref ref = new(bytes) domCg_param_type::domHalf2x2;
	return ref;
}


daeMetaElement *
domCg_param_type::domHalf2x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "half2x2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domHalf2x2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domHalf2x2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_half2x2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domHalf2x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domHalf2x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domHalf2x3::create(daeInt bytes)
{
	domCg_param_type::domHalf2x3Ref ref = new(bytes) domCg_param_type::domHalf2x3;
	return ref;
}


daeMetaElement *
domCg_param_type::domHalf2x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "half2x3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domHalf2x3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domHalf2x3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_half2x3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domHalf2x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domHalf2x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domHalf2x4::create(daeInt bytes)
{
	domCg_param_type::domHalf2x4Ref ref = new(bytes) domCg_param_type::domHalf2x4;
	return ref;
}


daeMetaElement *
domCg_param_type::domHalf2x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "half2x4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domHalf2x4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domHalf2x4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_half2x4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domHalf2x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domHalf2x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domHalf3x1::create(daeInt bytes)
{
	domCg_param_type::domHalf3x1Ref ref = new(bytes) domCg_param_type::domHalf3x1;
	return ref;
}


daeMetaElement *
domCg_param_type::domHalf3x1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "half3x1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domHalf3x1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domHalf3x1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_half3x1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domHalf3x1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domHalf3x1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domHalf3x2::create(daeInt bytes)
{
	domCg_param_type::domHalf3x2Ref ref = new(bytes) domCg_param_type::domHalf3x2;
	return ref;
}


daeMetaElement *
domCg_param_type::domHalf3x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "half3x2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domHalf3x2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domHalf3x2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_half3x2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domHalf3x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domHalf3x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domHalf3x3::create(daeInt bytes)
{
	domCg_param_type::domHalf3x3Ref ref = new(bytes) domCg_param_type::domHalf3x3;
	return ref;
}


daeMetaElement *
domCg_param_type::domHalf3x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "half3x3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domHalf3x3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domHalf3x3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_half3x3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domHalf3x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domHalf3x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domHalf3x4::create(daeInt bytes)
{
	domCg_param_type::domHalf3x4Ref ref = new(bytes) domCg_param_type::domHalf3x4;
	return ref;
}


daeMetaElement *
domCg_param_type::domHalf3x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "half3x4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domHalf3x4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domHalf3x4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_half3x4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domHalf3x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domHalf3x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domHalf4x1::create(daeInt bytes)
{
	domCg_param_type::domHalf4x1Ref ref = new(bytes) domCg_param_type::domHalf4x1;
	return ref;
}


daeMetaElement *
domCg_param_type::domHalf4x1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "half4x1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domHalf4x1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domHalf4x1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_half4x1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domHalf4x1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domHalf4x1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domHalf4x2::create(daeInt bytes)
{
	domCg_param_type::domHalf4x2Ref ref = new(bytes) domCg_param_type::domHalf4x2;
	return ref;
}


daeMetaElement *
domCg_param_type::domHalf4x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "half4x2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domHalf4x2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domHalf4x2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_half4x2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domHalf4x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domHalf4x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domHalf4x3::create(daeInt bytes)
{
	domCg_param_type::domHalf4x3Ref ref = new(bytes) domCg_param_type::domHalf4x3;
	return ref;
}


daeMetaElement *
domCg_param_type::domHalf4x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "half4x3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domHalf4x3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domHalf4x3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_half4x3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domHalf4x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domHalf4x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domHalf4x4::create(daeInt bytes)
{
	domCg_param_type::domHalf4x4Ref ref = new(bytes) domCg_param_type::domHalf4x4;
	return ref;
}


daeMetaElement *
domCg_param_type::domHalf4x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "half4x4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domHalf4x4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domHalf4x4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_half4x4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domHalf4x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domHalf4x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFixed::create(daeInt bytes)
{
	domCg_param_type::domFixedRef ref = new(bytes) domCg_param_type::domFixed;
	return ref;
}


daeMetaElement *
domCg_param_type::domFixed::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fixed" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFixed::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFixed::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_fixed"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFixed , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFixed));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFixed1::create(daeInt bytes)
{
	domCg_param_type::domFixed1Ref ref = new(bytes) domCg_param_type::domFixed1;
	return ref;
}


daeMetaElement *
domCg_param_type::domFixed1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fixed1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFixed1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFixed1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_fixed1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFixed1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFixed1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFixed2::create(daeInt bytes)
{
	domCg_param_type::domFixed2Ref ref = new(bytes) domCg_param_type::domFixed2;
	return ref;
}


daeMetaElement *
domCg_param_type::domFixed2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fixed2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFixed2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFixed2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_fixed2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFixed2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFixed2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFixed3::create(daeInt bytes)
{
	domCg_param_type::domFixed3Ref ref = new(bytes) domCg_param_type::domFixed3;
	return ref;
}


daeMetaElement *
domCg_param_type::domFixed3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fixed3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFixed3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFixed3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_fixed3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFixed3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFixed3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFixed4::create(daeInt bytes)
{
	domCg_param_type::domFixed4Ref ref = new(bytes) domCg_param_type::domFixed4;
	return ref;
}


daeMetaElement *
domCg_param_type::domFixed4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fixed4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFixed4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFixed4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_fixed4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFixed4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFixed4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFixed1x1::create(daeInt bytes)
{
	domCg_param_type::domFixed1x1Ref ref = new(bytes) domCg_param_type::domFixed1x1;
	return ref;
}


daeMetaElement *
domCg_param_type::domFixed1x1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fixed1x1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFixed1x1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFixed1x1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_fixed1x1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFixed1x1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFixed1x1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFixed1x2::create(daeInt bytes)
{
	domCg_param_type::domFixed1x2Ref ref = new(bytes) domCg_param_type::domFixed1x2;
	return ref;
}


daeMetaElement *
domCg_param_type::domFixed1x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fixed1x2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFixed1x2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFixed1x2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_fixed1x2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFixed1x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFixed1x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFixed1x3::create(daeInt bytes)
{
	domCg_param_type::domFixed1x3Ref ref = new(bytes) domCg_param_type::domFixed1x3;
	return ref;
}


daeMetaElement *
domCg_param_type::domFixed1x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fixed1x3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFixed1x3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFixed1x3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_fixed1x3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFixed1x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFixed1x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFixed1x4::create(daeInt bytes)
{
	domCg_param_type::domFixed1x4Ref ref = new(bytes) domCg_param_type::domFixed1x4;
	return ref;
}


daeMetaElement *
domCg_param_type::domFixed1x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fixed1x4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFixed1x4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFixed1x4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_fixed1x4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFixed1x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFixed1x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFixed2x1::create(daeInt bytes)
{
	domCg_param_type::domFixed2x1Ref ref = new(bytes) domCg_param_type::domFixed2x1;
	return ref;
}


daeMetaElement *
domCg_param_type::domFixed2x1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fixed2x1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFixed2x1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFixed2x1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_fixed2x1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFixed2x1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFixed2x1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFixed2x2::create(daeInt bytes)
{
	domCg_param_type::domFixed2x2Ref ref = new(bytes) domCg_param_type::domFixed2x2;
	return ref;
}


daeMetaElement *
domCg_param_type::domFixed2x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fixed2x2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFixed2x2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFixed2x2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_fixed2x2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFixed2x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFixed2x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFixed2x3::create(daeInt bytes)
{
	domCg_param_type::domFixed2x3Ref ref = new(bytes) domCg_param_type::domFixed2x3;
	return ref;
}


daeMetaElement *
domCg_param_type::domFixed2x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fixed2x3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFixed2x3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFixed2x3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_fixed2x3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFixed2x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFixed2x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFixed2x4::create(daeInt bytes)
{
	domCg_param_type::domFixed2x4Ref ref = new(bytes) domCg_param_type::domFixed2x4;
	return ref;
}


daeMetaElement *
domCg_param_type::domFixed2x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fixed2x4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFixed2x4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFixed2x4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_fixed2x4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFixed2x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFixed2x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFixed3x1::create(daeInt bytes)
{
	domCg_param_type::domFixed3x1Ref ref = new(bytes) domCg_param_type::domFixed3x1;
	return ref;
}


daeMetaElement *
domCg_param_type::domFixed3x1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fixed3x1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFixed3x1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFixed3x1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_fixed3x1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFixed3x1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFixed3x1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFixed3x2::create(daeInt bytes)
{
	domCg_param_type::domFixed3x2Ref ref = new(bytes) domCg_param_type::domFixed3x2;
	return ref;
}


daeMetaElement *
domCg_param_type::domFixed3x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fixed3x2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFixed3x2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFixed3x2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_fixed3x2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFixed3x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFixed3x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFixed3x3::create(daeInt bytes)
{
	domCg_param_type::domFixed3x3Ref ref = new(bytes) domCg_param_type::domFixed3x3;
	return ref;
}


daeMetaElement *
domCg_param_type::domFixed3x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fixed3x3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFixed3x3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFixed3x3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_fixed3x3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFixed3x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFixed3x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFixed3x4::create(daeInt bytes)
{
	domCg_param_type::domFixed3x4Ref ref = new(bytes) domCg_param_type::domFixed3x4;
	return ref;
}


daeMetaElement *
domCg_param_type::domFixed3x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fixed3x4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFixed3x4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFixed3x4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_fixed3x4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFixed3x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFixed3x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFixed4x1::create(daeInt bytes)
{
	domCg_param_type::domFixed4x1Ref ref = new(bytes) domCg_param_type::domFixed4x1;
	return ref;
}


daeMetaElement *
domCg_param_type::domFixed4x1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fixed4x1" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFixed4x1::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFixed4x1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_fixed4x1"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFixed4x1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFixed4x1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFixed4x2::create(daeInt bytes)
{
	domCg_param_type::domFixed4x2Ref ref = new(bytes) domCg_param_type::domFixed4x2;
	return ref;
}


daeMetaElement *
domCg_param_type::domFixed4x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fixed4x2" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFixed4x2::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFixed4x2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_fixed4x2"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFixed4x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFixed4x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFixed4x3::create(daeInt bytes)
{
	domCg_param_type::domFixed4x3Ref ref = new(bytes) domCg_param_type::domFixed4x3;
	return ref;
}


daeMetaElement *
domCg_param_type::domFixed4x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fixed4x3" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFixed4x3::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFixed4x3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_fixed4x3"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFixed4x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFixed4x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domFixed4x4::create(daeInt bytes)
{
	domCg_param_type::domFixed4x4Ref ref = new(bytes) domCg_param_type::domFixed4x4;
	return ref;
}


daeMetaElement *
domCg_param_type::domFixed4x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fixed4x4" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domFixed4x4::_Meta);
	_Meta->registerConstructor(domCg_param_type::domFixed4x4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Cg_fixed4x4"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domFixed4x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domFixed4x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domString::create(daeInt bytes)
{
	domCg_param_type::domStringRef ref = new(bytes) domCg_param_type::domString;
	return ref;
}


daeMetaElement *
domCg_param_type::domString::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "string" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domString::_Meta);
	_Meta->registerConstructor(domCg_param_type::domString::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsString"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domString , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domString));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_param_type::domEnum::create(daeInt bytes)
{
	domCg_param_type::domEnumRef ref = new(bytes) domCg_param_type::domEnum;
	return ref;
}


daeMetaElement *
domCg_param_type::domEnum::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "enum" );
	_Meta->setStaticPointerAddress(&domCg_param_type::domEnum::_Meta);
	_Meta->registerConstructor(domCg_param_type::domEnum::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Gl_enumeration"));
		ma->setOffset( daeOffsetOf( domCg_param_type::domEnum , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_param_type::domEnum));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domCg_param_type::_Meta = NULL;
daeMetaElement * domCg_param_type::domBool::_Meta = NULL;
daeMetaElement * domCg_param_type::domBool1::_Meta = NULL;
daeMetaElement * domCg_param_type::domBool2::_Meta = NULL;
daeMetaElement * domCg_param_type::domBool3::_Meta = NULL;
daeMetaElement * domCg_param_type::domBool4::_Meta = NULL;
daeMetaElement * domCg_param_type::domBool1x1::_Meta = NULL;
daeMetaElement * domCg_param_type::domBool1x2::_Meta = NULL;
daeMetaElement * domCg_param_type::domBool1x3::_Meta = NULL;
daeMetaElement * domCg_param_type::domBool1x4::_Meta = NULL;
daeMetaElement * domCg_param_type::domBool2x1::_Meta = NULL;
daeMetaElement * domCg_param_type::domBool2x2::_Meta = NULL;
daeMetaElement * domCg_param_type::domBool2x3::_Meta = NULL;
daeMetaElement * domCg_param_type::domBool2x4::_Meta = NULL;
daeMetaElement * domCg_param_type::domBool3x1::_Meta = NULL;
daeMetaElement * domCg_param_type::domBool3x2::_Meta = NULL;
daeMetaElement * domCg_param_type::domBool3x3::_Meta = NULL;
daeMetaElement * domCg_param_type::domBool3x4::_Meta = NULL;
daeMetaElement * domCg_param_type::domBool4x1::_Meta = NULL;
daeMetaElement * domCg_param_type::domBool4x2::_Meta = NULL;
daeMetaElement * domCg_param_type::domBool4x3::_Meta = NULL;
daeMetaElement * domCg_param_type::domBool4x4::_Meta = NULL;
daeMetaElement * domCg_param_type::domFloat::_Meta = NULL;
daeMetaElement * domCg_param_type::domFloat1::_Meta = NULL;
daeMetaElement * domCg_param_type::domFloat2::_Meta = NULL;
daeMetaElement * domCg_param_type::domFloat3::_Meta = NULL;
daeMetaElement * domCg_param_type::domFloat4::_Meta = NULL;
daeMetaElement * domCg_param_type::domFloat1x1::_Meta = NULL;
daeMetaElement * domCg_param_type::domFloat1x2::_Meta = NULL;
daeMetaElement * domCg_param_type::domFloat1x3::_Meta = NULL;
daeMetaElement * domCg_param_type::domFloat1x4::_Meta = NULL;
daeMetaElement * domCg_param_type::domFloat2x1::_Meta = NULL;
daeMetaElement * domCg_param_type::domFloat2x2::_Meta = NULL;
daeMetaElement * domCg_param_type::domFloat2x3::_Meta = NULL;
daeMetaElement * domCg_param_type::domFloat2x4::_Meta = NULL;
daeMetaElement * domCg_param_type::domFloat3x1::_Meta = NULL;
daeMetaElement * domCg_param_type::domFloat3x2::_Meta = NULL;
daeMetaElement * domCg_param_type::domFloat3x3::_Meta = NULL;
daeMetaElement * domCg_param_type::domFloat3x4::_Meta = NULL;
daeMetaElement * domCg_param_type::domFloat4x1::_Meta = NULL;
daeMetaElement * domCg_param_type::domFloat4x2::_Meta = NULL;
daeMetaElement * domCg_param_type::domFloat4x3::_Meta = NULL;
daeMetaElement * domCg_param_type::domFloat4x4::_Meta = NULL;
daeMetaElement * domCg_param_type::domInt::_Meta = NULL;
daeMetaElement * domCg_param_type::domInt1::_Meta = NULL;
daeMetaElement * domCg_param_type::domInt2::_Meta = NULL;
daeMetaElement * domCg_param_type::domInt3::_Meta = NULL;
daeMetaElement * domCg_param_type::domInt4::_Meta = NULL;
daeMetaElement * domCg_param_type::domInt1x1::_Meta = NULL;
daeMetaElement * domCg_param_type::domInt1x2::_Meta = NULL;
daeMetaElement * domCg_param_type::domInt1x3::_Meta = NULL;
daeMetaElement * domCg_param_type::domInt1x4::_Meta = NULL;
daeMetaElement * domCg_param_type::domInt2x1::_Meta = NULL;
daeMetaElement * domCg_param_type::domInt2x2::_Meta = NULL;
daeMetaElement * domCg_param_type::domInt2x3::_Meta = NULL;
daeMetaElement * domCg_param_type::domInt2x4::_Meta = NULL;
daeMetaElement * domCg_param_type::domInt3x1::_Meta = NULL;
daeMetaElement * domCg_param_type::domInt3x2::_Meta = NULL;
daeMetaElement * domCg_param_type::domInt3x3::_Meta = NULL;
daeMetaElement * domCg_param_type::domInt3x4::_Meta = NULL;
daeMetaElement * domCg_param_type::domInt4x1::_Meta = NULL;
daeMetaElement * domCg_param_type::domInt4x2::_Meta = NULL;
daeMetaElement * domCg_param_type::domInt4x3::_Meta = NULL;
daeMetaElement * domCg_param_type::domInt4x4::_Meta = NULL;
daeMetaElement * domCg_param_type::domHalf::_Meta = NULL;
daeMetaElement * domCg_param_type::domHalf1::_Meta = NULL;
daeMetaElement * domCg_param_type::domHalf2::_Meta = NULL;
daeMetaElement * domCg_param_type::domHalf3::_Meta = NULL;
daeMetaElement * domCg_param_type::domHalf4::_Meta = NULL;
daeMetaElement * domCg_param_type::domHalf1x1::_Meta = NULL;
daeMetaElement * domCg_param_type::domHalf1x2::_Meta = NULL;
daeMetaElement * domCg_param_type::domHalf1x3::_Meta = NULL;
daeMetaElement * domCg_param_type::domHalf1x4::_Meta = NULL;
daeMetaElement * domCg_param_type::domHalf2x1::_Meta = NULL;
daeMetaElement * domCg_param_type::domHalf2x2::_Meta = NULL;
daeMetaElement * domCg_param_type::domHalf2x3::_Meta = NULL;
daeMetaElement * domCg_param_type::domHalf2x4::_Meta = NULL;
daeMetaElement * domCg_param_type::domHalf3x1::_Meta = NULL;
daeMetaElement * domCg_param_type::domHalf3x2::_Meta = NULL;
daeMetaElement * domCg_param_type::domHalf3x3::_Meta = NULL;
daeMetaElement * domCg_param_type::domHalf3x4::_Meta = NULL;
daeMetaElement * domCg_param_type::domHalf4x1::_Meta = NULL;
daeMetaElement * domCg_param_type::domHalf4x2::_Meta = NULL;
daeMetaElement * domCg_param_type::domHalf4x3::_Meta = NULL;
daeMetaElement * domCg_param_type::domHalf4x4::_Meta = NULL;
daeMetaElement * domCg_param_type::domFixed::_Meta = NULL;
daeMetaElement * domCg_param_type::domFixed1::_Meta = NULL;
daeMetaElement * domCg_param_type::domFixed2::_Meta = NULL;
daeMetaElement * domCg_param_type::domFixed3::_Meta = NULL;
daeMetaElement * domCg_param_type::domFixed4::_Meta = NULL;
daeMetaElement * domCg_param_type::domFixed1x1::_Meta = NULL;
daeMetaElement * domCg_param_type::domFixed1x2::_Meta = NULL;
daeMetaElement * domCg_param_type::domFixed1x3::_Meta = NULL;
daeMetaElement * domCg_param_type::domFixed1x4::_Meta = NULL;
daeMetaElement * domCg_param_type::domFixed2x1::_Meta = NULL;
daeMetaElement * domCg_param_type::domFixed2x2::_Meta = NULL;
daeMetaElement * domCg_param_type::domFixed2x3::_Meta = NULL;
daeMetaElement * domCg_param_type::domFixed2x4::_Meta = NULL;
daeMetaElement * domCg_param_type::domFixed3x1::_Meta = NULL;
daeMetaElement * domCg_param_type::domFixed3x2::_Meta = NULL;
daeMetaElement * domCg_param_type::domFixed3x3::_Meta = NULL;
daeMetaElement * domCg_param_type::domFixed3x4::_Meta = NULL;
daeMetaElement * domCg_param_type::domFixed4x1::_Meta = NULL;
daeMetaElement * domCg_param_type::domFixed4x2::_Meta = NULL;
daeMetaElement * domCg_param_type::domFixed4x3::_Meta = NULL;
daeMetaElement * domCg_param_type::domFixed4x4::_Meta = NULL;
daeMetaElement * domCg_param_type::domString::_Meta = NULL;
daeMetaElement * domCg_param_type::domEnum::_Meta = NULL;


