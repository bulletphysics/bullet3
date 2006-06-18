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
#include <dom/domGles_basic_type_common.h>

daeElementRef
domGles_basic_type_common::create(daeInt bytes)
{
	domGles_basic_type_commonRef ref = new(bytes) domGles_basic_type_common;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "gles_basic_type_common" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::create);

	_Meta->setIsTransparent( true );
	// Add elements: bool, bool2, bool3, bool4, int, int2, int3, int4, float, float2, float3, float4, float1x1, float1x2, float1x3, float1x4, float2x1, float2x2, float2x3, float2x4, float3x1, float3x2, float3x3, float3x4, float4x1, float4x2, float4x3, float4x4, surface, texture_pipeline, sampler_state, texture_unit, enum
    _Meta->appendElement(domGles_basic_type_common::domBool::registerElement(),daeOffsetOf(domGles_basic_type_common,elemBool));
    _Meta->appendElement(domGles_basic_type_common::domBool2::registerElement(),daeOffsetOf(domGles_basic_type_common,elemBool2));
    _Meta->appendElement(domGles_basic_type_common::domBool3::registerElement(),daeOffsetOf(domGles_basic_type_common,elemBool3));
    _Meta->appendElement(domGles_basic_type_common::domBool4::registerElement(),daeOffsetOf(domGles_basic_type_common,elemBool4));
    _Meta->appendElement(domGles_basic_type_common::domInt::registerElement(),daeOffsetOf(domGles_basic_type_common,elemInt));
    _Meta->appendElement(domGles_basic_type_common::domInt2::registerElement(),daeOffsetOf(domGles_basic_type_common,elemInt2));
    _Meta->appendElement(domGles_basic_type_common::domInt3::registerElement(),daeOffsetOf(domGles_basic_type_common,elemInt3));
    _Meta->appendElement(domGles_basic_type_common::domInt4::registerElement(),daeOffsetOf(domGles_basic_type_common,elemInt4));
    _Meta->appendElement(domGles_basic_type_common::domFloat::registerElement(),daeOffsetOf(domGles_basic_type_common,elemFloat));
    _Meta->appendElement(domGles_basic_type_common::domFloat2::registerElement(),daeOffsetOf(domGles_basic_type_common,elemFloat2));
    _Meta->appendElement(domGles_basic_type_common::domFloat3::registerElement(),daeOffsetOf(domGles_basic_type_common,elemFloat3));
    _Meta->appendElement(domGles_basic_type_common::domFloat4::registerElement(),daeOffsetOf(domGles_basic_type_common,elemFloat4));
    _Meta->appendElement(domGles_basic_type_common::domFloat1x1::registerElement(),daeOffsetOf(domGles_basic_type_common,elemFloat1x1));
    _Meta->appendElement(domGles_basic_type_common::domFloat1x2::registerElement(),daeOffsetOf(domGles_basic_type_common,elemFloat1x2));
    _Meta->appendElement(domGles_basic_type_common::domFloat1x3::registerElement(),daeOffsetOf(domGles_basic_type_common,elemFloat1x3));
    _Meta->appendElement(domGles_basic_type_common::domFloat1x4::registerElement(),daeOffsetOf(domGles_basic_type_common,elemFloat1x4));
    _Meta->appendElement(domGles_basic_type_common::domFloat2x1::registerElement(),daeOffsetOf(domGles_basic_type_common,elemFloat2x1));
    _Meta->appendElement(domGles_basic_type_common::domFloat2x2::registerElement(),daeOffsetOf(domGles_basic_type_common,elemFloat2x2));
    _Meta->appendElement(domGles_basic_type_common::domFloat2x3::registerElement(),daeOffsetOf(domGles_basic_type_common,elemFloat2x3));
    _Meta->appendElement(domGles_basic_type_common::domFloat2x4::registerElement(),daeOffsetOf(domGles_basic_type_common,elemFloat2x4));
    _Meta->appendElement(domGles_basic_type_common::domFloat3x1::registerElement(),daeOffsetOf(domGles_basic_type_common,elemFloat3x1));
    _Meta->appendElement(domGles_basic_type_common::domFloat3x2::registerElement(),daeOffsetOf(domGles_basic_type_common,elemFloat3x2));
    _Meta->appendElement(domGles_basic_type_common::domFloat3x3::registerElement(),daeOffsetOf(domGles_basic_type_common,elemFloat3x3));
    _Meta->appendElement(domGles_basic_type_common::domFloat3x4::registerElement(),daeOffsetOf(domGles_basic_type_common,elemFloat3x4));
    _Meta->appendElement(domGles_basic_type_common::domFloat4x1::registerElement(),daeOffsetOf(domGles_basic_type_common,elemFloat4x1));
    _Meta->appendElement(domGles_basic_type_common::domFloat4x2::registerElement(),daeOffsetOf(domGles_basic_type_common,elemFloat4x2));
    _Meta->appendElement(domGles_basic_type_common::domFloat4x3::registerElement(),daeOffsetOf(domGles_basic_type_common,elemFloat4x3));
    _Meta->appendElement(domGles_basic_type_common::domFloat4x4::registerElement(),daeOffsetOf(domGles_basic_type_common,elemFloat4x4));
    _Meta->appendElement(domFx_surface_common::registerElement(),daeOffsetOf(domGles_basic_type_common,elemSurface),"surface"); 
    _Meta->appendElement(domGles_texture_pipeline::registerElement(),daeOffsetOf(domGles_basic_type_common,elemTexture_pipeline),"texture_pipeline"); 
    _Meta->appendElement(domGles_sampler_state::registerElement(),daeOffsetOf(domGles_basic_type_common,elemSampler_state),"sampler_state"); 
    _Meta->appendElement(domGles_texture_unit::registerElement(),daeOffsetOf(domGles_basic_type_common,elemTexture_unit),"texture_unit"); 
    _Meta->appendElement(domGles_basic_type_common::domEnum::registerElement(),daeOffsetOf(domGles_basic_type_common,elemEnum));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domGles_basic_type_common,_contents));

	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domBool::create(daeInt bytes)
{
	domGles_basic_type_common::domBoolRef ref = new(bytes) domGles_basic_type_common::domBool;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domBool::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domBool::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domBool::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domBool , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domBool));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domBool2::create(daeInt bytes)
{
	domGles_basic_type_common::domBool2Ref ref = new(bytes) domGles_basic_type_common::domBool2;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domBool2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool2" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domBool2::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domBool2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Bool2"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domBool2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domBool2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domBool3::create(daeInt bytes)
{
	domGles_basic_type_common::domBool3Ref ref = new(bytes) domGles_basic_type_common::domBool3;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domBool3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool3" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domBool3::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domBool3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Bool3"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domBool3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domBool3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domBool4::create(daeInt bytes)
{
	domGles_basic_type_common::domBool4Ref ref = new(bytes) domGles_basic_type_common::domBool4;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domBool4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool4" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domBool4::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domBool4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Bool4"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domBool4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domBool4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domInt::create(daeInt bytes)
{
	domGles_basic_type_common::domIntRef ref = new(bytes) domGles_basic_type_common::domInt;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domInt::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domInt::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domInt::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Int"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domInt , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domInt));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domInt2::create(daeInt bytes)
{
	domGles_basic_type_common::domInt2Ref ref = new(bytes) domGles_basic_type_common::domInt2;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domInt2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int2" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domInt2::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domInt2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Int2"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domInt2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domInt2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domInt3::create(daeInt bytes)
{
	domGles_basic_type_common::domInt3Ref ref = new(bytes) domGles_basic_type_common::domInt3;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domInt3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int3" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domInt3::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domInt3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Int3"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domInt3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domInt3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domInt4::create(daeInt bytes)
{
	domGles_basic_type_common::domInt4Ref ref = new(bytes) domGles_basic_type_common::domInt4;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domInt4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int4" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domInt4::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domInt4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Int4"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domInt4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domInt4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domFloat::create(daeInt bytes)
{
	domGles_basic_type_common::domFloatRef ref = new(bytes) domGles_basic_type_common::domFloat;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domFloat::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domFloat::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domFloat::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domFloat , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domFloat));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domFloat2::create(daeInt bytes)
{
	domGles_basic_type_common::domFloat2Ref ref = new(bytes) domGles_basic_type_common::domFloat2;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domFloat2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float2" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domFloat2::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domFloat2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float2"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domFloat2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domFloat2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domFloat3::create(daeInt bytes)
{
	domGles_basic_type_common::domFloat3Ref ref = new(bytes) domGles_basic_type_common::domFloat3;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domFloat3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float3" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domFloat3::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domFloat3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float3"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domFloat3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domFloat3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domFloat4::create(daeInt bytes)
{
	domGles_basic_type_common::domFloat4Ref ref = new(bytes) domGles_basic_type_common::domFloat4;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domFloat4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float4" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domFloat4::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domFloat4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domFloat4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domFloat4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domFloat1x1::create(daeInt bytes)
{
	domGles_basic_type_common::domFloat1x1Ref ref = new(bytes) domGles_basic_type_common::domFloat1x1;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domFloat1x1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float1x1" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domFloat1x1::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domFloat1x1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domFloat1x1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domFloat1x1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domFloat1x2::create(daeInt bytes)
{
	domGles_basic_type_common::domFloat1x2Ref ref = new(bytes) domGles_basic_type_common::domFloat1x2;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domFloat1x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float1x2" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domFloat1x2::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domFloat1x2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float2"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domFloat1x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domFloat1x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domFloat1x3::create(daeInt bytes)
{
	domGles_basic_type_common::domFloat1x3Ref ref = new(bytes) domGles_basic_type_common::domFloat1x3;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domFloat1x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float1x3" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domFloat1x3::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domFloat1x3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float3"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domFloat1x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domFloat1x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domFloat1x4::create(daeInt bytes)
{
	domGles_basic_type_common::domFloat1x4Ref ref = new(bytes) domGles_basic_type_common::domFloat1x4;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domFloat1x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float1x4" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domFloat1x4::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domFloat1x4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domFloat1x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domFloat1x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domFloat2x1::create(daeInt bytes)
{
	domGles_basic_type_common::domFloat2x1Ref ref = new(bytes) domGles_basic_type_common::domFloat2x1;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domFloat2x1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float2x1" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domFloat2x1::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domFloat2x1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float2"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domFloat2x1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domFloat2x1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domFloat2x2::create(daeInt bytes)
{
	domGles_basic_type_common::domFloat2x2Ref ref = new(bytes) domGles_basic_type_common::domFloat2x2;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domFloat2x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float2x2" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domFloat2x2::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domFloat2x2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float2x2"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domFloat2x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domFloat2x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domFloat2x3::create(daeInt bytes)
{
	domGles_basic_type_common::domFloat2x3Ref ref = new(bytes) domGles_basic_type_common::domFloat2x3;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domFloat2x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float2x3" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domFloat2x3::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domFloat2x3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float2x3"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domFloat2x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domFloat2x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domFloat2x4::create(daeInt bytes)
{
	domGles_basic_type_common::domFloat2x4Ref ref = new(bytes) domGles_basic_type_common::domFloat2x4;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domFloat2x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float2x4" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domFloat2x4::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domFloat2x4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float2x4"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domFloat2x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domFloat2x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domFloat3x1::create(daeInt bytes)
{
	domGles_basic_type_common::domFloat3x1Ref ref = new(bytes) domGles_basic_type_common::domFloat3x1;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domFloat3x1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float3x1" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domFloat3x1::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domFloat3x1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float3"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domFloat3x1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domFloat3x1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domFloat3x2::create(daeInt bytes)
{
	domGles_basic_type_common::domFloat3x2Ref ref = new(bytes) domGles_basic_type_common::domFloat3x2;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domFloat3x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float3x2" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domFloat3x2::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domFloat3x2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float3x2"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domFloat3x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domFloat3x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domFloat3x3::create(daeInt bytes)
{
	domGles_basic_type_common::domFloat3x3Ref ref = new(bytes) domGles_basic_type_common::domFloat3x3;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domFloat3x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float3x3" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domFloat3x3::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domFloat3x3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float3x3"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domFloat3x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domFloat3x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domFloat3x4::create(daeInt bytes)
{
	domGles_basic_type_common::domFloat3x4Ref ref = new(bytes) domGles_basic_type_common::domFloat3x4;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domFloat3x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float3x4" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domFloat3x4::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domFloat3x4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float3x4"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domFloat3x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domFloat3x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domFloat4x1::create(daeInt bytes)
{
	domGles_basic_type_common::domFloat4x1Ref ref = new(bytes) domGles_basic_type_common::domFloat4x1;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domFloat4x1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float4x1" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domFloat4x1::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domFloat4x1::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domFloat4x1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domFloat4x1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domFloat4x2::create(daeInt bytes)
{
	domGles_basic_type_common::domFloat4x2Ref ref = new(bytes) domGles_basic_type_common::domFloat4x2;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domFloat4x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float4x2" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domFloat4x2::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domFloat4x2::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float4x2"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domFloat4x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domFloat4x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domFloat4x3::create(daeInt bytes)
{
	domGles_basic_type_common::domFloat4x3Ref ref = new(bytes) domGles_basic_type_common::domFloat4x3;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domFloat4x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float4x3" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domFloat4x3::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domFloat4x3::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float4x3"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domFloat4x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domFloat4x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domFloat4x4::create(daeInt bytes)
{
	domGles_basic_type_common::domFloat4x4Ref ref = new(bytes) domGles_basic_type_common::domFloat4x4;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domFloat4x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float4x4" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domFloat4x4::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domFloat4x4::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float4x4"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domFloat4x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domFloat4x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_basic_type_common::domEnum::create(daeInt bytes)
{
	domGles_basic_type_common::domEnumRef ref = new(bytes) domGles_basic_type_common::domEnum;
	return ref;
}


daeMetaElement *
domGles_basic_type_common::domEnum::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "enum" );
	_Meta->setStaticPointerAddress(&domGles_basic_type_common::domEnum::_Meta);
	_Meta->registerConstructor(domGles_basic_type_common::domEnum::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Gles_enumeration"));
		ma->setOffset( daeOffsetOf( domGles_basic_type_common::domEnum , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_basic_type_common::domEnum));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domGles_basic_type_common::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domBool::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domBool2::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domBool3::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domBool4::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domInt::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domInt2::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domInt3::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domInt4::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domFloat::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domFloat2::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domFloat3::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domFloat4::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domFloat1x1::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domFloat1x2::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domFloat1x3::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domFloat1x4::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domFloat2x1::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domFloat2x2::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domFloat2x3::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domFloat2x4::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domFloat3x1::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domFloat3x2::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domFloat3x3::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domFloat3x4::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domFloat4x1::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domFloat4x2::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domFloat4x3::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domFloat4x4::_Meta = NULL;
daeMetaElement * domGles_basic_type_common::domEnum::_Meta = NULL;


