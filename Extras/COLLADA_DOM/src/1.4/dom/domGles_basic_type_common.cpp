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
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

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
	_Meta->registerClass(domGles_basic_type_common::create, &_Meta);

	_Meta->setIsTransparent( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "bool" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemBool) );
	mea->setElementType( domGles_basic_type_common::domBool::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "bool2" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemBool2) );
	mea->setElementType( domGles_basic_type_common::domBool2::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "bool3" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemBool3) );
	mea->setElementType( domGles_basic_type_common::domBool3::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "bool4" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemBool4) );
	mea->setElementType( domGles_basic_type_common::domBool4::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "int" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemInt) );
	mea->setElementType( domGles_basic_type_common::domInt::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "int2" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemInt2) );
	mea->setElementType( domGles_basic_type_common::domInt2::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "int3" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemInt3) );
	mea->setElementType( domGles_basic_type_common::domInt3::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "int4" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemInt4) );
	mea->setElementType( domGles_basic_type_common::domInt4::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemFloat) );
	mea->setElementType( domGles_basic_type_common::domFloat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float2" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemFloat2) );
	mea->setElementType( domGles_basic_type_common::domFloat2::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float3" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemFloat3) );
	mea->setElementType( domGles_basic_type_common::domFloat3::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float4" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemFloat4) );
	mea->setElementType( domGles_basic_type_common::domFloat4::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float1x1" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemFloat1x1) );
	mea->setElementType( domGles_basic_type_common::domFloat1x1::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float1x2" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemFloat1x2) );
	mea->setElementType( domGles_basic_type_common::domFloat1x2::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float1x3" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemFloat1x3) );
	mea->setElementType( domGles_basic_type_common::domFloat1x3::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float1x4" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemFloat1x4) );
	mea->setElementType( domGles_basic_type_common::domFloat1x4::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float2x1" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemFloat2x1) );
	mea->setElementType( domGles_basic_type_common::domFloat2x1::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float2x2" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemFloat2x2) );
	mea->setElementType( domGles_basic_type_common::domFloat2x2::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float2x3" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemFloat2x3) );
	mea->setElementType( domGles_basic_type_common::domFloat2x3::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float2x4" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemFloat2x4) );
	mea->setElementType( domGles_basic_type_common::domFloat2x4::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float3x1" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemFloat3x1) );
	mea->setElementType( domGles_basic_type_common::domFloat3x1::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float3x2" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemFloat3x2) );
	mea->setElementType( domGles_basic_type_common::domFloat3x2::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float3x3" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemFloat3x3) );
	mea->setElementType( domGles_basic_type_common::domFloat3x3::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float3x4" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemFloat3x4) );
	mea->setElementType( domGles_basic_type_common::domFloat3x4::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float4x1" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemFloat4x1) );
	mea->setElementType( domGles_basic_type_common::domFloat4x1::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float4x2" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemFloat4x2) );
	mea->setElementType( domGles_basic_type_common::domFloat4x2::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float4x3" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemFloat4x3) );
	mea->setElementType( domGles_basic_type_common::domFloat4x3::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float4x4" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemFloat4x4) );
	mea->setElementType( domGles_basic_type_common::domFloat4x4::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "surface" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemSurface) );
	mea->setElementType( domFx_surface_common::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "texture_pipeline" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemTexture_pipeline) );
	mea->setElementType( domGles_texture_pipeline::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "sampler_state" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemSampler_state) );
	mea->setElementType( domGles_sampler_state::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "texture_unit" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemTexture_unit) );
	mea->setElementType( domGles_texture_unit::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "enum" );
	mea->setOffset( daeOffsetOf(domGles_basic_type_common,elemEnum) );
	mea->setElementType( domGles_basic_type_common::domEnum::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domGles_basic_type_common,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domGles_basic_type_common,_contentsOrder));

	
	
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
	_Meta->registerClass(domGles_basic_type_common::domBool::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domBool2::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domBool3::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domBool4::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domInt::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domInt2::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domInt3::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domInt4::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domFloat::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domFloat2::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domFloat3::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domFloat4::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domFloat1x1::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domFloat1x2::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domFloat1x3::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domFloat1x4::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domFloat2x1::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domFloat2x2::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domFloat2x3::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domFloat2x4::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domFloat3x1::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domFloat3x2::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domFloat3x3::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domFloat3x4::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domFloat4x1::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domFloat4x2::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domFloat4x3::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domFloat4x4::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domGles_basic_type_common::domEnum::create, &_Meta);

	_Meta->setIsInnerClass( true );
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


