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
#include <dom/domGlsl_param_type.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domGlsl_param_type::create(daeInt bytes)
{
	domGlsl_param_typeRef ref = new(bytes) domGlsl_param_type;
	return ref;
}


daeMetaElement *
domGlsl_param_type::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "glsl_param_type" );
	_Meta->registerClass(domGlsl_param_type::create, &_Meta);

	_Meta->setIsTransparent( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "bool" );
	mea->setOffset( daeOffsetOf(domGlsl_param_type,elemBool) );
	mea->setElementType( domGlsl_param_type::domBool::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "bool2" );
	mea->setOffset( daeOffsetOf(domGlsl_param_type,elemBool2) );
	mea->setElementType( domGlsl_param_type::domBool2::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "bool3" );
	mea->setOffset( daeOffsetOf(domGlsl_param_type,elemBool3) );
	mea->setElementType( domGlsl_param_type::domBool3::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "bool4" );
	mea->setOffset( daeOffsetOf(domGlsl_param_type,elemBool4) );
	mea->setElementType( domGlsl_param_type::domBool4::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float" );
	mea->setOffset( daeOffsetOf(domGlsl_param_type,elemFloat) );
	mea->setElementType( domGlsl_param_type::domFloat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float2" );
	mea->setOffset( daeOffsetOf(domGlsl_param_type,elemFloat2) );
	mea->setElementType( domGlsl_param_type::domFloat2::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float3" );
	mea->setOffset( daeOffsetOf(domGlsl_param_type,elemFloat3) );
	mea->setElementType( domGlsl_param_type::domFloat3::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float4" );
	mea->setOffset( daeOffsetOf(domGlsl_param_type,elemFloat4) );
	mea->setElementType( domGlsl_param_type::domFloat4::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float2x2" );
	mea->setOffset( daeOffsetOf(domGlsl_param_type,elemFloat2x2) );
	mea->setElementType( domGlsl_param_type::domFloat2x2::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float3x3" );
	mea->setOffset( daeOffsetOf(domGlsl_param_type,elemFloat3x3) );
	mea->setElementType( domGlsl_param_type::domFloat3x3::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float4x4" );
	mea->setOffset( daeOffsetOf(domGlsl_param_type,elemFloat4x4) );
	mea->setElementType( domGlsl_param_type::domFloat4x4::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "int" );
	mea->setOffset( daeOffsetOf(domGlsl_param_type,elemInt) );
	mea->setElementType( domGlsl_param_type::domInt::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "int2" );
	mea->setOffset( daeOffsetOf(domGlsl_param_type,elemInt2) );
	mea->setElementType( domGlsl_param_type::domInt2::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "int3" );
	mea->setOffset( daeOffsetOf(domGlsl_param_type,elemInt3) );
	mea->setElementType( domGlsl_param_type::domInt3::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "int4" );
	mea->setOffset( daeOffsetOf(domGlsl_param_type,elemInt4) );
	mea->setElementType( domGlsl_param_type::domInt4::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "surface" );
	mea->setOffset( daeOffsetOf(domGlsl_param_type,elemSurface) );
	mea->setElementType( domGlsl_surface_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "sampler1D" );
	mea->setOffset( daeOffsetOf(domGlsl_param_type,elemSampler1D) );
	mea->setElementType( domGl_sampler1D::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "sampler2D" );
	mea->setOffset( daeOffsetOf(domGlsl_param_type,elemSampler2D) );
	mea->setElementType( domGl_sampler2D::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "sampler3D" );
	mea->setOffset( daeOffsetOf(domGlsl_param_type,elemSampler3D) );
	mea->setElementType( domGl_sampler3D::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "samplerCUBE" );
	mea->setOffset( daeOffsetOf(domGlsl_param_type,elemSamplerCUBE) );
	mea->setElementType( domGl_samplerCUBE::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "samplerRECT" );
	mea->setOffset( daeOffsetOf(domGlsl_param_type,elemSamplerRECT) );
	mea->setElementType( domGl_samplerRECT::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "samplerDEPTH" );
	mea->setOffset( daeOffsetOf(domGlsl_param_type,elemSamplerDEPTH) );
	mea->setElementType( domGl_samplerDEPTH::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "enum" );
	mea->setOffset( daeOffsetOf(domGlsl_param_type,elemEnum) );
	mea->setElementType( domGlsl_param_type::domEnum::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domGlsl_param_type,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domGlsl_param_type,_contentsOrder));

	
	
	_Meta->setElementSize(sizeof(domGlsl_param_type));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGlsl_param_type::domBool::create(daeInt bytes)
{
	domGlsl_param_type::domBoolRef ref = new(bytes) domGlsl_param_type::domBool;
	return ref;
}


daeMetaElement *
domGlsl_param_type::domBool::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool" );
	_Meta->registerClass(domGlsl_param_type::domBool::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Glsl_bool"));
		ma->setOffset( daeOffsetOf( domGlsl_param_type::domBool , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGlsl_param_type::domBool));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGlsl_param_type::domBool2::create(daeInt bytes)
{
	domGlsl_param_type::domBool2Ref ref = new(bytes) domGlsl_param_type::domBool2;
	return ref;
}


daeMetaElement *
domGlsl_param_type::domBool2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool2" );
	_Meta->registerClass(domGlsl_param_type::domBool2::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Glsl_bool2"));
		ma->setOffset( daeOffsetOf( domGlsl_param_type::domBool2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGlsl_param_type::domBool2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGlsl_param_type::domBool3::create(daeInt bytes)
{
	domGlsl_param_type::domBool3Ref ref = new(bytes) domGlsl_param_type::domBool3;
	return ref;
}


daeMetaElement *
domGlsl_param_type::domBool3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool3" );
	_Meta->registerClass(domGlsl_param_type::domBool3::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Glsl_bool3"));
		ma->setOffset( daeOffsetOf( domGlsl_param_type::domBool3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGlsl_param_type::domBool3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGlsl_param_type::domBool4::create(daeInt bytes)
{
	domGlsl_param_type::domBool4Ref ref = new(bytes) domGlsl_param_type::domBool4;
	return ref;
}


daeMetaElement *
domGlsl_param_type::domBool4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bool4" );
	_Meta->registerClass(domGlsl_param_type::domBool4::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Glsl_bool4"));
		ma->setOffset( daeOffsetOf( domGlsl_param_type::domBool4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGlsl_param_type::domBool4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGlsl_param_type::domFloat::create(daeInt bytes)
{
	domGlsl_param_type::domFloatRef ref = new(bytes) domGlsl_param_type::domFloat;
	return ref;
}


daeMetaElement *
domGlsl_param_type::domFloat::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float" );
	_Meta->registerClass(domGlsl_param_type::domFloat::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Glsl_float"));
		ma->setOffset( daeOffsetOf( domGlsl_param_type::domFloat , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGlsl_param_type::domFloat));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGlsl_param_type::domFloat2::create(daeInt bytes)
{
	domGlsl_param_type::domFloat2Ref ref = new(bytes) domGlsl_param_type::domFloat2;
	return ref;
}


daeMetaElement *
domGlsl_param_type::domFloat2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float2" );
	_Meta->registerClass(domGlsl_param_type::domFloat2::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Glsl_float2"));
		ma->setOffset( daeOffsetOf( domGlsl_param_type::domFloat2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGlsl_param_type::domFloat2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGlsl_param_type::domFloat3::create(daeInt bytes)
{
	domGlsl_param_type::domFloat3Ref ref = new(bytes) domGlsl_param_type::domFloat3;
	return ref;
}


daeMetaElement *
domGlsl_param_type::domFloat3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float3" );
	_Meta->registerClass(domGlsl_param_type::domFloat3::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Glsl_float3"));
		ma->setOffset( daeOffsetOf( domGlsl_param_type::domFloat3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGlsl_param_type::domFloat3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGlsl_param_type::domFloat4::create(daeInt bytes)
{
	domGlsl_param_type::domFloat4Ref ref = new(bytes) domGlsl_param_type::domFloat4;
	return ref;
}


daeMetaElement *
domGlsl_param_type::domFloat4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float4" );
	_Meta->registerClass(domGlsl_param_type::domFloat4::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Glsl_float4"));
		ma->setOffset( daeOffsetOf( domGlsl_param_type::domFloat4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGlsl_param_type::domFloat4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGlsl_param_type::domFloat2x2::create(daeInt bytes)
{
	domGlsl_param_type::domFloat2x2Ref ref = new(bytes) domGlsl_param_type::domFloat2x2;
	return ref;
}


daeMetaElement *
domGlsl_param_type::domFloat2x2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float2x2" );
	_Meta->registerClass(domGlsl_param_type::domFloat2x2::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Glsl_float2x2"));
		ma->setOffset( daeOffsetOf( domGlsl_param_type::domFloat2x2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGlsl_param_type::domFloat2x2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGlsl_param_type::domFloat3x3::create(daeInt bytes)
{
	domGlsl_param_type::domFloat3x3Ref ref = new(bytes) domGlsl_param_type::domFloat3x3;
	return ref;
}


daeMetaElement *
domGlsl_param_type::domFloat3x3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float3x3" );
	_Meta->registerClass(domGlsl_param_type::domFloat3x3::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Glsl_float3x3"));
		ma->setOffset( daeOffsetOf( domGlsl_param_type::domFloat3x3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGlsl_param_type::domFloat3x3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGlsl_param_type::domFloat4x4::create(daeInt bytes)
{
	domGlsl_param_type::domFloat4x4Ref ref = new(bytes) domGlsl_param_type::domFloat4x4;
	return ref;
}


daeMetaElement *
domGlsl_param_type::domFloat4x4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float4x4" );
	_Meta->registerClass(domGlsl_param_type::domFloat4x4::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Glsl_float4x4"));
		ma->setOffset( daeOffsetOf( domGlsl_param_type::domFloat4x4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGlsl_param_type::domFloat4x4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGlsl_param_type::domInt::create(daeInt bytes)
{
	domGlsl_param_type::domIntRef ref = new(bytes) domGlsl_param_type::domInt;
	return ref;
}


daeMetaElement *
domGlsl_param_type::domInt::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int" );
	_Meta->registerClass(domGlsl_param_type::domInt::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Glsl_int"));
		ma->setOffset( daeOffsetOf( domGlsl_param_type::domInt , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGlsl_param_type::domInt));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGlsl_param_type::domInt2::create(daeInt bytes)
{
	domGlsl_param_type::domInt2Ref ref = new(bytes) domGlsl_param_type::domInt2;
	return ref;
}


daeMetaElement *
domGlsl_param_type::domInt2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int2" );
	_Meta->registerClass(domGlsl_param_type::domInt2::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Glsl_int2"));
		ma->setOffset( daeOffsetOf( domGlsl_param_type::domInt2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGlsl_param_type::domInt2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGlsl_param_type::domInt3::create(daeInt bytes)
{
	domGlsl_param_type::domInt3Ref ref = new(bytes) domGlsl_param_type::domInt3;
	return ref;
}


daeMetaElement *
domGlsl_param_type::domInt3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int3" );
	_Meta->registerClass(domGlsl_param_type::domInt3::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Glsl_int3"));
		ma->setOffset( daeOffsetOf( domGlsl_param_type::domInt3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGlsl_param_type::domInt3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGlsl_param_type::domInt4::create(daeInt bytes)
{
	domGlsl_param_type::domInt4Ref ref = new(bytes) domGlsl_param_type::domInt4;
	return ref;
}


daeMetaElement *
domGlsl_param_type::domInt4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int4" );
	_Meta->registerClass(domGlsl_param_type::domInt4::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Glsl_int4"));
		ma->setOffset( daeOffsetOf( domGlsl_param_type::domInt4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGlsl_param_type::domInt4));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGlsl_param_type::domEnum::create(daeInt bytes)
{
	domGlsl_param_type::domEnumRef ref = new(bytes) domGlsl_param_type::domEnum;
	return ref;
}


daeMetaElement *
domGlsl_param_type::domEnum::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "enum" );
	_Meta->registerClass(domGlsl_param_type::domEnum::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Gl_enumeration"));
		ma->setOffset( daeOffsetOf( domGlsl_param_type::domEnum , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGlsl_param_type::domEnum));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domGlsl_param_type::_Meta = NULL;
daeMetaElement * domGlsl_param_type::domBool::_Meta = NULL;
daeMetaElement * domGlsl_param_type::domBool2::_Meta = NULL;
daeMetaElement * domGlsl_param_type::domBool3::_Meta = NULL;
daeMetaElement * domGlsl_param_type::domBool4::_Meta = NULL;
daeMetaElement * domGlsl_param_type::domFloat::_Meta = NULL;
daeMetaElement * domGlsl_param_type::domFloat2::_Meta = NULL;
daeMetaElement * domGlsl_param_type::domFloat3::_Meta = NULL;
daeMetaElement * domGlsl_param_type::domFloat4::_Meta = NULL;
daeMetaElement * domGlsl_param_type::domFloat2x2::_Meta = NULL;
daeMetaElement * domGlsl_param_type::domFloat3x3::_Meta = NULL;
daeMetaElement * domGlsl_param_type::domFloat4x4::_Meta = NULL;
daeMetaElement * domGlsl_param_type::domInt::_Meta = NULL;
daeMetaElement * domGlsl_param_type::domInt2::_Meta = NULL;
daeMetaElement * domGlsl_param_type::domInt3::_Meta = NULL;
daeMetaElement * domGlsl_param_type::domInt4::_Meta = NULL;
daeMetaElement * domGlsl_param_type::domEnum::_Meta = NULL;


