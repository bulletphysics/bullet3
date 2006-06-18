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
	_Meta->setStaticPointerAddress(&domGlsl_param_type::_Meta);
	_Meta->registerConstructor(domGlsl_param_type::create);

	_Meta->setIsTransparent( true );
	// Add elements: bool, bool2, bool3, bool4, float, float2, float3, float4, float2x2, float3x3, float4x4, int, int2, int3, int4, surface, sampler1D, sampler2D, sampler3D, samplerCUBE, samplerRECT, samplerDEPTH, enum
    _Meta->appendElement(domGlsl_param_type::domBool::registerElement(),daeOffsetOf(domGlsl_param_type,elemBool));
    _Meta->appendElement(domGlsl_param_type::domBool2::registerElement(),daeOffsetOf(domGlsl_param_type,elemBool2));
    _Meta->appendElement(domGlsl_param_type::domBool3::registerElement(),daeOffsetOf(domGlsl_param_type,elemBool3));
    _Meta->appendElement(domGlsl_param_type::domBool4::registerElement(),daeOffsetOf(domGlsl_param_type,elemBool4));
    _Meta->appendElement(domGlsl_param_type::domFloat::registerElement(),daeOffsetOf(domGlsl_param_type,elemFloat));
    _Meta->appendElement(domGlsl_param_type::domFloat2::registerElement(),daeOffsetOf(domGlsl_param_type,elemFloat2));
    _Meta->appendElement(domGlsl_param_type::domFloat3::registerElement(),daeOffsetOf(domGlsl_param_type,elemFloat3));
    _Meta->appendElement(domGlsl_param_type::domFloat4::registerElement(),daeOffsetOf(domGlsl_param_type,elemFloat4));
    _Meta->appendElement(domGlsl_param_type::domFloat2x2::registerElement(),daeOffsetOf(domGlsl_param_type,elemFloat2x2));
    _Meta->appendElement(domGlsl_param_type::domFloat3x3::registerElement(),daeOffsetOf(domGlsl_param_type,elemFloat3x3));
    _Meta->appendElement(domGlsl_param_type::domFloat4x4::registerElement(),daeOffsetOf(domGlsl_param_type,elemFloat4x4));
    _Meta->appendElement(domGlsl_param_type::domInt::registerElement(),daeOffsetOf(domGlsl_param_type,elemInt));
    _Meta->appendElement(domGlsl_param_type::domInt2::registerElement(),daeOffsetOf(domGlsl_param_type,elemInt2));
    _Meta->appendElement(domGlsl_param_type::domInt3::registerElement(),daeOffsetOf(domGlsl_param_type,elemInt3));
    _Meta->appendElement(domGlsl_param_type::domInt4::registerElement(),daeOffsetOf(domGlsl_param_type,elemInt4));
    _Meta->appendElement(domFx_surface_common::registerElement(),daeOffsetOf(domGlsl_param_type,elemSurface),"surface"); 
    _Meta->appendElement(domGl_sampler1D::registerElement(),daeOffsetOf(domGlsl_param_type,elemSampler1D),"sampler1D"); 
    _Meta->appendElement(domGl_sampler2D::registerElement(),daeOffsetOf(domGlsl_param_type,elemSampler2D),"sampler2D"); 
    _Meta->appendElement(domGl_sampler3D::registerElement(),daeOffsetOf(domGlsl_param_type,elemSampler3D),"sampler3D"); 
    _Meta->appendElement(domGl_samplerCUBE::registerElement(),daeOffsetOf(domGlsl_param_type,elemSamplerCUBE),"samplerCUBE"); 
    _Meta->appendElement(domGl_samplerRECT::registerElement(),daeOffsetOf(domGlsl_param_type,elemSamplerRECT),"samplerRECT"); 
    _Meta->appendElement(domGl_samplerDEPTH::registerElement(),daeOffsetOf(domGlsl_param_type,elemSamplerDEPTH),"samplerDEPTH"); 
    _Meta->appendElement(domGlsl_param_type::domEnum::registerElement(),daeOffsetOf(domGlsl_param_type,elemEnum));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domGlsl_param_type,_contents));

	
	
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
	_Meta->setStaticPointerAddress(&domGlsl_param_type::domBool::_Meta);
	_Meta->registerConstructor(domGlsl_param_type::domBool::create);

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
	_Meta->setStaticPointerAddress(&domGlsl_param_type::domBool2::_Meta);
	_Meta->registerConstructor(domGlsl_param_type::domBool2::create);

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
	_Meta->setStaticPointerAddress(&domGlsl_param_type::domBool3::_Meta);
	_Meta->registerConstructor(domGlsl_param_type::domBool3::create);

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
	_Meta->setStaticPointerAddress(&domGlsl_param_type::domBool4::_Meta);
	_Meta->registerConstructor(domGlsl_param_type::domBool4::create);

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
	_Meta->setStaticPointerAddress(&domGlsl_param_type::domFloat::_Meta);
	_Meta->registerConstructor(domGlsl_param_type::domFloat::create);

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
	_Meta->setStaticPointerAddress(&domGlsl_param_type::domFloat2::_Meta);
	_Meta->registerConstructor(domGlsl_param_type::domFloat2::create);

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
	_Meta->setStaticPointerAddress(&domGlsl_param_type::domFloat3::_Meta);
	_Meta->registerConstructor(domGlsl_param_type::domFloat3::create);

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
	_Meta->setStaticPointerAddress(&domGlsl_param_type::domFloat4::_Meta);
	_Meta->registerConstructor(domGlsl_param_type::domFloat4::create);

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
	_Meta->setStaticPointerAddress(&domGlsl_param_type::domFloat2x2::_Meta);
	_Meta->registerConstructor(domGlsl_param_type::domFloat2x2::create);

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
	_Meta->setStaticPointerAddress(&domGlsl_param_type::domFloat3x3::_Meta);
	_Meta->registerConstructor(domGlsl_param_type::domFloat3x3::create);

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
	_Meta->setStaticPointerAddress(&domGlsl_param_type::domFloat4x4::_Meta);
	_Meta->registerConstructor(domGlsl_param_type::domFloat4x4::create);

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
	_Meta->setStaticPointerAddress(&domGlsl_param_type::domInt::_Meta);
	_Meta->registerConstructor(domGlsl_param_type::domInt::create);

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
	_Meta->setStaticPointerAddress(&domGlsl_param_type::domInt2::_Meta);
	_Meta->registerConstructor(domGlsl_param_type::domInt2::create);

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
	_Meta->setStaticPointerAddress(&domGlsl_param_type::domInt3::_Meta);
	_Meta->registerConstructor(domGlsl_param_type::domInt3::create);

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
	_Meta->setStaticPointerAddress(&domGlsl_param_type::domInt4::_Meta);
	_Meta->registerConstructor(domGlsl_param_type::domInt4::create);

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
	_Meta->setStaticPointerAddress(&domGlsl_param_type::domEnum::_Meta);
	_Meta->registerConstructor(domGlsl_param_type::domEnum::create);

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


