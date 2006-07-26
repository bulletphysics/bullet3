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
#include <dom/domFx_surface_common.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domFx_surface_common::create(daeInt bytes)
{
	domFx_surface_commonRef ref = new(bytes) domFx_surface_common;
	return ref;
}


daeMetaElement *
domFx_surface_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fx_surface_common" );
	_Meta->registerConstructor(domFx_surface_common::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "fx_surface_init_common" );
	mea->setOffset( daeOffsetOf(domFx_surface_common,elemFx_surface_init_common) );
	mea->setElementType( domFx_surface_init_common::registerElement() );
	cm->appendChild( new daeMetaGroup( mea, _Meta, cm, 0, 0, 1 ) );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "format" );
	mea->setOffset( daeOffsetOf(domFx_surface_common,elemFormat) );
	mea->setElementType( domFx_surface_common::domFormat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 0, 1 );
	mea->setName( "format_hint" );
	mea->setOffset( daeOffsetOf(domFx_surface_common,elemFormat_hint) );
	mea->setElementType( domFx_surface_format_hint_common::registerElement() );
	cm->appendChild( mea );
	
	cm = new daeMetaChoice( _Meta, cm, 3, 0, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "size" );
	mea->setOffset( daeOffsetOf(domFx_surface_common,elemSize) );
	mea->setElementType( domFx_surface_common::domSize::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "viewport_ratio" );
	mea->setOffset( daeOffsetOf(domFx_surface_common,elemViewport_ratio) );
	mea->setElementType( domFx_surface_common::domViewport_ratio::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	mea = new daeMetaElementAttribute( _Meta, cm, 4, 0, 1 );
	mea->setName( "mip_levels" );
	mea->setOffset( daeOffsetOf(domFx_surface_common,elemMip_levels) );
	mea->setElementType( domFx_surface_common::domMip_levels::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 5, 0, 1 );
	mea->setName( "mipmap_generate" );
	mea->setOffset( daeOffsetOf(domFx_surface_common,elemMipmap_generate) );
	mea->setElementType( domFx_surface_common::domMipmap_generate::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 6, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domFx_surface_common,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 6 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domFx_surface_common,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domFx_surface_common,_contentsOrder));


	//	Add attribute: type
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "type" );
		ma->setType( daeAtomicType::get("Fx_surface_type_enum"));
		ma->setOffset( daeOffsetOf( domFx_surface_common , attrType ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_surface_common));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_surface_common::domFormat::create(daeInt bytes)
{
	domFx_surface_common::domFormatRef ref = new(bytes) domFx_surface_common::domFormat;
	return ref;
}


daeMetaElement *
domFx_surface_common::domFormat::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "format" );
	_Meta->registerConstructor(domFx_surface_common::domFormat::create);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsToken"));
		ma->setOffset( daeOffsetOf( domFx_surface_common::domFormat , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_surface_common::domFormat));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_surface_common::domSize::create(daeInt bytes)
{
	domFx_surface_common::domSizeRef ref = new(bytes) domFx_surface_common::domSize;
	return ref;
}


daeMetaElement *
domFx_surface_common::domSize::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "size" );
	_Meta->registerConstructor(domFx_surface_common::domSize::create);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Int3"));
		ma->setOffset( daeOffsetOf( domFx_surface_common::domSize , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_surface_common::domSize));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_surface_common::domViewport_ratio::create(daeInt bytes)
{
	domFx_surface_common::domViewport_ratioRef ref = new(bytes) domFx_surface_common::domViewport_ratio;
	return ref;
}


daeMetaElement *
domFx_surface_common::domViewport_ratio::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "viewport_ratio" );
	_Meta->registerConstructor(domFx_surface_common::domViewport_ratio::create);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float2"));
		ma->setOffset( daeOffsetOf( domFx_surface_common::domViewport_ratio , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_surface_common::domViewport_ratio));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_surface_common::domMip_levels::create(daeInt bytes)
{
	domFx_surface_common::domMip_levelsRef ref = new(bytes) domFx_surface_common::domMip_levels;
	return ref;
}


daeMetaElement *
domFx_surface_common::domMip_levels::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "mip_levels" );
	_Meta->registerConstructor(domFx_surface_common::domMip_levels::create);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsUnsignedInt"));
		ma->setOffset( daeOffsetOf( domFx_surface_common::domMip_levels , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_surface_common::domMip_levels));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_surface_common::domMipmap_generate::create(daeInt bytes)
{
	domFx_surface_common::domMipmap_generateRef ref = new(bytes) domFx_surface_common::domMipmap_generate;
	return ref;
}


daeMetaElement *
domFx_surface_common::domMipmap_generate::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "mipmap_generate" );
	_Meta->registerConstructor(domFx_surface_common::domMipmap_generate::create);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsBoolean"));
		ma->setOffset( daeOffsetOf( domFx_surface_common::domMipmap_generate , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_surface_common::domMipmap_generate));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domFx_surface_common::_Meta = NULL;
daeMetaElement * domFx_surface_common::domFormat::_Meta = NULL;
daeMetaElement * domFx_surface_common::domSize::_Meta = NULL;
daeMetaElement * domFx_surface_common::domViewport_ratio::_Meta = NULL;
daeMetaElement * domFx_surface_common::domMip_levels::_Meta = NULL;
daeMetaElement * domFx_surface_common::domMipmap_generate::_Meta = NULL;


//Backwards Compatibility functions
domFx_surface_common_complexType::domInit_from_Array &domFx_surface_common_complexType::getInit_from_array() { 
	if (elemFx_surface_init_common != NULL ) {
		return elemFx_surface_init_common->getInit_from_array(); 
	}
	return emptyArray;
}

const domFx_surface_common_complexType::domInit_from_Array &domFx_surface_common_complexType::getInit_from_array() const { 
	if (elemFx_surface_init_common != NULL ) {
		return elemFx_surface_init_common->getInit_from_array(); 
	}
	return emptyArray;
}

