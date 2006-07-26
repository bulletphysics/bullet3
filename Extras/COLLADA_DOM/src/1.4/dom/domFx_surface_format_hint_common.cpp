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
#include <dom/domFx_surface_format_hint_common.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domFx_surface_format_hint_common::create(daeInt bytes)
{
	domFx_surface_format_hint_commonRef ref = new(bytes) domFx_surface_format_hint_common;
	return ref;
}


daeMetaElement *
domFx_surface_format_hint_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fx_surface_format_hint_common" );
	_Meta->registerConstructor(domFx_surface_format_hint_common::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "channels" );
	mea->setOffset( daeOffsetOf(domFx_surface_format_hint_common,elemChannels) );
	mea->setElementType( domFx_surface_format_hint_common::domChannels::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 1, 1 );
	mea->setName( "range" );
	mea->setOffset( daeOffsetOf(domFx_surface_format_hint_common,elemRange) );
	mea->setElementType( domFx_surface_format_hint_common::domRange::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 0, 1 );
	mea->setName( "precision" );
	mea->setOffset( daeOffsetOf(domFx_surface_format_hint_common,elemPrecision) );
	mea->setElementType( domFx_surface_format_hint_common::domPrecision::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3, 0, -1 );
	mea->setName( "option" );
	mea->setOffset( daeOffsetOf(domFx_surface_format_hint_common,elemOption_array) );
	mea->setElementType( domFx_surface_format_hint_common::domOption::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 4, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domFx_surface_format_hint_common,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 4 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domFx_surface_format_hint_common));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_surface_format_hint_common::domChannels::create(daeInt bytes)
{
	domFx_surface_format_hint_common::domChannelsRef ref = new(bytes) domFx_surface_format_hint_common::domChannels;
	return ref;
}


daeMetaElement *
domFx_surface_format_hint_common::domChannels::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "channels" );
	_Meta->registerConstructor(domFx_surface_format_hint_common::domChannels::create);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_surface_format_hint_channels_enum"));
		ma->setOffset( daeOffsetOf( domFx_surface_format_hint_common::domChannels , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_surface_format_hint_common::domChannels));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_surface_format_hint_common::domRange::create(daeInt bytes)
{
	domFx_surface_format_hint_common::domRangeRef ref = new(bytes) domFx_surface_format_hint_common::domRange;
	return ref;
}


daeMetaElement *
domFx_surface_format_hint_common::domRange::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "range" );
	_Meta->registerConstructor(domFx_surface_format_hint_common::domRange::create);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_surface_format_hint_range_enum"));
		ma->setOffset( daeOffsetOf( domFx_surface_format_hint_common::domRange , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_surface_format_hint_common::domRange));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_surface_format_hint_common::domPrecision::create(daeInt bytes)
{
	domFx_surface_format_hint_common::domPrecisionRef ref = new(bytes) domFx_surface_format_hint_common::domPrecision;
	return ref;
}


daeMetaElement *
domFx_surface_format_hint_common::domPrecision::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "precision" );
	_Meta->registerConstructor(domFx_surface_format_hint_common::domPrecision::create);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_surface_format_hint_precision_enum"));
		ma->setOffset( daeOffsetOf( domFx_surface_format_hint_common::domPrecision , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_surface_format_hint_common::domPrecision));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_surface_format_hint_common::domOption::create(daeInt bytes)
{
	domFx_surface_format_hint_common::domOptionRef ref = new(bytes) domFx_surface_format_hint_common::domOption;
	return ref;
}


daeMetaElement *
domFx_surface_format_hint_common::domOption::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "option" );
	_Meta->registerConstructor(domFx_surface_format_hint_common::domOption::create);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_surface_format_hint_option_enum"));
		ma->setOffset( daeOffsetOf( domFx_surface_format_hint_common::domOption , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_surface_format_hint_common::domOption));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domFx_surface_format_hint_common::_Meta = NULL;
daeMetaElement * domFx_surface_format_hint_common::domChannels::_Meta = NULL;
daeMetaElement * domFx_surface_format_hint_common::domRange::_Meta = NULL;
daeMetaElement * domFx_surface_format_hint_common::domPrecision::_Meta = NULL;
daeMetaElement * domFx_surface_format_hint_common::domOption::_Meta = NULL;


