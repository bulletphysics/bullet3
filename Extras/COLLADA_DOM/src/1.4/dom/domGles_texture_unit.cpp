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
#include <dom/domGles_texture_unit.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domGles_texture_unit::create(daeInt bytes)
{
	domGles_texture_unitRef ref = new(bytes) domGles_texture_unit;
	return ref;
}


daeMetaElement *
domGles_texture_unit::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "gles_texture_unit" );
	_Meta->registerClass(domGles_texture_unit::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "surface" );
	mea->setOffset( daeOffsetOf(domGles_texture_unit,elemSurface) );
	mea->setElementType( domGles_texture_unit::domSurface::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "sampler_state" );
	mea->setOffset( daeOffsetOf(domGles_texture_unit,elemSampler_state) );
	mea->setElementType( domGles_texture_unit::domSampler_state::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 0, 1 );
	mea->setName( "texcoord" );
	mea->setOffset( daeOffsetOf(domGles_texture_unit,elemTexcoord) );
	mea->setElementType( domGles_texture_unit::domTexcoord::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domGles_texture_unit,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 3 );
	_Meta->setCMRoot( cm );	

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_texture_unit , attrSid ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_texture_unit));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_texture_unit::domSurface::create(daeInt bytes)
{
	domGles_texture_unit::domSurfaceRef ref = new(bytes) domGles_texture_unit::domSurface;
	return ref;
}


daeMetaElement *
domGles_texture_unit::domSurface::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "surface" );
	_Meta->registerClass(domGles_texture_unit::domSurface::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_texture_unit::domSurface , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_texture_unit::domSurface));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_texture_unit::domSampler_state::create(daeInt bytes)
{
	domGles_texture_unit::domSampler_stateRef ref = new(bytes) domGles_texture_unit::domSampler_state;
	return ref;
}


daeMetaElement *
domGles_texture_unit::domSampler_state::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "sampler_state" );
	_Meta->registerClass(domGles_texture_unit::domSampler_state::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_texture_unit::domSampler_state , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_texture_unit::domSampler_state));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_texture_unit::domTexcoord::create(daeInt bytes)
{
	domGles_texture_unit::domTexcoordRef ref = new(bytes) domGles_texture_unit::domTexcoord;
	return ref;
}


daeMetaElement *
domGles_texture_unit::domTexcoord::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "texcoord" );
	_Meta->registerClass(domGles_texture_unit::domTexcoord::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: semantic
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "semantic" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_texture_unit::domTexcoord , attrSemantic ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_texture_unit::domTexcoord));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domGles_texture_unit::_Meta = NULL;
daeMetaElement * domGles_texture_unit::domSurface::_Meta = NULL;
daeMetaElement * domGles_texture_unit::domSampler_state::_Meta = NULL;
daeMetaElement * domGles_texture_unit::domTexcoord::_Meta = NULL;


