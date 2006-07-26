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
#include <dom/domFx_surface_init_common.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domFx_surface_init_common::create(daeInt bytes)
{
	domFx_surface_init_commonRef ref = new(bytes) domFx_surface_init_common;
	return ref;
}


daeMetaElement *
domFx_surface_init_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fx_surface_init_common" );
	_Meta->registerConstructor(domFx_surface_init_common::create);

	_Meta->setIsTransparent( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "init_as_null" );
	mea->setOffset( daeOffsetOf(domFx_surface_init_common,elemInit_as_null) );
	mea->setElementType( domFx_surface_init_common::domInit_as_null::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "init_as_target" );
	mea->setOffset( daeOffsetOf(domFx_surface_init_common,elemInit_as_target) );
	mea->setElementType( domFx_surface_init_common::domInit_as_target::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "init_cube" );
	mea->setOffset( daeOffsetOf(domFx_surface_init_common,elemInit_cube) );
	mea->setElementType( domFx_surface_init_cube_common::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "init_volume" );
	mea->setOffset( daeOffsetOf(domFx_surface_init_common,elemInit_volume) );
	mea->setElementType( domFx_surface_init_volume_common::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "init_planar" );
	mea->setOffset( daeOffsetOf(domFx_surface_init_common,elemInit_planar) );
	mea->setElementType( domFx_surface_init_planar_common::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, -1 );
	mea->setName( "init_from" );
	mea->setOffset( daeOffsetOf(domFx_surface_init_common,elemInit_from_array) );
	mea->setElementType( domFx_surface_init_from_common::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domFx_surface_init_common,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domFx_surface_init_common,_contentsOrder));

	
	
	_Meta->setElementSize(sizeof(domFx_surface_init_common));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_surface_init_common::domInit_as_null::create(daeInt bytes)
{
	domFx_surface_init_common::domInit_as_nullRef ref = new(bytes) domFx_surface_init_common::domInit_as_null;
	return ref;
}


daeMetaElement *
domFx_surface_init_common::domInit_as_null::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "init_as_null" );
	_Meta->registerConstructor(domFx_surface_init_common::domInit_as_null::create);

	_Meta->setIsInnerClass( true );
	
	
	_Meta->setElementSize(sizeof(domFx_surface_init_common::domInit_as_null));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_surface_init_common::domInit_as_target::create(daeInt bytes)
{
	domFx_surface_init_common::domInit_as_targetRef ref = new(bytes) domFx_surface_init_common::domInit_as_target;
	return ref;
}


daeMetaElement *
domFx_surface_init_common::domInit_as_target::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "init_as_target" );
	_Meta->registerConstructor(domFx_surface_init_common::domInit_as_target::create);

	_Meta->setIsInnerClass( true );
	
	
	_Meta->setElementSize(sizeof(domFx_surface_init_common::domInit_as_target));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domFx_surface_init_common::_Meta = NULL;
daeMetaElement * domFx_surface_init_common::domInit_as_null::_Meta = NULL;
daeMetaElement * domFx_surface_init_common::domInit_as_target::_Meta = NULL;


