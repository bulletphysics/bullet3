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
#include <dom/domFx_surface_init_volume_common.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domFx_surface_init_volume_common::create(daeInt bytes)
{
	domFx_surface_init_volume_commonRef ref = new(bytes) domFx_surface_init_volume_common;
	return ref;
}


daeMetaElement *
domFx_surface_init_volume_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fx_surface_init_volume_common" );
	_Meta->registerConstructor(domFx_surface_init_volume_common::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "all" );
	mea->setOffset( daeOffsetOf(domFx_surface_init_volume_common,elemAll) );
	mea->setElementType( domFx_surface_init_volume_common::domAll::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "primary" );
	mea->setOffset( daeOffsetOf(domFx_surface_init_volume_common,elemPrimary) );
	mea->setElementType( domFx_surface_init_volume_common::domPrimary::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domFx_surface_init_volume_common,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domFx_surface_init_volume_common,_contentsOrder));

	
	
	_Meta->setElementSize(sizeof(domFx_surface_init_volume_common));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_surface_init_volume_common::domAll::create(daeInt bytes)
{
	domFx_surface_init_volume_common::domAllRef ref = new(bytes) domFx_surface_init_volume_common::domAll;
	return ref;
}


daeMetaElement *
domFx_surface_init_volume_common::domAll::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "all" );
	_Meta->registerConstructor(domFx_surface_init_volume_common::domAll::create);

	_Meta->setIsInnerClass( true );

	//	Add attribute: ref
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( daeAtomicType::get("xsIDREF"));
		ma->setOffset( daeOffsetOf( domFx_surface_init_volume_common::domAll , attrRef ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_surface_init_volume_common::domAll));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_surface_init_volume_common::domPrimary::create(daeInt bytes)
{
	domFx_surface_init_volume_common::domPrimaryRef ref = new(bytes) domFx_surface_init_volume_common::domPrimary;
	return ref;
}


daeMetaElement *
domFx_surface_init_volume_common::domPrimary::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "primary" );
	_Meta->registerConstructor(domFx_surface_init_volume_common::domPrimary::create);

	_Meta->setIsInnerClass( true );

	//	Add attribute: ref
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( daeAtomicType::get("xsIDREF"));
		ma->setOffset( daeOffsetOf( domFx_surface_init_volume_common::domPrimary , attrRef ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_surface_init_volume_common::domPrimary));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domFx_surface_init_volume_common::_Meta = NULL;
daeMetaElement * domFx_surface_init_volume_common::domAll::_Meta = NULL;
daeMetaElement * domFx_surface_init_volume_common::domPrimary::_Meta = NULL;


