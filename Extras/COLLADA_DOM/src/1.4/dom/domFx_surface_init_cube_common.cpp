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
#include <dom/domFx_surface_init_cube_common.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domFx_surface_init_cube_common::create(daeInt bytes)
{
	domFx_surface_init_cube_commonRef ref = new(bytes) domFx_surface_init_cube_common;
	return ref;
}


daeMetaElement *
domFx_surface_init_cube_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fx_surface_init_cube_common" );
	_Meta->registerClass(domFx_surface_init_cube_common::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "all" );
	mea->setOffset( daeOffsetOf(domFx_surface_init_cube_common,elemAll) );
	mea->setElementType( domFx_surface_init_cube_common::domAll::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "primary" );
	mea->setOffset( daeOffsetOf(domFx_surface_init_cube_common,elemPrimary) );
	mea->setElementType( domFx_surface_init_cube_common::domPrimary::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 6, 6 );
	mea->setName( "face" );
	mea->setOffset( daeOffsetOf(domFx_surface_init_cube_common,elemFace_array) );
	mea->setElementType( domFx_surface_init_cube_common::domFace::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domFx_surface_init_cube_common,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domFx_surface_init_cube_common,_contentsOrder));

	
	
	_Meta->setElementSize(sizeof(domFx_surface_init_cube_common));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_surface_init_cube_common::domAll::create(daeInt bytes)
{
	domFx_surface_init_cube_common::domAllRef ref = new(bytes) domFx_surface_init_cube_common::domAll;
	return ref;
}


daeMetaElement *
domFx_surface_init_cube_common::domAll::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "all" );
	_Meta->registerClass(domFx_surface_init_cube_common::domAll::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: ref
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( daeAtomicType::get("xsIDREF"));
		ma->setOffset( daeOffsetOf( domFx_surface_init_cube_common::domAll , attrRef ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_surface_init_cube_common::domAll));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_surface_init_cube_common::domPrimary::create(daeInt bytes)
{
	domFx_surface_init_cube_common::domPrimaryRef ref = new(bytes) domFx_surface_init_cube_common::domPrimary;
	return ref;
}


daeMetaElement *
domFx_surface_init_cube_common::domPrimary::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "primary" );
	_Meta->registerClass(domFx_surface_init_cube_common::domPrimary::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 0, 1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 6, 6 );
	mea->setName( "order" );
	mea->setOffset( daeOffsetOf(domFx_surface_init_cube_common::domPrimary,elemOrder_array) );
	mea->setElementType( domFx_surface_init_cube_common::domPrimary::domOrder::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	_Meta->setCMRoot( cm );	

	//	Add attribute: ref
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( daeAtomicType::get("xsIDREF"));
		ma->setOffset( daeOffsetOf( domFx_surface_init_cube_common::domPrimary , attrRef ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_surface_init_cube_common::domPrimary));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_surface_init_cube_common::domPrimary::domOrder::create(daeInt bytes)
{
	domFx_surface_init_cube_common::domPrimary::domOrderRef ref = new(bytes) domFx_surface_init_cube_common::domPrimary::domOrder;
	return ref;
}


daeMetaElement *
domFx_surface_init_cube_common::domPrimary::domOrder::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "order" );
	_Meta->registerClass(domFx_surface_init_cube_common::domPrimary::domOrder::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_surface_face_enum"));
		ma->setOffset( daeOffsetOf( domFx_surface_init_cube_common::domPrimary::domOrder , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_surface_init_cube_common::domPrimary::domOrder));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_surface_init_cube_common::domFace::create(daeInt bytes)
{
	domFx_surface_init_cube_common::domFaceRef ref = new(bytes) domFx_surface_init_cube_common::domFace;
	return ref;
}


daeMetaElement *
domFx_surface_init_cube_common::domFace::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "face" );
	_Meta->registerClass(domFx_surface_init_cube_common::domFace::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: ref
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( daeAtomicType::get("xsIDREF"));
		ma->setOffset( daeOffsetOf( domFx_surface_init_cube_common::domFace , attrRef ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_surface_init_cube_common::domFace));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domFx_surface_init_cube_common::_Meta = NULL;
daeMetaElement * domFx_surface_init_cube_common::domAll::_Meta = NULL;
daeMetaElement * domFx_surface_init_cube_common::domPrimary::_Meta = NULL;
daeMetaElement * domFx_surface_init_cube_common::domPrimary::domOrder::_Meta = NULL;
daeMetaElement * domFx_surface_init_cube_common::domFace::_Meta = NULL;


