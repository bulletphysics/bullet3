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
#include <dom/domCamera.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domCamera::create(daeInt bytes)
{
	domCameraRef ref = new(bytes) domCamera;
	return ref;
}


daeMetaElement *
domCamera::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "camera" );
	_Meta->registerConstructor(domCamera::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domCamera,elemAsset) );
	mea->setElementType( domAsset::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 1, 1 );
	mea->setName( "optics" );
	mea->setOffset( daeOffsetOf(domCamera,elemOptics) );
	mea->setElementType( domCamera::domOptics::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 0, 1 );
	mea->setName( "imager" );
	mea->setOffset( daeOffsetOf(domCamera,elemImager) );
	mea->setElementType( domCamera::domImager::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domCamera,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 3 );
	_Meta->setCMRoot( cm );	

	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domCamera , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domCamera , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCamera));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCamera::domOptics::create(daeInt bytes)
{
	domCamera::domOpticsRef ref = new(bytes) domCamera::domOptics;
	return ref;
}


daeMetaElement *
domCamera::domOptics::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "optics" );
	_Meta->registerConstructor(domCamera::domOptics::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "technique_common" );
	mea->setOffset( daeOffsetOf(domCamera::domOptics,elemTechnique_common) );
	mea->setElementType( domCamera::domOptics::domTechnique_common::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 1, 0, -1 );
	mea->setName( "technique" );
	mea->setOffset( daeOffsetOf(domCamera::domOptics,elemTechnique_array) );
	mea->setElementType( domTechnique::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 2, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domCamera::domOptics,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 2 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domCamera::domOptics));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCamera::domOptics::domTechnique_common::create(daeInt bytes)
{
	domCamera::domOptics::domTechnique_commonRef ref = new(bytes) domCamera::domOptics::domTechnique_common;
	return ref;
}


daeMetaElement *
domCamera::domOptics::domTechnique_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "technique_common" );
	_Meta->registerConstructor(domCamera::domOptics::domTechnique_common::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "orthographic" );
	mea->setOffset( daeOffsetOf(domCamera::domOptics::domTechnique_common,elemOrthographic) );
	mea->setElementType( domCamera::domOptics::domTechnique_common::domOrthographic::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "perspective" );
	mea->setOffset( daeOffsetOf(domCamera::domOptics::domTechnique_common,elemPerspective) );
	mea->setElementType( domCamera::domOptics::domTechnique_common::domPerspective::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domCamera::domOptics::domTechnique_common,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domCamera::domOptics::domTechnique_common,_contentsOrder));

	
	
	_Meta->setElementSize(sizeof(domCamera::domOptics::domTechnique_common));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCamera::domOptics::domTechnique_common::domOrthographic::create(daeInt bytes)
{
	domCamera::domOptics::domTechnique_common::domOrthographicRef ref = new(bytes) domCamera::domOptics::domTechnique_common::domOrthographic;
	return ref;
}


daeMetaElement *
domCamera::domOptics::domTechnique_common::domOrthographic::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "orthographic" );
	_Meta->registerConstructor(domCamera::domOptics::domTechnique_common::domOrthographic::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	cm = new daeMetaChoice( _Meta, cm, 0, 1, 1 );

	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "xmag" );
	mea->setOffset( daeOffsetOf(domCamera::domOptics::domTechnique_common::domOrthographic,elemXmag) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	cm = new daeMetaChoice( _Meta, cm, 1, 0, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "ymag" );
	mea->setOffset( daeOffsetOf(domCamera::domOptics::domTechnique_common::domOrthographic,elemYmag) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "aspect_ratio" );
	mea->setOffset( daeOffsetOf(domCamera::domOptics::domTechnique_common::domOrthographic,elemAspect_ratio) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	cm->setMaxOrdinal( 1 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	cm = new daeMetaSequence( _Meta, cm, 2, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "ymag" );
	mea->setOffset( daeOffsetOf(domCamera::domOptics::domTechnique_common::domOrthographic,elemYmag) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "aspect_ratio" );
	mea->setOffset( daeOffsetOf(domCamera::domOptics::domTechnique_common::domOrthographic,elemAspect_ratio) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 1 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	cm->setMaxOrdinal( 3 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 1, 1 );
	mea->setName( "znear" );
	mea->setOffset( daeOffsetOf(domCamera::domOptics::domTechnique_common::domOrthographic,elemZnear) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 1, 1 );
	mea->setName( "zfar" );
	mea->setOffset( daeOffsetOf(domCamera::domOptics::domTechnique_common::domOrthographic,elemZfar) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 2 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domCamera::domOptics::domTechnique_common::domOrthographic,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domCamera::domOptics::domTechnique_common::domOrthographic,_contentsOrder));

	
	
	_Meta->setElementSize(sizeof(domCamera::domOptics::domTechnique_common::domOrthographic));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCamera::domOptics::domTechnique_common::domPerspective::create(daeInt bytes)
{
	domCamera::domOptics::domTechnique_common::domPerspectiveRef ref = new(bytes) domCamera::domOptics::domTechnique_common::domPerspective;
	return ref;
}


daeMetaElement *
domCamera::domOptics::domTechnique_common::domPerspective::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "perspective" );
	_Meta->registerConstructor(domCamera::domOptics::domTechnique_common::domPerspective::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	cm = new daeMetaChoice( _Meta, cm, 0, 1, 1 );

	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "xfov" );
	mea->setOffset( daeOffsetOf(domCamera::domOptics::domTechnique_common::domPerspective,elemXfov) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	cm = new daeMetaChoice( _Meta, cm, 1, 0, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "yfov" );
	mea->setOffset( daeOffsetOf(domCamera::domOptics::domTechnique_common::domPerspective,elemYfov) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "aspect_ratio" );
	mea->setOffset( daeOffsetOf(domCamera::domOptics::domTechnique_common::domPerspective,elemAspect_ratio) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	cm->setMaxOrdinal( 1 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	cm = new daeMetaSequence( _Meta, cm, 2, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "yfov" );
	mea->setOffset( daeOffsetOf(domCamera::domOptics::domTechnique_common::domPerspective,elemYfov) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "aspect_ratio" );
	mea->setOffset( daeOffsetOf(domCamera::domOptics::domTechnique_common::domPerspective,elemAspect_ratio) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 1 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	cm->setMaxOrdinal( 3 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 1, 1 );
	mea->setName( "znear" );
	mea->setOffset( daeOffsetOf(domCamera::domOptics::domTechnique_common::domPerspective,elemZnear) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 1, 1 );
	mea->setName( "zfar" );
	mea->setOffset( daeOffsetOf(domCamera::domOptics::domTechnique_common::domPerspective,elemZfar) );
	mea->setElementType( domTargetableFloat::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 2 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domCamera::domOptics::domTechnique_common::domPerspective,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domCamera::domOptics::domTechnique_common::domPerspective,_contentsOrder));

	
	
	_Meta->setElementSize(sizeof(domCamera::domOptics::domTechnique_common::domPerspective));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCamera::domImager::create(daeInt bytes)
{
	domCamera::domImagerRef ref = new(bytes) domCamera::domImager;
	return ref;
}


daeMetaElement *
domCamera::domImager::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "imager" );
	_Meta->registerConstructor(domCamera::domImager::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, -1 );
	mea->setName( "technique" );
	mea->setOffset( daeOffsetOf(domCamera::domImager,elemTechnique_array) );
	mea->setElementType( domTechnique::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 1, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domCamera::domImager,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 1 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domCamera::domImager));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domCamera::_Meta = NULL;
daeMetaElement * domCamera::domOptics::_Meta = NULL;
daeMetaElement * domCamera::domOptics::domTechnique_common::_Meta = NULL;
daeMetaElement * domCamera::domOptics::domTechnique_common::domOrthographic::_Meta = NULL;
daeMetaElement * domCamera::domOptics::domTechnique_common::domPerspective::_Meta = NULL;
daeMetaElement * domCamera::domImager::_Meta = NULL;


