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
#include <dom/domSpline.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domSpline::create(daeInt bytes)
{
	domSplineRef ref = new(bytes) domSpline;
	return ref;
}


daeMetaElement *
domSpline::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "spline" );
	_Meta->registerClass(domSpline::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, -1 );
	mea->setName( "source" );
	mea->setOffset( daeOffsetOf(domSpline,elemSource_array) );
	mea->setElementType( domSource::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 1, 1 );
	mea->setName( "control_vertices" );
	mea->setOffset( daeOffsetOf(domSpline,elemControl_vertices) );
	mea->setElementType( domSpline::domControl_vertices::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 2, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domSpline,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 2 );
	_Meta->setCMRoot( cm );	

	//	Add attribute: closed
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "closed" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domSpline , attrClosed ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domSpline));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domSpline::domControl_vertices::create(daeInt bytes)
{
	domSpline::domControl_verticesRef ref = new(bytes) domSpline::domControl_vertices;
	return ref;
}


daeMetaElement *
domSpline::domControl_vertices::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "control_vertices" );
	_Meta->registerClass(domSpline::domControl_vertices::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, -1 );
	mea->setName( "input" );
	mea->setOffset( daeOffsetOf(domSpline::domControl_vertices,elemInput_array) );
	mea->setElementType( domInputLocal::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 1, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domSpline::domControl_vertices,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 1 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domSpline::domControl_vertices));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domSpline::_Meta = NULL;
daeMetaElement * domSpline::domControl_vertices::_Meta = NULL;


