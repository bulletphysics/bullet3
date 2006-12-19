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
#include <dom/domSphere.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domSphere::create(daeInt bytes)
{
	domSphereRef ref = new(bytes) domSphere;
	return ref;
}


daeMetaElement *
domSphere::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "sphere" );
	_Meta->registerClass(domSphere::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "radius" );
	mea->setOffset( daeOffsetOf(domSphere,elemRadius) );
	mea->setElementType( domSphere::domRadius::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 1, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domSphere,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 1 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domSphere));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domSphere::domRadius::create(daeInt bytes)
{
	domSphere::domRadiusRef ref = new(bytes) domSphere::domRadius;
	return ref;
}


daeMetaElement *
domSphere::domRadius::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "radius" );
	_Meta->registerClass(domSphere::domRadius::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domSphere::domRadius , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domSphere::domRadius));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domSphere::_Meta = NULL;
daeMetaElement * domSphere::domRadius::_Meta = NULL;


