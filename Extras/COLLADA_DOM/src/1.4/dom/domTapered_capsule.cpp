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
#include <dom/domTapered_capsule.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domTapered_capsule::create(daeInt bytes)
{
	domTapered_capsuleRef ref = new(bytes) domTapered_capsule;
	return ref;
}


daeMetaElement *
domTapered_capsule::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "tapered_capsule" );
	_Meta->registerClass(domTapered_capsule::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "height" );
	mea->setOffset( daeOffsetOf(domTapered_capsule,elemHeight) );
	mea->setElementType( domTapered_capsule::domHeight::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 1, 1 );
	mea->setName( "radius1" );
	mea->setOffset( daeOffsetOf(domTapered_capsule,elemRadius1) );
	mea->setElementType( domTapered_capsule::domRadius1::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 1, 1 );
	mea->setName( "radius2" );
	mea->setOffset( daeOffsetOf(domTapered_capsule,elemRadius2) );
	mea->setElementType( domTapered_capsule::domRadius2::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domTapered_capsule,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 3 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domTapered_capsule));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domTapered_capsule::domHeight::create(daeInt bytes)
{
	domTapered_capsule::domHeightRef ref = new(bytes) domTapered_capsule::domHeight;
	return ref;
}


daeMetaElement *
domTapered_capsule::domHeight::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "height" );
	_Meta->registerClass(domTapered_capsule::domHeight::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domTapered_capsule::domHeight , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domTapered_capsule::domHeight));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domTapered_capsule::domRadius1::create(daeInt bytes)
{
	domTapered_capsule::domRadius1Ref ref = new(bytes) domTapered_capsule::domRadius1;
	return ref;
}


daeMetaElement *
domTapered_capsule::domRadius1::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "radius1" );
	_Meta->registerClass(domTapered_capsule::domRadius1::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float2"));
		ma->setOffset( daeOffsetOf( domTapered_capsule::domRadius1 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domTapered_capsule::domRadius1));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domTapered_capsule::domRadius2::create(daeInt bytes)
{
	domTapered_capsule::domRadius2Ref ref = new(bytes) domTapered_capsule::domRadius2;
	return ref;
}


daeMetaElement *
domTapered_capsule::domRadius2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "radius2" );
	_Meta->registerClass(domTapered_capsule::domRadius2::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float2"));
		ma->setOffset( daeOffsetOf( domTapered_capsule::domRadius2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domTapered_capsule::domRadius2));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domTapered_capsule::_Meta = NULL;
daeMetaElement * domTapered_capsule::domHeight::_Meta = NULL;
daeMetaElement * domTapered_capsule::domRadius1::_Meta = NULL;
daeMetaElement * domTapered_capsule::domRadius2::_Meta = NULL;


