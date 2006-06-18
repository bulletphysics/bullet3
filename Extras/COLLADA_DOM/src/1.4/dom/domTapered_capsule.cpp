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
	_Meta->setStaticPointerAddress(&domTapered_capsule::_Meta);
	_Meta->registerConstructor(domTapered_capsule::create);

	// Add elements: height, radius1, radius2, extra
    _Meta->appendElement(domTapered_capsule::domHeight::registerElement(),daeOffsetOf(domTapered_capsule,elemHeight));
    _Meta->appendElement(domTapered_capsule::domRadius1::registerElement(),daeOffsetOf(domTapered_capsule,elemRadius1));
    _Meta->appendElement(domTapered_capsule::domRadius2::registerElement(),daeOffsetOf(domTapered_capsule,elemRadius2));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domTapered_capsule,elemExtra_array));
	
	
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
	_Meta->setStaticPointerAddress(&domTapered_capsule::domHeight::_Meta);
	_Meta->registerConstructor(domTapered_capsule::domHeight::create);

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
	_Meta->setStaticPointerAddress(&domTapered_capsule::domRadius1::_Meta);
	_Meta->registerConstructor(domTapered_capsule::domRadius1::create);

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
	_Meta->setStaticPointerAddress(&domTapered_capsule::domRadius2::_Meta);
	_Meta->registerConstructor(domTapered_capsule::domRadius2::create);

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


