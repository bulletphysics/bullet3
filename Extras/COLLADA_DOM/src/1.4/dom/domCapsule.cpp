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
#include <dom/domCapsule.h>

daeElementRef
domCapsule::create(daeInt bytes)
{
	domCapsuleRef ref = new(bytes) domCapsule;
	return ref;
}


daeMetaElement *
domCapsule::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "capsule" );
	_Meta->setStaticPointerAddress(&domCapsule::_Meta);
	_Meta->registerConstructor(domCapsule::create);

	// Add elements: height, radius, extra
    _Meta->appendElement(domCapsule::domHeight::registerElement(),daeOffsetOf(domCapsule,elemHeight));
    _Meta->appendElement(domCapsule::domRadius::registerElement(),daeOffsetOf(domCapsule,elemRadius));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domCapsule,elemExtra_array));
	
	
	_Meta->setElementSize(sizeof(domCapsule));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCapsule::domHeight::create(daeInt bytes)
{
	domCapsule::domHeightRef ref = new(bytes) domCapsule::domHeight;
	return ref;
}


daeMetaElement *
domCapsule::domHeight::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "height" );
	_Meta->setStaticPointerAddress(&domCapsule::domHeight::_Meta);
	_Meta->registerConstructor(domCapsule::domHeight::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domCapsule::domHeight , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCapsule::domHeight));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCapsule::domRadius::create(daeInt bytes)
{
	domCapsule::domRadiusRef ref = new(bytes) domCapsule::domRadius;
	return ref;
}


daeMetaElement *
domCapsule::domRadius::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "radius" );
	_Meta->setStaticPointerAddress(&domCapsule::domRadius::_Meta);
	_Meta->registerConstructor(domCapsule::domRadius::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float2"));
		ma->setOffset( daeOffsetOf( domCapsule::domRadius , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCapsule::domRadius));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domCapsule::_Meta = NULL;
daeMetaElement * domCapsule::domHeight::_Meta = NULL;
daeMetaElement * domCapsule::domRadius::_Meta = NULL;


