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
#include <dom/domEllipsoid.h>

daeElementRef
domEllipsoid::create(daeInt bytes)
{
	domEllipsoidRef ref = new(bytes) domEllipsoid;
	return ref;
}


daeMetaElement *
domEllipsoid::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "ellipsoid" );
	_Meta->setStaticPointerAddress(&domEllipsoid::_Meta);
	_Meta->registerConstructor(domEllipsoid::create);

	// Add elements: size
    _Meta->appendElement(domEllipsoid::domSize::registerElement(),daeOffsetOf(domEllipsoid,elemSize));
	
	
	_Meta->setElementSize(sizeof(domEllipsoid));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domEllipsoid::domSize::create(daeInt bytes)
{
	domEllipsoid::domSizeRef ref = new(bytes) domEllipsoid::domSize;
	return ref;
}


daeMetaElement *
domEllipsoid::domSize::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "size" );
	_Meta->setStaticPointerAddress(&domEllipsoid::domSize::_Meta);
	_Meta->registerConstructor(domEllipsoid::domSize::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float3"));
		ma->setOffset( daeOffsetOf( domEllipsoid::domSize , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domEllipsoid::domSize));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domEllipsoid::_Meta = NULL;
daeMetaElement * domEllipsoid::domSize::_Meta = NULL;


