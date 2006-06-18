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
	_Meta->setStaticPointerAddress(&domSphere::_Meta);
	_Meta->registerConstructor(domSphere::create);

	// Add elements: radius, extra
    _Meta->appendElement(domSphere::domRadius::registerElement(),daeOffsetOf(domSphere,elemRadius));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domSphere,elemExtra_array));
	
	
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
	_Meta->setStaticPointerAddress(&domSphere::domRadius::_Meta);
	_Meta->registerConstructor(domSphere::domRadius::create);

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


