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
#include <dom/domPlane.h>

daeElementRef
domPlane::create(daeInt bytes)
{
	domPlaneRef ref = new(bytes) domPlane;
	return ref;
}


daeMetaElement *
domPlane::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "plane" );
	_Meta->setStaticPointerAddress(&domPlane::_Meta);
	_Meta->registerConstructor(domPlane::create);

	// Add elements: equation, extra
    _Meta->appendElement(domPlane::domEquation::registerElement(),daeOffsetOf(domPlane,elemEquation));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domPlane,elemExtra_array));
	
	
	_Meta->setElementSize(sizeof(domPlane));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domPlane::domEquation::create(daeInt bytes)
{
	domPlane::domEquationRef ref = new(bytes) domPlane::domEquation;
	return ref;
}


daeMetaElement *
domPlane::domEquation::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "equation" );
	_Meta->setStaticPointerAddress(&domPlane::domEquation::_Meta);
	_Meta->registerConstructor(domPlane::domEquation::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domPlane::domEquation , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domPlane::domEquation));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domPlane::_Meta = NULL;
daeMetaElement * domPlane::domEquation::_Meta = NULL;


