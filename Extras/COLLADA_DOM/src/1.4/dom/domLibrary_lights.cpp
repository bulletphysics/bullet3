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
#include <dom/domLibrary_lights.h>

daeElementRef
domLibrary_lights::create(daeInt bytes)
{
	domLibrary_lightsRef ref = new(bytes) domLibrary_lights;
	return ref;
}


daeMetaElement *
domLibrary_lights::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "library_lights" );
	_Meta->setStaticPointerAddress(&domLibrary_lights::_Meta);
	_Meta->registerConstructor(domLibrary_lights::create);

	// Add elements: asset, light, extra
    _Meta->appendElement(domAsset::registerElement(),daeOffsetOf(domLibrary_lights,elemAsset));
    _Meta->appendArrayElement(domLight::registerElement(),daeOffsetOf(domLibrary_lights,elemLight_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domLibrary_lights,elemExtra_array));

	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domLibrary_lights , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domLibrary_lights , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domLibrary_lights));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domLibrary_lights::_Meta = NULL;


