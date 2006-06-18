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
#include <dom/domLibrary_controllers.h>

daeElementRef
domLibrary_controllers::create(daeInt bytes)
{
	domLibrary_controllersRef ref = new(bytes) domLibrary_controllers;
	return ref;
}


daeMetaElement *
domLibrary_controllers::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "library_controllers" );
	_Meta->setStaticPointerAddress(&domLibrary_controllers::_Meta);
	_Meta->registerConstructor(domLibrary_controllers::create);

	// Add elements: asset, controller, extra
    _Meta->appendElement(domAsset::registerElement(),daeOffsetOf(domLibrary_controllers,elemAsset));
    _Meta->appendArrayElement(domController::registerElement(),daeOffsetOf(domLibrary_controllers,elemController_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domLibrary_controllers,elemExtra_array));

	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domLibrary_controllers , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domLibrary_controllers , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domLibrary_controllers));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domLibrary_controllers::_Meta = NULL;


