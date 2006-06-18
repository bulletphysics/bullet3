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
#include <dom/domController.h>

daeElementRef
domController::create(daeInt bytes)
{
	domControllerRef ref = new(bytes) domController;
	return ref;
}


daeMetaElement *
domController::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "controller" );
	_Meta->setStaticPointerAddress(&domController::_Meta);
	_Meta->registerConstructor(domController::create);

	// Add elements: asset, skin, morph, extra
    _Meta->appendElement(domAsset::registerElement(),daeOffsetOf(domController,elemAsset));
    _Meta->appendElement(domSkin::registerElement(),daeOffsetOf(domController,elemSkin));
    _Meta->appendElement(domMorph::registerElement(),daeOffsetOf(domController,elemMorph));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domController,elemExtra_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domController,_contents));


	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domController , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domController , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domController));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domController::_Meta = NULL;


