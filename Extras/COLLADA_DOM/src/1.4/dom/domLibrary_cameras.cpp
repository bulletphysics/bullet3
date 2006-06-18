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
#include <dom/domLibrary_cameras.h>

daeElementRef
domLibrary_cameras::create(daeInt bytes)
{
	domLibrary_camerasRef ref = new(bytes) domLibrary_cameras;
	return ref;
}


daeMetaElement *
domLibrary_cameras::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "library_cameras" );
	_Meta->setStaticPointerAddress(&domLibrary_cameras::_Meta);
	_Meta->registerConstructor(domLibrary_cameras::create);

	// Add elements: asset, camera, extra
    _Meta->appendElement(domAsset::registerElement(),daeOffsetOf(domLibrary_cameras,elemAsset));
    _Meta->appendArrayElement(domCamera::registerElement(),daeOffsetOf(domLibrary_cameras,elemCamera_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domLibrary_cameras,elemExtra_array));

	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domLibrary_cameras , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domLibrary_cameras , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domLibrary_cameras));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domLibrary_cameras::_Meta = NULL;


