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
#include <dom/domLibrary_materials.h>

daeElementRef
domLibrary_materials::create(daeInt bytes)
{
	domLibrary_materialsRef ref = new(bytes) domLibrary_materials;
	return ref;
}


daeMetaElement *
domLibrary_materials::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "library_materials" );
	_Meta->setStaticPointerAddress(&domLibrary_materials::_Meta);
	_Meta->registerConstructor(domLibrary_materials::create);

	// Add elements: asset, material, extra
    _Meta->appendElement(domAsset::registerElement(),daeOffsetOf(domLibrary_materials,elemAsset));
    _Meta->appendArrayElement(domMaterial::registerElement(),daeOffsetOf(domLibrary_materials,elemMaterial_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domLibrary_materials,elemExtra_array));

	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domLibrary_materials , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domLibrary_materials , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domLibrary_materials));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domLibrary_materials::_Meta = NULL;


