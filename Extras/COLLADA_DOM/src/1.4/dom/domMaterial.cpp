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
#include <dom/domMaterial.h>

daeElementRef
domMaterial::create(daeInt bytes)
{
	domMaterialRef ref = new(bytes) domMaterial;
	return ref;
}


daeMetaElement *
domMaterial::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "material" );
	_Meta->setStaticPointerAddress(&domMaterial::_Meta);
	_Meta->registerConstructor(domMaterial::create);

	// Add elements: asset, instance_effect, extra
    _Meta->appendElement(domAsset::registerElement(),daeOffsetOf(domMaterial,elemAsset));
    _Meta->appendElement(domInstance_effect::registerElement(),daeOffsetOf(domMaterial,elemInstance_effect));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domMaterial,elemExtra_array));

	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domMaterial , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domMaterial , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domMaterial));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domMaterial::_Meta = NULL;


