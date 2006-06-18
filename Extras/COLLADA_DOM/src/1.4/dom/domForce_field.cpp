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
#include <dom/domForce_field.h>

daeElementRef
domForce_field::create(daeInt bytes)
{
	domForce_fieldRef ref = new(bytes) domForce_field;
	return ref;
}


daeMetaElement *
domForce_field::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "force_field" );
	_Meta->setStaticPointerAddress(&domForce_field::_Meta);
	_Meta->registerConstructor(domForce_field::create);

	// Add elements: asset, technique, extra
    _Meta->appendElement(domAsset::registerElement(),daeOffsetOf(domForce_field,elemAsset));
    _Meta->appendArrayElement(domTechnique::registerElement(),daeOffsetOf(domForce_field,elemTechnique_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domForce_field,elemExtra_array));

	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domForce_field , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domForce_field , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domForce_field));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domForce_field::_Meta = NULL;


