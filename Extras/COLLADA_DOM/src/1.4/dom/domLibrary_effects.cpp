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
#include <dom/domLibrary_effects.h>

daeElementRef
domLibrary_effects::create(daeInt bytes)
{
	domLibrary_effectsRef ref = new(bytes) domLibrary_effects;
	return ref;
}


daeMetaElement *
domLibrary_effects::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "library_effects" );
	_Meta->setStaticPointerAddress(&domLibrary_effects::_Meta);
	_Meta->registerConstructor(domLibrary_effects::create);

	// Add elements: asset, effect, extra
    _Meta->appendElement(domAsset::registerElement(),daeOffsetOf(domLibrary_effects,elemAsset));
    _Meta->appendArrayElement(domEffect::registerElement(),daeOffsetOf(domLibrary_effects,elemEffect_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domLibrary_effects,elemExtra_array));

	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domLibrary_effects , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domLibrary_effects , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domLibrary_effects));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domLibrary_effects::_Meta = NULL;


