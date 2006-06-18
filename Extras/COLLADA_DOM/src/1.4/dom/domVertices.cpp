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
#include <dom/domVertices.h>

daeElementRef
domVertices::create(daeInt bytes)
{
	domVerticesRef ref = new(bytes) domVertices;
	return ref;
}


daeMetaElement *
domVertices::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "vertices" );
	_Meta->setStaticPointerAddress(&domVertices::_Meta);
	_Meta->registerConstructor(domVertices::create);

	// Add elements: input, extra
    _Meta->appendArrayElement(domInputLocal::registerElement(),daeOffsetOf(domVertices,elemInput_array),"input"); 
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domVertices,elemExtra_array));

	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domVertices , attrId ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domVertices , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domVertices));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domVertices::_Meta = NULL;


