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
#include <dom/domPolylist.h>

daeElementRef
domPolylist::create(daeInt bytes)
{
	domPolylistRef ref = new(bytes) domPolylist;
	return ref;
}


daeMetaElement *
domPolylist::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "polylist" );
	_Meta->setStaticPointerAddress(&domPolylist::_Meta);
	_Meta->registerConstructor(domPolylist::create);

	// Add elements: input, vcount, p, extra
    _Meta->appendArrayElement(domInputLocalOffset::registerElement(),daeOffsetOf(domPolylist,elemInput_array),"input"); 
    _Meta->appendElement(domPolylist::domVcount::registerElement(),daeOffsetOf(domPolylist,elemVcount));
    _Meta->appendElement(domP::registerElement(),daeOffsetOf(domPolylist,elemP));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domPolylist,elemExtra_array));

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domPolylist , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: count
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "count" );
		ma->setType( daeAtomicType::get("Uint"));
		ma->setOffset( daeOffsetOf( domPolylist , attrCount ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: material
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "material" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domPolylist , attrMaterial ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domPolylist));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domPolylist::domVcount::create(daeInt bytes)
{
	domPolylist::domVcountRef ref = new(bytes) domPolylist::domVcount;
	return ref;
}


daeMetaElement *
domPolylist::domVcount::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "vcount" );
	_Meta->setStaticPointerAddress(&domPolylist::domVcount::_Meta);
	_Meta->registerConstructor(domPolylist::domVcount::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("ListOfUInts"));
		ma->setOffset( daeOffsetOf( domPolylist::domVcount , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domPolylist::domVcount));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domPolylist::_Meta = NULL;
daeMetaElement * domPolylist::domVcount::_Meta = NULL;


