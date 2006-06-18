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
#include <dom/domInt_array.h>

daeElementRef
domInt_array::create(daeInt bytes)
{
	domInt_arrayRef ref = new(bytes) domInt_array;
	return ref;
}


daeMetaElement *
domInt_array::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "int_array" );
	_Meta->setStaticPointerAddress(&domInt_array::_Meta);
	_Meta->registerConstructor(domInt_array::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("ListOfInts"));
		ma->setOffset( daeOffsetOf( domInt_array , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domInt_array , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInt_array , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: count
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "count" );
		ma->setType( daeAtomicType::get("Uint"));
		ma->setOffset( daeOffsetOf( domInt_array , attrCount ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: minInclusive
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "minInclusive" );
		ma->setType( daeAtomicType::get("xsInteger"));
		ma->setOffset( daeOffsetOf( domInt_array , attrMinInclusive ));
		ma->setContainer( _Meta );
		ma->setDefault( "-2147483648");
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: maxInclusive
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "maxInclusive" );
		ma->setType( daeAtomicType::get("xsInteger"));
		ma->setOffset( daeOffsetOf( domInt_array , attrMaxInclusive ));
		ma->setContainer( _Meta );
		ma->setDefault( "2147483647");
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domInt_array));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domInt_array::_Meta = NULL;


