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
#include <dom/domFloat_array.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domFloat_array::create(daeInt bytes)
{
	domFloat_arrayRef ref = new(bytes) domFloat_array;
	return ref;
}


daeMetaElement *
domFloat_array::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float_array" );
	_Meta->registerConstructor(domFloat_array::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("ListOfFloats"));
		ma->setOffset( daeOffsetOf( domFloat_array , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domFloat_array , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domFloat_array , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: count
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "count" );
		ma->setType( daeAtomicType::get("Uint"));
		ma->setOffset( daeOffsetOf( domFloat_array , attrCount ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: digits
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "digits" );
		ma->setType( daeAtomicType::get("xsShort"));
		ma->setOffset( daeOffsetOf( domFloat_array , attrDigits ));
		ma->setContainer( _Meta );
		ma->setDefault( "6");
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: magnitude
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "magnitude" );
		ma->setType( daeAtomicType::get("xsShort"));
		ma->setOffset( daeOffsetOf( domFloat_array , attrMagnitude ));
		ma->setContainer( _Meta );
		ma->setDefault( "38");
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFloat_array));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domFloat_array::_Meta = NULL;


