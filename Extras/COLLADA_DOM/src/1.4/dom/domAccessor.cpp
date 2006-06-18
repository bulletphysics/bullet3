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
#include <dom/domAccessor.h>

daeElementRef
domAccessor::create(daeInt bytes)
{
	domAccessorRef ref = new(bytes) domAccessor;
	ref->attrSource.setContainer( (domAccessor*)ref );
	return ref;
}


daeMetaElement *
domAccessor::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "accessor" );
	_Meta->setStaticPointerAddress(&domAccessor::_Meta);
	_Meta->registerConstructor(domAccessor::create);

	// Add elements: param
    _Meta->appendArrayElement(domParam::registerElement(),daeOffsetOf(domAccessor,elemParam_array));

	//	Add attribute: count
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "count" );
		ma->setType( daeAtomicType::get("Uint"));
		ma->setOffset( daeOffsetOf( domAccessor , attrCount ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: offset
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "offset" );
		ma->setType( daeAtomicType::get("Uint"));
		ma->setOffset( daeOffsetOf( domAccessor , attrOffset ));
		ma->setContainer( _Meta );
		ma->setDefault( "0");
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: source
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "source" );
		ma->setType( daeAtomicType::get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domAccessor , attrSource ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: stride
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "stride" );
		ma->setType( daeAtomicType::get("Uint"));
		ma->setOffset( daeOffsetOf( domAccessor , attrStride ));
		ma->setContainer( _Meta );
		ma->setDefault( "1");
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domAccessor));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domAccessor::_Meta = NULL;


