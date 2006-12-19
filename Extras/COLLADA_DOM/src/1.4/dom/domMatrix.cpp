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
#include <dom/domMatrix.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domMatrix::create(daeInt bytes)
{
	domMatrixRef ref = new(bytes) domMatrix;
	return ref;
}


daeMetaElement *
domMatrix::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "matrix" );
	_Meta->registerClass(domMatrix::create, &_Meta);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float4x4"));
		ma->setOffset( daeOffsetOf( domMatrix , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domMatrix , attrSid ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domMatrix));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domMatrix::_Meta = NULL;


