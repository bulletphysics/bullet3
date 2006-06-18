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
#include <dom/domFx_stenciltarget_common.h>

daeElementRef
domFx_stenciltarget_common::create(daeInt bytes)
{
	domFx_stenciltarget_commonRef ref = new(bytes) domFx_stenciltarget_common;
	return ref;
}


daeMetaElement *
domFx_stenciltarget_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fx_stenciltarget_common" );
	_Meta->setStaticPointerAddress(&domFx_stenciltarget_common::_Meta);
	_Meta->registerConstructor(domFx_stenciltarget_common::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domFx_stenciltarget_common , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("xsNonNegativeInteger"));
		ma->setOffset( daeOffsetOf( domFx_stenciltarget_common , attrIndex ));
		ma->setContainer( _Meta );
		ma->setDefault( "0");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: slice
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "slice" );
		ma->setType( daeAtomicType::get("xsNonNegativeInteger"));
		ma->setOffset( daeOffsetOf( domFx_stenciltarget_common , attrSlice ));
		ma->setContainer( _Meta );
		ma->setDefault( "0");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_stenciltarget_common));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domFx_stenciltarget_common::_Meta = NULL;


