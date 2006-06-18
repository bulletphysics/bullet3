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
#include <dom/domFx_annotate_common.h>

daeElementRef
domFx_annotate_common::create(daeInt bytes)
{
	domFx_annotate_commonRef ref = new(bytes) domFx_annotate_common;
	return ref;
}


daeMetaElement *
domFx_annotate_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fx_annotate_common" );
	_Meta->setStaticPointerAddress(&domFx_annotate_common::_Meta);
	_Meta->registerConstructor(domFx_annotate_common::create);

	// Add elements: fx_annotate_type_common
    _Meta->appendElement(domFx_annotate_type_common::registerElement(),daeOffsetOf(domFx_annotate_common,elemFx_annotate_type_common));
	_Meta->appendPossibleChild( "bool", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float2x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float3x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float4x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "string", _Meta->getMetaElements()[0]);

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domFx_annotate_common , attrName ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_annotate_common));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domFx_annotate_common::_Meta = NULL;


