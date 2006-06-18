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
#include <dom/domFx_setparam_common.h>

daeElementRef
domFx_setparam_common::create(daeInt bytes)
{
	domFx_setparam_commonRef ref = new(bytes) domFx_setparam_common;
	return ref;
}


daeMetaElement *
domFx_setparam_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fx_setparam_common" );
	_Meta->setStaticPointerAddress(&domFx_setparam_common::_Meta);
	_Meta->registerConstructor(domFx_setparam_common::create);

	// Add elements: annotate, fx_basic_type_common
    _Meta->appendArrayElement(domFx_annotate_common::registerElement(),daeOffsetOf(domFx_setparam_common,elemAnnotate_array),"annotate"); 
    _Meta->appendElement(domFx_basic_type_common::registerElement(),daeOffsetOf(domFx_setparam_common,elemFx_basic_type_common));
	_Meta->appendPossibleChild( "bool", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float1x1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float1x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float1x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float1x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float2x1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float2x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float2x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float2x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float3x1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float3x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float3x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float3x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float4x1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float4x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float4x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float4x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "surface", _Meta->getMetaElements()[1], "fx_surface_common");
	_Meta->appendPossibleChild( "sampler1D", _Meta->getMetaElements()[1], "fx_sampler1D_common");
	_Meta->appendPossibleChild( "sampler2D", _Meta->getMetaElements()[1], "fx_sampler2D_common");
	_Meta->appendPossibleChild( "sampler3D", _Meta->getMetaElements()[1], "fx_sampler3D_common");
	_Meta->appendPossibleChild( "samplerCUBE", _Meta->getMetaElements()[1], "fx_samplerCUBE_common");
	_Meta->appendPossibleChild( "samplerRECT", _Meta->getMetaElements()[1], "fx_samplerRECT_common");
	_Meta->appendPossibleChild( "samplerDEPTH", _Meta->getMetaElements()[1], "fx_samplerDEPTH_common");
	_Meta->appendPossibleChild( "enum", _Meta->getMetaElements()[1]);

	//	Add attribute: ref
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domFx_setparam_common , attrRef ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_setparam_common));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domFx_setparam_common::_Meta = NULL;


