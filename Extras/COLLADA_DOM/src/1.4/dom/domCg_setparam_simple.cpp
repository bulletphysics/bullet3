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
#include <dom/domCg_setparam_simple.h>

daeElementRef
domCg_setparam_simple::create(daeInt bytes)
{
	domCg_setparam_simpleRef ref = new(bytes) domCg_setparam_simple;
	return ref;
}


daeMetaElement *
domCg_setparam_simple::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "cg_setparam_simple" );
	_Meta->setStaticPointerAddress(&domCg_setparam_simple::_Meta);
	_Meta->registerConstructor(domCg_setparam_simple::create);

	// Add elements: annotate, cg_param_type
    _Meta->appendArrayElement(domFx_annotate_common::registerElement(),daeOffsetOf(domCg_setparam_simple,elemAnnotate_array),"annotate"); 
    _Meta->appendElement(domCg_param_type::registerElement(),daeOffsetOf(domCg_setparam_simple,elemCg_param_type));
	_Meta->appendPossibleChild( "bool", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool1x1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool1x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool1x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool1x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool2x1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool2x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool2x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool2x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool3x1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool3x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool3x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool3x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool4x1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool4x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool4x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool4x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float1", _Meta->getMetaElements()[1]);
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
	_Meta->appendPossibleChild( "int", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int1x1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int1x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int1x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int1x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int2x1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int2x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int2x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int2x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int3x1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int3x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int3x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int3x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int4x1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int4x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int4x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int4x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "half", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "half1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "half2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "half3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "half4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "half1x1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "half1x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "half1x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "half1x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "half2x1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "half2x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "half2x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "half2x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "half3x1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "half3x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "half3x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "half3x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "half4x1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "half4x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "half4x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "half4x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "fixed", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "fixed1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "fixed2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "fixed3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "fixed4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "fixed1x1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "fixed1x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "fixed1x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "fixed1x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "fixed2x1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "fixed2x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "fixed2x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "fixed2x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "fixed3x1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "fixed3x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "fixed3x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "fixed3x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "fixed4x1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "fixed4x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "fixed4x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "fixed4x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "surface", _Meta->getMetaElements()[1], "cg_surface_type");
	_Meta->appendPossibleChild( "sampler1D", _Meta->getMetaElements()[1], "cg_sampler1D");
	_Meta->appendPossibleChild( "sampler2D", _Meta->getMetaElements()[1], "cg_sampler2D");
	_Meta->appendPossibleChild( "sampler3D", _Meta->getMetaElements()[1], "cg_sampler3D");
	_Meta->appendPossibleChild( "samplerRECT", _Meta->getMetaElements()[1], "cg_samplerRECT");
	_Meta->appendPossibleChild( "samplerCUBE", _Meta->getMetaElements()[1], "cg_samplerCUBE");
	_Meta->appendPossibleChild( "samplerDEPTH", _Meta->getMetaElements()[1], "cg_samplerDEPTH");
	_Meta->appendPossibleChild( "string", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "enum", _Meta->getMetaElements()[1]);

	//	Add attribute: ref
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( daeAtomicType::get("Cg_identifier"));
		ma->setOffset( daeOffsetOf( domCg_setparam_simple , attrRef ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_setparam_simple));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domCg_setparam_simple::_Meta = NULL;


