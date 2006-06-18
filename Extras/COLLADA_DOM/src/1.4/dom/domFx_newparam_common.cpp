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
#include <dom/domFx_newparam_common.h>

daeElementRef
domFx_newparam_common::create(daeInt bytes)
{
	domFx_newparam_commonRef ref = new(bytes) domFx_newparam_common;
	return ref;
}


daeMetaElement *
domFx_newparam_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fx_newparam_common" );
	_Meta->setStaticPointerAddress(&domFx_newparam_common::_Meta);
	_Meta->registerConstructor(domFx_newparam_common::create);

	// Add elements: annotate, semantic, modifier, fx_basic_type_common
    _Meta->appendArrayElement(domFx_annotate_common::registerElement(),daeOffsetOf(domFx_newparam_common,elemAnnotate_array),"annotate"); 
    _Meta->appendElement(domFx_newparam_common::domSemantic::registerElement(),daeOffsetOf(domFx_newparam_common,elemSemantic));
    _Meta->appendElement(domFx_newparam_common::domModifier::registerElement(),daeOffsetOf(domFx_newparam_common,elemModifier));
    _Meta->appendElement(domFx_basic_type_common::registerElement(),daeOffsetOf(domFx_newparam_common,elemFx_basic_type_common));
	_Meta->appendPossibleChild( "bool", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "bool2", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "bool3", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "bool4", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "int", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "int2", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "int3", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "int4", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float2", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float3", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float4", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float1x1", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float1x2", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float1x3", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float1x4", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float2x1", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float2x2", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float2x3", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float2x4", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float3x1", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float3x2", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float3x3", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float3x4", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float4x1", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float4x2", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float4x3", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float4x4", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "surface", _Meta->getMetaElements()[3], "fx_surface_common");
	_Meta->appendPossibleChild( "sampler1D", _Meta->getMetaElements()[3], "fx_sampler1D_common");
	_Meta->appendPossibleChild( "sampler2D", _Meta->getMetaElements()[3], "fx_sampler2D_common");
	_Meta->appendPossibleChild( "sampler3D", _Meta->getMetaElements()[3], "fx_sampler3D_common");
	_Meta->appendPossibleChild( "samplerCUBE", _Meta->getMetaElements()[3], "fx_samplerCUBE_common");
	_Meta->appendPossibleChild( "samplerRECT", _Meta->getMetaElements()[3], "fx_samplerRECT_common");
	_Meta->appendPossibleChild( "samplerDEPTH", _Meta->getMetaElements()[3], "fx_samplerDEPTH_common");
	_Meta->appendPossibleChild( "enum", _Meta->getMetaElements()[3]);

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domFx_newparam_common , attrSid ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_newparam_common));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_newparam_common::domSemantic::create(daeInt bytes)
{
	domFx_newparam_common::domSemanticRef ref = new(bytes) domFx_newparam_common::domSemantic;
	return ref;
}


daeMetaElement *
domFx_newparam_common::domSemantic::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "semantic" );
	_Meta->setStaticPointerAddress(&domFx_newparam_common::domSemantic::_Meta);
	_Meta->registerConstructor(domFx_newparam_common::domSemantic::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domFx_newparam_common::domSemantic , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_newparam_common::domSemantic));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domFx_newparam_common::domModifier::create(daeInt bytes)
{
	domFx_newparam_common::domModifierRef ref = new(bytes) domFx_newparam_common::domModifier;
	return ref;
}


daeMetaElement *
domFx_newparam_common::domModifier::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "modifier" );
	_Meta->setStaticPointerAddress(&domFx_newparam_common::domModifier::_Meta);
	_Meta->registerConstructor(domFx_newparam_common::domModifier::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_modifier_enum_common"));
		ma->setOffset( daeOffsetOf( domFx_newparam_common::domModifier , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domFx_newparam_common::domModifier));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domFx_newparam_common::_Meta = NULL;
daeMetaElement * domFx_newparam_common::domSemantic::_Meta = NULL;
daeMetaElement * domFx_newparam_common::domModifier::_Meta = NULL;


