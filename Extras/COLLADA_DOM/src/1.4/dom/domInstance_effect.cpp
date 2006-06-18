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
#include <dom/domInstance_effect.h>

daeElementRef
domInstance_effect::create(daeInt bytes)
{
	domInstance_effectRef ref = new(bytes) domInstance_effect;
	ref->attrUrl.setContainer( (domInstance_effect*)ref );
	return ref;
}


daeMetaElement *
domInstance_effect::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "instance_effect" );
	_Meta->setStaticPointerAddress(&domInstance_effect::_Meta);
	_Meta->registerConstructor(domInstance_effect::create);

	// Add elements: technique_hint, setparam, extra
    _Meta->appendArrayElement(domInstance_effect::domTechnique_hint::registerElement(),daeOffsetOf(domInstance_effect,elemTechnique_hint_array));
    _Meta->appendArrayElement(domInstance_effect::domSetparam::registerElement(),daeOffsetOf(domInstance_effect,elemSetparam_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domInstance_effect,elemExtra_array));

	//	Add attribute: url
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "url" );
		ma->setType( daeAtomicType::get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domInstance_effect , attrUrl ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domInstance_effect));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domInstance_effect::domTechnique_hint::create(daeInt bytes)
{
	domInstance_effect::domTechnique_hintRef ref = new(bytes) domInstance_effect::domTechnique_hint;
	return ref;
}


daeMetaElement *
domInstance_effect::domTechnique_hint::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "technique_hint" );
	_Meta->setStaticPointerAddress(&domInstance_effect::domTechnique_hint::_Meta);
	_Meta->registerConstructor(domInstance_effect::domTechnique_hint::create);


	//	Add attribute: platform
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "platform" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_effect::domTechnique_hint , attrPlatform ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: ref
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_effect::domTechnique_hint , attrRef ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domInstance_effect::domTechnique_hint));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domInstance_effect::domSetparam::create(daeInt bytes)
{
	domInstance_effect::domSetparamRef ref = new(bytes) domInstance_effect::domSetparam;
	return ref;
}


daeMetaElement *
domInstance_effect::domSetparam::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "setparam" );
	_Meta->setStaticPointerAddress(&domInstance_effect::domSetparam::_Meta);
	_Meta->registerConstructor(domInstance_effect::domSetparam::create);

	// Add elements: fx_basic_type_common
    _Meta->appendElement(domFx_basic_type_common::registerElement(),daeOffsetOf(domInstance_effect::domSetparam,elemFx_basic_type_common));
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
	_Meta->appendPossibleChild( "float1x1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float1x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float1x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float1x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float2x1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float2x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float2x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float2x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float3x1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float3x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float3x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float3x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float4x1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float4x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float4x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float4x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "surface", _Meta->getMetaElements()[0], "fx_surface_common");
	_Meta->appendPossibleChild( "sampler1D", _Meta->getMetaElements()[0], "fx_sampler1D_common");
	_Meta->appendPossibleChild( "sampler2D", _Meta->getMetaElements()[0], "fx_sampler2D_common");
	_Meta->appendPossibleChild( "sampler3D", _Meta->getMetaElements()[0], "fx_sampler3D_common");
	_Meta->appendPossibleChild( "samplerCUBE", _Meta->getMetaElements()[0], "fx_samplerCUBE_common");
	_Meta->appendPossibleChild( "samplerRECT", _Meta->getMetaElements()[0], "fx_samplerRECT_common");
	_Meta->appendPossibleChild( "samplerDEPTH", _Meta->getMetaElements()[0], "fx_samplerDEPTH_common");
	_Meta->appendPossibleChild( "enum", _Meta->getMetaElements()[0]);

	//	Add attribute: ref
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_effect::domSetparam , attrRef ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domInstance_effect::domSetparam));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domInstance_effect::_Meta = NULL;
daeMetaElement * domInstance_effect::domTechnique_hint::_Meta = NULL;
daeMetaElement * domInstance_effect::domSetparam::_Meta = NULL;


