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
#include <dom/domGlsl_newparam.h>

daeElementRef
domGlsl_newparam::create(daeInt bytes)
{
	domGlsl_newparamRef ref = new(bytes) domGlsl_newparam;
	return ref;
}


daeMetaElement *
domGlsl_newparam::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "glsl_newparam" );
	_Meta->setStaticPointerAddress(&domGlsl_newparam::_Meta);
	_Meta->registerConstructor(domGlsl_newparam::create);

	// Add elements: annotate, semantic, modifier, glsl_param_type, array
    _Meta->appendArrayElement(domFx_annotate_common::registerElement(),daeOffsetOf(domGlsl_newparam,elemAnnotate_array),"annotate"); 
    _Meta->appendElement(domGlsl_newparam::domSemantic::registerElement(),daeOffsetOf(domGlsl_newparam,elemSemantic));
    _Meta->appendElement(domGlsl_newparam::domModifier::registerElement(),daeOffsetOf(domGlsl_newparam,elemModifier));
    _Meta->appendElement(domGlsl_param_type::registerElement(),daeOffsetOf(domGlsl_newparam,elemGlsl_param_type));
	_Meta->appendPossibleChild( "bool", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "bool2", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "bool3", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "bool4", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float2", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float3", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float4", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float2x2", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float3x3", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "float4x4", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "int", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "int2", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "int3", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "int4", _Meta->getMetaElements()[3]);
	_Meta->appendPossibleChild( "surface", _Meta->getMetaElements()[3], "fx_surface_common");
	_Meta->appendPossibleChild( "sampler1D", _Meta->getMetaElements()[3], "gl_sampler1D");
	_Meta->appendPossibleChild( "sampler2D", _Meta->getMetaElements()[3], "gl_sampler2D");
	_Meta->appendPossibleChild( "sampler3D", _Meta->getMetaElements()[3], "gl_sampler3D");
	_Meta->appendPossibleChild( "samplerCUBE", _Meta->getMetaElements()[3], "gl_samplerCUBE");
	_Meta->appendPossibleChild( "samplerRECT", _Meta->getMetaElements()[3], "gl_samplerRECT");
	_Meta->appendPossibleChild( "samplerDEPTH", _Meta->getMetaElements()[3], "gl_samplerDEPTH");
	_Meta->appendPossibleChild( "enum", _Meta->getMetaElements()[3]);
    _Meta->appendElement(domGlsl_newarray_type::registerElement(),daeOffsetOf(domGlsl_newparam,elemArray),"array"); 
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domGlsl_newparam,_contents));


	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("Glsl_identifier"));
		ma->setOffset( daeOffsetOf( domGlsl_newparam , attrSid ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGlsl_newparam));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGlsl_newparam::domSemantic::create(daeInt bytes)
{
	domGlsl_newparam::domSemanticRef ref = new(bytes) domGlsl_newparam::domSemantic;
	return ref;
}


daeMetaElement *
domGlsl_newparam::domSemantic::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "semantic" );
	_Meta->setStaticPointerAddress(&domGlsl_newparam::domSemantic::_Meta);
	_Meta->registerConstructor(domGlsl_newparam::domSemantic::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGlsl_newparam::domSemantic , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGlsl_newparam::domSemantic));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGlsl_newparam::domModifier::create(daeInt bytes)
{
	domGlsl_newparam::domModifierRef ref = new(bytes) domGlsl_newparam::domModifier;
	return ref;
}


daeMetaElement *
domGlsl_newparam::domModifier::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "modifier" );
	_Meta->setStaticPointerAddress(&domGlsl_newparam::domModifier::_Meta);
	_Meta->registerConstructor(domGlsl_newparam::domModifier::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_modifier_enum_common"));
		ma->setOffset( daeOffsetOf( domGlsl_newparam::domModifier , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGlsl_newparam::domModifier));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domGlsl_newparam::_Meta = NULL;
daeMetaElement * domGlsl_newparam::domSemantic::_Meta = NULL;
daeMetaElement * domGlsl_newparam::domModifier::_Meta = NULL;


