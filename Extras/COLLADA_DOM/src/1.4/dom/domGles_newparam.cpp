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
#include <dom/domGles_newparam.h>

daeElementRef
domGles_newparam::create(daeInt bytes)
{
	domGles_newparamRef ref = new(bytes) domGles_newparam;
	return ref;
}


daeMetaElement *
domGles_newparam::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "gles_newparam" );
	_Meta->setStaticPointerAddress(&domGles_newparam::_Meta);
	_Meta->registerConstructor(domGles_newparam::create);

	// Add elements: annotate, semantic, modifier, gles_basic_type_common
    _Meta->appendArrayElement(domFx_annotate_common::registerElement(),daeOffsetOf(domGles_newparam,elemAnnotate_array),"annotate"); 
    _Meta->appendElement(domGles_newparam::domSemantic::registerElement(),daeOffsetOf(domGles_newparam,elemSemantic));
    _Meta->appendElement(domGles_newparam::domModifier::registerElement(),daeOffsetOf(domGles_newparam,elemModifier));
    _Meta->appendElement(domGles_basic_type_common::registerElement(),daeOffsetOf(domGles_newparam,elemGles_basic_type_common));
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
	_Meta->appendPossibleChild( "texture_pipeline", _Meta->getMetaElements()[3], "gles_texture_pipeline");
	_Meta->appendPossibleChild( "sampler_state", _Meta->getMetaElements()[3], "gles_sampler_state");
	_Meta->appendPossibleChild( "texture_unit", _Meta->getMetaElements()[3], "gles_texture_unit");
	_Meta->appendPossibleChild( "enum", _Meta->getMetaElements()[3]);

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_newparam , attrSid ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_newparam));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_newparam::domSemantic::create(daeInt bytes)
{
	domGles_newparam::domSemanticRef ref = new(bytes) domGles_newparam::domSemantic;
	return ref;
}


daeMetaElement *
domGles_newparam::domSemantic::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "semantic" );
	_Meta->setStaticPointerAddress(&domGles_newparam::domSemantic::_Meta);
	_Meta->registerConstructor(domGles_newparam::domSemantic::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_newparam::domSemantic , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_newparam::domSemantic));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_newparam::domModifier::create(daeInt bytes)
{
	domGles_newparam::domModifierRef ref = new(bytes) domGles_newparam::domModifier;
	return ref;
}


daeMetaElement *
domGles_newparam::domModifier::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "modifier" );
	_Meta->setStaticPointerAddress(&domGles_newparam::domModifier::_Meta);
	_Meta->registerConstructor(domGles_newparam::domModifier::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_modifier_enum_common"));
		ma->setOffset( daeOffsetOf( domGles_newparam::domModifier , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_newparam::domModifier));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domGles_newparam::_Meta = NULL;
daeMetaElement * domGles_newparam::domSemantic::_Meta = NULL;
daeMetaElement * domGles_newparam::domModifier::_Meta = NULL;


