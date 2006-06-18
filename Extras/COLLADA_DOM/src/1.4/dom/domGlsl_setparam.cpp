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
#include <dom/domGlsl_setparam.h>

daeElementRef
domGlsl_setparam::create(daeInt bytes)
{
	domGlsl_setparamRef ref = new(bytes) domGlsl_setparam;
	return ref;
}


daeMetaElement *
domGlsl_setparam::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "glsl_setparam" );
	_Meta->setStaticPointerAddress(&domGlsl_setparam::_Meta);
	_Meta->registerConstructor(domGlsl_setparam::create);

	// Add elements: annotate, glsl_param_type, array
    _Meta->appendArrayElement(domFx_annotate_common::registerElement(),daeOffsetOf(domGlsl_setparam,elemAnnotate_array),"annotate"); 
    _Meta->appendElement(domGlsl_param_type::registerElement(),daeOffsetOf(domGlsl_setparam,elemGlsl_param_type));
	_Meta->appendPossibleChild( "bool", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float2x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float3x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float4x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "surface", _Meta->getMetaElements()[1], "fx_surface_common");
	_Meta->appendPossibleChild( "sampler1D", _Meta->getMetaElements()[1], "gl_sampler1D");
	_Meta->appendPossibleChild( "sampler2D", _Meta->getMetaElements()[1], "gl_sampler2D");
	_Meta->appendPossibleChild( "sampler3D", _Meta->getMetaElements()[1], "gl_sampler3D");
	_Meta->appendPossibleChild( "samplerCUBE", _Meta->getMetaElements()[1], "gl_samplerCUBE");
	_Meta->appendPossibleChild( "samplerRECT", _Meta->getMetaElements()[1], "gl_samplerRECT");
	_Meta->appendPossibleChild( "samplerDEPTH", _Meta->getMetaElements()[1], "gl_samplerDEPTH");
	_Meta->appendPossibleChild( "enum", _Meta->getMetaElements()[1]);
    _Meta->appendElement(domGlsl_setarray_type::registerElement(),daeOffsetOf(domGlsl_setparam,elemArray),"array"); 
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domGlsl_setparam,_contents));


	//	Add attribute: ref
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( daeAtomicType::get("Glsl_identifier"));
		ma->setOffset( daeOffsetOf( domGlsl_setparam , attrRef ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: program
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "program" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGlsl_setparam , attrProgram ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGlsl_setparam));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domGlsl_setparam::_Meta = NULL;


