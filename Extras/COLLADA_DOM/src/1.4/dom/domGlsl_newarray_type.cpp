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
#include <dom/domGlsl_newarray_type.h>

daeElementRef
domGlsl_newarray_type::create(daeInt bytes)
{
	domGlsl_newarray_typeRef ref = new(bytes) domGlsl_newarray_type;
	return ref;
}


daeMetaElement *
domGlsl_newarray_type::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "glsl_newarray_type" );
	_Meta->setStaticPointerAddress(&domGlsl_newarray_type::_Meta);
	_Meta->registerConstructor(domGlsl_newarray_type::create);

	// Add elements: glsl_param_type, array
    _Meta->appendArrayElement(domGlsl_param_type::registerElement(),daeOffsetOf(domGlsl_newarray_type,elemGlsl_param_type_array));
	_Meta->appendPossibleChild( "bool", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float2x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float3x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float4x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "surface", _Meta->getMetaElements()[0], "fx_surface_common");
	_Meta->appendPossibleChild( "sampler1D", _Meta->getMetaElements()[0], "gl_sampler1D");
	_Meta->appendPossibleChild( "sampler2D", _Meta->getMetaElements()[0], "gl_sampler2D");
	_Meta->appendPossibleChild( "sampler3D", _Meta->getMetaElements()[0], "gl_sampler3D");
	_Meta->appendPossibleChild( "samplerCUBE", _Meta->getMetaElements()[0], "gl_samplerCUBE");
	_Meta->appendPossibleChild( "samplerRECT", _Meta->getMetaElements()[0], "gl_samplerRECT");
	_Meta->appendPossibleChild( "samplerDEPTH", _Meta->getMetaElements()[0], "gl_samplerDEPTH");
	_Meta->appendPossibleChild( "enum", _Meta->getMetaElements()[0]);
    _Meta->appendArrayElement(domGlsl_newarray_type::registerElement(),daeOffsetOf(domGlsl_newarray_type,elemArray_array),"array"); 
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domGlsl_newarray_type,_contents));


	//	Add attribute: length
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "length" );
		ma->setType( daeAtomicType::get("xsPositiveInteger"));
		ma->setOffset( daeOffsetOf( domGlsl_newarray_type , attrLength ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGlsl_newarray_type));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domGlsl_newarray_type::_Meta = NULL;


