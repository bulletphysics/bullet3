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
#include <dom/domGlsl_surface_type.h>

daeElementRef
domGlsl_surface_type::create(daeInt bytes)
{
	domGlsl_surface_typeRef ref = new(bytes) domGlsl_surface_type;
	return ref;
}


daeMetaElement *
domGlsl_surface_type::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "glsl_surface_type" );
	_Meta->setStaticPointerAddress(&domGlsl_surface_type::_Meta);
	_Meta->registerConstructor(domGlsl_surface_type::create);

	// Add elements: init_from, format, size, viewport_ratio, mip_levels, mipmap_generate, generator
    _Meta->appendArrayElement(domInit_from::registerElement(),daeOffsetOf(domGlsl_surface_type,elemInit_from_array));
    _Meta->appendElement(domFormat::registerElement(),daeOffsetOf(domGlsl_surface_type,elemFormat));
    _Meta->appendElement(domSize::registerElement(),daeOffsetOf(domGlsl_surface_type,elemSize));
    _Meta->appendElement(domViewport_ratio::registerElement(),daeOffsetOf(domGlsl_surface_type,elemViewport_ratio));
    _Meta->appendElement(domMip_levels::registerElement(),daeOffsetOf(domGlsl_surface_type,elemMip_levels));
    _Meta->appendElement(domMipmap_generate::registerElement(),daeOffsetOf(domGlsl_surface_type,elemMipmap_generate));
    _Meta->appendElement(domGenerator::registerElement(),daeOffsetOf(domGlsl_surface_type,elemGenerator));

	//	Add attribute: type
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "type" );
		ma->setType( daeAtomicType::get("Fx_surface_type_enum"));
		ma->setOffset( daeOffsetOf( domGlsl_surface_type , attrType ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGlsl_surface_type));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGlsl_surface_type::domGenerator::create(daeInt bytes)
{
	domGlsl_surface_type::domGeneratorRef ref = new(bytes) domGlsl_surface_type::domGenerator;
	return ref;
}


daeMetaElement *
domGlsl_surface_type::domGenerator::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "generator" );
	_Meta->setStaticPointerAddress(&domGlsl_surface_type::domGenerator::_Meta);
	_Meta->registerConstructor(domGlsl_surface_type::domGenerator::create);

	// Add elements: annotate, code, include, name, setparam
    _Meta->appendArrayElement(domFx_annotate_common::registerElement(),daeOffsetOf(domGlsl_surface_type::domGenerator,elemAnnotate_array),"annotate"); 
    _Meta->appendArrayElement(domFx_code_profile::registerElement(),daeOffsetOf(domGlsl_surface_type::domGenerator,elemCode_array),"code"); 
    _Meta->appendArrayElement(domFx_include_common::registerElement(),daeOffsetOf(domGlsl_surface_type::domGenerator,elemInclude_array),"include"); 
    _Meta->appendElement(domGlsl_surface_type::domGenerator::domName::registerElement(),daeOffsetOf(domGlsl_surface_type::domGenerator,elemName));
    _Meta->appendArrayElement(domGlsl_setparam_simple::registerElement(),daeOffsetOf(domGlsl_surface_type::domGenerator,elemSetparam_array),"setparam"); 
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domGlsl_surface_type::domGenerator,_contents));

	
	
	_Meta->setElementSize(sizeof(domGlsl_surface_type::domGenerator));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGlsl_surface_type::domGenerator::domName::create(daeInt bytes)
{
	domGlsl_surface_type::domGenerator::domNameRef ref = new(bytes) domGlsl_surface_type::domGenerator::domName;
	return ref;
}


daeMetaElement *
domGlsl_surface_type::domGenerator::domName::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "name" );
	_Meta->setStaticPointerAddress(&domGlsl_surface_type::domGenerator::domName::_Meta);
	_Meta->registerConstructor(domGlsl_surface_type::domGenerator::domName::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGlsl_surface_type::domGenerator::domName , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: source
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "source" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGlsl_surface_type::domGenerator::domName , attrSource ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGlsl_surface_type::domGenerator::domName));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domGlsl_surface_type::_Meta = NULL;
daeMetaElement * domGlsl_surface_type::domGenerator::_Meta = NULL;
daeMetaElement * domGlsl_surface_type::domGenerator::domName::_Meta = NULL;


