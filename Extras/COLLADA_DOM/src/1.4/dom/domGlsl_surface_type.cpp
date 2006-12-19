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
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

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
	_Meta->registerClass(domGlsl_surface_type::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "fx_surface_init_common" );
	mea->setOffset( daeOffsetOf(domGlsl_surface_type,elemFx_surface_init_common) );
	mea->setElementType( domFx_surface_init_common::registerElement() );
	cm->appendChild( new daeMetaGroup( mea, _Meta, cm, 0, 0, 1 ) );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "format" );
	mea->setOffset( daeOffsetOf(domGlsl_surface_type,elemFormat) );
	mea->setElementType( domFormat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 0, 1 );
	mea->setName( "format_hint" );
	mea->setOffset( daeOffsetOf(domGlsl_surface_type,elemFormat_hint) );
	mea->setElementType( domFx_surface_format_hint_common::registerElement() );
	cm->appendChild( mea );
	
	cm = new daeMetaChoice( _Meta, cm, 3, 0, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "size" );
	mea->setOffset( daeOffsetOf(domGlsl_surface_type,elemSize) );
	mea->setElementType( domSize::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "viewport_ratio" );
	mea->setOffset( daeOffsetOf(domGlsl_surface_type,elemViewport_ratio) );
	mea->setElementType( domViewport_ratio::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	mea = new daeMetaElementAttribute( _Meta, cm, 4, 0, 1 );
	mea->setName( "mip_levels" );
	mea->setOffset( daeOffsetOf(domGlsl_surface_type,elemMip_levels) );
	mea->setElementType( domMip_levels::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 5, 0, 1 );
	mea->setName( "mipmap_generate" );
	mea->setOffset( daeOffsetOf(domGlsl_surface_type,elemMipmap_generate) );
	mea->setElementType( domMipmap_generate::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 6, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domGlsl_surface_type,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 6 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	cm = new daeMetaSequence( _Meta, cm, 7, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "generator" );
	mea->setOffset( daeOffsetOf(domGlsl_surface_type,elemGenerator) );
	mea->setElementType( domGenerator::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	cm->setMaxOrdinal( 7 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domGlsl_surface_type,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domGlsl_surface_type,_contentsOrder));


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
	_Meta->registerClass(domGlsl_surface_type::domGenerator::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 0, -1 );
	mea->setName( "annotate" );
	mea->setOffset( daeOffsetOf(domGlsl_surface_type::domGenerator,elemAnnotate_array) );
	mea->setElementType( domFx_annotate_common::registerElement() );
	cm->appendChild( mea );
	
	cm = new daeMetaChoice( _Meta, cm, 1, 1, -1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "code" );
	mea->setOffset( daeOffsetOf(domGlsl_surface_type::domGenerator,elemCode_array) );
	mea->setElementType( domFx_code_profile::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "include" );
	mea->setOffset( daeOffsetOf(domGlsl_surface_type::domGenerator,elemInclude_array) );
	mea->setElementType( domFx_include_common::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	mea = new daeMetaElementAttribute( _Meta, cm, 3002, 1, 1 );
	mea->setName( "name" );
	mea->setOffset( daeOffsetOf(domGlsl_surface_type::domGenerator,elemName) );
	mea->setElementType( domGlsl_surface_type::domGenerator::domName::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3003, 0, -1 );
	mea->setName( "setparam" );
	mea->setOffset( daeOffsetOf(domGlsl_surface_type::domGenerator,elemSetparam_array) );
	mea->setElementType( domGlsl_setparam_simple::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 3003 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domGlsl_surface_type::domGenerator,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domGlsl_surface_type::domGenerator,_contentsOrder));

	
	
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
	_Meta->registerClass(domGlsl_surface_type::domGenerator::domName::create, &_Meta);

	_Meta->setIsInnerClass( true );
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


