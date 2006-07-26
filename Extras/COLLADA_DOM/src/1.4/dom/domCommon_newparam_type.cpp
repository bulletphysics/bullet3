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
#include <dom/domCommon_newparam_type.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domCommon_newparam_type::create(daeInt bytes)
{
	domCommon_newparam_typeRef ref = new(bytes) domCommon_newparam_type;
	return ref;
}


daeMetaElement *
domCommon_newparam_type::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "common_newparam_type" );
	_Meta->registerConstructor(domCommon_newparam_type::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "semantic" );
	mea->setOffset( daeOffsetOf(domCommon_newparam_type,elemSemantic) );
	mea->setElementType( domCommon_newparam_type::domSemantic::registerElement() );
	cm->appendChild( mea );
	
	cm = new daeMetaChoice( _Meta, cm, 1, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float" );
	mea->setOffset( daeOffsetOf(domCommon_newparam_type,elemFloat) );
	mea->setElementType( domCommon_newparam_type::domFloat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float2" );
	mea->setOffset( daeOffsetOf(domCommon_newparam_type,elemFloat2) );
	mea->setElementType( domCommon_newparam_type::domFloat2::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float3" );
	mea->setOffset( daeOffsetOf(domCommon_newparam_type,elemFloat3) );
	mea->setElementType( domCommon_newparam_type::domFloat3::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "float4" );
	mea->setOffset( daeOffsetOf(domCommon_newparam_type,elemFloat4) );
	mea->setElementType( domCommon_newparam_type::domFloat4::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "surface" );
	mea->setOffset( daeOffsetOf(domCommon_newparam_type,elemSurface) );
	mea->setElementType( domFx_surface_common::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "sampler2D" );
	mea->setOffset( daeOffsetOf(domCommon_newparam_type,elemSampler2D) );
	mea->setElementType( domFx_sampler2D_common::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	cm->setMaxOrdinal( 1 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domCommon_newparam_type,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domCommon_newparam_type,_contentsOrder));


	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domCommon_newparam_type , attrSid ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCommon_newparam_type));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCommon_newparam_type::domSemantic::create(daeInt bytes)
{
	domCommon_newparam_type::domSemanticRef ref = new(bytes) domCommon_newparam_type::domSemantic;
	return ref;
}


daeMetaElement *
domCommon_newparam_type::domSemantic::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "semantic" );
	_Meta->registerConstructor(domCommon_newparam_type::domSemantic::create);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domCommon_newparam_type::domSemantic , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCommon_newparam_type::domSemantic));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCommon_newparam_type::domFloat::create(daeInt bytes)
{
	domCommon_newparam_type::domFloatRef ref = new(bytes) domCommon_newparam_type::domFloat;
	return ref;
}


daeMetaElement *
domCommon_newparam_type::domFloat::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float" );
	_Meta->registerConstructor(domCommon_newparam_type::domFloat::create);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domCommon_newparam_type::domFloat , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCommon_newparam_type::domFloat));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCommon_newparam_type::domFloat2::create(daeInt bytes)
{
	domCommon_newparam_type::domFloat2Ref ref = new(bytes) domCommon_newparam_type::domFloat2;
	return ref;
}


daeMetaElement *
domCommon_newparam_type::domFloat2::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float2" );
	_Meta->registerConstructor(domCommon_newparam_type::domFloat2::create);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float2"));
		ma->setOffset( daeOffsetOf( domCommon_newparam_type::domFloat2 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCommon_newparam_type::domFloat2));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCommon_newparam_type::domFloat3::create(daeInt bytes)
{
	domCommon_newparam_type::domFloat3Ref ref = new(bytes) domCommon_newparam_type::domFloat3;
	return ref;
}


daeMetaElement *
domCommon_newparam_type::domFloat3::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float3" );
	_Meta->registerConstructor(domCommon_newparam_type::domFloat3::create);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float3"));
		ma->setOffset( daeOffsetOf( domCommon_newparam_type::domFloat3 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCommon_newparam_type::domFloat3));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCommon_newparam_type::domFloat4::create(daeInt bytes)
{
	domCommon_newparam_type::domFloat4Ref ref = new(bytes) domCommon_newparam_type::domFloat4;
	return ref;
}


daeMetaElement *
domCommon_newparam_type::domFloat4::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "float4" );
	_Meta->registerConstructor(domCommon_newparam_type::domFloat4::create);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domCommon_newparam_type::domFloat4 , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCommon_newparam_type::domFloat4));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domCommon_newparam_type::_Meta = NULL;
daeMetaElement * domCommon_newparam_type::domSemantic::_Meta = NULL;
daeMetaElement * domCommon_newparam_type::domFloat::_Meta = NULL;
daeMetaElement * domCommon_newparam_type::domFloat2::_Meta = NULL;
daeMetaElement * domCommon_newparam_type::domFloat3::_Meta = NULL;
daeMetaElement * domCommon_newparam_type::domFloat4::_Meta = NULL;


