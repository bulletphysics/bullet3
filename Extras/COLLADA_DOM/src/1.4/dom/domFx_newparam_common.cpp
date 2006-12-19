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
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

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
	_Meta->registerClass(domFx_newparam_common::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 0, -1 );
	mea->setName( "annotate" );
	mea->setOffset( daeOffsetOf(domFx_newparam_common,elemAnnotate_array) );
	mea->setElementType( domFx_annotate_common::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "semantic" );
	mea->setOffset( daeOffsetOf(domFx_newparam_common,elemSemantic) );
	mea->setElementType( domFx_newparam_common::domSemantic::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 0, 1 );
	mea->setName( "modifier" );
	mea->setOffset( daeOffsetOf(domFx_newparam_common,elemModifier) );
	mea->setElementType( domFx_newparam_common::domModifier::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 3, 1, 1 );
	mea->setName( "fx_basic_type_common" );
	mea->setOffset( daeOffsetOf(domFx_newparam_common,elemFx_basic_type_common) );
	mea->setElementType( domFx_basic_type_common::registerElement() );
	cm->appendChild( new daeMetaGroup( mea, _Meta, cm, 3, 1, 1 ) );
	
	cm->setMaxOrdinal( 3 );
	_Meta->setCMRoot( cm );	

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
	_Meta->registerClass(domFx_newparam_common::domSemantic::create, &_Meta);

	_Meta->setIsInnerClass( true );
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
	_Meta->registerClass(domFx_newparam_common::domModifier::create, &_Meta);

	_Meta->setIsInnerClass( true );
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


