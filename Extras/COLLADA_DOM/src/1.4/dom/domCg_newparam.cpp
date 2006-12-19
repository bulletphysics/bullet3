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
#include <dom/domCg_newparam.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domCg_newparam::create(daeInt bytes)
{
	domCg_newparamRef ref = new(bytes) domCg_newparam;
	return ref;
}


daeMetaElement *
domCg_newparam::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "cg_newparam" );
	_Meta->registerClass(domCg_newparam::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 0, -1 );
	mea->setName( "annotate" );
	mea->setOffset( daeOffsetOf(domCg_newparam,elemAnnotate_array) );
	mea->setElementType( domFx_annotate_common::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "semantic" );
	mea->setOffset( daeOffsetOf(domCg_newparam,elemSemantic) );
	mea->setElementType( domCg_newparam::domSemantic::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 0, 1 );
	mea->setName( "modifier" );
	mea->setOffset( daeOffsetOf(domCg_newparam,elemModifier) );
	mea->setElementType( domCg_newparam::domModifier::registerElement() );
	cm->appendChild( mea );
	
	cm = new daeMetaChoice( _Meta, cm, 3, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "cg_param_type" );
	mea->setOffset( daeOffsetOf(domCg_newparam,elemCg_param_type) );
	mea->setElementType( domCg_param_type::registerElement() );
	cm->appendChild( new daeMetaGroup( mea, _Meta, cm, 0, 1, 1 ) );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "usertype" );
	mea->setOffset( daeOffsetOf(domCg_newparam,elemUsertype) );
	mea->setElementType( domCg_setuser_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "array" );
	mea->setOffset( daeOffsetOf(domCg_newparam,elemArray) );
	mea->setElementType( domCg_newarray_type::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	cm->setMaxOrdinal( 3 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domCg_newparam,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domCg_newparam,_contentsOrder));


	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("Cg_identifier"));
		ma->setOffset( daeOffsetOf( domCg_newparam , attrSid ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_newparam));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_newparam::domSemantic::create(daeInt bytes)
{
	domCg_newparam::domSemanticRef ref = new(bytes) domCg_newparam::domSemantic;
	return ref;
}


daeMetaElement *
domCg_newparam::domSemantic::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "semantic" );
	_Meta->registerClass(domCg_newparam::domSemantic::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domCg_newparam::domSemantic , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_newparam::domSemantic));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCg_newparam::domModifier::create(daeInt bytes)
{
	domCg_newparam::domModifierRef ref = new(bytes) domCg_newparam::domModifier;
	return ref;
}


daeMetaElement *
domCg_newparam::domModifier::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "modifier" );
	_Meta->registerClass(domCg_newparam::domModifier::create, &_Meta);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_modifier_enum_common"));
		ma->setOffset( daeOffsetOf( domCg_newparam::domModifier , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_newparam::domModifier));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domCg_newparam::_Meta = NULL;
daeMetaElement * domCg_newparam::domSemantic::_Meta = NULL;
daeMetaElement * domCg_newparam::domModifier::_Meta = NULL;


