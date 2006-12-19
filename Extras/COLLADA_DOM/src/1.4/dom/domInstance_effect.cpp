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
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

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
	_Meta->registerClass(domInstance_effect::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 0, -1 );
	mea->setName( "technique_hint" );
	mea->setOffset( daeOffsetOf(domInstance_effect,elemTechnique_hint_array) );
	mea->setElementType( domInstance_effect::domTechnique_hint::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 1, 0, -1 );
	mea->setName( "setparam" );
	mea->setOffset( daeOffsetOf(domInstance_effect,elemSetparam_array) );
	mea->setElementType( domInstance_effect::domSetparam::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 2, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domInstance_effect,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 2 );
	_Meta->setCMRoot( cm );	

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

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_effect , attrSid ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_effect , attrName ));
		ma->setContainer( _Meta );
	
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
	_Meta->registerClass(domInstance_effect::domTechnique_hint::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: platform
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "platform" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_effect::domTechnique_hint , attrPlatform ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: profile
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "profile" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domInstance_effect::domTechnique_hint , attrProfile ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
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
	_Meta->registerClass(domInstance_effect::domSetparam::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "fx_basic_type_common" );
	mea->setOffset( daeOffsetOf(domInstance_effect::domSetparam,elemFx_basic_type_common) );
	mea->setElementType( domFx_basic_type_common::registerElement() );
	cm->appendChild( new daeMetaGroup( mea, _Meta, cm, 0, 1, 1 ) );
	
	cm->setMaxOrdinal( 0 );
	_Meta->setCMRoot( cm );	

	//	Add attribute: ref
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( daeAtomicType::get("xsToken"));
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


