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
#include <dom/domEffect.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domEffect::create(daeInt bytes)
{
	domEffectRef ref = new(bytes) domEffect;
	return ref;
}

#include <dom/domProfile_GLSL.h>
#include <dom/domProfile_COMMON.h>
#include <dom/domProfile_CG.h>
#include <dom/domProfile_GLES.h>

daeMetaElement *
domEffect::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "effect" );
	_Meta->registerConstructor(domEffect::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domEffect,elemAsset) );
	mea->setElementType( domAsset::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 1, 0, -1 );
	mea->setName( "annotate" );
	mea->setOffset( daeOffsetOf(domEffect,elemAnnotate_array) );
	mea->setElementType( domFx_annotate_common::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 2, 0, -1 );
	mea->setName( "image" );
	mea->setOffset( daeOffsetOf(domEffect,elemImage_array) );
	mea->setElementType( domImage::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3, 0, -1 );
	mea->setName( "newparam" );
	mea->setOffset( daeOffsetOf(domEffect,elemNewparam_array) );
	mea->setElementType( domFx_newparam_common::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 4, 1, -1 );
	mea->setName( "fx_profile_abstract" );
	mea->setOffset( daeOffsetOf(domEffect,elemFx_profile_abstract_array) );
	mea->setElementType( domFx_profile_abstract::registerElement() );
	cm->appendChild( mea );
	
    
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 4, 1, -1 );
	mea->setName( "profile_GLSL" );
	mea->setOffset( daeOffsetOf(domEffect,elemFx_profile_abstract_array) );
	mea->setElementType( domProfile_GLSL::registerElement() );
	cm->appendChild( mea );
	
    
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 4, 1, -1 );
	mea->setName( "profile_COMMON" );
	mea->setOffset( daeOffsetOf(domEffect,elemFx_profile_abstract_array) );
	mea->setElementType( domProfile_COMMON::registerElement() );
	cm->appendChild( mea );
	
    
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 4, 1, -1 );
	mea->setName( "profile_CG" );
	mea->setOffset( daeOffsetOf(domEffect,elemFx_profile_abstract_array) );
	mea->setElementType( domProfile_CG::registerElement() );
	cm->appendChild( mea );
	
    
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 4, 1, -1 );
	mea->setName( "profile_GLES" );
	mea->setOffset( daeOffsetOf(domEffect,elemFx_profile_abstract_array) );
	mea->setElementType( domProfile_GLES::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 5, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domEffect,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 5 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domEffect,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domEffect,_contentsOrder));


	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domEffect , attrId ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domEffect , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domEffect));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domEffect::_Meta = NULL;


