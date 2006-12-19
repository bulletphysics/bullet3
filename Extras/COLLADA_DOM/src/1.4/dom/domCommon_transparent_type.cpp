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
#include <dom/domCommon_transparent_type.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domCommon_transparent_type::create(daeInt bytes)
{
	domCommon_transparent_typeRef ref = new(bytes) domCommon_transparent_type;
	return ref;
}


daeMetaElement *
domCommon_transparent_type::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "common_transparent_type" );
	_Meta->registerClass(domCommon_transparent_type::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	cm = new daeMetaChoice( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "color" );
	mea->setOffset( daeOffsetOf(domCommon_transparent_type,elemColor) );
	mea->setElementType( domColor::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "param" );
	mea->setOffset( daeOffsetOf(domCommon_transparent_type,elemParam) );
	mea->setElementType( domParam::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "texture" );
	mea->setOffset( daeOffsetOf(domCommon_transparent_type,elemTexture) );
	mea->setElementType( domTexture::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	cm->setMaxOrdinal( 0 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domCommon_transparent_type,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domCommon_transparent_type,_contentsOrder));


	//	Add attribute: opaque
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "opaque" );
		ma->setType( daeAtomicType::get("Fx_opaque_enum"));
		ma->setOffset( daeOffsetOf( domCommon_transparent_type , attrOpaque ));
		ma->setContainer( _Meta );
		ma->setDefault( "A_ONE");
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCommon_transparent_type));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domCommon_transparent_type::_Meta = NULL;


