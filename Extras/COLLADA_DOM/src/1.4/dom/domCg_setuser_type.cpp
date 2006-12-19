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
#include <dom/domCg_setuser_type.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domCg_setuser_type::create(daeInt bytes)
{
	domCg_setuser_typeRef ref = new(bytes) domCg_setuser_type;
	return ref;
}


daeMetaElement *
domCg_setuser_type::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "cg_setuser_type" );
	_Meta->registerClass(domCg_setuser_type::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( _Meta, cm, 0, 0, 1 );

	cm = new daeMetaChoice( _Meta, cm, 0, 1, -1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "cg_param_type" );
	mea->setOffset( daeOffsetOf(domCg_setuser_type,elemCg_param_type_array) );
	mea->setElementType( domCg_param_type::registerElement() );
	cm->appendChild( new daeMetaGroup( mea, _Meta, cm, 0, 1, 1 ) );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "array" );
	mea->setOffset( daeOffsetOf(domCg_setuser_type,elemArray_array) );
	mea->setElementType( domCg_setarray_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "usertype" );
	mea->setOffset( daeOffsetOf(domCg_setuser_type,elemUsertype_array) );
	mea->setElementType( domCg_setuser_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "connect_param" );
	mea->setOffset( daeOffsetOf(domCg_setuser_type,elemConnect_param_array) );
	mea->setElementType( domCg_connect_param::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3001, 1, -1 );
	mea->setName( "setparam" );
	mea->setOffset( daeOffsetOf(domCg_setuser_type,elemSetparam_array) );
	mea->setElementType( domCg_setparam::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domCg_setuser_type,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domCg_setuser_type,_contentsOrder));


	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("Cg_identifier"));
		ma->setOffset( daeOffsetOf( domCg_setuser_type , attrName ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: source
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "source" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domCg_setuser_type , attrSource ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_setuser_type));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domCg_setuser_type::_Meta = NULL;


