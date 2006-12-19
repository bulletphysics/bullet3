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
#include <dom/domCg_setparam.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domCg_setparam::create(daeInt bytes)
{
	domCg_setparamRef ref = new(bytes) domCg_setparam;
	return ref;
}


daeMetaElement *
domCg_setparam::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "cg_setparam" );
	_Meta->registerClass(domCg_setparam::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "cg_param_type" );
	mea->setOffset( daeOffsetOf(domCg_setparam,elemCg_param_type) );
	mea->setElementType( domCg_param_type::registerElement() );
	cm->appendChild( new daeMetaGroup( mea, _Meta, cm, 0, 1, 1 ) );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "usertype" );
	mea->setOffset( daeOffsetOf(domCg_setparam,elemUsertype) );
	mea->setElementType( domCg_setuser_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "array" );
	mea->setOffset( daeOffsetOf(domCg_setparam,elemArray) );
	mea->setElementType( domCg_setarray_type::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "connect_param" );
	mea->setOffset( daeOffsetOf(domCg_setparam,elemConnect_param) );
	mea->setElementType( domCg_connect_param::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domCg_setparam,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domCg_setparam,_contentsOrder));


	//	Add attribute: ref
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( daeAtomicType::get("Cg_identifier"));
		ma->setOffset( daeOffsetOf( domCg_setparam , attrRef ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: program
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "program" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domCg_setparam , attrProgram ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCg_setparam));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domCg_setparam::_Meta = NULL;


