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
#include <dom/domMorph.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domMorph::create(daeInt bytes)
{
	domMorphRef ref = new(bytes) domMorph;
	ref->attrSource.setContainer( (domMorph*)ref );
	return ref;
}


daeMetaElement *
domMorph::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "morph" );
	_Meta->registerClass(domMorph::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 2, -1 );
	mea->setName( "source" );
	mea->setOffset( daeOffsetOf(domMorph,elemSource_array) );
	mea->setElementType( domSource::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 1, 1 );
	mea->setName( "targets" );
	mea->setOffset( daeOffsetOf(domMorph,elemTargets) );
	mea->setElementType( domMorph::domTargets::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 2, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domMorph,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 2 );
	_Meta->setCMRoot( cm );	

	//	Add attribute: method
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "method" );
		ma->setType( daeAtomicType::get("MorphMethodType"));
		ma->setOffset( daeOffsetOf( domMorph , attrMethod ));
		ma->setContainer( _Meta );
		ma->setDefault( "NORMALIZED");
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: source
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "source" );
		ma->setType( daeAtomicType::get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domMorph , attrSource ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domMorph));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domMorph::domTargets::create(daeInt bytes)
{
	domMorph::domTargetsRef ref = new(bytes) domMorph::domTargets;
	return ref;
}


daeMetaElement *
domMorph::domTargets::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "targets" );
	_Meta->registerClass(domMorph::domTargets::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 2, -1 );
	mea->setName( "input" );
	mea->setOffset( daeOffsetOf(domMorph::domTargets,elemInput_array) );
	mea->setElementType( domInputLocal::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 1, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domMorph::domTargets,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 1 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domMorph::domTargets));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domMorph::_Meta = NULL;
daeMetaElement * domMorph::domTargets::_Meta = NULL;


