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
#include <dom/domSkin.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domSkin::create(daeInt bytes)
{
	domSkinRef ref = new(bytes) domSkin;
	ref->attrSource.setContainer( (domSkin*)ref );
	return ref;
}


daeMetaElement *
domSkin::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "skin" );
	_Meta->registerConstructor(domSkin::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "bind_shape_matrix" );
	mea->setOffset( daeOffsetOf(domSkin,elemBind_shape_matrix) );
	mea->setElementType( domSkin::domBind_shape_matrix::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 1, 3, -1 );
	mea->setName( "source" );
	mea->setOffset( daeOffsetOf(domSkin,elemSource_array) );
	mea->setElementType( domSource::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 1, 1 );
	mea->setName( "joints" );
	mea->setOffset( daeOffsetOf(domSkin,elemJoints) );
	mea->setElementType( domSkin::domJoints::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 3, 1, 1 );
	mea->setName( "vertex_weights" );
	mea->setOffset( daeOffsetOf(domSkin,elemVertex_weights) );
	mea->setElementType( domSkin::domVertex_weights::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 4, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domSkin,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 4 );
	_Meta->setCMRoot( cm );	

	//	Add attribute: source
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "source" );
		ma->setType( daeAtomicType::get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domSkin , attrSource ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domSkin));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domSkin::domBind_shape_matrix::create(daeInt bytes)
{
	domSkin::domBind_shape_matrixRef ref = new(bytes) domSkin::domBind_shape_matrix;
	return ref;
}


daeMetaElement *
domSkin::domBind_shape_matrix::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bind_shape_matrix" );
	_Meta->registerConstructor(domSkin::domBind_shape_matrix::create);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float4x4"));
		ma->setOffset( daeOffsetOf( domSkin::domBind_shape_matrix , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domSkin::domBind_shape_matrix));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domSkin::domJoints::create(daeInt bytes)
{
	domSkin::domJointsRef ref = new(bytes) domSkin::domJoints;
	return ref;
}


daeMetaElement *
domSkin::domJoints::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "joints" );
	_Meta->registerConstructor(domSkin::domJoints::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 2, -1 );
	mea->setName( "input" );
	mea->setOffset( daeOffsetOf(domSkin::domJoints,elemInput_array) );
	mea->setElementType( domInputLocal::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 1, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domSkin::domJoints,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 1 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domSkin::domJoints));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domSkin::domVertex_weights::create(daeInt bytes)
{
	domSkin::domVertex_weightsRef ref = new(bytes) domSkin::domVertex_weights;
	return ref;
}


daeMetaElement *
domSkin::domVertex_weights::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "vertex_weights" );
	_Meta->registerConstructor(domSkin::domVertex_weights::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 2, -1 );
	mea->setName( "input" );
	mea->setOffset( daeOffsetOf(domSkin::domVertex_weights,elemInput_array) );
	mea->setElementType( domInputLocalOffset::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "vcount" );
	mea->setOffset( daeOffsetOf(domSkin::domVertex_weights,elemVcount) );
	mea->setElementType( domSkin::domVertex_weights::domVcount::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 0, 1 );
	mea->setName( "v" );
	mea->setOffset( daeOffsetOf(domSkin::domVertex_weights,elemV) );
	mea->setElementType( domSkin::domVertex_weights::domV::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domSkin::domVertex_weights,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 3 );
	_Meta->setCMRoot( cm );	

	//	Add attribute: count
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "count" );
		ma->setType( daeAtomicType::get("Uint"));
		ma->setOffset( daeOffsetOf( domSkin::domVertex_weights , attrCount ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domSkin::domVertex_weights));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domSkin::domVertex_weights::domVcount::create(daeInt bytes)
{
	domSkin::domVertex_weights::domVcountRef ref = new(bytes) domSkin::domVertex_weights::domVcount;
	return ref;
}


daeMetaElement *
domSkin::domVertex_weights::domVcount::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "vcount" );
	_Meta->registerConstructor(domSkin::domVertex_weights::domVcount::create);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("ListOfUInts"));
		ma->setOffset( daeOffsetOf( domSkin::domVertex_weights::domVcount , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domSkin::domVertex_weights::domVcount));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domSkin::domVertex_weights::domV::create(daeInt bytes)
{
	domSkin::domVertex_weights::domVRef ref = new(bytes) domSkin::domVertex_weights::domV;
	return ref;
}


daeMetaElement *
domSkin::domVertex_weights::domV::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "v" );
	_Meta->registerConstructor(domSkin::domVertex_weights::domV::create);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("ListOfInts"));
		ma->setOffset( daeOffsetOf( domSkin::domVertex_weights::domV , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domSkin::domVertex_weights::domV));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domSkin::_Meta = NULL;
daeMetaElement * domSkin::domBind_shape_matrix::_Meta = NULL;
daeMetaElement * domSkin::domJoints::_Meta = NULL;
daeMetaElement * domSkin::domVertex_weights::_Meta = NULL;
daeMetaElement * domSkin::domVertex_weights::domVcount::_Meta = NULL;
daeMetaElement * domSkin::domVertex_weights::domV::_Meta = NULL;


