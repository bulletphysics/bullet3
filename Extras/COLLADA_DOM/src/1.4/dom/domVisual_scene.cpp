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
#include <dom/domVisual_scene.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domVisual_scene::create(daeInt bytes)
{
	domVisual_sceneRef ref = new(bytes) domVisual_scene;
	return ref;
}


daeMetaElement *
domVisual_scene::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "visual_scene" );
	_Meta->registerConstructor(domVisual_scene::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domVisual_scene,elemAsset) );
	mea->setElementType( domAsset::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 1, 1, -1 );
	mea->setName( "node" );
	mea->setOffset( daeOffsetOf(domVisual_scene,elemNode_array) );
	mea->setElementType( domNode::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 2, 0, -1 );
	mea->setName( "evaluate_scene" );
	mea->setOffset( daeOffsetOf(domVisual_scene,elemEvaluate_scene_array) );
	mea->setElementType( domVisual_scene::domEvaluate_scene::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domVisual_scene,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 3 );
	_Meta->setCMRoot( cm );	

	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domVisual_scene , attrId ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domVisual_scene , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domVisual_scene));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domVisual_scene::domEvaluate_scene::create(daeInt bytes)
{
	domVisual_scene::domEvaluate_sceneRef ref = new(bytes) domVisual_scene::domEvaluate_scene;
	return ref;
}


daeMetaElement *
domVisual_scene::domEvaluate_scene::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "evaluate_scene" );
	_Meta->registerConstructor(domVisual_scene::domEvaluate_scene::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, -1 );
	mea->setName( "render" );
	mea->setOffset( daeOffsetOf(domVisual_scene::domEvaluate_scene,elemRender_array) );
	mea->setElementType( domVisual_scene::domEvaluate_scene::domRender::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	_Meta->setCMRoot( cm );	

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domVisual_scene::domEvaluate_scene , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domVisual_scene::domEvaluate_scene));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domVisual_scene::domEvaluate_scene::domRender::create(daeInt bytes)
{
	domVisual_scene::domEvaluate_scene::domRenderRef ref = new(bytes) domVisual_scene::domEvaluate_scene::domRender;
	ref->attrCamera_node.setContainer( (domVisual_scene::domEvaluate_scene::domRender*)ref );
	return ref;
}


daeMetaElement *
domVisual_scene::domEvaluate_scene::domRender::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "render" );
	_Meta->registerConstructor(domVisual_scene::domEvaluate_scene::domRender::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 0, -1 );
	mea->setName( "layer" );
	mea->setOffset( daeOffsetOf(domVisual_scene::domEvaluate_scene::domRender,elemLayer_array) );
	mea->setElementType( domVisual_scene::domEvaluate_scene::domRender::domLayer::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "instance_effect" );
	mea->setOffset( daeOffsetOf(domVisual_scene::domEvaluate_scene::domRender,elemInstance_effect) );
	mea->setElementType( domInstance_effect::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 1 );
	_Meta->setCMRoot( cm );	

	//	Add attribute: camera_node
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "camera_node" );
		ma->setType( daeAtomicType::get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domVisual_scene::domEvaluate_scene::domRender , attrCamera_node ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domVisual_scene::domEvaluate_scene::domRender));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domVisual_scene::domEvaluate_scene::domRender::domLayer::create(daeInt bytes)
{
	domVisual_scene::domEvaluate_scene::domRender::domLayerRef ref = new(bytes) domVisual_scene::domEvaluate_scene::domRender::domLayer;
	return ref;
}


daeMetaElement *
domVisual_scene::domEvaluate_scene::domRender::domLayer::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "layer" );
	_Meta->registerConstructor(domVisual_scene::domEvaluate_scene::domRender::domLayer::create);

	_Meta->setIsInnerClass( true );
	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domVisual_scene::domEvaluate_scene::domRender::domLayer , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domVisual_scene::domEvaluate_scene::domRender::domLayer));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domVisual_scene::_Meta = NULL;
daeMetaElement * domVisual_scene::domEvaluate_scene::_Meta = NULL;
daeMetaElement * domVisual_scene::domEvaluate_scene::domRender::_Meta = NULL;
daeMetaElement * domVisual_scene::domEvaluate_scene::domRender::domLayer::_Meta = NULL;


