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
#include <dom/domNode.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domNode::create(daeInt bytes)
{
	domNodeRef ref = new(bytes) domNode;
	return ref;
}


daeMetaElement *
domNode::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "node" );
	_Meta->registerClass(domNode::create, &_Meta);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domNode,elemAsset) );
	mea->setElementType( domAsset::registerElement() );
	cm->appendChild( mea );
	
	cm = new daeMetaChoice( _Meta, cm, 1, 0, -1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "lookat" );
	mea->setOffset( daeOffsetOf(domNode,elemLookat_array) );
	mea->setElementType( domLookat::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "matrix" );
	mea->setOffset( daeOffsetOf(domNode,elemMatrix_array) );
	mea->setElementType( domMatrix::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "rotate" );
	mea->setOffset( daeOffsetOf(domNode,elemRotate_array) );
	mea->setElementType( domRotate::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "scale" );
	mea->setOffset( daeOffsetOf(domNode,elemScale_array) );
	mea->setElementType( domScale::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "skew" );
	mea->setOffset( daeOffsetOf(domNode,elemSkew_array) );
	mea->setElementType( domSkew::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "translate" );
	mea->setOffset( daeOffsetOf(domNode,elemTranslate_array) );
	mea->setElementType( domTranslate::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3002, 0, -1 );
	mea->setName( "instance_camera" );
	mea->setOffset( daeOffsetOf(domNode,elemInstance_camera_array) );
	mea->setElementType( domInstance_camera::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3003, 0, -1 );
	mea->setName( "instance_controller" );
	mea->setOffset( daeOffsetOf(domNode,elemInstance_controller_array) );
	mea->setElementType( domInstance_controller::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3004, 0, -1 );
	mea->setName( "instance_geometry" );
	mea->setOffset( daeOffsetOf(domNode,elemInstance_geometry_array) );
	mea->setElementType( domInstance_geometry::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3005, 0, -1 );
	mea->setName( "instance_light" );
	mea->setOffset( daeOffsetOf(domNode,elemInstance_light_array) );
	mea->setElementType( domInstance_light::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3006, 0, -1 );
	mea->setName( "instance_node" );
	mea->setOffset( daeOffsetOf(domNode,elemInstance_node_array) );
	mea->setElementType( domInstance_node::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3007, 0, -1 );
	mea->setName( "node" );
	mea->setOffset( daeOffsetOf(domNode,elemNode_array) );
	mea->setElementType( domNode::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3008, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domNode,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 3008 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domNode,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domNode,_contentsOrder));


	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domNode , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domNode , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domNode , attrSid ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: type
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "type" );
		ma->setType( daeAtomicType::get("NodeType"));
		ma->setOffset( daeOffsetOf( domNode , attrType ));
		ma->setContainer( _Meta );
		ma->setDefault( "NODE");
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: layer
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "layer" );
		ma->setType( daeAtomicType::get("ListOfNames"));
		ma->setOffset( daeOffsetOf( domNode , attrLayer ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domNode));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domNode::_Meta = NULL;


