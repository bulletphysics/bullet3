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
	_Meta->setStaticPointerAddress(&domNode::_Meta);
	_Meta->registerConstructor(domNode::create);

	// Add elements: asset, lookat, matrix, rotate, scale, skew, translate, instance_camera, instance_controller, instance_geometry, instance_light, instance_node, node, extra
    _Meta->appendElement(domAsset::registerElement(),daeOffsetOf(domNode,elemAsset));
    _Meta->appendArrayElement(domLookat::registerElement(),daeOffsetOf(domNode,elemLookat_array));
    _Meta->appendArrayElement(domMatrix::registerElement(),daeOffsetOf(domNode,elemMatrix_array));
    _Meta->appendArrayElement(domRotate::registerElement(),daeOffsetOf(domNode,elemRotate_array));
    _Meta->appendArrayElement(domScale::registerElement(),daeOffsetOf(domNode,elemScale_array));
    _Meta->appendArrayElement(domSkew::registerElement(),daeOffsetOf(domNode,elemSkew_array));
    _Meta->appendArrayElement(domTranslate::registerElement(),daeOffsetOf(domNode,elemTranslate_array));
    _Meta->appendArrayElement(domInstance_camera::registerElement(),daeOffsetOf(domNode,elemInstance_camera_array));
    _Meta->appendArrayElement(domInstance_controller::registerElement(),daeOffsetOf(domNode,elemInstance_controller_array));
    _Meta->appendArrayElement(domInstance_geometry::registerElement(),daeOffsetOf(domNode,elemInstance_geometry_array));
    _Meta->appendArrayElement(domInstance_light::registerElement(),daeOffsetOf(domNode,elemInstance_light_array));
    _Meta->appendArrayElement(domInstance_node::registerElement(),daeOffsetOf(domNode,elemInstance_node_array));
    _Meta->appendArrayElement(domNode::registerElement(),daeOffsetOf(domNode,elemNode_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domNode,elemExtra_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domNode,_contents));


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


