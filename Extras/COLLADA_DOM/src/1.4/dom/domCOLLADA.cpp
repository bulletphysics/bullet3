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
#include <dom/domCOLLADA.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

extern daeString COLLADA_VERSION;
extern daeString COLLADA_NAMESPACE;

daeElementRef
domCOLLADA::create(daeInt bytes)
{
	domCOLLADARef ref = new(bytes) domCOLLADA;
	ref->attrXmlns.setContainer( (domCOLLADA*)ref );
	ref->attrXml_base.setContainer( (domCOLLADA*)ref );
	ref->_meta = _Meta;
	ref->_validAttributeArray.setCount( ref->_meta->getMetaAttributes().getCount() );
	ref->setAttribute("version", COLLADA_VERSION );
	ref->setAttribute("xmlns", COLLADA_NAMESPACE );
	ref->_meta = NULL;
	return ref;
}


daeMetaElement *
domCOLLADA::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "COLLADA" );
	_Meta->registerConstructor(domCOLLADA::create);

	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "asset" );
	mea->setOffset( daeOffsetOf(domCOLLADA,elemAsset) );
	mea->setElementType( domAsset::registerElement() );
	cm->appendChild( mea );
	
	cm = new daeMetaChoice( _Meta, cm, 1, 0, -1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "library_animations" );
	mea->setOffset( daeOffsetOf(domCOLLADA,elemLibrary_animations_array) );
	mea->setElementType( domLibrary_animations::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "library_animation_clips" );
	mea->setOffset( daeOffsetOf(domCOLLADA,elemLibrary_animation_clips_array) );
	mea->setElementType( domLibrary_animation_clips::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "library_cameras" );
	mea->setOffset( daeOffsetOf(domCOLLADA,elemLibrary_cameras_array) );
	mea->setElementType( domLibrary_cameras::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "library_controllers" );
	mea->setOffset( daeOffsetOf(domCOLLADA,elemLibrary_controllers_array) );
	mea->setElementType( domLibrary_controllers::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "library_geometries" );
	mea->setOffset( daeOffsetOf(domCOLLADA,elemLibrary_geometries_array) );
	mea->setElementType( domLibrary_geometries::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "library_effects" );
	mea->setOffset( daeOffsetOf(domCOLLADA,elemLibrary_effects_array) );
	mea->setElementType( domLibrary_effects::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "library_force_fields" );
	mea->setOffset( daeOffsetOf(domCOLLADA,elemLibrary_force_fields_array) );
	mea->setElementType( domLibrary_force_fields::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "library_images" );
	mea->setOffset( daeOffsetOf(domCOLLADA,elemLibrary_images_array) );
	mea->setElementType( domLibrary_images::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "library_lights" );
	mea->setOffset( daeOffsetOf(domCOLLADA,elemLibrary_lights_array) );
	mea->setElementType( domLibrary_lights::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "library_materials" );
	mea->setOffset( daeOffsetOf(domCOLLADA,elemLibrary_materials_array) );
	mea->setElementType( domLibrary_materials::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "library_nodes" );
	mea->setOffset( daeOffsetOf(domCOLLADA,elemLibrary_nodes_array) );
	mea->setElementType( domLibrary_nodes::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "library_physics_materials" );
	mea->setOffset( daeOffsetOf(domCOLLADA,elemLibrary_physics_materials_array) );
	mea->setElementType( domLibrary_physics_materials::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "library_physics_models" );
	mea->setOffset( daeOffsetOf(domCOLLADA,elemLibrary_physics_models_array) );
	mea->setElementType( domLibrary_physics_models::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "library_physics_scenes" );
	mea->setOffset( daeOffsetOf(domCOLLADA,elemLibrary_physics_scenes_array) );
	mea->setElementType( domLibrary_physics_scenes::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "library_visual_scenes" );
	mea->setOffset( daeOffsetOf(domCOLLADA,elemLibrary_visual_scenes_array) );
	mea->setElementType( domLibrary_visual_scenes::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	cm->getParent()->appendChild( cm );
	cm = cm->getParent();
	
	mea = new daeMetaElementAttribute( _Meta, cm, 3002, 0, 1 );
	mea->setName( "scene" );
	mea->setOffset( daeOffsetOf(domCOLLADA,elemScene) );
	mea->setElementType( domCOLLADA::domScene::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 3003, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domCOLLADA,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 3003 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domCOLLADA,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domCOLLADA,_contentsOrder));

    //	Add attribute: xmlns
    {
		daeMetaAttribute* ma = new daeMetaAttribute;
		ma->setName( "xmlns" );
		ma->setType( daeAtomicType::get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domCOLLADA , attrXmlns ));
		ma->setContainer( _Meta );
		//ma->setIsRequired( true );
		_Meta->appendAttribute(ma);
	}
    
	//	Add attribute: version
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "version" );
		ma->setType( daeAtomicType::get("VersionType"));
		ma->setOffset( daeOffsetOf( domCOLLADA , attrVersion ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: xml_base
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "xml_base" );
		ma->setType( daeAtomicType::get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domCOLLADA , attrXml_base ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domCOLLADA));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domCOLLADA::domScene::create(daeInt bytes)
{
	domCOLLADA::domSceneRef ref = new(bytes) domCOLLADA::domScene;
	return ref;
}


daeMetaElement *
domCOLLADA::domScene::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "scene" );
	_Meta->registerConstructor(domCOLLADA::domScene::create);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementArrayAttribute( _Meta, cm, 0, 0, -1 );
	mea->setName( "instance_physics_scene" );
	mea->setOffset( daeOffsetOf(domCOLLADA::domScene,elemInstance_physics_scene_array) );
	mea->setElementType( domInstanceWithExtra::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 0, 1 );
	mea->setName( "instance_visual_scene" );
	mea->setOffset( daeOffsetOf(domCOLLADA::domScene,elemInstance_visual_scene) );
	mea->setElementType( domInstanceWithExtra::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementArrayAttribute( _Meta, cm, 2, 0, -1 );
	mea->setName( "extra" );
	mea->setOffset( daeOffsetOf(domCOLLADA::domScene,elemExtra_array) );
	mea->setElementType( domExtra::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 2 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domCOLLADA::domScene));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domCOLLADA::_Meta = NULL;
daeMetaElement * domCOLLADA::domScene::_Meta = NULL;


