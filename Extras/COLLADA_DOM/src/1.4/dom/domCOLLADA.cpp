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

extern daeString COLLADA_VERSION;
extern daeString COLLADA_NAMESPACE;

daeElementRef
domCOLLADA::create(daeInt bytes)
{
	domCOLLADARef ref = new(bytes) domCOLLADA;
	ref->attrXmlns.setContainer( (domCOLLADA*)ref );
	ref->setAttribute("version", COLLADA_VERSION );
	ref->setAttribute("xmlns", COLLADA_NAMESPACE );
	return ref;
}


daeMetaElement *
domCOLLADA::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "COLLADA" );
	_Meta->setStaticPointerAddress(&domCOLLADA::_Meta);
	_Meta->registerConstructor(domCOLLADA::create);

	// Add elements: asset, library_animations, library_animation_clips, library_cameras, library_controllers, library_geometries, library_effects, library_force_fields, library_images, library_lights, library_materials, library_nodes, library_physics_materials, library_physics_models, library_physics_scenes, library_visual_scenes, scene, extra
    _Meta->appendElement(domAsset::registerElement(),daeOffsetOf(domCOLLADA,elemAsset));
    _Meta->appendArrayElement(domLibrary_animations::registerElement(),daeOffsetOf(domCOLLADA,elemLibrary_animations_array));
    _Meta->appendArrayElement(domLibrary_animation_clips::registerElement(),daeOffsetOf(domCOLLADA,elemLibrary_animation_clips_array));
    _Meta->appendArrayElement(domLibrary_cameras::registerElement(),daeOffsetOf(domCOLLADA,elemLibrary_cameras_array));
    _Meta->appendArrayElement(domLibrary_controllers::registerElement(),daeOffsetOf(domCOLLADA,elemLibrary_controllers_array));
    _Meta->appendArrayElement(domLibrary_geometries::registerElement(),daeOffsetOf(domCOLLADA,elemLibrary_geometries_array));
    _Meta->appendArrayElement(domLibrary_effects::registerElement(),daeOffsetOf(domCOLLADA,elemLibrary_effects_array));
    _Meta->appendArrayElement(domLibrary_force_fields::registerElement(),daeOffsetOf(domCOLLADA,elemLibrary_force_fields_array));
    _Meta->appendArrayElement(domLibrary_images::registerElement(),daeOffsetOf(domCOLLADA,elemLibrary_images_array));
    _Meta->appendArrayElement(domLibrary_lights::registerElement(),daeOffsetOf(domCOLLADA,elemLibrary_lights_array));
    _Meta->appendArrayElement(domLibrary_materials::registerElement(),daeOffsetOf(domCOLLADA,elemLibrary_materials_array));
    _Meta->appendArrayElement(domLibrary_nodes::registerElement(),daeOffsetOf(domCOLLADA,elemLibrary_nodes_array));
    _Meta->appendArrayElement(domLibrary_physics_materials::registerElement(),daeOffsetOf(domCOLLADA,elemLibrary_physics_materials_array));
    _Meta->appendArrayElement(domLibrary_physics_models::registerElement(),daeOffsetOf(domCOLLADA,elemLibrary_physics_models_array));
    _Meta->appendArrayElement(domLibrary_physics_scenes::registerElement(),daeOffsetOf(domCOLLADA,elemLibrary_physics_scenes_array));
    _Meta->appendArrayElement(domLibrary_visual_scenes::registerElement(),daeOffsetOf(domCOLLADA,elemLibrary_visual_scenes_array));
    _Meta->appendElement(domCOLLADA::domScene::registerElement(),daeOffsetOf(domCOLLADA,elemScene));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domCOLLADA,elemExtra_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domCOLLADA,_contents));

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
	_Meta->setStaticPointerAddress(&domCOLLADA::domScene::_Meta);
	_Meta->registerConstructor(domCOLLADA::domScene::create);

	// Add elements: instance_physics_scene, instance_visual_scene, extra
    _Meta->appendArrayElement(domInstanceWithExtra::registerElement(),daeOffsetOf(domCOLLADA::domScene,elemInstance_physics_scene_array),"instance_physics_scene"); 
    _Meta->appendElement(domInstanceWithExtra::registerElement(),daeOffsetOf(domCOLLADA::domScene,elemInstance_visual_scene),"instance_visual_scene"); 
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domCOLLADA::domScene,elemExtra_array));
	
	
	_Meta->setElementSize(sizeof(domCOLLADA::domScene));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domCOLLADA::_Meta = NULL;
daeMetaElement * domCOLLADA::domScene::_Meta = NULL;


