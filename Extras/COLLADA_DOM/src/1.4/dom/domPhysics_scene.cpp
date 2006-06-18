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
#include <dom/domPhysics_scene.h>

daeElementRef
domPhysics_scene::create(daeInt bytes)
{
	domPhysics_sceneRef ref = new(bytes) domPhysics_scene;
	return ref;
}


daeMetaElement *
domPhysics_scene::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "physics_scene" );
	_Meta->setStaticPointerAddress(&domPhysics_scene::_Meta);
	_Meta->registerConstructor(domPhysics_scene::create);

	// Add elements: asset, instance_force_field, instance_physics_model, technique_common, technique, extra
    _Meta->appendElement(domAsset::registerElement(),daeOffsetOf(domPhysics_scene,elemAsset));
    _Meta->appendArrayElement(domInstance_force_field::registerElement(),daeOffsetOf(domPhysics_scene,elemInstance_force_field_array));
    _Meta->appendArrayElement(domInstance_physics_model::registerElement(),daeOffsetOf(domPhysics_scene,elemInstance_physics_model_array));
    _Meta->appendElement(domPhysics_scene::domTechnique_common::registerElement(),daeOffsetOf(domPhysics_scene,elemTechnique_common));
    _Meta->appendArrayElement(domTechnique::registerElement(),daeOffsetOf(domPhysics_scene,elemTechnique_array));
    _Meta->appendArrayElement(domExtra::registerElement(),daeOffsetOf(domPhysics_scene,elemExtra_array));

	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domPhysics_scene , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: name
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "name" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domPhysics_scene , attrName ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domPhysics_scene));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domPhysics_scene::domTechnique_common::create(daeInt bytes)
{
	domPhysics_scene::domTechnique_commonRef ref = new(bytes) domPhysics_scene::domTechnique_common;
	return ref;
}


daeMetaElement *
domPhysics_scene::domTechnique_common::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "technique_common" );
	_Meta->setStaticPointerAddress(&domPhysics_scene::domTechnique_common::_Meta);
	_Meta->registerConstructor(domPhysics_scene::domTechnique_common::create);

	// Add elements: gravity, time_step
    _Meta->appendElement(domTargetableFloat3::registerElement(),daeOffsetOf(domPhysics_scene::domTechnique_common,elemGravity),"gravity"); 
    _Meta->appendElement(domTargetableFloat::registerElement(),daeOffsetOf(domPhysics_scene::domTechnique_common,elemTime_step),"time_step"); 
	
	
	_Meta->setElementSize(sizeof(domPhysics_scene::domTechnique_common));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domPhysics_scene::_Meta = NULL;
daeMetaElement * domPhysics_scene::domTechnique_common::_Meta = NULL;


