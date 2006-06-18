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
#include <dom/domTechnique.h>

#include <dom/domCOLLADA.h>
#include <dom/domIDREF_array.h>
#include <dom/domName_array.h>
#include <dom/domBool_array.h>
#include <dom/domFloat_array.h>
#include <dom/domInt_array.h>
#include <dom/domAccessor.h>
#include <dom/domParam.h>
#include <dom/domSource.h>
#include <dom/domGeometry.h>
#include <dom/domMesh.h>
#include <dom/domSpline.h>
#include <dom/domP.h>
#include <dom/domLines.h>
#include <dom/domLinestrips.h>
#include <dom/domPolygons.h>
#include <dom/domPolylist.h>
#include <dom/domTriangles.h>
#include <dom/domTrifans.h>
#include <dom/domTristrips.h>
#include <dom/domVertices.h>
#include <dom/domLookat.h>
#include <dom/domMatrix.h>
#include <dom/domRotate.h>
#include <dom/domScale.h>
#include <dom/domSkew.h>
#include <dom/domTranslate.h>
#include <dom/domImage.h>
#include <dom/domLight.h>
#include <dom/domMaterial.h>
#include <dom/domCamera.h>
#include <dom/domAnimation.h>
#include <dom/domAnimation_clip.h>
#include <dom/domChannel.h>
#include <dom/domSampler.h>
#include <dom/domController.h>
#include <dom/domSkin.h>
#include <dom/domMorph.h>
#include <dom/domAsset.h>
#include <dom/domExtra.h>
#include <dom/domTechnique.h>
#include <dom/domNode.h>
#include <dom/domVisual_scene.h>
#include <dom/domBind_material.h>
#include <dom/domInstance_camera.h>
#include <dom/domInstance_controller.h>
#include <dom/domInstance_effect.h>
#include <dom/domInstance_force_field.h>
#include <dom/domInstance_geometry.h>
#include <dom/domInstance_light.h>
#include <dom/domInstance_material.h>
#include <dom/domInstance_node.h>
#include <dom/domInstance_physics_material.h>
#include <dom/domInstance_physics_model.h>
#include <dom/domInstance_rigid_body.h>
#include <dom/domInstance_rigid_constraint.h>
#include <dom/domLibrary_animations.h>
#include <dom/domLibrary_animation_clips.h>
#include <dom/domLibrary_cameras.h>
#include <dom/domLibrary_controllers.h>
#include <dom/domLibrary_geometries.h>
#include <dom/domLibrary_effects.h>
#include <dom/domLibrary_force_fields.h>
#include <dom/domLibrary_images.h>
#include <dom/domLibrary_lights.h>
#include <dom/domLibrary_materials.h>
#include <dom/domLibrary_nodes.h>
#include <dom/domLibrary_physics_materials.h>
#include <dom/domLibrary_physics_models.h>
#include <dom/domLibrary_physics_scenes.h>
#include <dom/domLibrary_visual_scenes.h>
#include <dom/domEffect.h>
#include <dom/domProfile_GLSL.h>
#include <dom/domProfile_COMMON.h>
#include <dom/domProfile_CG.h>
#include <dom/domProfile_GLES.h>
#include <dom/domBox.h>
#include <dom/domPlane.h>
#include <dom/domSphere.h>
#include <dom/domEllipsoid.h>
#include <dom/domCylinder.h>
#include <dom/domTapered_cylinder.h>
#include <dom/domCapsule.h>
#include <dom/domTapered_capsule.h>
#include <dom/domConvex_mesh.h>
#include <dom/domForce_field.h>
#include <dom/domPhysics_material.h>
#include <dom/domPhysics_scene.h>
#include <dom/domRigid_body.h>
#include <dom/domRigid_constraint.h>
#include <dom/domPhysics_model.h>

daeElementRef
domTechnique::create(daeInt bytes)
{
	domTechniqueRef ref = new(bytes) domTechnique;
	ref->attrXmlns.setContainer( (domTechnique*)ref );
	return ref;
}


daeMetaElement *
domTechnique::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "technique" );
	_Meta->setStaticPointerAddress(&domTechnique::_Meta);
	_Meta->registerConstructor(domTechnique::create);

	// Add elements: 
	_Meta->setAllowsAny( true );
	_Meta->appendArrayElement( domCOLLADA::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domIDREF_array::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domName_array::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domBool_array::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domFloat_array::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domInt_array::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domAccessor::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domParam::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domSource::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domGeometry::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domMesh::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domSpline::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domP::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domLines::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domLinestrips::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domPolygons::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domPolylist::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domTriangles::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domTrifans::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domTristrips::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domVertices::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domLookat::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domMatrix::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domRotate::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domScale::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domSkew::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domTranslate::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domImage::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domLight::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domMaterial::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domCamera::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domAnimation::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domAnimation_clip::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domChannel::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domSampler::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domController::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domSkin::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domMorph::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domAsset::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domExtra::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domTechnique::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domNode::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domVisual_scene::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domBind_material::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domInstance_camera::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domInstance_controller::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domInstance_effect::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domInstance_force_field::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domInstance_geometry::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domInstance_light::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domInstance_material::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domInstance_node::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domInstance_physics_material::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domInstance_physics_model::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domInstance_rigid_body::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domInstance_rigid_constraint::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domLibrary_animations::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domLibrary_animation_clips::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domLibrary_cameras::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domLibrary_controllers::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domLibrary_geometries::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domLibrary_effects::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domLibrary_force_fields::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domLibrary_images::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domLibrary_lights::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domLibrary_materials::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domLibrary_nodes::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domLibrary_physics_materials::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domLibrary_physics_models::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domLibrary_physics_scenes::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domLibrary_visual_scenes::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domEffect::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domProfile_GLSL::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domProfile_COMMON::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domProfile_CG::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domProfile_GLES::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domBox::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domPlane::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domSphere::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domEllipsoid::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domCylinder::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domTapered_cylinder::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domCapsule::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domTapered_capsule::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domConvex_mesh::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domForce_field::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domPhysics_material::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domPhysics_scene::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domRigid_body::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domRigid_constraint::registerElement(), daeOffsetOf(domTechnique, _contents));
	_Meta->appendArrayElement( domPhysics_model::registerElement(), daeOffsetOf(domTechnique, _contents));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domTechnique,_contents));

    //	Add attribute: xmlns
    {
		daeMetaAttribute* ma = new daeMetaAttribute;
		ma->setName( "xmlns" );
		ma->setType( daeAtomicType::get("xsAnyURI"));
		ma->setOffset( daeOffsetOf( domTechnique , attrXmlns ));
		ma->setContainer( _Meta );
		//ma->setIsRequired( true );
		_Meta->appendAttribute(ma);
	}
    
	//	Add attribute: profile
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "profile" );
		ma->setType( daeAtomicType::get("xsNMTOKEN"));
		ma->setOffset( daeOffsetOf( domTechnique , attrProfile ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domTechnique));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domTechnique::_Meta = NULL;


