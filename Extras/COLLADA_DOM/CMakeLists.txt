INCLUDE_DIRECTORIES(
${BULLET_PHYSICS_SOURCE_DIR}/Extras/LibXML/include ${BULLET_PHYSICS_SOURCE_DIR}/Extras/COLLADA_DOM/include ${BULLET_PHYSICS_SOURCE_DIR}/Extras/COLLADA_DOM/include/1.4
)

ADD_DEFINITIONS( -D_CRT_SECURE_NO_WARNINGS)
ADD_DEFINITIONS( -D_CRT_SECURE_NO_DEPRECATE)


ADD_LIBRARY(ColladaDom
			src/1.4/dom/domAccessor.cpp
			src/1.4/dom/domAnimation.cpp
			src/1.4/dom/domAnimation_clip.cpp
			src/1.4/dom/domAsset.cpp
			src/1.4/dom/domBind_material.cpp
			src/1.4/dom/domBool_array.cpp
			src/1.4/dom/domBox.cpp
			src/1.4/dom/domCamera.cpp
			src/1.4/dom/domCapsule.cpp
			src/1.4/dom/domCg_connect_param.cpp
			src/1.4/dom/domCg_newarray_type.cpp
			src/1.4/dom/domCg_newparam.cpp
			src/1.4/dom/domCg_param_type.cpp
			src/1.4/dom/domCg_sampler1D.cpp
			src/1.4/dom/domCg_sampler2D.cpp
			src/1.4/dom/domCg_sampler3D.cpp
			src/1.4/dom/domCg_samplerCUBE.cpp
			src/1.4/dom/domCg_samplerDEPTH.cpp
			src/1.4/dom/domCg_samplerRECT.cpp
			src/1.4/dom/domCg_setarray_type.cpp
			src/1.4/dom/domCg_setparam.cpp
			src/1.4/dom/domCg_setparam_simple.cpp
			src/1.4/dom/domCg_setuser_type.cpp
			src/1.4/dom/domCg_surface_type.cpp
			src/1.4/dom/domChannel.cpp
			src/1.4/dom/domCOLLADA.cpp
			src/1.4/dom/domCommon_color_or_texture_type.cpp
			src/1.4/dom/domCommon_float_or_param_type.cpp
			src/1.4/dom/domCommon_newparam_type.cpp
			src/1.4/dom/domCommon_transparent_type.cpp
			src/1.4/dom/domConstants.cpp
			src/1.4/dom/domController.cpp
			src/1.4/dom/domConvex_mesh.cpp
			src/1.4/dom/domCylinder.cpp
			src/1.4/dom/domEffect.cpp
			src/1.4/dom/domEllipsoid.cpp
			src/1.4/dom/domExtra.cpp
			src/1.4/dom/domFloat_array.cpp
			src/1.4/dom/domForce_field.cpp
			src/1.4/dom/domFx_annotate_common.cpp
			src/1.4/dom/domFx_annotate_type_common.cpp
			src/1.4/dom/domFx_basic_type_common.cpp
			src/1.4/dom/domFx_clearcolor_common.cpp
			src/1.4/dom/domFx_cleardepth_common.cpp
			src/1.4/dom/domFx_clearstencil_common.cpp
			src/1.4/dom/domFx_code_profile.cpp
			src/1.4/dom/domFx_colortarget_common.cpp
			src/1.4/dom/domFx_depthtarget_common.cpp
			src/1.4/dom/domFx_include_common.cpp
			src/1.4/dom/domFx_newparam_common.cpp
			src/1.4/dom/domFx_profile_abstract.cpp
			src/1.4/dom/domFx_sampler1D_common.cpp
			src/1.4/dom/domFx_sampler2D_common.cpp
			src/1.4/dom/domFx_sampler3D_common.cpp
			src/1.4/dom/domFx_samplerCUBE_common.cpp
			src/1.4/dom/domFx_samplerDEPTH_common.cpp
			src/1.4/dom/domFx_samplerRECT_common.cpp
			src/1.4/dom/domFx_stenciltarget_common.cpp
			src/1.4/dom/domFx_surface_common.cpp
			src/1.4/dom/domFx_surface_format_hint_common.cpp
			src/1.4/dom/domFx_surface_init_common.cpp
			src/1.4/dom/domFx_surface_init_cube_common.cpp
			src/1.4/dom/domFx_surface_init_from_common.cpp
			src/1.4/dom/domFx_surface_init_planar_common.cpp
			src/1.4/dom/domFx_surface_init_volume_common.cpp
			src/1.4/dom/domGeometry.cpp
			src/1.4/dom/domGl_hook_abstract.cpp
			src/1.4/dom/domGl_pipeline_settings.cpp
			src/1.4/dom/domGl_sampler1D.cpp
			src/1.4/dom/domGl_sampler2D.cpp
			src/1.4/dom/domGl_sampler3D.cpp
			src/1.4/dom/domGl_samplerCUBE.cpp
			src/1.4/dom/domGl_samplerDEPTH.cpp
			src/1.4/dom/domGl_samplerRECT.cpp
			src/1.4/dom/domGles_basic_type_common.cpp
			src/1.4/dom/domGles_newparam.cpp
			src/1.4/dom/domGles_pipeline_settings.cpp
			src/1.4/dom/domGles_sampler_state.cpp
			src/1.4/dom/domGles_texcombiner_argumentAlpha_type.cpp
			src/1.4/dom/domGles_texcombiner_argumentRGB_type.cpp
			src/1.4/dom/domGles_texcombiner_command_type.cpp
			src/1.4/dom/domGles_texcombiner_commandAlpha_type.cpp
			src/1.4/dom/domGles_texcombiner_commandRGB_type.cpp
			src/1.4/dom/domGles_texenv_command_type.cpp
			src/1.4/dom/domGles_texture_constant_type.cpp
			src/1.4/dom/domGles_texture_pipeline.cpp
			src/1.4/dom/domGles_texture_unit.cpp
			src/1.4/dom/domGlsl_newarray_type.cpp
			src/1.4/dom/domGlsl_newparam.cpp
			src/1.4/dom/domGlsl_param_type.cpp
			src/1.4/dom/domGlsl_setarray_type.cpp
			src/1.4/dom/domGlsl_setparam.cpp
			src/1.4/dom/domGlsl_setparam_simple.cpp
			src/1.4/dom/domGlsl_surface_type.cpp
			src/1.4/dom/domIDREF_array.cpp
			src/1.4/dom/domImage.cpp
			src/1.4/dom/domInputGlobal.cpp
			src/1.4/dom/domInputLocal.cpp
			src/1.4/dom/domInputLocalOffset.cpp
			src/1.4/dom/domInstance_camera.cpp
			src/1.4/dom/domInstance_controller.cpp
			src/1.4/dom/domInstance_effect.cpp
			src/1.4/dom/domInstance_force_field.cpp
			src/1.4/dom/domInstance_geometry.cpp
			src/1.4/dom/domInstance_light.cpp
			src/1.4/dom/domInstance_material.cpp
			src/1.4/dom/domInstance_node.cpp
			src/1.4/dom/domInstance_physics_material.cpp
			src/1.4/dom/domInstance_physics_model.cpp
			src/1.4/dom/domInstance_rigid_body.cpp
			src/1.4/dom/domInstance_rigid_constraint.cpp
			src/1.4/dom/domInstanceWithExtra.cpp
			src/1.4/dom/domInt_array.cpp
			src/1.4/dom/domLibrary_animation_clips.cpp
			src/1.4/dom/domLibrary_animations.cpp
			src/1.4/dom/domLibrary_cameras.cpp
			src/1.4/dom/domLibrary_controllers.cpp
			src/1.4/dom/domLibrary_effects.cpp
			src/1.4/dom/domLibrary_force_fields.cpp
			src/1.4/dom/domLibrary_geometries.cpp
			src/1.4/dom/domLibrary_images.cpp
			src/1.4/dom/domLibrary_lights.cpp
			src/1.4/dom/domLibrary_materials.cpp
			src/1.4/dom/domLibrary_nodes.cpp
			src/1.4/dom/domLibrary_physics_materials.cpp
			src/1.4/dom/domLibrary_physics_models.cpp
			src/1.4/dom/domLibrary_physics_scenes.cpp
			src/1.4/dom/domLibrary_visual_scenes.cpp
			src/1.4/dom/domLight.cpp
			src/1.4/dom/domLines.cpp
			src/1.4/dom/domLinestrips.cpp
			src/1.4/dom/domLookat.cpp
			src/1.4/dom/domMaterial.cpp
			src/1.4/dom/domMatrix.cpp
			src/1.4/dom/domMesh.cpp
			src/1.4/dom/domMorph.cpp
			src/1.4/dom/domName_array.cpp
			src/1.4/dom/domNode.cpp
			src/1.4/dom/domP.cpp
			src/1.4/dom/domParam.cpp
			src/1.4/dom/domPhysics_material.cpp
			src/1.4/dom/domPhysics_model.cpp
			src/1.4/dom/domPhysics_scene.cpp
			src/1.4/dom/domPlane.cpp
			src/1.4/dom/domPolygons.cpp
			src/1.4/dom/domPolylist.cpp
			src/1.4/dom/domProfile_CG.cpp
			src/1.4/dom/domProfile_COMMON.cpp
			src/1.4/dom/domProfile_GLES.cpp
			src/1.4/dom/domProfile_GLSL.cpp
			src/1.4/dom/domRigid_body.cpp
			src/1.4/dom/domRigid_constraint.cpp
			src/1.4/dom/domRotate.cpp
			src/1.4/dom/domSampler.cpp
			src/1.4/dom/domScale.cpp
			src/1.4/dom/domSkew.cpp
			src/1.4/dom/domSkin.cpp
			src/1.4/dom/domSource.cpp
			src/1.4/dom/domSphere.cpp
			src/1.4/dom/domSpline.cpp
			src/1.4/dom/domTapered_capsule.cpp
			src/1.4/dom/domTapered_cylinder.cpp
			src/1.4/dom/domTargetableFloat.cpp
			src/1.4/dom/domTargetableFloat3.cpp
			src/1.4/dom/domTechnique.cpp
			src/1.4/dom/domTranslate.cpp
			src/1.4/dom/domTriangles.cpp
			src/1.4/dom/domTrifans.cpp
			src/1.4/dom/domTristrips.cpp
			src/1.4/dom/domTypes.cpp
			src/1.4/dom/domVertices.cpp
			src/1.4/dom/domVisual_scene.cpp
			src/dae/dae.cpp
			src/dae/daeArray.cpp
			src/dae/daeAtomicType.cpp
			src/dae/daeDocument.cpp
			src/dae/daeDom.cpp
			src/dae/daeElement.cpp
			src/dae/daeError.cpp
			src/dae/daeErrorHandler.cpp
			src/dae/daeIDRef.cpp
			src/dae/daeMemorySystem.cpp
			src/dae/daeMetaAny.cpp
			src/dae/daeMetaAttribute.cpp
			src/dae/daeMetaChoice.cpp
			src/dae/daeMetaCMPolicy.cpp
			src/dae/daeMetaElement.cpp
			src/dae/daeMetaElementAttribute.cpp
			src/dae/daeMetaGroup.cpp
			src/dae/daeMetaSequence.cpp
			src/dae/daeSIDResolver.cpp
			src/dae/daeStringRef.cpp
			src/dae/daeStringTable.cpp
			src/dae/daeURI.cpp
			src/dae/domAny.cpp
			src/modules/LIBXMLPlugin/daeLIBXMLPlugin.cpp
			src/modules/LIBXMLPlugin/daeLIBXMLResolver.cpp
			src/modules/stdErrPlugin/stdErrPlugin.cpp
			src/modules/STLDatabase/daeSTLDatabase.cpp
)

IF (BUILD_SHARED_LIBS)
  TARGET_LINK_LIBRARIES(ColladaDom XML)
ENDIF (BUILD_SHARED_LIBS)

#INSTALL of other files requires CMake 2.6
IF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.5)
	IF(INSTALL_EXTRA_LIBS)
		IF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
			INSTALL(TARGETS ColladaDom DESTINATION .)
		ELSE (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
		INSTALL(TARGETS ColladaDom DESTINATION lib)
			INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/1.4 DESTINATION include/1.4 FILES_MATCHING PATTERN "*.h")
			INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/dae DESTINATION include/dae FILES_MATCHING PATTERN "*.h")
			INSTALL(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}/include/modules DESTINATION include/modules FILES_MATCHING PATTERN "*.h")
		ENDIF (APPLE AND BUILD_SHARED_LIBS AND FRAMEWORK)
	ENDIF (INSTALL_EXTRA_LIBS)
ENDIF (${CMAKE_MAJOR_VERSION}.${CMAKE_MINOR_VERSION} GREATER 2.5)
