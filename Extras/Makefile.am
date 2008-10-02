noinst_LIBRARIES	= libxml.a libbulletcolladaconverter.a libcollada_dom.a libbulletmultithreaded.a

libbulletmultithreaded_a_CXXFLAGS = $(CXXFLAGS) -I../src -IBulletMultiThreaded
libbulletmultithreaded_a_SOURCES =\
		BulletMultiThreaded/SpuCollisionObjectWrapper.cpp \
		BulletMultiThreaded/SpuSampleTask/SpuSampleTask.cpp \
		BulletMultiThreaded/SpuLibspe2Support.cpp \
		BulletMultiThreaded/SpuBatchRaycaster.cpp \
		BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGjkEpa2.cpp \
		BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuEpaPenetrationDepthSolver.cpp \
		BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuContactResult.cpp \
		BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.cpp \
		BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGjkPairDetector.cpp \
		BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuVoronoiSimplexSolver.cpp \
		BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuMinkowskiPenetrationDepthSolver.cpp \
		BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuCollisionShapes.cpp \
		BulletMultiThreaded/SpuRaycastTask/SpuRaycastTask.cpp \
		BulletMultiThreaded/SpuRaycastTask/SpuSubSimplexConvexCast.cpp \
		BulletMultiThreaded/SpuParallelSolver.cpp \
		BulletMultiThreaded/SpuSolverTask/SpuParallellSolverTask.cpp \
		BulletMultiThreaded/btThreadSupportInterface.cpp \
		BulletMultiThreaded/SequentialThreadSupport.cpp \
		BulletMultiThreaded/SpuGatheringCollisionDispatcher.cpp \
		BulletMultiThreaded/Win32ThreadSupport.cpp \
		BulletMultiThreaded/SpuFakeDma.cpp \
		BulletMultiThreaded/SpuRaycastTaskProcess.cpp \
		BulletMultiThreaded/PosixThreadSupport.cpp \
		BulletMultiThreaded/SpuCollisionTaskProcess.cpp \
		BulletMultiThreaded/SpuContactManifoldCollisionAlgorithm.cpp \
		BulletMultiThreaded/SpuSampleTaskProcess.cpp \
		BulletMultiThreaded/SpuSampleTask/SpuSampleTask.h \
		BulletMultiThreaded/PpuAddressSpace.h \
		BulletMultiThreaded/SpuSampleTaskProcess.h \
		BulletMultiThreaded/SequentialThreadSupport.h \
		BulletMultiThreaded/PlatformDefinitions.h \
		BulletMultiThreaded/Win32ThreadSupport.h \
		BulletMultiThreaded/SpuContactManifoldCollisionAlgorithm.h \
		BulletMultiThreaded/SpuParallelSolver.h \
		BulletMultiThreaded/btThreadSupportInterface.h \
		BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGatheringCollisionTask.h \
		BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGjkEpa2.h \
		BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuEpaPenetrationDepthSolver.h \
		BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuVoronoiSimplexSolver.h \
		BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuConvexPenetrationDepthSolver.h \
		BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuPreferredPenetrationDirections.h \
		BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuCollisionShapes.h \
		BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuGjkPairDetector.h \
		BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuLocalSupport.h \
		BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuMinkowskiPenetrationDepthSolver.h \
		BulletMultiThreaded/SpuNarrowPhaseCollisionTask/SpuContactResult.h \
		BulletMultiThreaded/SpuRaycastTask/SpuSubSimplexConvexCast.h \
		BulletMultiThreaded/SpuRaycastTask/SpuRaycastTask.h \
		BulletMultiThreaded/SpuSolverTask/SpuParallellSolverTask.h \
		BulletMultiThreaded/SpuGatheringCollisionDispatcher.h \
		BulletMultiThreaded/SpuFakeDma.h \
		BulletMultiThreaded/SpuSync.h \
		BulletMultiThreaded/SpuCollisionObjectWrapper.h \
		BulletMultiThreaded/SpuDoubleBuffer.h \
		BulletMultiThreaded/SpuCollisionTaskProcess.h \
		BulletMultiThreaded/SpuBatchRaycaster.h \
		BulletMultiThreaded/PosixThreadSupport.h \
		BulletMultiThreaded/SpuLibspe2Support.h \
		BulletMultiThreaded/SpuRaycastTaskProcess.h


libxml_a_CFLAGS = -ILibXML/ -ILibXML/include/ -D_CRT_SECURE_NO_WARNINGS -D_CRT_SECURE_NO_DEPRECATE $(CXXFLAGS)
libxml_a_SOURCES =\
		LibXML/c14n.c\
		LibXML/catalog.c\
		LibXML/chvalid.c\
		LibXML/debugXML.c\
		LibXML/dict.c\
		LibXML/DOCBparser.c\
		LibXML/encoding.c\
		LibXML/entities.c\
		LibXML/error.c\
		LibXML/globals.c\
		LibXML/hash.c\
		LibXML/HTMLparser.c\
		LibXML/HTMLtree.c\
		LibXML/legacy.c\
		LibXML/list.c\
		LibXML/nanoftp.c\
		LibXML/nanohttp.c\
		LibXML/parser.c\
		LibXML/parserInternals.c\
		LibXML/pattern.c\
		LibXML/relaxng.c\
		LibXML/SAX.c\
		LibXML/SAX2.c\
		LibXML/schematron.c\
		LibXML/threads.c\
		LibXML/tree.c\
		LibXML/trio.c\
		LibXML/trionan.c\
		LibXML/triostr.c\
		LibXML/uri.c\
		LibXML/valid.c\
		LibXML/xinclude.c\
		LibXML/xlink.c\
		LibXML/xmlIO.c\
		LibXML/xmlmemory.c\
		LibXML/xmlmodule.c\
		LibXML/xmlreader.c\
		LibXML/xmlregexp.c\
		LibXML/xmlsave.c\
		LibXML/xmlschemas.c\
		LibXML/xmlschemastypes.c\
		LibXML/xmlstring.c\
		LibXML/xmlunicode.c\
		LibXML/xmlwriter.c\
		LibXML/xpath.c\
		LibXML/xpointer.c

libbulletcolladaconverter_a_CXXFLAGS = -I../src -ICOLLADA_DOM/include/1.4 -ICOLLADA_DOM/include $(CXXFLAGS)
libbulletcolladaconverter_a_SOURCES = BulletColladaConverter/ColladaConverter.cpp  BulletColladaConverter/ColladaConverter.h

libcollada_dom_a_CXXFLAGS = -ILibXML/include -ICOLLADA_DOM/include -ICOLLADA_DOM/include/1.4 $(CXXFLAGS)
libcollada_dom_a_SOURCES = \
COLLADA_DOM/src/dae/daeDocument.cpp \
COLLADA_DOM/src/dae/daeMemorySystem.cpp \
COLLADA_DOM/src/dae/daeMetaElement.cpp \
COLLADA_DOM/src/dae/daeMetaSequence.cpp \
COLLADA_DOM/src/dae/daeURI.cpp \
COLLADA_DOM/src/dae/daeMetaAttribute.cpp \
COLLADA_DOM/src/dae/daeArray.cpp \
COLLADA_DOM/src/dae/daeAtomicType.cpp \
COLLADA_DOM/src/dae/dae.cpp \
COLLADA_DOM/src/dae/daeMetaAny.cpp \
COLLADA_DOM/src/dae/daeErrorHandler.cpp \
COLLADA_DOM/src/dae/daeMetaCMPolicy.cpp \
COLLADA_DOM/src/dae/daeStringRef.cpp \
COLLADA_DOM/src/dae/daeElement.cpp \
COLLADA_DOM/src/dae/daeMetaElementAttribute.cpp \
COLLADA_DOM/src/dae/daeIDRef.cpp \
COLLADA_DOM/src/dae/daeSIDResolver.cpp \
COLLADA_DOM/src/dae/daeMetaGroup.cpp \
COLLADA_DOM/src/dae/domAny.cpp \
COLLADA_DOM/src/dae/daeMetaChoice.cpp \
COLLADA_DOM/src/dae/daeDom.cpp \
COLLADA_DOM/src/dae/daeError.cpp \
COLLADA_DOM/src/dae/daeStringTable.cpp \
COLLADA_DOM/src/1.4/dom/domFx_surface_init_from_common.cpp \
COLLADA_DOM/src/1.4/dom/domTapered_capsule.cpp \
COLLADA_DOM/src/1.4/dom/domFx_surface_format_hint_common.cpp \
COLLADA_DOM/src/1.4/dom/domMaterial.cpp \
COLLADA_DOM/src/1.4/dom/domRigid_body.cpp \
COLLADA_DOM/src/1.4/dom/domInputLocal.cpp \
COLLADA_DOM/src/1.4/dom/domLibrary_images.cpp \
COLLADA_DOM/src/1.4/dom/domFx_sampler3D_common.cpp \
COLLADA_DOM/src/1.4/dom/domTriangles.cpp \
COLLADA_DOM/src/1.4/dom/domTargetableFloat3.cpp \
COLLADA_DOM/src/1.4/dom/domFx_depthtarget_common.cpp \
COLLADA_DOM/src/1.4/dom/domGl_samplerRECT.cpp \
COLLADA_DOM/src/1.4/dom/domLibrary_geometries.cpp \
COLLADA_DOM/src/1.4/dom/domFx_profile_abstract.cpp \
COLLADA_DOM/src/1.4/dom/domInstance_material.cpp \
COLLADA_DOM/src/1.4/dom/domRigid_constraint.cpp \
COLLADA_DOM/src/1.4/dom/domFx_stenciltarget_common.cpp \
COLLADA_DOM/src/1.4/dom/domGl_samplerCUBE.cpp \
COLLADA_DOM/src/1.4/dom/domLibrary_physics_materials.cpp \
COLLADA_DOM/src/1.4/dom/domInstance_camera.cpp \
COLLADA_DOM/src/1.4/dom/domExtra.cpp \
COLLADA_DOM/src/1.4/dom/domFx_surface_init_common.cpp \
COLLADA_DOM/src/1.4/dom/domGles_texture_constant_type.cpp \
COLLADA_DOM/src/1.4/dom/domFx_code_profile.cpp \
COLLADA_DOM/src/1.4/dom/domGl_hook_abstract.cpp \
COLLADA_DOM/src/1.4/dom/domAnimation.cpp \
COLLADA_DOM/src/1.4/dom/domCg_setuser_type.cpp \
COLLADA_DOM/src/1.4/dom/domCapsule.cpp \
COLLADA_DOM/src/1.4/dom/domFx_colortarget_common.cpp \
COLLADA_DOM/src/1.4/dom/domFx_surface_common.cpp \
COLLADA_DOM/src/1.4/dom/domProfile_GLES.cpp \
COLLADA_DOM/src/1.4/dom/domGlsl_setparam.cpp \
COLLADA_DOM/src/1.4/dom/domVertices.cpp \
COLLADA_DOM/src/1.4/dom/domTrifans.cpp \
COLLADA_DOM/src/1.4/dom/domCg_sampler1D.cpp \
COLLADA_DOM/src/1.4/dom/domVisual_scene.cpp \
COLLADA_DOM/src/1.4/dom/domFx_cleardepth_common.cpp \
COLLADA_DOM/src/1.4/dom/domFx_samplerCUBE_common.cpp \
COLLADA_DOM/src/1.4/dom/domLookat.cpp \
COLLADA_DOM/src/1.4/dom/domGlsl_setparam_simple.cpp \
COLLADA_DOM/src/1.4/dom/domFx_surface_init_volume_common.cpp \
COLLADA_DOM/src/1.4/dom/domLibrary_lights.cpp \
COLLADA_DOM/src/1.4/dom/domFx_sampler1D_common.cpp \
COLLADA_DOM/src/1.4/dom/domLines.cpp \
COLLADA_DOM/src/1.4/dom/domFx_samplerDEPTH_common.cpp \
COLLADA_DOM/src/1.4/dom/domCg_newparam.cpp \
COLLADA_DOM/src/1.4/dom/domLibrary_cameras.cpp \
COLLADA_DOM/src/1.4/dom/domCg_setparam_simple.cpp \
COLLADA_DOM/src/1.4/dom/domImage.cpp \
COLLADA_DOM/src/1.4/dom/domEffect.cpp \
COLLADA_DOM/src/1.4/dom/domCommon_newparam_type.cpp \
COLLADA_DOM/src/1.4/dom/domInputGlobal.cpp \
COLLADA_DOM/src/1.4/dom/domFx_samplerRECT_common.cpp \
COLLADA_DOM/src/1.4/dom/domEllipsoid.cpp \
COLLADA_DOM/src/1.4/dom/domGles_basic_type_common.cpp \
COLLADA_DOM/src/1.4/dom/domName_array.cpp \
COLLADA_DOM/src/1.4/dom/domGles_sampler_state.cpp \
COLLADA_DOM/src/1.4/dom/domCg_connect_param.cpp \
COLLADA_DOM/src/1.4/dom/domAnimation_clip.cpp \
COLLADA_DOM/src/1.4/dom/domAsset.cpp \
COLLADA_DOM/src/1.4/dom/domRotate.cpp \
COLLADA_DOM/src/1.4/dom/domGles_texcombiner_commandAlpha_type.cpp \
COLLADA_DOM/src/1.4/dom/domNode.cpp \
COLLADA_DOM/src/1.4/dom/domTargetableFloat.cpp \
COLLADA_DOM/src/1.4/dom/domMorph.cpp \
COLLADA_DOM/src/1.4/dom/domLibrary_physics_scenes.cpp \
COLLADA_DOM/src/1.4/dom/domCamera.cpp \
COLLADA_DOM/src/1.4/dom/domProfile_CG.cpp \
COLLADA_DOM/src/1.4/dom/domCommon_float_or_param_type.cpp \
COLLADA_DOM/src/1.4/dom/domSpline.cpp \
COLLADA_DOM/src/1.4/dom/domGles_texcombiner_argumentRGB_type.cpp \
COLLADA_DOM/src/1.4/dom/domGlsl_setarray_type.cpp \
COLLADA_DOM/src/1.4/dom/domGles_texenv_command_type.cpp \
COLLADA_DOM/src/1.4/dom/domFx_annotate_common.cpp \
COLLADA_DOM/src/1.4/dom/domTristrips.cpp \
COLLADA_DOM/src/1.4/dom/domLibrary_materials.cpp \
COLLADA_DOM/src/1.4/dom/domFx_basic_type_common.cpp \
COLLADA_DOM/src/1.4/dom/domInstance_node.cpp \
COLLADA_DOM/src/1.4/dom/domCommon_transparent_type.cpp \
COLLADA_DOM/src/1.4/dom/domCg_setparam.cpp \
COLLADA_DOM/src/1.4/dom/domLibrary_physics_models.cpp \
COLLADA_DOM/src/1.4/dom/domFx_surface_init_planar_common.cpp \
COLLADA_DOM/src/1.4/dom/domSource.cpp \
COLLADA_DOM/src/1.4/dom/domInstance_rigid_body.cpp \
COLLADA_DOM/src/1.4/dom/domLibrary_force_fields.cpp \
COLLADA_DOM/src/1.4/dom/domLibrary_controllers.cpp \
COLLADA_DOM/src/1.4/dom/domTypes.cpp \
COLLADA_DOM/src/1.4/dom/domGles_newparam.cpp \
COLLADA_DOM/src/1.4/dom/domTapered_cylinder.cpp \
COLLADA_DOM/src/1.4/dom/domP.cpp \
COLLADA_DOM/src/1.4/dom/domCg_samplerCUBE.cpp \
COLLADA_DOM/src/1.4/dom/domGl_sampler1D.cpp \
COLLADA_DOM/src/1.4/dom/domSkin.cpp \
COLLADA_DOM/src/1.4/dom/domFx_surface_init_cube_common.cpp \
COLLADA_DOM/src/1.4/dom/domCg_sampler2D.cpp \
COLLADA_DOM/src/1.4/dom/domGlsl_newarray_type.cpp \
COLLADA_DOM/src/1.4/dom/domInstance_light.cpp \
COLLADA_DOM/src/1.4/dom/domPhysics_material.cpp \
COLLADA_DOM/src/1.4/dom/domLibrary_effects.cpp \
COLLADA_DOM/src/1.4/dom/domGl_sampler2D.cpp \
COLLADA_DOM/src/1.4/dom/domGlsl_param_type.cpp \
COLLADA_DOM/src/1.4/dom/domLibrary_visual_scenes.cpp \
COLLADA_DOM/src/1.4/dom/domFx_newparam_common.cpp \
COLLADA_DOM/src/1.4/dom/domPhysics_scene.cpp \
COLLADA_DOM/src/1.4/dom/domInstanceWithExtra.cpp \
COLLADA_DOM/src/1.4/dom/domFx_clearcolor_common.cpp \
COLLADA_DOM/src/1.4/dom/domCg_sampler3D.cpp \
COLLADA_DOM/src/1.4/dom/domInputLocalOffset.cpp \
COLLADA_DOM/src/1.4/dom/domGles_pipeline_settings.cpp \
COLLADA_DOM/src/1.4/dom/domParam.cpp \
COLLADA_DOM/src/1.4/dom/domGles_texture_pipeline.cpp \
COLLADA_DOM/src/1.4/dom/domGeometry.cpp \
COLLADA_DOM/src/1.4/dom/domPlane.cpp \
COLLADA_DOM/src/1.4/dom/domGles_texture_unit.cpp \
COLLADA_DOM/src/1.4/dom/domGlsl_newparam.cpp \
COLLADA_DOM/src/1.4/dom/domController.cpp \
COLLADA_DOM/src/1.4/dom/domAccessor.cpp \
COLLADA_DOM/src/1.4/dom/domSkew.cpp \
COLLADA_DOM/src/1.4/dom/domPolygons.cpp \
COLLADA_DOM/src/1.4/dom/domInstance_effect.cpp \
COLLADA_DOM/src/1.4/dom/domCg_samplerDEPTH.cpp \
COLLADA_DOM/src/1.4/dom/domMatrix.cpp \
COLLADA_DOM/src/1.4/dom/domGlsl_surface_type.cpp \
COLLADA_DOM/src/1.4/dom/domInstance_physics_material.cpp \
COLLADA_DOM/src/1.4/dom/domFx_clearstencil_common.cpp \
COLLADA_DOM/src/1.4/dom/domFx_sampler2D_common.cpp \
COLLADA_DOM/src/1.4/dom/domGl_pipeline_settings.cpp \
COLLADA_DOM/src/1.4/dom/domInstance_geometry.cpp \
COLLADA_DOM/src/1.4/dom/domCommon_color_or_texture_type.cpp \
COLLADA_DOM/src/1.4/dom/domFloat_array.cpp \
COLLADA_DOM/src/1.4/dom/domCg_param_type.cpp \
COLLADA_DOM/src/1.4/dom/domTechnique.cpp \
COLLADA_DOM/src/1.4/dom/domIDREF_array.cpp \
COLLADA_DOM/src/1.4/dom/domForce_field.cpp \
COLLADA_DOM/src/1.4/dom/domFx_include_common.cpp \
COLLADA_DOM/src/1.4/dom/domCg_newarray_type.cpp \
COLLADA_DOM/src/1.4/dom/domLight.cpp \
COLLADA_DOM/src/1.4/dom/domBool_array.cpp \
COLLADA_DOM/src/1.4/dom/domCylinder.cpp \
COLLADA_DOM/src/1.4/dom/domLibrary_animation_clips.cpp \
COLLADA_DOM/src/1.4/dom/domGl_sampler3D.cpp \
COLLADA_DOM/src/1.4/dom/domCg_surface_type.cpp \
COLLADA_DOM/src/1.4/dom/domPhysics_model.cpp \
COLLADA_DOM/src/1.4/dom/domInt_array.cpp \
COLLADA_DOM/src/1.4/dom/domInstance_physics_model.cpp \
COLLADA_DOM/src/1.4/dom/domCg_setarray_type.cpp \
COLLADA_DOM/src/1.4/dom/domGles_texcombiner_commandRGB_type.cpp \
COLLADA_DOM/src/1.4/dom/domGles_texcombiner_argumentAlpha_type.cpp \
COLLADA_DOM/src/1.4/dom/domTranslate.cpp \
COLLADA_DOM/src/1.4/dom/domBox.cpp \
COLLADA_DOM/src/1.4/dom/domInstance_rigid_constraint.cpp \
COLLADA_DOM/src/1.4/dom/domCOLLADA.cpp \
COLLADA_DOM/src/1.4/dom/domChannel.cpp \
COLLADA_DOM/src/1.4/dom/domPolylist.cpp \
COLLADA_DOM/src/1.4/dom/domSphere.cpp \
COLLADA_DOM/src/1.4/dom/domInstance_controller.cpp \
COLLADA_DOM/src/1.4/dom/domGles_texcombiner_command_type.cpp \
COLLADA_DOM/src/1.4/dom/domConstants.cpp \
COLLADA_DOM/src/1.4/dom/domMesh.cpp \
COLLADA_DOM/src/1.4/dom/domGl_samplerDEPTH.cpp \
COLLADA_DOM/src/1.4/dom/domProfile_GLSL.cpp \
COLLADA_DOM/src/1.4/dom/domSampler.cpp \
COLLADA_DOM/src/1.4/dom/domProfile_COMMON.cpp \
COLLADA_DOM/src/1.4/dom/domConvex_mesh.cpp \
COLLADA_DOM/src/1.4/dom/domLibrary_nodes.cpp \
COLLADA_DOM/src/1.4/dom/domScale.cpp \
COLLADA_DOM/src/1.4/dom/domLinestrips.cpp \
COLLADA_DOM/src/1.4/dom/domBind_material.cpp \
COLLADA_DOM/src/1.4/dom/domLibrary_animations.cpp \
COLLADA_DOM/src/1.4/dom/domFx_annotate_type_common.cpp \
COLLADA_DOM/src/1.4/dom/domCg_samplerRECT.cpp \
COLLADA_DOM/src/1.4/dom/domInstance_force_field.cpp \
COLLADA_DOM/src/modules/LIBXMLPlugin/daeLIBXMLResolver.cpp \
COLLADA_DOM/src/modules/LIBXMLPlugin/daeLIBXMLPlugin.cpp \
COLLADA_DOM/src/modules/stdErrPlugin/stdErrPlugin.cpp \
COLLADA_DOM/src/modules/STLDatabase/daeSTLDatabase.cpp \
COLLADA_DOM/include/dae/daeSmartRef.h \
COLLADA_DOM/include/dae/daeInterface.h \
COLLADA_DOM/include/dae/daeIOPlugin.h \
COLLADA_DOM/include/dae/daeIntegrationObject.h \
COLLADA_DOM/include/dae/daeStringRef.h \
COLLADA_DOM/include/dae/daeSIDResolver.h \
COLLADA_DOM/include/dae/daeMetaSequence.h \
COLLADA_DOM/include/dae/daeIDRef.h \
COLLADA_DOM/include/dae/daeTypes.h \
COLLADA_DOM/include/dae/daeDom.h \
COLLADA_DOM/include/dae/daeDatabase.h \
COLLADA_DOM/include/dae/daeDocument.h \
COLLADA_DOM/include/dae/daeMetaAny.h \
COLLADA_DOM/include/dae/daeArrayTypes.h \
COLLADA_DOM/include/dae/daeMetaAttribute.h \
COLLADA_DOM/include/dae/daeMetaChoice.h \
COLLADA_DOM/include/dae/daeError.h \
COLLADA_DOM/include/dae/daeAtomicType.h \
COLLADA_DOM/include/dae/daeStringTable.h \
COLLADA_DOM/include/dae/daeMetaGroup.h \
COLLADA_DOM/include/dae/daeMemorySystem.h \
COLLADA_DOM/include/dae/daeErrorHandler.h \
COLLADA_DOM/include/dae/daeMetaElementAttribute.h \
COLLADA_DOM/include/dae/daeDomTypes.h \
COLLADA_DOM/include/dae/daeMetaCMPolicy.h \
COLLADA_DOM/include/dae/daeWin32Platform.h \
COLLADA_DOM/include/dae/daeArray.h \
COLLADA_DOM/include/dae/daeGenericPlatform.h \
COLLADA_DOM/include/dae/daeElement.h \
COLLADA_DOM/include/dae/domAny.h \
COLLADA_DOM/include/dae/daeURI.h \
COLLADA_DOM/include/dae/daeMetaElement.h \
COLLADA_DOM/include/dae/daeGCCPlatform.h \
COLLADA_DOM/include/1.4/dom/domCommon_float_or_param_type.h \
COLLADA_DOM/include/1.4/dom/domGles_sampler_state.h \
COLLADA_DOM/include/1.4/dom/domFx_newparam_common.h \
COLLADA_DOM/include/1.4/dom/domGl_samplerDEPTH.h \
COLLADA_DOM/include/1.4/dom/domFx_annotate_common.h \
COLLADA_DOM/include/1.4/dom/domLibrary_physics_scenes.h \
COLLADA_DOM/include/1.4/dom/domFx_clearstencil_common.h \
COLLADA_DOM/include/1.4/dom/domCommon_newparam_type.h \
COLLADA_DOM/include/1.4/dom/domFx_surface_common.h \
COLLADA_DOM/include/1.4/dom/domEllipsoid.h \
COLLADA_DOM/include/1.4/dom/domLibrary_physics_materials.h \
COLLADA_DOM/include/1.4/dom/domInputLocal.h \
COLLADA_DOM/include/1.4/dom/domFx_basic_type_common.h \
COLLADA_DOM/include/1.4/dom/domAnimation.h \
COLLADA_DOM/include/1.4/dom/domFx_samplerRECT_common.h \
COLLADA_DOM/include/1.4/dom/domCOLLADA.h \
COLLADA_DOM/include/1.4/dom/domSkin.h \
COLLADA_DOM/include/1.4/dom/domGles_basic_type_common.h \
COLLADA_DOM/include/1.4/dom/domCylinder.h \
COLLADA_DOM/include/1.4/dom/domInputLocalOffset.h \
COLLADA_DOM/include/1.4/dom/domGles_texcombiner_commandAlpha_type.h \
COLLADA_DOM/include/1.4/dom/domInstance_effect.h \
COLLADA_DOM/include/1.4/dom/domFx_surface_format_hint_common.h \
COLLADA_DOM/include/1.4/dom/domFx_surface_init_cube_common.h \
COLLADA_DOM/include/1.4/dom/domFx_cleardepth_common.h \
COLLADA_DOM/include/1.4/dom/domGl_samplerCUBE.h \
COLLADA_DOM/include/1.4/dom/domFx_surface_init_planar_common.h \
COLLADA_DOM/include/1.4/dom/domAnimation_clip.h \
COLLADA_DOM/include/1.4/dom/domGl_hook_abstract.h \
COLLADA_DOM/include/1.4/dom/domTechnique.h \
COLLADA_DOM/include/1.4/dom/domInputGlobal.h \
COLLADA_DOM/include/1.4/dom/domLibrary_controllers.h \
COLLADA_DOM/include/1.4/dom/domName_array.h \
COLLADA_DOM/include/1.4/dom/domLight.h \
COLLADA_DOM/include/1.4/dom/domTranslate.h \
COLLADA_DOM/include/1.4/dom/domCg_newarray_type.h \
COLLADA_DOM/include/1.4/dom/domGlsl_param_type.h \
COLLADA_DOM/include/1.4/dom/domTargetableFloat.h \
COLLADA_DOM/include/1.4/dom/domParam.h \
COLLADA_DOM/include/1.4/dom/domSampler.h \
COLLADA_DOM/include/1.4/dom/domProfile_GLSL.h \
COLLADA_DOM/include/1.4/dom/domFx_surface_init_volume_common.h \
COLLADA_DOM/include/1.4/dom/domLibrary_geometries.h \
COLLADA_DOM/include/1.4/dom/domP.h \
COLLADA_DOM/include/1.4/dom/domInt_array.h \
COLLADA_DOM/include/1.4/dom/domInstance_geometry.h \
COLLADA_DOM/include/1.4/dom/domFx_samplerDEPTH_common.h \
COLLADA_DOM/include/1.4/dom/domFx_surface_init_from_common.h \
COLLADA_DOM/include/1.4/dom/domLibrary_materials.h \
COLLADA_DOM/include/1.4/dom/domTypes.h \
COLLADA_DOM/include/1.4/dom/domGles_texenv_command_type.h \
COLLADA_DOM/include/1.4/dom/domFx_setparam_common.h \
COLLADA_DOM/include/1.4/dom/domForce_field.h \
COLLADA_DOM/include/1.4/dom/domCg_setuser_type.h \
COLLADA_DOM/include/1.4/dom/domLibrary_lights.h \
COLLADA_DOM/include/1.4/dom/domScale.h \
COLLADA_DOM/include/1.4/dom/domExtra.h \
COLLADA_DOM/include/1.4/dom/domTriangles.h \
COLLADA_DOM/include/1.4/dom/domLibrary_animation_clips.h \
COLLADA_DOM/include/1.4/dom/domLookat.h \
COLLADA_DOM/include/1.4/dom/domGles_texture_pipeline.h \
COLLADA_DOM/include/1.4/dom/domFx_depthtarget_common.h \
COLLADA_DOM/include/1.4/dom/domGles_pipeline_settings.h \
COLLADA_DOM/include/1.4/dom/domEffect.h \
COLLADA_DOM/include/1.4/dom/domLines.h \
COLLADA_DOM/include/1.4/dom/domCg_setparam.h \
COLLADA_DOM/include/1.4/dom/domVertices.h \
COLLADA_DOM/include/1.4/dom/domPolylist.h \
COLLADA_DOM/include/1.4/dom/domInstance_rigid_body.h \
COLLADA_DOM/include/1.4/dom/domGlsl_newparam.h \
COLLADA_DOM/include/1.4/dom/domInstance_camera.h \
COLLADA_DOM/include/1.4/dom/domCg_param_type.h \
COLLADA_DOM/include/1.4/dom/domCg_sampler2D.h \
COLLADA_DOM/include/1.4/dom/domFx_clearcolor_common.h \
COLLADA_DOM/include/1.4/dom/domProfile_CG.h \
COLLADA_DOM/include/1.4/dom/domAccessor.h \
COLLADA_DOM/include/1.4/dom/domFx_stenciltarget_common.h \
COLLADA_DOM/include/1.4/dom/domGlsl_newarray_type.h \
COLLADA_DOM/include/1.4/dom/domCg_setarray_type.h \
COLLADA_DOM/include/1.4/dom/domChannel.h \
COLLADA_DOM/include/1.4/dom/domCg_sampler1D.h \
COLLADA_DOM/include/1.4/dom/domImage.h \
COLLADA_DOM/include/1.4/dom/domSphere.h \
COLLADA_DOM/include/1.4/dom/domCommon_color_or_texture_type.h \
COLLADA_DOM/include/1.4/dom/domInstanceWithExtra.h \
COLLADA_DOM/include/1.4/dom/domLibrary_nodes.h \
COLLADA_DOM/include/1.4/dom/domTrifans.h \
COLLADA_DOM/include/1.4/dom/domCg_sampler3D.h \
COLLADA_DOM/include/1.4/dom/domCg_samplerCUBE.h \
COLLADA_DOM/include/1.4/dom/domElements.h \
COLLADA_DOM/include/1.4/dom/domBind_material.h \
COLLADA_DOM/include/1.4/dom/domMesh.h \
COLLADA_DOM/include/1.4/dom/domInstance_controller.h \
COLLADA_DOM/include/1.4/dom/domRigid_body.h \
COLLADA_DOM/include/1.4/dom/domCg_samplerRECT.h \
COLLADA_DOM/include/1.4/dom/domPhysics_model.h \
COLLADA_DOM/include/1.4/dom/domAsset.h \
COLLADA_DOM/include/1.4/dom/domGles_texture_constant_type.h \
COLLADA_DOM/include/1.4/dom/domPlane.h \
COLLADA_DOM/include/1.4/dom/domFx_annotate_type_common.h \
COLLADA_DOM/include/1.4/dom/domController.h \
COLLADA_DOM/include/1.4/dom/domInstance_force_field.h \
COLLADA_DOM/include/1.4/dom/domPhysics_material.h \
COLLADA_DOM/include/1.4/dom/domSource.h \
COLLADA_DOM/include/1.4/dom/domGles_texcombiner_commandRGB_type.h \
COLLADA_DOM/include/1.4/dom/domCg_samplerDEPTH.h \
COLLADA_DOM/include/1.4/dom/domCg_newparam.h \
COLLADA_DOM/include/1.4/dom/domFx_surface_init_common.h \
COLLADA_DOM/include/1.4/dom/domFx_sampler2D_common.h \
COLLADA_DOM/include/1.4/dom/domTapered_cylinder.h \
COLLADA_DOM/include/1.4/dom/domConstants.h \
COLLADA_DOM/include/1.4/dom/domSpline.h \
COLLADA_DOM/include/1.4/dom/domTristrips.h \
COLLADA_DOM/include/1.4/dom/domTapered_capsule.h \
COLLADA_DOM/include/1.4/dom/domLibrary_force_fields.h \
COLLADA_DOM/include/1.4/dom/domPhysics_scene.h \
COLLADA_DOM/include/1.4/dom/domInstance_light.h \
COLLADA_DOM/include/1.4/dom/domCg_surface_type.h \
COLLADA_DOM/include/1.4/dom/domFx_profile_abstract.h \
COLLADA_DOM/include/1.4/dom/domMorph.h \
COLLADA_DOM/include/1.4/dom/domFx_sampler3D_common.h \
COLLADA_DOM/include/1.4/dom/domProfile_GLES.h \
COLLADA_DOM/include/1.4/dom/domGles_texcombiner_command_type.h \
COLLADA_DOM/include/1.4/dom/domGles_newparam.h \
COLLADA_DOM/include/1.4/dom/domRotate.h \
COLLADA_DOM/include/1.4/dom/domNode.h \
COLLADA_DOM/include/1.4/dom/domBox.h \
COLLADA_DOM/include/1.4/dom/domFx_samplerCUBE_common.h \
COLLADA_DOM/include/1.4/dom/domInstance_physics_model.h \
COLLADA_DOM/include/1.4/dom/domGl_sampler1D.h \
COLLADA_DOM/include/1.4/dom/domGles_texcombiner_argumentRGB_type.h \
COLLADA_DOM/include/1.4/dom/domVisual_scene.h \
COLLADA_DOM/include/1.4/dom/domRigid_constraint.h \
COLLADA_DOM/include/1.4/dom/domInstance_rigid_constraint.h \
COLLADA_DOM/include/1.4/dom/domIDREF_array.h \
COLLADA_DOM/include/1.4/dom/domLibrary_visual_scenes.h \
COLLADA_DOM/include/1.4/dom/domCapsule.h \
COLLADA_DOM/include/1.4/dom/domPolygons.h \
COLLADA_DOM/include/1.4/dom/domGles_texture_unit.h \
COLLADA_DOM/include/1.4/dom/domCamera.h \
COLLADA_DOM/include/1.4/dom/domGl_sampler3D.h \
COLLADA_DOM/include/1.4/dom/domLibrary_images.h \
COLLADA_DOM/include/1.4/dom/domInstance_node.h \
COLLADA_DOM/include/1.4/dom/domLinestrips.h \
COLLADA_DOM/include/1.4/dom/domMaterial.h \
COLLADA_DOM/include/1.4/dom/domProfile_COMMON.h \
COLLADA_DOM/include/1.4/dom/domSkew.h \
COLLADA_DOM/include/1.4/dom/domFx_include_common.h \
COLLADA_DOM/include/1.4/dom/domGeometry.h \
COLLADA_DOM/include/1.4/dom/domMatrix.h \
COLLADA_DOM/include/1.4/dom/domBool_array.h \
COLLADA_DOM/include/1.4/dom/domLibrary_animations.h \
COLLADA_DOM/include/1.4/dom/domGl_pipeline_settings.h \
COLLADA_DOM/include/1.4/dom/domCg_setparam_simple.h \
COLLADA_DOM/include/1.4/dom/domConvex_mesh.h \
COLLADA_DOM/include/1.4/dom/domLibrary_cameras.h \
COLLADA_DOM/include/1.4/dom/domGles_texcombiner_argumentAlpha_type.h \
COLLADA_DOM/include/1.4/dom/domFx_colortarget_common.h \
COLLADA_DOM/include/1.4/dom/domInstance_material.h \
COLLADA_DOM/include/1.4/dom/domGlsl_setparam.h \
COLLADA_DOM/include/1.4/dom/domGlsl_surface_type.h \
COLLADA_DOM/include/1.4/dom/domTargetableFloat3.h \
COLLADA_DOM/include/1.4/dom/domCg_connect_param.h \
COLLADA_DOM/include/1.4/dom/domFx_code_profile.h \
COLLADA_DOM/include/1.4/dom/domFx_sampler1D_common.h \
COLLADA_DOM/include/1.4/dom/domLibrary_physics_models.h \
COLLADA_DOM/include/1.4/dom/domFloat_array.h \
COLLADA_DOM/include/1.4/dom/domInstance_physics_material.h \
COLLADA_DOM/include/1.4/dom/domGlsl_setarray_type.h \
COLLADA_DOM/include/1.4/dom/domGl_sampler2D.h \
COLLADA_DOM/include/1.4/dom/domLibrary_effects.h \
COLLADA_DOM/include/1.4/dom/domCommon_transparent_type.h \
COLLADA_DOM/include/1.4/dom/domGl_samplerRECT.h \
COLLADA_DOM/include/1.4/dom/domGlsl_setparam_simple.h \
COLLADA_DOM/include/modules/daeLIBXMLPlugin.h \
COLLADA_DOM/include/modules/daeLIBXMLResolver.h \
COLLADA_DOM/include/modules/daeSTLDatabase.h \
COLLADA_DOM/include/modules/stdErrPlugin.h \
COLLADA_DOM/include/dom.h \
COLLADA_DOM/include/dae.h

