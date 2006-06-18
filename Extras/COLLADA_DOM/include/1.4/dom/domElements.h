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

#ifndef __DOM_ELEMENTS_H__
#define __DOM_ELEMENTS_H__

#include <dom/domTypes.h>

class domInputGlobal;

typedef daeSmartRef<domInputGlobal> domInputGlobalRef;
typedef daeTArray<domInputGlobalRef> domInputGlobal_Array;

class domInputLocal;

typedef daeSmartRef<domInputLocal> domInputLocalRef;
typedef daeTArray<domInputLocalRef> domInputLocal_Array;

class domInputLocalOffset;

typedef daeSmartRef<domInputLocalOffset> domInputLocalOffsetRef;
typedef daeTArray<domInputLocalOffsetRef> domInputLocalOffset_Array;

class domInstanceWithExtra;

typedef daeSmartRef<domInstanceWithExtra> domInstanceWithExtraRef;
typedef daeTArray<domInstanceWithExtraRef> domInstanceWithExtra_Array;

class domTargetableFloat;

typedef daeSmartRef<domTargetableFloat> domTargetableFloatRef;
typedef daeTArray<domTargetableFloatRef> domTargetableFloat_Array;

class domTargetableFloat3;

typedef daeSmartRef<domTargetableFloat3> domTargetableFloat3Ref;
typedef daeTArray<domTargetableFloat3Ref> domTargetableFloat3_Array;

class domFx_surface_common;

typedef daeSmartRef<domFx_surface_common> domFx_surface_commonRef;
typedef daeTArray<domFx_surface_commonRef> domFx_surface_common_Array;

class domFx_sampler1D_common;

typedef daeSmartRef<domFx_sampler1D_common> domFx_sampler1D_commonRef;
typedef daeTArray<domFx_sampler1D_commonRef> domFx_sampler1D_common_Array;

class domFx_sampler2D_common;

typedef daeSmartRef<domFx_sampler2D_common> domFx_sampler2D_commonRef;
typedef daeTArray<domFx_sampler2D_commonRef> domFx_sampler2D_common_Array;

class domFx_sampler3D_common;

typedef daeSmartRef<domFx_sampler3D_common> domFx_sampler3D_commonRef;
typedef daeTArray<domFx_sampler3D_commonRef> domFx_sampler3D_common_Array;

class domFx_samplerCUBE_common;

typedef daeSmartRef<domFx_samplerCUBE_common> domFx_samplerCUBE_commonRef;
typedef daeTArray<domFx_samplerCUBE_commonRef> domFx_samplerCUBE_common_Array;

class domFx_samplerRECT_common;

typedef daeSmartRef<domFx_samplerRECT_common> domFx_samplerRECT_commonRef;
typedef daeTArray<domFx_samplerRECT_commonRef> domFx_samplerRECT_common_Array;

class domFx_samplerDEPTH_common;

typedef daeSmartRef<domFx_samplerDEPTH_common> domFx_samplerDEPTH_commonRef;
typedef daeTArray<domFx_samplerDEPTH_commonRef> domFx_samplerDEPTH_common_Array;

class domFx_colortarget_common;

typedef daeSmartRef<domFx_colortarget_common> domFx_colortarget_commonRef;
typedef daeTArray<domFx_colortarget_commonRef> domFx_colortarget_common_Array;

class domFx_depthtarget_common;

typedef daeSmartRef<domFx_depthtarget_common> domFx_depthtarget_commonRef;
typedef daeTArray<domFx_depthtarget_commonRef> domFx_depthtarget_common_Array;

class domFx_stenciltarget_common;

typedef daeSmartRef<domFx_stenciltarget_common> domFx_stenciltarget_commonRef;
typedef daeTArray<domFx_stenciltarget_commonRef> domFx_stenciltarget_common_Array;

class domFx_clearcolor_common;

typedef daeSmartRef<domFx_clearcolor_common> domFx_clearcolor_commonRef;
typedef daeTArray<domFx_clearcolor_commonRef> domFx_clearcolor_common_Array;

class domFx_cleardepth_common;

typedef daeSmartRef<domFx_cleardepth_common> domFx_cleardepth_commonRef;
typedef daeTArray<domFx_cleardepth_commonRef> domFx_cleardepth_common_Array;

class domFx_clearstencil_common;

typedef daeSmartRef<domFx_clearstencil_common> domFx_clearstencil_commonRef;
typedef daeTArray<domFx_clearstencil_commonRef> domFx_clearstencil_common_Array;

class domFx_annotate_common;

typedef daeSmartRef<domFx_annotate_common> domFx_annotate_commonRef;
typedef daeTArray<domFx_annotate_commonRef> domFx_annotate_common_Array;

class domFx_include_common;

typedef daeSmartRef<domFx_include_common> domFx_include_commonRef;
typedef daeTArray<domFx_include_commonRef> domFx_include_common_Array;

class domFx_newparam_common;

typedef daeSmartRef<domFx_newparam_common> domFx_newparam_commonRef;
typedef daeTArray<domFx_newparam_commonRef> domFx_newparam_common_Array;

class domFx_setparam_common;

typedef daeSmartRef<domFx_setparam_common> domFx_setparam_commonRef;
typedef daeTArray<domFx_setparam_commonRef> domFx_setparam_common_Array;

class domFx_code_profile;

typedef daeSmartRef<domFx_code_profile> domFx_code_profileRef;
typedef daeTArray<domFx_code_profileRef> domFx_code_profile_Array;

class domGl_sampler1D;

typedef daeSmartRef<domGl_sampler1D> domGl_sampler1DRef;
typedef daeTArray<domGl_sampler1DRef> domGl_sampler1D_Array;

class domGl_sampler2D;

typedef daeSmartRef<domGl_sampler2D> domGl_sampler2DRef;
typedef daeTArray<domGl_sampler2DRef> domGl_sampler2D_Array;

class domGl_sampler3D;

typedef daeSmartRef<domGl_sampler3D> domGl_sampler3DRef;
typedef daeTArray<domGl_sampler3DRef> domGl_sampler3D_Array;

class domGl_samplerCUBE;

typedef daeSmartRef<domGl_samplerCUBE> domGl_samplerCUBERef;
typedef daeTArray<domGl_samplerCUBERef> domGl_samplerCUBE_Array;

class domGl_samplerRECT;

typedef daeSmartRef<domGl_samplerRECT> domGl_samplerRECTRef;
typedef daeTArray<domGl_samplerRECTRef> domGl_samplerRECT_Array;

class domGl_samplerDEPTH;

typedef daeSmartRef<domGl_samplerDEPTH> domGl_samplerDEPTHRef;
typedef daeTArray<domGl_samplerDEPTHRef> domGl_samplerDEPTH_Array;

class domGlsl_newarray_type;

typedef daeSmartRef<domGlsl_newarray_type> domGlsl_newarray_typeRef;
typedef daeTArray<domGlsl_newarray_typeRef> domGlsl_newarray_type_Array;

class domGlsl_setarray_type;

typedef daeSmartRef<domGlsl_setarray_type> domGlsl_setarray_typeRef;
typedef daeTArray<domGlsl_setarray_typeRef> domGlsl_setarray_type_Array;

class domGlsl_surface_type;

typedef daeSmartRef<domGlsl_surface_type> domGlsl_surface_typeRef;
typedef daeTArray<domGlsl_surface_typeRef> domGlsl_surface_type_Array;

class domGlsl_newparam;

typedef daeSmartRef<domGlsl_newparam> domGlsl_newparamRef;
typedef daeTArray<domGlsl_newparamRef> domGlsl_newparam_Array;

class domGlsl_setparam_simple;

typedef daeSmartRef<domGlsl_setparam_simple> domGlsl_setparam_simpleRef;
typedef daeTArray<domGlsl_setparam_simpleRef> domGlsl_setparam_simple_Array;

class domGlsl_setparam;

typedef daeSmartRef<domGlsl_setparam> domGlsl_setparamRef;
typedef daeTArray<domGlsl_setparamRef> domGlsl_setparam_Array;

class domCommon_float_or_param_type;

typedef daeSmartRef<domCommon_float_or_param_type> domCommon_float_or_param_typeRef;
typedef daeTArray<domCommon_float_or_param_typeRef> domCommon_float_or_param_type_Array;

class domCommon_color_or_texture_type;

typedef daeSmartRef<domCommon_color_or_texture_type> domCommon_color_or_texture_typeRef;
typedef daeTArray<domCommon_color_or_texture_typeRef> domCommon_color_or_texture_type_Array;

class domCommon_newparam_type;

typedef daeSmartRef<domCommon_newparam_type> domCommon_newparam_typeRef;
typedef daeTArray<domCommon_newparam_typeRef> domCommon_newparam_type_Array;

class domCg_sampler1D;

typedef daeSmartRef<domCg_sampler1D> domCg_sampler1DRef;
typedef daeTArray<domCg_sampler1DRef> domCg_sampler1D_Array;

class domCg_sampler2D;

typedef daeSmartRef<domCg_sampler2D> domCg_sampler2DRef;
typedef daeTArray<domCg_sampler2DRef> domCg_sampler2D_Array;

class domCg_sampler3D;

typedef daeSmartRef<domCg_sampler3D> domCg_sampler3DRef;
typedef daeTArray<domCg_sampler3DRef> domCg_sampler3D_Array;

class domCg_samplerCUBE;

typedef daeSmartRef<domCg_samplerCUBE> domCg_samplerCUBERef;
typedef daeTArray<domCg_samplerCUBERef> domCg_samplerCUBE_Array;

class domCg_samplerRECT;

typedef daeSmartRef<domCg_samplerRECT> domCg_samplerRECTRef;
typedef daeTArray<domCg_samplerRECTRef> domCg_samplerRECT_Array;

class domCg_samplerDEPTH;

typedef daeSmartRef<domCg_samplerDEPTH> domCg_samplerDEPTHRef;
typedef daeTArray<domCg_samplerDEPTHRef> domCg_samplerDEPTH_Array;

class domCg_connect_param;

typedef daeSmartRef<domCg_connect_param> domCg_connect_paramRef;
typedef daeTArray<domCg_connect_paramRef> domCg_connect_param_Array;

class domCg_newarray_type;

typedef daeSmartRef<domCg_newarray_type> domCg_newarray_typeRef;
typedef daeTArray<domCg_newarray_typeRef> domCg_newarray_type_Array;

class domCg_setarray_type;

typedef daeSmartRef<domCg_setarray_type> domCg_setarray_typeRef;
typedef daeTArray<domCg_setarray_typeRef> domCg_setarray_type_Array;

class domCg_setuser_type;

typedef daeSmartRef<domCg_setuser_type> domCg_setuser_typeRef;
typedef daeTArray<domCg_setuser_typeRef> domCg_setuser_type_Array;

class domCg_surface_type;

typedef daeSmartRef<domCg_surface_type> domCg_surface_typeRef;
typedef daeTArray<domCg_surface_typeRef> domCg_surface_type_Array;

class domCg_newparam;

typedef daeSmartRef<domCg_newparam> domCg_newparamRef;
typedef daeTArray<domCg_newparamRef> domCg_newparam_Array;

class domCg_setparam_simple;

typedef daeSmartRef<domCg_setparam_simple> domCg_setparam_simpleRef;
typedef daeTArray<domCg_setparam_simpleRef> domCg_setparam_simple_Array;

class domCg_setparam;

typedef daeSmartRef<domCg_setparam> domCg_setparamRef;
typedef daeTArray<domCg_setparamRef> domCg_setparam_Array;

class domGles_texture_constant_type;

typedef daeSmartRef<domGles_texture_constant_type> domGles_texture_constant_typeRef;
typedef daeTArray<domGles_texture_constant_typeRef> domGles_texture_constant_type_Array;

class domGles_texenv_command_type;

typedef daeSmartRef<domGles_texenv_command_type> domGles_texenv_command_typeRef;
typedef daeTArray<domGles_texenv_command_typeRef> domGles_texenv_command_type_Array;

class domGles_texcombiner_argumentRGB_type;

typedef daeSmartRef<domGles_texcombiner_argumentRGB_type> domGles_texcombiner_argumentRGB_typeRef;
typedef daeTArray<domGles_texcombiner_argumentRGB_typeRef> domGles_texcombiner_argumentRGB_type_Array;

class domGles_texcombiner_argumentAlpha_type;

typedef daeSmartRef<domGles_texcombiner_argumentAlpha_type> domGles_texcombiner_argumentAlpha_typeRef;
typedef daeTArray<domGles_texcombiner_argumentAlpha_typeRef> domGles_texcombiner_argumentAlpha_type_Array;

class domGles_texcombiner_commandRGB_type;

typedef daeSmartRef<domGles_texcombiner_commandRGB_type> domGles_texcombiner_commandRGB_typeRef;
typedef daeTArray<domGles_texcombiner_commandRGB_typeRef> domGles_texcombiner_commandRGB_type_Array;

class domGles_texcombiner_commandAlpha_type;

typedef daeSmartRef<domGles_texcombiner_commandAlpha_type> domGles_texcombiner_commandAlpha_typeRef;
typedef daeTArray<domGles_texcombiner_commandAlpha_typeRef> domGles_texcombiner_commandAlpha_type_Array;

class domGles_texcombiner_command_type;

typedef daeSmartRef<domGles_texcombiner_command_type> domGles_texcombiner_command_typeRef;
typedef daeTArray<domGles_texcombiner_command_typeRef> domGles_texcombiner_command_type_Array;

class domGles_texture_pipeline;

typedef daeSmartRef<domGles_texture_pipeline> domGles_texture_pipelineRef;
typedef daeTArray<domGles_texture_pipelineRef> domGles_texture_pipeline_Array;

class domGles_texture_unit;

typedef daeSmartRef<domGles_texture_unit> domGles_texture_unitRef;
typedef daeTArray<domGles_texture_unitRef> domGles_texture_unit_Array;

class domGles_sampler_state;

typedef daeSmartRef<domGles_sampler_state> domGles_sampler_stateRef;
typedef daeTArray<domGles_sampler_stateRef> domGles_sampler_state_Array;

class domGles_newparam;

typedef daeSmartRef<domGles_newparam> domGles_newparamRef;
typedef daeTArray<domGles_newparamRef> domGles_newparam_Array;

class domFx_annotate_type_common;

typedef daeSmartRef<domFx_annotate_type_common> domFx_annotate_type_commonRef;
typedef daeTArray<domFx_annotate_type_commonRef> domFx_annotate_type_common_Array;

class domFx_basic_type_common;

typedef daeSmartRef<domFx_basic_type_common> domFx_basic_type_commonRef;
typedef daeTArray<domFx_basic_type_commonRef> domFx_basic_type_common_Array;

class domGl_pipeline_settings;

typedef daeSmartRef<domGl_pipeline_settings> domGl_pipeline_settingsRef;
typedef daeTArray<domGl_pipeline_settingsRef> domGl_pipeline_settings_Array;

class domGlsl_param_type;

typedef daeSmartRef<domGlsl_param_type> domGlsl_param_typeRef;
typedef daeTArray<domGlsl_param_typeRef> domGlsl_param_type_Array;

class domCg_param_type;

typedef daeSmartRef<domCg_param_type> domCg_param_typeRef;
typedef daeTArray<domCg_param_typeRef> domCg_param_type_Array;

class domGles_pipeline_settings;

typedef daeSmartRef<domGles_pipeline_settings> domGles_pipeline_settingsRef;
typedef daeTArray<domGles_pipeline_settingsRef> domGles_pipeline_settings_Array;

class domGles_basic_type_common;

typedef daeSmartRef<domGles_basic_type_common> domGles_basic_type_commonRef;
typedef daeTArray<domGles_basic_type_commonRef> domGles_basic_type_common_Array;

class domCOLLADA;

typedef daeSmartRef<domCOLLADA> domCOLLADARef;
typedef daeTArray<domCOLLADARef> domCOLLADA_Array;

class domIDREF_array;

typedef daeSmartRef<domIDREF_array> domIDREF_arrayRef;
typedef daeTArray<domIDREF_arrayRef> domIDREF_array_Array;

class domName_array;

typedef daeSmartRef<domName_array> domName_arrayRef;
typedef daeTArray<domName_arrayRef> domName_array_Array;

class domBool_array;

typedef daeSmartRef<domBool_array> domBool_arrayRef;
typedef daeTArray<domBool_arrayRef> domBool_array_Array;

class domFloat_array;

typedef daeSmartRef<domFloat_array> domFloat_arrayRef;
typedef daeTArray<domFloat_arrayRef> domFloat_array_Array;

class domInt_array;

typedef daeSmartRef<domInt_array> domInt_arrayRef;
typedef daeTArray<domInt_arrayRef> domInt_array_Array;

class domAccessor;

typedef daeSmartRef<domAccessor> domAccessorRef;
typedef daeTArray<domAccessorRef> domAccessor_Array;

class domParam;

typedef daeSmartRef<domParam> domParamRef;
typedef daeTArray<domParamRef> domParam_Array;

class domSource;

typedef daeSmartRef<domSource> domSourceRef;
typedef daeTArray<domSourceRef> domSource_Array;

class domGeometry;

typedef daeSmartRef<domGeometry> domGeometryRef;
typedef daeTArray<domGeometryRef> domGeometry_Array;

class domMesh;

typedef daeSmartRef<domMesh> domMeshRef;
typedef daeTArray<domMeshRef> domMesh_Array;

class domSpline;

typedef daeSmartRef<domSpline> domSplineRef;
typedef daeTArray<domSplineRef> domSpline_Array;

class domP;

typedef daeSmartRef<domP> domPRef;
typedef daeTArray<domPRef> domP_Array;

class domLines;

typedef daeSmartRef<domLines> domLinesRef;
typedef daeTArray<domLinesRef> domLines_Array;

class domLinestrips;

typedef daeSmartRef<domLinestrips> domLinestripsRef;
typedef daeTArray<domLinestripsRef> domLinestrips_Array;

class domPolygons;

typedef daeSmartRef<domPolygons> domPolygonsRef;
typedef daeTArray<domPolygonsRef> domPolygons_Array;

class domPolylist;

typedef daeSmartRef<domPolylist> domPolylistRef;
typedef daeTArray<domPolylistRef> domPolylist_Array;

class domTriangles;

typedef daeSmartRef<domTriangles> domTrianglesRef;
typedef daeTArray<domTrianglesRef> domTriangles_Array;

class domTrifans;

typedef daeSmartRef<domTrifans> domTrifansRef;
typedef daeTArray<domTrifansRef> domTrifans_Array;

class domTristrips;

typedef daeSmartRef<domTristrips> domTristripsRef;
typedef daeTArray<domTristripsRef> domTristrips_Array;

class domVertices;

typedef daeSmartRef<domVertices> domVerticesRef;
typedef daeTArray<domVerticesRef> domVertices_Array;

class domLookat;

typedef daeSmartRef<domLookat> domLookatRef;
typedef daeTArray<domLookatRef> domLookat_Array;

class domMatrix;

typedef daeSmartRef<domMatrix> domMatrixRef;
typedef daeTArray<domMatrixRef> domMatrix_Array;

class domRotate;

typedef daeSmartRef<domRotate> domRotateRef;
typedef daeTArray<domRotateRef> domRotate_Array;

class domScale;

typedef daeSmartRef<domScale> domScaleRef;
typedef daeTArray<domScaleRef> domScale_Array;

class domSkew;

typedef daeSmartRef<domSkew> domSkewRef;
typedef daeTArray<domSkewRef> domSkew_Array;

class domTranslate;

typedef daeSmartRef<domTranslate> domTranslateRef;
typedef daeTArray<domTranslateRef> domTranslate_Array;

class domImage;

typedef daeSmartRef<domImage> domImageRef;
typedef daeTArray<domImageRef> domImage_Array;

class domLight;

typedef daeSmartRef<domLight> domLightRef;
typedef daeTArray<domLightRef> domLight_Array;

class domMaterial;

typedef daeSmartRef<domMaterial> domMaterialRef;
typedef daeTArray<domMaterialRef> domMaterial_Array;

class domCamera;

typedef daeSmartRef<domCamera> domCameraRef;
typedef daeTArray<domCameraRef> domCamera_Array;

class domAnimation;

typedef daeSmartRef<domAnimation> domAnimationRef;
typedef daeTArray<domAnimationRef> domAnimation_Array;

class domAnimation_clip;

typedef daeSmartRef<domAnimation_clip> domAnimation_clipRef;
typedef daeTArray<domAnimation_clipRef> domAnimation_clip_Array;

class domChannel;

typedef daeSmartRef<domChannel> domChannelRef;
typedef daeTArray<domChannelRef> domChannel_Array;

class domSampler;

typedef daeSmartRef<domSampler> domSamplerRef;
typedef daeTArray<domSamplerRef> domSampler_Array;

class domController;

typedef daeSmartRef<domController> domControllerRef;
typedef daeTArray<domControllerRef> domController_Array;

class domSkin;

typedef daeSmartRef<domSkin> domSkinRef;
typedef daeTArray<domSkinRef> domSkin_Array;

class domMorph;

typedef daeSmartRef<domMorph> domMorphRef;
typedef daeTArray<domMorphRef> domMorph_Array;

class domAsset;

typedef daeSmartRef<domAsset> domAssetRef;
typedef daeTArray<domAssetRef> domAsset_Array;

class domExtra;

typedef daeSmartRef<domExtra> domExtraRef;
typedef daeTArray<domExtraRef> domExtra_Array;

class domTechnique;

typedef daeSmartRef<domTechnique> domTechniqueRef;
typedef daeTArray<domTechniqueRef> domTechnique_Array;

class domNode;

typedef daeSmartRef<domNode> domNodeRef;
typedef daeTArray<domNodeRef> domNode_Array;

class domVisual_scene;

typedef daeSmartRef<domVisual_scene> domVisual_sceneRef;
typedef daeTArray<domVisual_sceneRef> domVisual_scene_Array;

class domBind_material;

typedef daeSmartRef<domBind_material> domBind_materialRef;
typedef daeTArray<domBind_materialRef> domBind_material_Array;

class domInstance_camera;

typedef daeSmartRef<domInstance_camera> domInstance_cameraRef;
typedef daeTArray<domInstance_cameraRef> domInstance_camera_Array;

class domInstance_controller;

typedef daeSmartRef<domInstance_controller> domInstance_controllerRef;
typedef daeTArray<domInstance_controllerRef> domInstance_controller_Array;

class domInstance_effect;

typedef daeSmartRef<domInstance_effect> domInstance_effectRef;
typedef daeTArray<domInstance_effectRef> domInstance_effect_Array;

class domInstance_force_field;

typedef daeSmartRef<domInstance_force_field> domInstance_force_fieldRef;
typedef daeTArray<domInstance_force_fieldRef> domInstance_force_field_Array;

class domInstance_geometry;

typedef daeSmartRef<domInstance_geometry> domInstance_geometryRef;
typedef daeTArray<domInstance_geometryRef> domInstance_geometry_Array;

class domInstance_light;

typedef daeSmartRef<domInstance_light> domInstance_lightRef;
typedef daeTArray<domInstance_lightRef> domInstance_light_Array;

class domInstance_material;

typedef daeSmartRef<domInstance_material> domInstance_materialRef;
typedef daeTArray<domInstance_materialRef> domInstance_material_Array;

class domInstance_node;

typedef daeSmartRef<domInstance_node> domInstance_nodeRef;
typedef daeTArray<domInstance_nodeRef> domInstance_node_Array;

class domInstance_physics_material;

typedef daeSmartRef<domInstance_physics_material> domInstance_physics_materialRef;
typedef daeTArray<domInstance_physics_materialRef> domInstance_physics_material_Array;

class domInstance_physics_model;

typedef daeSmartRef<domInstance_physics_model> domInstance_physics_modelRef;
typedef daeTArray<domInstance_physics_modelRef> domInstance_physics_model_Array;

class domInstance_rigid_body;

typedef daeSmartRef<domInstance_rigid_body> domInstance_rigid_bodyRef;
typedef daeTArray<domInstance_rigid_bodyRef> domInstance_rigid_body_Array;

class domInstance_rigid_constraint;

typedef daeSmartRef<domInstance_rigid_constraint> domInstance_rigid_constraintRef;
typedef daeTArray<domInstance_rigid_constraintRef> domInstance_rigid_constraint_Array;

class domLibrary_animations;

typedef daeSmartRef<domLibrary_animations> domLibrary_animationsRef;
typedef daeTArray<domLibrary_animationsRef> domLibrary_animations_Array;

class domLibrary_animation_clips;

typedef daeSmartRef<domLibrary_animation_clips> domLibrary_animation_clipsRef;
typedef daeTArray<domLibrary_animation_clipsRef> domLibrary_animation_clips_Array;

class domLibrary_cameras;

typedef daeSmartRef<domLibrary_cameras> domLibrary_camerasRef;
typedef daeTArray<domLibrary_camerasRef> domLibrary_cameras_Array;

class domLibrary_controllers;

typedef daeSmartRef<domLibrary_controllers> domLibrary_controllersRef;
typedef daeTArray<domLibrary_controllersRef> domLibrary_controllers_Array;

class domLibrary_geometries;

typedef daeSmartRef<domLibrary_geometries> domLibrary_geometriesRef;
typedef daeTArray<domLibrary_geometriesRef> domLibrary_geometries_Array;

class domLibrary_effects;

typedef daeSmartRef<domLibrary_effects> domLibrary_effectsRef;
typedef daeTArray<domLibrary_effectsRef> domLibrary_effects_Array;

class domLibrary_force_fields;

typedef daeSmartRef<domLibrary_force_fields> domLibrary_force_fieldsRef;
typedef daeTArray<domLibrary_force_fieldsRef> domLibrary_force_fields_Array;

class domLibrary_images;

typedef daeSmartRef<domLibrary_images> domLibrary_imagesRef;
typedef daeTArray<domLibrary_imagesRef> domLibrary_images_Array;

class domLibrary_lights;

typedef daeSmartRef<domLibrary_lights> domLibrary_lightsRef;
typedef daeTArray<domLibrary_lightsRef> domLibrary_lights_Array;

class domLibrary_materials;

typedef daeSmartRef<domLibrary_materials> domLibrary_materialsRef;
typedef daeTArray<domLibrary_materialsRef> domLibrary_materials_Array;

class domLibrary_nodes;

typedef daeSmartRef<domLibrary_nodes> domLibrary_nodesRef;
typedef daeTArray<domLibrary_nodesRef> domLibrary_nodes_Array;

class domLibrary_physics_materials;

typedef daeSmartRef<domLibrary_physics_materials> domLibrary_physics_materialsRef;
typedef daeTArray<domLibrary_physics_materialsRef> domLibrary_physics_materials_Array;

class domLibrary_physics_models;

typedef daeSmartRef<domLibrary_physics_models> domLibrary_physics_modelsRef;
typedef daeTArray<domLibrary_physics_modelsRef> domLibrary_physics_models_Array;

class domLibrary_physics_scenes;

typedef daeSmartRef<domLibrary_physics_scenes> domLibrary_physics_scenesRef;
typedef daeTArray<domLibrary_physics_scenesRef> domLibrary_physics_scenes_Array;

class domLibrary_visual_scenes;

typedef daeSmartRef<domLibrary_visual_scenes> domLibrary_visual_scenesRef;
typedef daeTArray<domLibrary_visual_scenesRef> domLibrary_visual_scenes_Array;

class domFx_profile_abstract;

typedef daeSmartRef<domFx_profile_abstract> domFx_profile_abstractRef;
typedef daeTArray<domFx_profile_abstractRef> domFx_profile_abstract_Array;

class domEffect;

typedef daeSmartRef<domEffect> domEffectRef;
typedef daeTArray<domEffectRef> domEffect_Array;

class domGl_hook_abstract;

typedef daeSmartRef<domGl_hook_abstract> domGl_hook_abstractRef;
typedef daeTArray<domGl_hook_abstractRef> domGl_hook_abstract_Array;

class domProfile_GLSL;

typedef daeSmartRef<domProfile_GLSL> domProfile_GLSLRef;
typedef daeTArray<domProfile_GLSLRef> domProfile_GLSL_Array;

class domProfile_COMMON;

typedef daeSmartRef<domProfile_COMMON> domProfile_COMMONRef;
typedef daeTArray<domProfile_COMMONRef> domProfile_COMMON_Array;

class domProfile_CG;

typedef daeSmartRef<domProfile_CG> domProfile_CGRef;
typedef daeTArray<domProfile_CGRef> domProfile_CG_Array;

class domProfile_GLES;

typedef daeSmartRef<domProfile_GLES> domProfile_GLESRef;
typedef daeTArray<domProfile_GLESRef> domProfile_GLES_Array;

class domBox;

typedef daeSmartRef<domBox> domBoxRef;
typedef daeTArray<domBoxRef> domBox_Array;

class domPlane;

typedef daeSmartRef<domPlane> domPlaneRef;
typedef daeTArray<domPlaneRef> domPlane_Array;

class domSphere;

typedef daeSmartRef<domSphere> domSphereRef;
typedef daeTArray<domSphereRef> domSphere_Array;

class domEllipsoid;

typedef daeSmartRef<domEllipsoid> domEllipsoidRef;
typedef daeTArray<domEllipsoidRef> domEllipsoid_Array;

class domCylinder;

typedef daeSmartRef<domCylinder> domCylinderRef;
typedef daeTArray<domCylinderRef> domCylinder_Array;

class domTapered_cylinder;

typedef daeSmartRef<domTapered_cylinder> domTapered_cylinderRef;
typedef daeTArray<domTapered_cylinderRef> domTapered_cylinder_Array;

class domCapsule;

typedef daeSmartRef<domCapsule> domCapsuleRef;
typedef daeTArray<domCapsuleRef> domCapsule_Array;

class domTapered_capsule;

typedef daeSmartRef<domTapered_capsule> domTapered_capsuleRef;
typedef daeTArray<domTapered_capsuleRef> domTapered_capsule_Array;

class domConvex_mesh;

typedef daeSmartRef<domConvex_mesh> domConvex_meshRef;
typedef daeTArray<domConvex_meshRef> domConvex_mesh_Array;

class domForce_field;

typedef daeSmartRef<domForce_field> domForce_fieldRef;
typedef daeTArray<domForce_fieldRef> domForce_field_Array;

class domPhysics_material;

typedef daeSmartRef<domPhysics_material> domPhysics_materialRef;
typedef daeTArray<domPhysics_materialRef> domPhysics_material_Array;

class domPhysics_scene;

typedef daeSmartRef<domPhysics_scene> domPhysics_sceneRef;
typedef daeTArray<domPhysics_sceneRef> domPhysics_scene_Array;

class domRigid_body;

typedef daeSmartRef<domRigid_body> domRigid_bodyRef;
typedef daeTArray<domRigid_bodyRef> domRigid_body_Array;

class domRigid_constraint;

typedef daeSmartRef<domRigid_constraint> domRigid_constraintRef;
typedef daeTArray<domRigid_constraintRef> domRigid_constraint_Array;

class domPhysics_model;

typedef daeSmartRef<domPhysics_model> domPhysics_modelRef;
typedef daeTArray<domPhysics_modelRef> domPhysics_model_Array;


#endif //__DOM_ELEMENTS_H__

