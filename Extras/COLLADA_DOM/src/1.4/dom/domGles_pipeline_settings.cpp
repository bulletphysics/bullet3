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
#include <dom/domGles_pipeline_settings.h>
#include <dae/daeMetaCMPolicy.h>
#include <dae/daeMetaSequence.h>
#include <dae/daeMetaChoice.h>
#include <dae/daeMetaGroup.h>
#include <dae/daeMetaAny.h>
#include <dae/daeMetaElementAttribute.h>

daeElementRef
domGles_pipeline_settings::create(daeInt bytes)
{
	domGles_pipeline_settingsRef ref = new(bytes) domGles_pipeline_settings;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "gles_pipeline_settings" );
	_Meta->registerClass(domGles_pipeline_settings::create, &_Meta);

	_Meta->setIsTransparent( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaChoice( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "alpha_func" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemAlpha_func) );
	mea->setElementType( domGles_pipeline_settings::domAlpha_func::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "blend_func" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemBlend_func) );
	mea->setElementType( domGles_pipeline_settings::domBlend_func::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "clear_color" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemClear_color) );
	mea->setElementType( domGles_pipeline_settings::domClear_color::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "clear_stencil" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemClear_stencil) );
	mea->setElementType( domGles_pipeline_settings::domClear_stencil::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "clear_depth" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemClear_depth) );
	mea->setElementType( domGles_pipeline_settings::domClear_depth::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "clip_plane" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemClip_plane) );
	mea->setElementType( domGles_pipeline_settings::domClip_plane::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "color_mask" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemColor_mask) );
	mea->setElementType( domGles_pipeline_settings::domColor_mask::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "cull_face" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemCull_face) );
	mea->setElementType( domGles_pipeline_settings::domCull_face::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "depth_func" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemDepth_func) );
	mea->setElementType( domGles_pipeline_settings::domDepth_func::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "depth_mask" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemDepth_mask) );
	mea->setElementType( domGles_pipeline_settings::domDepth_mask::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "depth_range" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemDepth_range) );
	mea->setElementType( domGles_pipeline_settings::domDepth_range::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "fog_color" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemFog_color) );
	mea->setElementType( domGles_pipeline_settings::domFog_color::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "fog_density" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemFog_density) );
	mea->setElementType( domGles_pipeline_settings::domFog_density::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "fog_mode" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemFog_mode) );
	mea->setElementType( domGles_pipeline_settings::domFog_mode::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "fog_start" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemFog_start) );
	mea->setElementType( domGles_pipeline_settings::domFog_start::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "fog_end" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemFog_end) );
	mea->setElementType( domGles_pipeline_settings::domFog_end::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "front_face" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemFront_face) );
	mea->setElementType( domGles_pipeline_settings::domFront_face::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "texture_pipeline" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemTexture_pipeline) );
	mea->setElementType( domGles_pipeline_settings::domTexture_pipeline::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "logic_op" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemLogic_op) );
	mea->setElementType( domGles_pipeline_settings::domLogic_op::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "light_ambient" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemLight_ambient) );
	mea->setElementType( domGles_pipeline_settings::domLight_ambient::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "light_diffuse" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemLight_diffuse) );
	mea->setElementType( domGles_pipeline_settings::domLight_diffuse::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "light_specular" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemLight_specular) );
	mea->setElementType( domGles_pipeline_settings::domLight_specular::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "light_position" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemLight_position) );
	mea->setElementType( domGles_pipeline_settings::domLight_position::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "light_constant_attenuation" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemLight_constant_attenuation) );
	mea->setElementType( domGles_pipeline_settings::domLight_constant_attenuation::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "light_linear_attenutation" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemLight_linear_attenutation) );
	mea->setElementType( domGles_pipeline_settings::domLight_linear_attenutation::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "light_quadratic_attenuation" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemLight_quadratic_attenuation) );
	mea->setElementType( domGles_pipeline_settings::domLight_quadratic_attenuation::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "light_spot_cutoff" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemLight_spot_cutoff) );
	mea->setElementType( domGles_pipeline_settings::domLight_spot_cutoff::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "light_spot_direction" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemLight_spot_direction) );
	mea->setElementType( domGles_pipeline_settings::domLight_spot_direction::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "light_spot_exponent" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemLight_spot_exponent) );
	mea->setElementType( domGles_pipeline_settings::domLight_spot_exponent::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "light_model_ambient" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemLight_model_ambient) );
	mea->setElementType( domGles_pipeline_settings::domLight_model_ambient::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "line_width" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemLine_width) );
	mea->setElementType( domGles_pipeline_settings::domLine_width::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "material_ambient" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemMaterial_ambient) );
	mea->setElementType( domGles_pipeline_settings::domMaterial_ambient::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "material_diffuse" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemMaterial_diffuse) );
	mea->setElementType( domGles_pipeline_settings::domMaterial_diffuse::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "material_emission" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemMaterial_emission) );
	mea->setElementType( domGles_pipeline_settings::domMaterial_emission::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "material_shininess" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemMaterial_shininess) );
	mea->setElementType( domGles_pipeline_settings::domMaterial_shininess::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "material_specular" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemMaterial_specular) );
	mea->setElementType( domGles_pipeline_settings::domMaterial_specular::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "model_view_matrix" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemModel_view_matrix) );
	mea->setElementType( domGles_pipeline_settings::domModel_view_matrix::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "point_distance_attenuation" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemPoint_distance_attenuation) );
	mea->setElementType( domGles_pipeline_settings::domPoint_distance_attenuation::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "point_fade_threshold_size" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemPoint_fade_threshold_size) );
	mea->setElementType( domGles_pipeline_settings::domPoint_fade_threshold_size::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "point_size" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemPoint_size) );
	mea->setElementType( domGles_pipeline_settings::domPoint_size::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "point_size_min" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemPoint_size_min) );
	mea->setElementType( domGles_pipeline_settings::domPoint_size_min::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "point_size_max" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemPoint_size_max) );
	mea->setElementType( domGles_pipeline_settings::domPoint_size_max::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "polygon_offset" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemPolygon_offset) );
	mea->setElementType( domGles_pipeline_settings::domPolygon_offset::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "projection_matrix" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemProjection_matrix) );
	mea->setElementType( domGles_pipeline_settings::domProjection_matrix::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "scissor" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemScissor) );
	mea->setElementType( domGles_pipeline_settings::domScissor::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "shade_model" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemShade_model) );
	mea->setElementType( domGles_pipeline_settings::domShade_model::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "stencil_func" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemStencil_func) );
	mea->setElementType( domGles_pipeline_settings::domStencil_func::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "stencil_mask" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemStencil_mask) );
	mea->setElementType( domGles_pipeline_settings::domStencil_mask::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "stencil_op" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemStencil_op) );
	mea->setElementType( domGles_pipeline_settings::domStencil_op::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "alpha_test_enable" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemAlpha_test_enable) );
	mea->setElementType( domGles_pipeline_settings::domAlpha_test_enable::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "blend_enable" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemBlend_enable) );
	mea->setElementType( domGles_pipeline_settings::domBlend_enable::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "clip_plane_enable" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemClip_plane_enable) );
	mea->setElementType( domGles_pipeline_settings::domClip_plane_enable::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "color_logic_op_enable" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemColor_logic_op_enable) );
	mea->setElementType( domGles_pipeline_settings::domColor_logic_op_enable::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "color_material_enable" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemColor_material_enable) );
	mea->setElementType( domGles_pipeline_settings::domColor_material_enable::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "cull_face_enable" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemCull_face_enable) );
	mea->setElementType( domGles_pipeline_settings::domCull_face_enable::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "depth_test_enable" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemDepth_test_enable) );
	mea->setElementType( domGles_pipeline_settings::domDepth_test_enable::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "dither_enable" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemDither_enable) );
	mea->setElementType( domGles_pipeline_settings::domDither_enable::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "fog_enable" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemFog_enable) );
	mea->setElementType( domGles_pipeline_settings::domFog_enable::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "texture_pipeline_enable" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemTexture_pipeline_enable) );
	mea->setElementType( domGles_pipeline_settings::domTexture_pipeline_enable::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "light_enable" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemLight_enable) );
	mea->setElementType( domGles_pipeline_settings::domLight_enable::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "lighting_enable" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemLighting_enable) );
	mea->setElementType( domGles_pipeline_settings::domLighting_enable::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "light_model_two_side_enable" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemLight_model_two_side_enable) );
	mea->setElementType( domGles_pipeline_settings::domLight_model_two_side_enable::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "line_smooth_enable" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemLine_smooth_enable) );
	mea->setElementType( domGles_pipeline_settings::domLine_smooth_enable::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "multisample_enable" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemMultisample_enable) );
	mea->setElementType( domGles_pipeline_settings::domMultisample_enable::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "normalize_enable" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemNormalize_enable) );
	mea->setElementType( domGles_pipeline_settings::domNormalize_enable::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "point_smooth_enable" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemPoint_smooth_enable) );
	mea->setElementType( domGles_pipeline_settings::domPoint_smooth_enable::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "polygon_offset_fill_enable" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemPolygon_offset_fill_enable) );
	mea->setElementType( domGles_pipeline_settings::domPolygon_offset_fill_enable::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "rescale_normal_enable" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemRescale_normal_enable) );
	mea->setElementType( domGles_pipeline_settings::domRescale_normal_enable::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "sample_alpha_to_coverage_enable" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemSample_alpha_to_coverage_enable) );
	mea->setElementType( domGles_pipeline_settings::domSample_alpha_to_coverage_enable::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "sample_alpha_to_one_enable" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemSample_alpha_to_one_enable) );
	mea->setElementType( domGles_pipeline_settings::domSample_alpha_to_one_enable::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "sample_coverage_enable" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemSample_coverage_enable) );
	mea->setElementType( domGles_pipeline_settings::domSample_coverage_enable::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "scissor_test_enable" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemScissor_test_enable) );
	mea->setElementType( domGles_pipeline_settings::domScissor_test_enable::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "stencil_test_enable" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings,elemStencil_test_enable) );
	mea->setElementType( domGles_pipeline_settings::domStencil_test_enable::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	_Meta->setCMRoot( cm );	
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domGles_pipeline_settings,_contents));
    _Meta->addContentsOrder(daeOffsetOf(domGles_pipeline_settings,_contentsOrder));

	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domAlpha_func::create(daeInt bytes)
{
	domGles_pipeline_settings::domAlpha_funcRef ref = new(bytes) domGles_pipeline_settings::domAlpha_func;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domAlpha_func::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "alpha_func" );
	_Meta->registerClass(domGles_pipeline_settings::domAlpha_func::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "func" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings::domAlpha_func,elemFunc) );
	mea->setElementType( domGles_pipeline_settings::domAlpha_func::domFunc::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 1, 1 );
	mea->setName( "value" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings::domAlpha_func,elemValue) );
	mea->setElementType( domGles_pipeline_settings::domAlpha_func::domValue::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 1 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domAlpha_func));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domAlpha_func::domFunc::create(daeInt bytes)
{
	domGles_pipeline_settings::domAlpha_func::domFuncRef ref = new(bytes) domGles_pipeline_settings::domAlpha_func::domFunc;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domAlpha_func::domFunc::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "func" );
	_Meta->registerClass(domGles_pipeline_settings::domAlpha_func::domFunc::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_func_type"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domAlpha_func::domFunc , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "ALWAYS");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domAlpha_func::domFunc , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domAlpha_func::domFunc));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domAlpha_func::domValue::create(daeInt bytes)
{
	domGles_pipeline_settings::domAlpha_func::domValueRef ref = new(bytes) domGles_pipeline_settings::domAlpha_func::domValue;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domAlpha_func::domValue::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "value" );
	_Meta->registerClass(domGles_pipeline_settings::domAlpha_func::domValue::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_alpha_value_type"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domAlpha_func::domValue , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "0.0");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domAlpha_func::domValue , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domAlpha_func::domValue));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domBlend_func::create(daeInt bytes)
{
	domGles_pipeline_settings::domBlend_funcRef ref = new(bytes) domGles_pipeline_settings::domBlend_func;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domBlend_func::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "blend_func" );
	_Meta->registerClass(domGles_pipeline_settings::domBlend_func::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "src" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings::domBlend_func,elemSrc) );
	mea->setElementType( domGles_pipeline_settings::domBlend_func::domSrc::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 1, 1 );
	mea->setName( "dest" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings::domBlend_func,elemDest) );
	mea->setElementType( domGles_pipeline_settings::domBlend_func::domDest::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 1 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domBlend_func));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domBlend_func::domSrc::create(daeInt bytes)
{
	domGles_pipeline_settings::domBlend_func::domSrcRef ref = new(bytes) domGles_pipeline_settings::domBlend_func::domSrc;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domBlend_func::domSrc::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "src" );
	_Meta->registerClass(domGles_pipeline_settings::domBlend_func::domSrc::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_blend_type"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domBlend_func::domSrc , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "ONE");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domBlend_func::domSrc , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domBlend_func::domSrc));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domBlend_func::domDest::create(daeInt bytes)
{
	domGles_pipeline_settings::domBlend_func::domDestRef ref = new(bytes) domGles_pipeline_settings::domBlend_func::domDest;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domBlend_func::domDest::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "dest" );
	_Meta->registerClass(domGles_pipeline_settings::domBlend_func::domDest::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_blend_type"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domBlend_func::domDest , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "ZERO");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domBlend_func::domDest , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domBlend_func::domDest));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domClear_color::create(daeInt bytes)
{
	domGles_pipeline_settings::domClear_colorRef ref = new(bytes) domGles_pipeline_settings::domClear_color;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domClear_color::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "clear_color" );
	_Meta->registerClass(domGles_pipeline_settings::domClear_color::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domClear_color , attrValue ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domClear_color , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domClear_color));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domClear_stencil::create(daeInt bytes)
{
	domGles_pipeline_settings::domClear_stencilRef ref = new(bytes) domGles_pipeline_settings::domClear_stencil;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domClear_stencil::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "clear_stencil" );
	_Meta->registerClass(domGles_pipeline_settings::domClear_stencil::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Int"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domClear_stencil , attrValue ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domClear_stencil , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domClear_stencil));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domClear_depth::create(daeInt bytes)
{
	domGles_pipeline_settings::domClear_depthRef ref = new(bytes) domGles_pipeline_settings::domClear_depth;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domClear_depth::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "clear_depth" );
	_Meta->registerClass(domGles_pipeline_settings::domClear_depth::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domClear_depth , attrValue ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domClear_depth , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domClear_depth));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domClip_plane::create(daeInt bytes)
{
	domGles_pipeline_settings::domClip_planeRef ref = new(bytes) domGles_pipeline_settings::domClip_plane;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domClip_plane::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "clip_plane" );
	_Meta->registerClass(domGles_pipeline_settings::domClip_plane::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool4"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domClip_plane , attrValue ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domClip_plane , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GLES_MAX_CLIP_PLANES_index"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domClip_plane , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domClip_plane));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domColor_mask::create(daeInt bytes)
{
	domGles_pipeline_settings::domColor_maskRef ref = new(bytes) domGles_pipeline_settings::domColor_mask;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domColor_mask::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "color_mask" );
	_Meta->registerClass(domGles_pipeline_settings::domColor_mask::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool4"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domColor_mask , attrValue ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domColor_mask , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domColor_mask));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domCull_face::create(daeInt bytes)
{
	domGles_pipeline_settings::domCull_faceRef ref = new(bytes) domGles_pipeline_settings::domCull_face;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domCull_face::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "cull_face" );
	_Meta->registerClass(domGles_pipeline_settings::domCull_face::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_face_type"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domCull_face , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "BACK");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domCull_face , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domCull_face));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domDepth_func::create(daeInt bytes)
{
	domGles_pipeline_settings::domDepth_funcRef ref = new(bytes) domGles_pipeline_settings::domDepth_func;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domDepth_func::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "depth_func" );
	_Meta->registerClass(domGles_pipeline_settings::domDepth_func::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_func_type"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domDepth_func , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "ALWAYS");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domDepth_func , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domDepth_func));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domDepth_mask::create(daeInt bytes)
{
	domGles_pipeline_settings::domDepth_maskRef ref = new(bytes) domGles_pipeline_settings::domDepth_mask;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domDepth_mask::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "depth_mask" );
	_Meta->registerClass(domGles_pipeline_settings::domDepth_mask::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domDepth_mask , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domDepth_mask , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domDepth_mask));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domDepth_range::create(daeInt bytes)
{
	domGles_pipeline_settings::domDepth_rangeRef ref = new(bytes) domGles_pipeline_settings::domDepth_range;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domDepth_range::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "depth_range" );
	_Meta->registerClass(domGles_pipeline_settings::domDepth_range::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float2"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domDepth_range , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "0 1");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domDepth_range , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domDepth_range));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domFog_color::create(daeInt bytes)
{
	domGles_pipeline_settings::domFog_colorRef ref = new(bytes) domGles_pipeline_settings::domFog_color;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domFog_color::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fog_color" );
	_Meta->registerClass(domGles_pipeline_settings::domFog_color::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domFog_color , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "0 0 0 0");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domFog_color , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domFog_color));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domFog_density::create(daeInt bytes)
{
	domGles_pipeline_settings::domFog_densityRef ref = new(bytes) domGles_pipeline_settings::domFog_density;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domFog_density::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fog_density" );
	_Meta->registerClass(domGles_pipeline_settings::domFog_density::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domFog_density , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "1");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domFog_density , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domFog_density));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domFog_mode::create(daeInt bytes)
{
	domGles_pipeline_settings::domFog_modeRef ref = new(bytes) domGles_pipeline_settings::domFog_mode;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domFog_mode::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fog_mode" );
	_Meta->registerClass(domGles_pipeline_settings::domFog_mode::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_fog_type"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domFog_mode , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "EXP");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domFog_mode , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domFog_mode));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domFog_start::create(daeInt bytes)
{
	domGles_pipeline_settings::domFog_startRef ref = new(bytes) domGles_pipeline_settings::domFog_start;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domFog_start::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fog_start" );
	_Meta->registerClass(domGles_pipeline_settings::domFog_start::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domFog_start , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "0");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domFog_start , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domFog_start));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domFog_end::create(daeInt bytes)
{
	domGles_pipeline_settings::domFog_endRef ref = new(bytes) domGles_pipeline_settings::domFog_end;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domFog_end::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fog_end" );
	_Meta->registerClass(domGles_pipeline_settings::domFog_end::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domFog_end , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "1");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domFog_end , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domFog_end));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domFront_face::create(daeInt bytes)
{
	domGles_pipeline_settings::domFront_faceRef ref = new(bytes) domGles_pipeline_settings::domFront_face;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domFront_face::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "front_face" );
	_Meta->registerClass(domGles_pipeline_settings::domFront_face::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_front_face_type"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domFront_face , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "CCW");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domFront_face , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domFront_face));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domTexture_pipeline::create(daeInt bytes)
{
	domGles_pipeline_settings::domTexture_pipelineRef ref = new(bytes) domGles_pipeline_settings::domTexture_pipeline;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domTexture_pipeline::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "texture_pipeline" );
	_Meta->registerClass(domGles_pipeline_settings::domTexture_pipeline::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 0, 1 );
	mea->setName( "value" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings::domTexture_pipeline,elemValue) );
	mea->setElementType( domGles_texture_pipeline::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 0 );
	_Meta->setCMRoot( cm );	

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domTexture_pipeline , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domTexture_pipeline));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domLogic_op::create(daeInt bytes)
{
	domGles_pipeline_settings::domLogic_opRef ref = new(bytes) domGles_pipeline_settings::domLogic_op;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domLogic_op::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "logic_op" );
	_Meta->registerClass(domGles_pipeline_settings::domLogic_op::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_logic_op_type"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLogic_op , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "COPY");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLogic_op , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domLogic_op));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domLight_ambient::create(daeInt bytes)
{
	domGles_pipeline_settings::domLight_ambientRef ref = new(bytes) domGles_pipeline_settings::domLight_ambient;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domLight_ambient::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_ambient" );
	_Meta->registerClass(domGles_pipeline_settings::domLight_ambient::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_ambient , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "0 0 0 1");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_ambient , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GLES_MAX_LIGHTS_index"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_ambient , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domLight_ambient));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domLight_diffuse::create(daeInt bytes)
{
	domGles_pipeline_settings::domLight_diffuseRef ref = new(bytes) domGles_pipeline_settings::domLight_diffuse;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domLight_diffuse::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_diffuse" );
	_Meta->registerClass(domGles_pipeline_settings::domLight_diffuse::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_diffuse , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "0 0 0 0");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_diffuse , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GLES_MAX_LIGHTS_index"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_diffuse , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domLight_diffuse));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domLight_specular::create(daeInt bytes)
{
	domGles_pipeline_settings::domLight_specularRef ref = new(bytes) domGles_pipeline_settings::domLight_specular;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domLight_specular::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_specular" );
	_Meta->registerClass(domGles_pipeline_settings::domLight_specular::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_specular , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "0 0 0 0");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_specular , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GLES_MAX_LIGHTS_index"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_specular , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domLight_specular));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domLight_position::create(daeInt bytes)
{
	domGles_pipeline_settings::domLight_positionRef ref = new(bytes) domGles_pipeline_settings::domLight_position;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domLight_position::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_position" );
	_Meta->registerClass(domGles_pipeline_settings::domLight_position::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_position , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "0 0 1 0");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_position , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GLES_MAX_LIGHTS_index"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_position , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domLight_position));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domLight_constant_attenuation::create(daeInt bytes)
{
	domGles_pipeline_settings::domLight_constant_attenuationRef ref = new(bytes) domGles_pipeline_settings::domLight_constant_attenuation;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domLight_constant_attenuation::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_constant_attenuation" );
	_Meta->registerClass(domGles_pipeline_settings::domLight_constant_attenuation::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_constant_attenuation , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "1");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_constant_attenuation , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GLES_MAX_LIGHTS_index"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_constant_attenuation , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domLight_constant_attenuation));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domLight_linear_attenutation::create(daeInt bytes)
{
	domGles_pipeline_settings::domLight_linear_attenutationRef ref = new(bytes) domGles_pipeline_settings::domLight_linear_attenutation;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domLight_linear_attenutation::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_linear_attenutation" );
	_Meta->registerClass(domGles_pipeline_settings::domLight_linear_attenutation::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_linear_attenutation , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "1");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_linear_attenutation , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GLES_MAX_LIGHTS_index"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_linear_attenutation , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domLight_linear_attenutation));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domLight_quadratic_attenuation::create(daeInt bytes)
{
	domGles_pipeline_settings::domLight_quadratic_attenuationRef ref = new(bytes) domGles_pipeline_settings::domLight_quadratic_attenuation;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domLight_quadratic_attenuation::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_quadratic_attenuation" );
	_Meta->registerClass(domGles_pipeline_settings::domLight_quadratic_attenuation::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_quadratic_attenuation , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "1");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_quadratic_attenuation , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GLES_MAX_LIGHTS_index"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_quadratic_attenuation , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domLight_quadratic_attenuation));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domLight_spot_cutoff::create(daeInt bytes)
{
	domGles_pipeline_settings::domLight_spot_cutoffRef ref = new(bytes) domGles_pipeline_settings::domLight_spot_cutoff;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domLight_spot_cutoff::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_spot_cutoff" );
	_Meta->registerClass(domGles_pipeline_settings::domLight_spot_cutoff::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_spot_cutoff , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "180");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_spot_cutoff , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GLES_MAX_LIGHTS_index"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_spot_cutoff , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domLight_spot_cutoff));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domLight_spot_direction::create(daeInt bytes)
{
	domGles_pipeline_settings::domLight_spot_directionRef ref = new(bytes) domGles_pipeline_settings::domLight_spot_direction;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domLight_spot_direction::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_spot_direction" );
	_Meta->registerClass(domGles_pipeline_settings::domLight_spot_direction::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float3"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_spot_direction , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "0 0 -1");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_spot_direction , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GLES_MAX_LIGHTS_index"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_spot_direction , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domLight_spot_direction));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domLight_spot_exponent::create(daeInt bytes)
{
	domGles_pipeline_settings::domLight_spot_exponentRef ref = new(bytes) domGles_pipeline_settings::domLight_spot_exponent;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domLight_spot_exponent::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_spot_exponent" );
	_Meta->registerClass(domGles_pipeline_settings::domLight_spot_exponent::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_spot_exponent , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "0");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_spot_exponent , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GLES_MAX_LIGHTS_index"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_spot_exponent , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domLight_spot_exponent));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domLight_model_ambient::create(daeInt bytes)
{
	domGles_pipeline_settings::domLight_model_ambientRef ref = new(bytes) domGles_pipeline_settings::domLight_model_ambient;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domLight_model_ambient::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_model_ambient" );
	_Meta->registerClass(domGles_pipeline_settings::domLight_model_ambient::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_model_ambient , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "0.2 0.2 0.2 1.0");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_model_ambient , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domLight_model_ambient));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domLine_width::create(daeInt bytes)
{
	domGles_pipeline_settings::domLine_widthRef ref = new(bytes) domGles_pipeline_settings::domLine_width;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domLine_width::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "line_width" );
	_Meta->registerClass(domGles_pipeline_settings::domLine_width::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLine_width , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "1");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLine_width , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domLine_width));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domMaterial_ambient::create(daeInt bytes)
{
	domGles_pipeline_settings::domMaterial_ambientRef ref = new(bytes) domGles_pipeline_settings::domMaterial_ambient;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domMaterial_ambient::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "material_ambient" );
	_Meta->registerClass(domGles_pipeline_settings::domMaterial_ambient::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domMaterial_ambient , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "0.2 0.2 0.2 1.0");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domMaterial_ambient , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domMaterial_ambient));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domMaterial_diffuse::create(daeInt bytes)
{
	domGles_pipeline_settings::domMaterial_diffuseRef ref = new(bytes) domGles_pipeline_settings::domMaterial_diffuse;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domMaterial_diffuse::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "material_diffuse" );
	_Meta->registerClass(domGles_pipeline_settings::domMaterial_diffuse::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domMaterial_diffuse , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "0.8 0.8 0.8 1.0");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domMaterial_diffuse , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domMaterial_diffuse));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domMaterial_emission::create(daeInt bytes)
{
	domGles_pipeline_settings::domMaterial_emissionRef ref = new(bytes) domGles_pipeline_settings::domMaterial_emission;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domMaterial_emission::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "material_emission" );
	_Meta->registerClass(domGles_pipeline_settings::domMaterial_emission::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domMaterial_emission , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "0 0 0 1");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domMaterial_emission , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domMaterial_emission));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domMaterial_shininess::create(daeInt bytes)
{
	domGles_pipeline_settings::domMaterial_shininessRef ref = new(bytes) domGles_pipeline_settings::domMaterial_shininess;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domMaterial_shininess::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "material_shininess" );
	_Meta->registerClass(domGles_pipeline_settings::domMaterial_shininess::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domMaterial_shininess , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "0");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domMaterial_shininess , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domMaterial_shininess));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domMaterial_specular::create(daeInt bytes)
{
	domGles_pipeline_settings::domMaterial_specularRef ref = new(bytes) domGles_pipeline_settings::domMaterial_specular;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domMaterial_specular::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "material_specular" );
	_Meta->registerClass(domGles_pipeline_settings::domMaterial_specular::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domMaterial_specular , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "0 0 0 1");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domMaterial_specular , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domMaterial_specular));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domModel_view_matrix::create(daeInt bytes)
{
	domGles_pipeline_settings::domModel_view_matrixRef ref = new(bytes) domGles_pipeline_settings::domModel_view_matrix;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domModel_view_matrix::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "model_view_matrix" );
	_Meta->registerClass(domGles_pipeline_settings::domModel_view_matrix::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4x4"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domModel_view_matrix , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domModel_view_matrix , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domModel_view_matrix));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domPoint_distance_attenuation::create(daeInt bytes)
{
	domGles_pipeline_settings::domPoint_distance_attenuationRef ref = new(bytes) domGles_pipeline_settings::domPoint_distance_attenuation;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domPoint_distance_attenuation::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "point_distance_attenuation" );
	_Meta->registerClass(domGles_pipeline_settings::domPoint_distance_attenuation::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float3"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domPoint_distance_attenuation , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "1 0 0");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domPoint_distance_attenuation , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domPoint_distance_attenuation));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domPoint_fade_threshold_size::create(daeInt bytes)
{
	domGles_pipeline_settings::domPoint_fade_threshold_sizeRef ref = new(bytes) domGles_pipeline_settings::domPoint_fade_threshold_size;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domPoint_fade_threshold_size::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "point_fade_threshold_size" );
	_Meta->registerClass(domGles_pipeline_settings::domPoint_fade_threshold_size::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domPoint_fade_threshold_size , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "1");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domPoint_fade_threshold_size , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domPoint_fade_threshold_size));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domPoint_size::create(daeInt bytes)
{
	domGles_pipeline_settings::domPoint_sizeRef ref = new(bytes) domGles_pipeline_settings::domPoint_size;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domPoint_size::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "point_size" );
	_Meta->registerClass(domGles_pipeline_settings::domPoint_size::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domPoint_size , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "1");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domPoint_size , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domPoint_size));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domPoint_size_min::create(daeInt bytes)
{
	domGles_pipeline_settings::domPoint_size_minRef ref = new(bytes) domGles_pipeline_settings::domPoint_size_min;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domPoint_size_min::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "point_size_min" );
	_Meta->registerClass(domGles_pipeline_settings::domPoint_size_min::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domPoint_size_min , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "0");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domPoint_size_min , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domPoint_size_min));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domPoint_size_max::create(daeInt bytes)
{
	domGles_pipeline_settings::domPoint_size_maxRef ref = new(bytes) domGles_pipeline_settings::domPoint_size_max;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domPoint_size_max::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "point_size_max" );
	_Meta->registerClass(domGles_pipeline_settings::domPoint_size_max::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domPoint_size_max , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "1");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domPoint_size_max , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domPoint_size_max));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domPolygon_offset::create(daeInt bytes)
{
	domGles_pipeline_settings::domPolygon_offsetRef ref = new(bytes) domGles_pipeline_settings::domPolygon_offset;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domPolygon_offset::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "polygon_offset" );
	_Meta->registerClass(domGles_pipeline_settings::domPolygon_offset::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float2"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domPolygon_offset , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "0 0");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domPolygon_offset , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domPolygon_offset));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domProjection_matrix::create(daeInt bytes)
{
	domGles_pipeline_settings::domProjection_matrixRef ref = new(bytes) domGles_pipeline_settings::domProjection_matrix;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domProjection_matrix::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "projection_matrix" );
	_Meta->registerClass(domGles_pipeline_settings::domProjection_matrix::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4x4"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domProjection_matrix , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domProjection_matrix , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domProjection_matrix));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domScissor::create(daeInt bytes)
{
	domGles_pipeline_settings::domScissorRef ref = new(bytes) domGles_pipeline_settings::domScissor;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domScissor::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "scissor" );
	_Meta->registerClass(domGles_pipeline_settings::domScissor::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Int4"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domScissor , attrValue ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domScissor , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domScissor));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domShade_model::create(daeInt bytes)
{
	domGles_pipeline_settings::domShade_modelRef ref = new(bytes) domGles_pipeline_settings::domShade_model;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domShade_model::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "shade_model" );
	_Meta->registerClass(domGles_pipeline_settings::domShade_model::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_shade_model_type"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domShade_model , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "SMOOTH");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domShade_model , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domShade_model));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domStencil_func::create(daeInt bytes)
{
	domGles_pipeline_settings::domStencil_funcRef ref = new(bytes) domGles_pipeline_settings::domStencil_func;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domStencil_func::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "stencil_func" );
	_Meta->registerClass(domGles_pipeline_settings::domStencil_func::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "func" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings::domStencil_func,elemFunc) );
	mea->setElementType( domGles_pipeline_settings::domStencil_func::domFunc::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 1, 1 );
	mea->setName( "ref" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings::domStencil_func,elemRef) );
	mea->setElementType( domGles_pipeline_settings::domStencil_func::domRef::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 1, 1 );
	mea->setName( "mask" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings::domStencil_func,elemMask) );
	mea->setElementType( domGles_pipeline_settings::domStencil_func::domMask::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 2 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domStencil_func));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domStencil_func::domFunc::create(daeInt bytes)
{
	domGles_pipeline_settings::domStencil_func::domFuncRef ref = new(bytes) domGles_pipeline_settings::domStencil_func::domFunc;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domStencil_func::domFunc::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "func" );
	_Meta->registerClass(domGles_pipeline_settings::domStencil_func::domFunc::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_func_type"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domStencil_func::domFunc , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "ALWAYS");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domStencil_func::domFunc , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domStencil_func::domFunc));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domStencil_func::domRef::create(daeInt bytes)
{
	domGles_pipeline_settings::domStencil_func::domRefRef ref = new(bytes) domGles_pipeline_settings::domStencil_func::domRef;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domStencil_func::domRef::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "ref" );
	_Meta->registerClass(domGles_pipeline_settings::domStencil_func::domRef::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("xsUnsignedByte"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domStencil_func::domRef , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "0");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domStencil_func::domRef , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domStencil_func::domRef));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domStencil_func::domMask::create(daeInt bytes)
{
	domGles_pipeline_settings::domStencil_func::domMaskRef ref = new(bytes) domGles_pipeline_settings::domStencil_func::domMask;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domStencil_func::domMask::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "mask" );
	_Meta->registerClass(domGles_pipeline_settings::domStencil_func::domMask::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("xsUnsignedByte"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domStencil_func::domMask , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "255");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domStencil_func::domMask , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domStencil_func::domMask));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domStencil_mask::create(daeInt bytes)
{
	domGles_pipeline_settings::domStencil_maskRef ref = new(bytes) domGles_pipeline_settings::domStencil_mask;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domStencil_mask::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "stencil_mask" );
	_Meta->registerClass(domGles_pipeline_settings::domStencil_mask::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Int"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domStencil_mask , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "4294967295");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domStencil_mask , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domStencil_mask));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domStencil_op::create(daeInt bytes)
{
	domGles_pipeline_settings::domStencil_opRef ref = new(bytes) domGles_pipeline_settings::domStencil_op;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domStencil_op::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "stencil_op" );
	_Meta->registerClass(domGles_pipeline_settings::domStencil_op::create, &_Meta);

	_Meta->setIsInnerClass( true );
	daeMetaCMPolicy *cm = NULL;
	daeMetaElementAttribute *mea = NULL;
	cm = new daeMetaSequence( _Meta, cm, 0, 1, 1 );

	mea = new daeMetaElementAttribute( _Meta, cm, 0, 1, 1 );
	mea->setName( "fail" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings::domStencil_op,elemFail) );
	mea->setElementType( domGles_pipeline_settings::domStencil_op::domFail::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 1, 1, 1 );
	mea->setName( "zfail" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings::domStencil_op,elemZfail) );
	mea->setElementType( domGles_pipeline_settings::domStencil_op::domZfail::registerElement() );
	cm->appendChild( mea );
	
	mea = new daeMetaElementAttribute( _Meta, cm, 2, 1, 1 );
	mea->setName( "zpass" );
	mea->setOffset( daeOffsetOf(domGles_pipeline_settings::domStencil_op,elemZpass) );
	mea->setElementType( domGles_pipeline_settings::domStencil_op::domZpass::registerElement() );
	cm->appendChild( mea );
	
	cm->setMaxOrdinal( 2 );
	_Meta->setCMRoot( cm );	
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domStencil_op));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domStencil_op::domFail::create(daeInt bytes)
{
	domGles_pipeline_settings::domStencil_op::domFailRef ref = new(bytes) domGles_pipeline_settings::domStencil_op::domFail;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domStencil_op::domFail::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fail" );
	_Meta->registerClass(domGles_pipeline_settings::domStencil_op::domFail::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gles_stencil_op_type"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domStencil_op::domFail , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "KEEP");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domStencil_op::domFail , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domStencil_op::domFail));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domStencil_op::domZfail::create(daeInt bytes)
{
	domGles_pipeline_settings::domStencil_op::domZfailRef ref = new(bytes) domGles_pipeline_settings::domStencil_op::domZfail;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domStencil_op::domZfail::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "zfail" );
	_Meta->registerClass(domGles_pipeline_settings::domStencil_op::domZfail::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gles_stencil_op_type"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domStencil_op::domZfail , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "KEEP");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domStencil_op::domZfail , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domStencil_op::domZfail));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domStencil_op::domZpass::create(daeInt bytes)
{
	domGles_pipeline_settings::domStencil_op::domZpassRef ref = new(bytes) domGles_pipeline_settings::domStencil_op::domZpass;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domStencil_op::domZpass::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "zpass" );
	_Meta->registerClass(domGles_pipeline_settings::domStencil_op::domZpass::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gles_stencil_op_type"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domStencil_op::domZpass , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "KEEP");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domStencil_op::domZpass , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domStencil_op::domZpass));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domAlpha_test_enable::create(daeInt bytes)
{
	domGles_pipeline_settings::domAlpha_test_enableRef ref = new(bytes) domGles_pipeline_settings::domAlpha_test_enable;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domAlpha_test_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "alpha_test_enable" );
	_Meta->registerClass(domGles_pipeline_settings::domAlpha_test_enable::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domAlpha_test_enable , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domAlpha_test_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domAlpha_test_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domBlend_enable::create(daeInt bytes)
{
	domGles_pipeline_settings::domBlend_enableRef ref = new(bytes) domGles_pipeline_settings::domBlend_enable;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domBlend_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "blend_enable" );
	_Meta->registerClass(domGles_pipeline_settings::domBlend_enable::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domBlend_enable , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domBlend_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domBlend_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domClip_plane_enable::create(daeInt bytes)
{
	domGles_pipeline_settings::domClip_plane_enableRef ref = new(bytes) domGles_pipeline_settings::domClip_plane_enable;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domClip_plane_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "clip_plane_enable" );
	_Meta->registerClass(domGles_pipeline_settings::domClip_plane_enable::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domClip_plane_enable , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domClip_plane_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GLES_MAX_CLIP_PLANES_index"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domClip_plane_enable , attrIndex ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domClip_plane_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domColor_logic_op_enable::create(daeInt bytes)
{
	domGles_pipeline_settings::domColor_logic_op_enableRef ref = new(bytes) domGles_pipeline_settings::domColor_logic_op_enable;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domColor_logic_op_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "color_logic_op_enable" );
	_Meta->registerClass(domGles_pipeline_settings::domColor_logic_op_enable::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domColor_logic_op_enable , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domColor_logic_op_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domColor_logic_op_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domColor_material_enable::create(daeInt bytes)
{
	domGles_pipeline_settings::domColor_material_enableRef ref = new(bytes) domGles_pipeline_settings::domColor_material_enable;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domColor_material_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "color_material_enable" );
	_Meta->registerClass(domGles_pipeline_settings::domColor_material_enable::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domColor_material_enable , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "true");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domColor_material_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domColor_material_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domCull_face_enable::create(daeInt bytes)
{
	domGles_pipeline_settings::domCull_face_enableRef ref = new(bytes) domGles_pipeline_settings::domCull_face_enable;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domCull_face_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "cull_face_enable" );
	_Meta->registerClass(domGles_pipeline_settings::domCull_face_enable::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domCull_face_enable , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domCull_face_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domCull_face_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domDepth_test_enable::create(daeInt bytes)
{
	domGles_pipeline_settings::domDepth_test_enableRef ref = new(bytes) domGles_pipeline_settings::domDepth_test_enable;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domDepth_test_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "depth_test_enable" );
	_Meta->registerClass(domGles_pipeline_settings::domDepth_test_enable::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domDepth_test_enable , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domDepth_test_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domDepth_test_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domDither_enable::create(daeInt bytes)
{
	domGles_pipeline_settings::domDither_enableRef ref = new(bytes) domGles_pipeline_settings::domDither_enable;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domDither_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "dither_enable" );
	_Meta->registerClass(domGles_pipeline_settings::domDither_enable::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domDither_enable , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domDither_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domDither_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domFog_enable::create(daeInt bytes)
{
	domGles_pipeline_settings::domFog_enableRef ref = new(bytes) domGles_pipeline_settings::domFog_enable;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domFog_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fog_enable" );
	_Meta->registerClass(domGles_pipeline_settings::domFog_enable::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domFog_enable , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domFog_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domFog_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domTexture_pipeline_enable::create(daeInt bytes)
{
	domGles_pipeline_settings::domTexture_pipeline_enableRef ref = new(bytes) domGles_pipeline_settings::domTexture_pipeline_enable;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domTexture_pipeline_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "texture_pipeline_enable" );
	_Meta->registerClass(domGles_pipeline_settings::domTexture_pipeline_enable::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domTexture_pipeline_enable , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domTexture_pipeline_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domTexture_pipeline_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domLight_enable::create(daeInt bytes)
{
	domGles_pipeline_settings::domLight_enableRef ref = new(bytes) domGles_pipeline_settings::domLight_enable;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domLight_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_enable" );
	_Meta->registerClass(domGles_pipeline_settings::domLight_enable::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_enable , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GLES_MAX_LIGHTS_index"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_enable , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domLight_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domLighting_enable::create(daeInt bytes)
{
	domGles_pipeline_settings::domLighting_enableRef ref = new(bytes) domGles_pipeline_settings::domLighting_enable;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domLighting_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "lighting_enable" );
	_Meta->registerClass(domGles_pipeline_settings::domLighting_enable::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLighting_enable , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLighting_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domLighting_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domLight_model_two_side_enable::create(daeInt bytes)
{
	domGles_pipeline_settings::domLight_model_two_side_enableRef ref = new(bytes) domGles_pipeline_settings::domLight_model_two_side_enable;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domLight_model_two_side_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_model_two_side_enable" );
	_Meta->registerClass(domGles_pipeline_settings::domLight_model_two_side_enable::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_model_two_side_enable , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLight_model_two_side_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domLight_model_two_side_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domLine_smooth_enable::create(daeInt bytes)
{
	domGles_pipeline_settings::domLine_smooth_enableRef ref = new(bytes) domGles_pipeline_settings::domLine_smooth_enable;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domLine_smooth_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "line_smooth_enable" );
	_Meta->registerClass(domGles_pipeline_settings::domLine_smooth_enable::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLine_smooth_enable , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domLine_smooth_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domLine_smooth_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domMultisample_enable::create(daeInt bytes)
{
	domGles_pipeline_settings::domMultisample_enableRef ref = new(bytes) domGles_pipeline_settings::domMultisample_enable;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domMultisample_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "multisample_enable" );
	_Meta->registerClass(domGles_pipeline_settings::domMultisample_enable::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domMultisample_enable , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domMultisample_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domMultisample_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domNormalize_enable::create(daeInt bytes)
{
	domGles_pipeline_settings::domNormalize_enableRef ref = new(bytes) domGles_pipeline_settings::domNormalize_enable;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domNormalize_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "normalize_enable" );
	_Meta->registerClass(domGles_pipeline_settings::domNormalize_enable::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domNormalize_enable , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domNormalize_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domNormalize_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domPoint_smooth_enable::create(daeInt bytes)
{
	domGles_pipeline_settings::domPoint_smooth_enableRef ref = new(bytes) domGles_pipeline_settings::domPoint_smooth_enable;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domPoint_smooth_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "point_smooth_enable" );
	_Meta->registerClass(domGles_pipeline_settings::domPoint_smooth_enable::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domPoint_smooth_enable , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domPoint_smooth_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domPoint_smooth_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domPolygon_offset_fill_enable::create(daeInt bytes)
{
	domGles_pipeline_settings::domPolygon_offset_fill_enableRef ref = new(bytes) domGles_pipeline_settings::domPolygon_offset_fill_enable;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domPolygon_offset_fill_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "polygon_offset_fill_enable" );
	_Meta->registerClass(domGles_pipeline_settings::domPolygon_offset_fill_enable::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domPolygon_offset_fill_enable , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domPolygon_offset_fill_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domPolygon_offset_fill_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domRescale_normal_enable::create(daeInt bytes)
{
	domGles_pipeline_settings::domRescale_normal_enableRef ref = new(bytes) domGles_pipeline_settings::domRescale_normal_enable;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domRescale_normal_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "rescale_normal_enable" );
	_Meta->registerClass(domGles_pipeline_settings::domRescale_normal_enable::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domRescale_normal_enable , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domRescale_normal_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domRescale_normal_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domSample_alpha_to_coverage_enable::create(daeInt bytes)
{
	domGles_pipeline_settings::domSample_alpha_to_coverage_enableRef ref = new(bytes) domGles_pipeline_settings::domSample_alpha_to_coverage_enable;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domSample_alpha_to_coverage_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "sample_alpha_to_coverage_enable" );
	_Meta->registerClass(domGles_pipeline_settings::domSample_alpha_to_coverage_enable::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domSample_alpha_to_coverage_enable , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domSample_alpha_to_coverage_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domSample_alpha_to_coverage_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domSample_alpha_to_one_enable::create(daeInt bytes)
{
	domGles_pipeline_settings::domSample_alpha_to_one_enableRef ref = new(bytes) domGles_pipeline_settings::domSample_alpha_to_one_enable;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domSample_alpha_to_one_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "sample_alpha_to_one_enable" );
	_Meta->registerClass(domGles_pipeline_settings::domSample_alpha_to_one_enable::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domSample_alpha_to_one_enable , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domSample_alpha_to_one_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domSample_alpha_to_one_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domSample_coverage_enable::create(daeInt bytes)
{
	domGles_pipeline_settings::domSample_coverage_enableRef ref = new(bytes) domGles_pipeline_settings::domSample_coverage_enable;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domSample_coverage_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "sample_coverage_enable" );
	_Meta->registerClass(domGles_pipeline_settings::domSample_coverage_enable::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domSample_coverage_enable , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domSample_coverage_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domSample_coverage_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domScissor_test_enable::create(daeInt bytes)
{
	domGles_pipeline_settings::domScissor_test_enableRef ref = new(bytes) domGles_pipeline_settings::domScissor_test_enable;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domScissor_test_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "scissor_test_enable" );
	_Meta->registerClass(domGles_pipeline_settings::domScissor_test_enable::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domScissor_test_enable , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domScissor_test_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domScissor_test_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGles_pipeline_settings::domStencil_test_enable::create(daeInt bytes)
{
	domGles_pipeline_settings::domStencil_test_enableRef ref = new(bytes) domGles_pipeline_settings::domStencil_test_enable;
	return ref;
}


daeMetaElement *
domGles_pipeline_settings::domStencil_test_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "stencil_test_enable" );
	_Meta->registerClass(domGles_pipeline_settings::domStencil_test_enable::create, &_Meta);

	_Meta->setIsInnerClass( true );

	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domStencil_test_enable , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "false");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGles_pipeline_settings::domStencil_test_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGles_pipeline_settings::domStencil_test_enable));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domGles_pipeline_settings::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domAlpha_func::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domAlpha_func::domFunc::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domAlpha_func::domValue::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domBlend_func::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domBlend_func::domSrc::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domBlend_func::domDest::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domClear_color::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domClear_stencil::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domClear_depth::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domClip_plane::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domColor_mask::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domCull_face::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domDepth_func::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domDepth_mask::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domDepth_range::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domFog_color::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domFog_density::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domFog_mode::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domFog_start::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domFog_end::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domFront_face::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domTexture_pipeline::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domLogic_op::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domLight_ambient::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domLight_diffuse::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domLight_specular::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domLight_position::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domLight_constant_attenuation::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domLight_linear_attenutation::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domLight_quadratic_attenuation::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domLight_spot_cutoff::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domLight_spot_direction::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domLight_spot_exponent::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domLight_model_ambient::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domLine_width::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domMaterial_ambient::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domMaterial_diffuse::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domMaterial_emission::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domMaterial_shininess::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domMaterial_specular::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domModel_view_matrix::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domPoint_distance_attenuation::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domPoint_fade_threshold_size::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domPoint_size::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domPoint_size_min::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domPoint_size_max::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domPolygon_offset::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domProjection_matrix::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domScissor::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domShade_model::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domStencil_func::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domStencil_func::domFunc::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domStencil_func::domRef::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domStencil_func::domMask::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domStencil_mask::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domStencil_op::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domStencil_op::domFail::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domStencil_op::domZfail::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domStencil_op::domZpass::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domAlpha_test_enable::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domBlend_enable::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domClip_plane_enable::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domColor_logic_op_enable::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domColor_material_enable::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domCull_face_enable::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domDepth_test_enable::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domDither_enable::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domFog_enable::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domTexture_pipeline_enable::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domLight_enable::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domLighting_enable::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domLight_model_two_side_enable::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domLine_smooth_enable::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domMultisample_enable::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domNormalize_enable::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domPoint_smooth_enable::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domPolygon_offset_fill_enable::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domRescale_normal_enable::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domSample_alpha_to_coverage_enable::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domSample_alpha_to_one_enable::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domSample_coverage_enable::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domScissor_test_enable::_Meta = NULL;
daeMetaElement * domGles_pipeline_settings::domStencil_test_enable::_Meta = NULL;


