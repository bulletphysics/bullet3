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
#include <dom/domGl_pipeline_settings.h>

daeElementRef
domGl_pipeline_settings::create(daeInt bytes)
{
	domGl_pipeline_settingsRef ref = new(bytes) domGl_pipeline_settings;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "gl_pipeline_settings" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::create);

	_Meta->setIsTransparent( true );
	// Add elements: alpha_func, blend_func, blend_func_separate, blend_equation, blend_equation_separate, color_material, cull_face, depth_func, fog_mode, fog_coord_src, front_face, light_model_color_control, logic_op, polygon_mode, shade_model, stencil_func, stencil_op, stencil_func_separate, stencil_op_separate, stencil_mask_separate, light_enable, light_ambient, light_diffuse, light_specular, light_position, light_constant_attenuation, light_linear_attenuation, light_quadratic_attenuation, light_spot_cutoff, light_spot_direction, light_spot_exponent, texture1D, texture2D, texture3D, textureCUBE, textureRECT, textureDEPTH, texture1D_enable, texture2D_enable, texture3D_enable, textureCUBE_enable, textureRECT_enable, textureDEPTH_enable, texture_env_color, texture_env_mode, clip_plane, clip_plane_enable, blend_color, clear_color, clear_stencil, clear_depth, color_mask, depth_bounds, depth_mask, depth_range, fog_density, fog_start, fog_end, fog_color, light_model_ambient, lighting_enable, line_stipple, line_width, material_ambient, material_diffuse, material_emission, material_shininess, material_specular, model_view_matrix, point_distance_attenuation, point_fade_threshold_size, point_size, point_size_min, point_size_max, polygon_offset, projection_matrix, scissor, stencil_mask, alpha_test_enable, auto_normal_enable, blend_enable, color_logic_op_enable, cull_face_enable, depth_bounds_enable, depth_clamp_enable, depth_test_enable, dither_enable, fog_enable, light_model_local_viewer_enable, light_model_two_side_enable, line_smooth_enable, line_stipple_enable, logic_op_enable, multisample_enable, normalize_enable, point_smooth_enable, polygon_offset_fill_enable, polygon_offset_line_enable, polygon_offset_point_enable, polygon_smooth_enable, polygon_stipple_enable, rescale_normal_enable, sample_alpha_to_coverage_enable, sample_alpha_to_one_enable, sample_coverage_enable, scissor_test_enable, stencil_test_enable, gl_hook_abstract
    _Meta->appendElement(domGl_pipeline_settings::domAlpha_func::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemAlpha_func));
    _Meta->appendElement(domGl_pipeline_settings::domBlend_func::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemBlend_func));
    _Meta->appendElement(domGl_pipeline_settings::domBlend_func_separate::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemBlend_func_separate));
    _Meta->appendElement(domGl_pipeline_settings::domBlend_equation::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemBlend_equation));
    _Meta->appendElement(domGl_pipeline_settings::domBlend_equation_separate::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemBlend_equation_separate));
    _Meta->appendElement(domGl_pipeline_settings::domColor_material::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemColor_material));
    _Meta->appendElement(domGl_pipeline_settings::domCull_face::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemCull_face));
    _Meta->appendElement(domGl_pipeline_settings::domDepth_func::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemDepth_func));
    _Meta->appendElement(domGl_pipeline_settings::domFog_mode::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemFog_mode));
    _Meta->appendElement(domGl_pipeline_settings::domFog_coord_src::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemFog_coord_src));
    _Meta->appendElement(domGl_pipeline_settings::domFront_face::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemFront_face));
    _Meta->appendElement(domGl_pipeline_settings::domLight_model_color_control::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemLight_model_color_control));
    _Meta->appendElement(domGl_pipeline_settings::domLogic_op::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemLogic_op));
    _Meta->appendElement(domGl_pipeline_settings::domPolygon_mode::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemPolygon_mode));
    _Meta->appendElement(domGl_pipeline_settings::domShade_model::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemShade_model));
    _Meta->appendElement(domGl_pipeline_settings::domStencil_func::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemStencil_func));
    _Meta->appendElement(domGl_pipeline_settings::domStencil_op::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemStencil_op));
    _Meta->appendElement(domGl_pipeline_settings::domStencil_func_separate::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemStencil_func_separate));
    _Meta->appendElement(domGl_pipeline_settings::domStencil_op_separate::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemStencil_op_separate));
    _Meta->appendElement(domGl_pipeline_settings::domStencil_mask_separate::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemStencil_mask_separate));
    _Meta->appendElement(domGl_pipeline_settings::domLight_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemLight_enable));
    _Meta->appendElement(domGl_pipeline_settings::domLight_ambient::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemLight_ambient));
    _Meta->appendElement(domGl_pipeline_settings::domLight_diffuse::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemLight_diffuse));
    _Meta->appendElement(domGl_pipeline_settings::domLight_specular::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemLight_specular));
    _Meta->appendElement(domGl_pipeline_settings::domLight_position::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemLight_position));
    _Meta->appendElement(domGl_pipeline_settings::domLight_constant_attenuation::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemLight_constant_attenuation));
    _Meta->appendElement(domGl_pipeline_settings::domLight_linear_attenuation::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemLight_linear_attenuation));
    _Meta->appendElement(domGl_pipeline_settings::domLight_quadratic_attenuation::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemLight_quadratic_attenuation));
    _Meta->appendElement(domGl_pipeline_settings::domLight_spot_cutoff::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemLight_spot_cutoff));
    _Meta->appendElement(domGl_pipeline_settings::domLight_spot_direction::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemLight_spot_direction));
    _Meta->appendElement(domGl_pipeline_settings::domLight_spot_exponent::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemLight_spot_exponent));
    _Meta->appendElement(domGl_pipeline_settings::domTexture1D::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemTexture1D));
    _Meta->appendElement(domGl_pipeline_settings::domTexture2D::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemTexture2D));
    _Meta->appendElement(domGl_pipeline_settings::domTexture3D::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemTexture3D));
    _Meta->appendElement(domGl_pipeline_settings::domTextureCUBE::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemTextureCUBE));
    _Meta->appendElement(domGl_pipeline_settings::domTextureRECT::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemTextureRECT));
    _Meta->appendElement(domGl_pipeline_settings::domTextureDEPTH::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemTextureDEPTH));
    _Meta->appendElement(domGl_pipeline_settings::domTexture1D_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemTexture1D_enable));
    _Meta->appendElement(domGl_pipeline_settings::domTexture2D_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemTexture2D_enable));
    _Meta->appendElement(domGl_pipeline_settings::domTexture3D_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemTexture3D_enable));
    _Meta->appendElement(domGl_pipeline_settings::domTextureCUBE_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemTextureCUBE_enable));
    _Meta->appendElement(domGl_pipeline_settings::domTextureRECT_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemTextureRECT_enable));
    _Meta->appendElement(domGl_pipeline_settings::domTextureDEPTH_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemTextureDEPTH_enable));
    _Meta->appendElement(domGl_pipeline_settings::domTexture_env_color::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemTexture_env_color));
    _Meta->appendElement(domGl_pipeline_settings::domTexture_env_mode::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemTexture_env_mode));
    _Meta->appendElement(domGl_pipeline_settings::domClip_plane::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemClip_plane));
    _Meta->appendElement(domGl_pipeline_settings::domClip_plane_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemClip_plane_enable));
    _Meta->appendElement(domGl_pipeline_settings::domBlend_color::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemBlend_color));
    _Meta->appendElement(domGl_pipeline_settings::domClear_color::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemClear_color));
    _Meta->appendElement(domGl_pipeline_settings::domClear_stencil::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemClear_stencil));
    _Meta->appendElement(domGl_pipeline_settings::domClear_depth::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemClear_depth));
    _Meta->appendElement(domGl_pipeline_settings::domColor_mask::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemColor_mask));
    _Meta->appendElement(domGl_pipeline_settings::domDepth_bounds::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemDepth_bounds));
    _Meta->appendElement(domGl_pipeline_settings::domDepth_mask::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemDepth_mask));
    _Meta->appendElement(domGl_pipeline_settings::domDepth_range::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemDepth_range));
    _Meta->appendElement(domGl_pipeline_settings::domFog_density::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemFog_density));
    _Meta->appendElement(domGl_pipeline_settings::domFog_start::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemFog_start));
    _Meta->appendElement(domGl_pipeline_settings::domFog_end::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemFog_end));
    _Meta->appendElement(domGl_pipeline_settings::domFog_color::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemFog_color));
    _Meta->appendElement(domGl_pipeline_settings::domLight_model_ambient::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemLight_model_ambient));
    _Meta->appendElement(domGl_pipeline_settings::domLighting_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemLighting_enable));
    _Meta->appendElement(domGl_pipeline_settings::domLine_stipple::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemLine_stipple));
    _Meta->appendElement(domGl_pipeline_settings::domLine_width::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemLine_width));
    _Meta->appendElement(domGl_pipeline_settings::domMaterial_ambient::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemMaterial_ambient));
    _Meta->appendElement(domGl_pipeline_settings::domMaterial_diffuse::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemMaterial_diffuse));
    _Meta->appendElement(domGl_pipeline_settings::domMaterial_emission::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemMaterial_emission));
    _Meta->appendElement(domGl_pipeline_settings::domMaterial_shininess::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemMaterial_shininess));
    _Meta->appendElement(domGl_pipeline_settings::domMaterial_specular::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemMaterial_specular));
    _Meta->appendElement(domGl_pipeline_settings::domModel_view_matrix::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemModel_view_matrix));
    _Meta->appendElement(domGl_pipeline_settings::domPoint_distance_attenuation::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemPoint_distance_attenuation));
    _Meta->appendElement(domGl_pipeline_settings::domPoint_fade_threshold_size::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemPoint_fade_threshold_size));
    _Meta->appendElement(domGl_pipeline_settings::domPoint_size::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemPoint_size));
    _Meta->appendElement(domGl_pipeline_settings::domPoint_size_min::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemPoint_size_min));
    _Meta->appendElement(domGl_pipeline_settings::domPoint_size_max::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemPoint_size_max));
    _Meta->appendElement(domGl_pipeline_settings::domPolygon_offset::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemPolygon_offset));
    _Meta->appendElement(domGl_pipeline_settings::domProjection_matrix::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemProjection_matrix));
    _Meta->appendElement(domGl_pipeline_settings::domScissor::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemScissor));
    _Meta->appendElement(domGl_pipeline_settings::domStencil_mask::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemStencil_mask));
    _Meta->appendElement(domGl_pipeline_settings::domAlpha_test_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemAlpha_test_enable));
    _Meta->appendElement(domGl_pipeline_settings::domAuto_normal_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemAuto_normal_enable));
    _Meta->appendElement(domGl_pipeline_settings::domBlend_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemBlend_enable));
    _Meta->appendElement(domGl_pipeline_settings::domColor_logic_op_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemColor_logic_op_enable));
    _Meta->appendElement(domGl_pipeline_settings::domCull_face_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemCull_face_enable));
    _Meta->appendElement(domGl_pipeline_settings::domDepth_bounds_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemDepth_bounds_enable));
    _Meta->appendElement(domGl_pipeline_settings::domDepth_clamp_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemDepth_clamp_enable));
    _Meta->appendElement(domGl_pipeline_settings::domDepth_test_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemDepth_test_enable));
    _Meta->appendElement(domGl_pipeline_settings::domDither_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemDither_enable));
    _Meta->appendElement(domGl_pipeline_settings::domFog_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemFog_enable));
    _Meta->appendElement(domGl_pipeline_settings::domLight_model_local_viewer_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemLight_model_local_viewer_enable));
    _Meta->appendElement(domGl_pipeline_settings::domLight_model_two_side_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemLight_model_two_side_enable));
    _Meta->appendElement(domGl_pipeline_settings::domLine_smooth_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemLine_smooth_enable));
    _Meta->appendElement(domGl_pipeline_settings::domLine_stipple_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemLine_stipple_enable));
    _Meta->appendElement(domGl_pipeline_settings::domLogic_op_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemLogic_op_enable));
    _Meta->appendElement(domGl_pipeline_settings::domMultisample_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemMultisample_enable));
    _Meta->appendElement(domGl_pipeline_settings::domNormalize_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemNormalize_enable));
    _Meta->appendElement(domGl_pipeline_settings::domPoint_smooth_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemPoint_smooth_enable));
    _Meta->appendElement(domGl_pipeline_settings::domPolygon_offset_fill_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemPolygon_offset_fill_enable));
    _Meta->appendElement(domGl_pipeline_settings::domPolygon_offset_line_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemPolygon_offset_line_enable));
    _Meta->appendElement(domGl_pipeline_settings::domPolygon_offset_point_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemPolygon_offset_point_enable));
    _Meta->appendElement(domGl_pipeline_settings::domPolygon_smooth_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemPolygon_smooth_enable));
    _Meta->appendElement(domGl_pipeline_settings::domPolygon_stipple_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemPolygon_stipple_enable));
    _Meta->appendElement(domGl_pipeline_settings::domRescale_normal_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemRescale_normal_enable));
    _Meta->appendElement(domGl_pipeline_settings::domSample_alpha_to_coverage_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemSample_alpha_to_coverage_enable));
    _Meta->appendElement(domGl_pipeline_settings::domSample_alpha_to_one_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemSample_alpha_to_one_enable));
    _Meta->appendElement(domGl_pipeline_settings::domSample_coverage_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemSample_coverage_enable));
    _Meta->appendElement(domGl_pipeline_settings::domScissor_test_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemScissor_test_enable));
    _Meta->appendElement(domGl_pipeline_settings::domStencil_test_enable::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemStencil_test_enable));
    _Meta->appendElement(domGl_hook_abstract::registerElement(),daeOffsetOf(domGl_pipeline_settings,elemGl_hook_abstract));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domGl_pipeline_settings,_contents));

	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domAlpha_func::create(daeInt bytes)
{
	domGl_pipeline_settings::domAlpha_funcRef ref = new(bytes) domGl_pipeline_settings::domAlpha_func;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domAlpha_func::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "alpha_func" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domAlpha_func::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domAlpha_func::create);

	// Add elements: func, value
    _Meta->appendElement(domGl_pipeline_settings::domAlpha_func::domFunc::registerElement(),daeOffsetOf(domGl_pipeline_settings::domAlpha_func,elemFunc));
    _Meta->appendElement(domGl_pipeline_settings::domAlpha_func::domValue::registerElement(),daeOffsetOf(domGl_pipeline_settings::domAlpha_func,elemValue));
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domAlpha_func));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domAlpha_func::domFunc::create(daeInt bytes)
{
	domGl_pipeline_settings::domAlpha_func::domFuncRef ref = new(bytes) domGl_pipeline_settings::domAlpha_func::domFunc;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domAlpha_func::domFunc::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "func" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domAlpha_func::domFunc::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domAlpha_func::domFunc::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_func_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domAlpha_func::domFunc , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domAlpha_func::domFunc , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domAlpha_func::domFunc));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domAlpha_func::domValue::create(daeInt bytes)
{
	domGl_pipeline_settings::domAlpha_func::domValueRef ref = new(bytes) domGl_pipeline_settings::domAlpha_func::domValue;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domAlpha_func::domValue::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "value" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domAlpha_func::domValue::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domAlpha_func::domValue::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_alpha_value_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domAlpha_func::domValue , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domAlpha_func::domValue , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domAlpha_func::domValue));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domBlend_func::create(daeInt bytes)
{
	domGl_pipeline_settings::domBlend_funcRef ref = new(bytes) domGl_pipeline_settings::domBlend_func;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domBlend_func::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "blend_func" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domBlend_func::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domBlend_func::create);

	// Add elements: src, dest
    _Meta->appendElement(domGl_pipeline_settings::domBlend_func::domSrc::registerElement(),daeOffsetOf(domGl_pipeline_settings::domBlend_func,elemSrc));
    _Meta->appendElement(domGl_pipeline_settings::domBlend_func::domDest::registerElement(),daeOffsetOf(domGl_pipeline_settings::domBlend_func,elemDest));
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domBlend_func));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domBlend_func::domSrc::create(daeInt bytes)
{
	domGl_pipeline_settings::domBlend_func::domSrcRef ref = new(bytes) domGl_pipeline_settings::domBlend_func::domSrc;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domBlend_func::domSrc::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "src" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domBlend_func::domSrc::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domBlend_func::domSrc::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_blend_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domBlend_func::domSrc , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domBlend_func::domSrc , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domBlend_func::domSrc));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domBlend_func::domDest::create(daeInt bytes)
{
	domGl_pipeline_settings::domBlend_func::domDestRef ref = new(bytes) domGl_pipeline_settings::domBlend_func::domDest;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domBlend_func::domDest::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "dest" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domBlend_func::domDest::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domBlend_func::domDest::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_blend_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domBlend_func::domDest , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domBlend_func::domDest , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domBlend_func::domDest));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domBlend_func_separate::create(daeInt bytes)
{
	domGl_pipeline_settings::domBlend_func_separateRef ref = new(bytes) domGl_pipeline_settings::domBlend_func_separate;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domBlend_func_separate::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "blend_func_separate" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domBlend_func_separate::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domBlend_func_separate::create);

	// Add elements: src_rgb, dest_rgb, src_alpha, dest_alpha
    _Meta->appendElement(domGl_pipeline_settings::domBlend_func_separate::domSrc_rgb::registerElement(),daeOffsetOf(domGl_pipeline_settings::domBlend_func_separate,elemSrc_rgb));
    _Meta->appendElement(domGl_pipeline_settings::domBlend_func_separate::domDest_rgb::registerElement(),daeOffsetOf(domGl_pipeline_settings::domBlend_func_separate,elemDest_rgb));
    _Meta->appendElement(domGl_pipeline_settings::domBlend_func_separate::domSrc_alpha::registerElement(),daeOffsetOf(domGl_pipeline_settings::domBlend_func_separate,elemSrc_alpha));
    _Meta->appendElement(domGl_pipeline_settings::domBlend_func_separate::domDest_alpha::registerElement(),daeOffsetOf(domGl_pipeline_settings::domBlend_func_separate,elemDest_alpha));
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domBlend_func_separate));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domBlend_func_separate::domSrc_rgb::create(daeInt bytes)
{
	domGl_pipeline_settings::domBlend_func_separate::domSrc_rgbRef ref = new(bytes) domGl_pipeline_settings::domBlend_func_separate::domSrc_rgb;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domBlend_func_separate::domSrc_rgb::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "src_rgb" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domBlend_func_separate::domSrc_rgb::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domBlend_func_separate::domSrc_rgb::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_blend_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domBlend_func_separate::domSrc_rgb , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domBlend_func_separate::domSrc_rgb , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domBlend_func_separate::domSrc_rgb));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domBlend_func_separate::domDest_rgb::create(daeInt bytes)
{
	domGl_pipeline_settings::domBlend_func_separate::domDest_rgbRef ref = new(bytes) domGl_pipeline_settings::domBlend_func_separate::domDest_rgb;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domBlend_func_separate::domDest_rgb::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "dest_rgb" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domBlend_func_separate::domDest_rgb::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domBlend_func_separate::domDest_rgb::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_blend_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domBlend_func_separate::domDest_rgb , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domBlend_func_separate::domDest_rgb , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domBlend_func_separate::domDest_rgb));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domBlend_func_separate::domSrc_alpha::create(daeInt bytes)
{
	domGl_pipeline_settings::domBlend_func_separate::domSrc_alphaRef ref = new(bytes) domGl_pipeline_settings::domBlend_func_separate::domSrc_alpha;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domBlend_func_separate::domSrc_alpha::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "src_alpha" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domBlend_func_separate::domSrc_alpha::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domBlend_func_separate::domSrc_alpha::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_blend_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domBlend_func_separate::domSrc_alpha , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domBlend_func_separate::domSrc_alpha , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domBlend_func_separate::domSrc_alpha));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domBlend_func_separate::domDest_alpha::create(daeInt bytes)
{
	domGl_pipeline_settings::domBlend_func_separate::domDest_alphaRef ref = new(bytes) domGl_pipeline_settings::domBlend_func_separate::domDest_alpha;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domBlend_func_separate::domDest_alpha::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "dest_alpha" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domBlend_func_separate::domDest_alpha::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domBlend_func_separate::domDest_alpha::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_blend_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domBlend_func_separate::domDest_alpha , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domBlend_func_separate::domDest_alpha , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domBlend_func_separate::domDest_alpha));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domBlend_equation::create(daeInt bytes)
{
	domGl_pipeline_settings::domBlend_equationRef ref = new(bytes) domGl_pipeline_settings::domBlend_equation;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domBlend_equation::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "blend_equation" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domBlend_equation::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domBlend_equation::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_blend_equation_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domBlend_equation , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "FUNC_ADD");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domBlend_equation , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domBlend_equation));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domBlend_equation_separate::create(daeInt bytes)
{
	domGl_pipeline_settings::domBlend_equation_separateRef ref = new(bytes) domGl_pipeline_settings::domBlend_equation_separate;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domBlend_equation_separate::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "blend_equation_separate" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domBlend_equation_separate::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domBlend_equation_separate::create);

	// Add elements: rgb, alpha
    _Meta->appendElement(domGl_pipeline_settings::domBlend_equation_separate::domRgb::registerElement(),daeOffsetOf(domGl_pipeline_settings::domBlend_equation_separate,elemRgb));
    _Meta->appendElement(domGl_pipeline_settings::domBlend_equation_separate::domAlpha::registerElement(),daeOffsetOf(domGl_pipeline_settings::domBlend_equation_separate,elemAlpha));
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domBlend_equation_separate));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domBlend_equation_separate::domRgb::create(daeInt bytes)
{
	domGl_pipeline_settings::domBlend_equation_separate::domRgbRef ref = new(bytes) domGl_pipeline_settings::domBlend_equation_separate::domRgb;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domBlend_equation_separate::domRgb::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "rgb" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domBlend_equation_separate::domRgb::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domBlend_equation_separate::domRgb::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_blend_equation_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domBlend_equation_separate::domRgb , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "FUNC_ADD");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domBlend_equation_separate::domRgb , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domBlend_equation_separate::domRgb));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domBlend_equation_separate::domAlpha::create(daeInt bytes)
{
	domGl_pipeline_settings::domBlend_equation_separate::domAlphaRef ref = new(bytes) domGl_pipeline_settings::domBlend_equation_separate::domAlpha;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domBlend_equation_separate::domAlpha::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "alpha" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domBlend_equation_separate::domAlpha::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domBlend_equation_separate::domAlpha::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_blend_equation_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domBlend_equation_separate::domAlpha , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "FUNC_ADD");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domBlend_equation_separate::domAlpha , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domBlend_equation_separate::domAlpha));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domColor_material::create(daeInt bytes)
{
	domGl_pipeline_settings::domColor_materialRef ref = new(bytes) domGl_pipeline_settings::domColor_material;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domColor_material::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "color_material" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domColor_material::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domColor_material::create);

	// Add elements: face, mode
    _Meta->appendElement(domGl_pipeline_settings::domColor_material::domFace::registerElement(),daeOffsetOf(domGl_pipeline_settings::domColor_material,elemFace));
    _Meta->appendElement(domGl_pipeline_settings::domColor_material::domMode::registerElement(),daeOffsetOf(domGl_pipeline_settings::domColor_material,elemMode));
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domColor_material));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domColor_material::domFace::create(daeInt bytes)
{
	domGl_pipeline_settings::domColor_material::domFaceRef ref = new(bytes) domGl_pipeline_settings::domColor_material::domFace;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domColor_material::domFace::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "face" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domColor_material::domFace::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domColor_material::domFace::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_face_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domColor_material::domFace , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "FRONT_AND_BACK");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domColor_material::domFace , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domColor_material::domFace));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domColor_material::domMode::create(daeInt bytes)
{
	domGl_pipeline_settings::domColor_material::domModeRef ref = new(bytes) domGl_pipeline_settings::domColor_material::domMode;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domColor_material::domMode::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "mode" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domColor_material::domMode::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domColor_material::domMode::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_material_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domColor_material::domMode , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "AMBIENT_AND_DIFFUSE");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domColor_material::domMode , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domColor_material::domMode));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domCull_face::create(daeInt bytes)
{
	domGl_pipeline_settings::domCull_faceRef ref = new(bytes) domGl_pipeline_settings::domCull_face;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domCull_face::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "cull_face" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domCull_face::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domCull_face::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_face_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domCull_face , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domCull_face , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domCull_face));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domDepth_func::create(daeInt bytes)
{
	domGl_pipeline_settings::domDepth_funcRef ref = new(bytes) domGl_pipeline_settings::domDepth_func;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domDepth_func::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "depth_func" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domDepth_func::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domDepth_func::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_func_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domDepth_func , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domDepth_func , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domDepth_func));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domFog_mode::create(daeInt bytes)
{
	domGl_pipeline_settings::domFog_modeRef ref = new(bytes) domGl_pipeline_settings::domFog_mode;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domFog_mode::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fog_mode" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domFog_mode::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domFog_mode::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_fog_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domFog_mode , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domFog_mode , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domFog_mode));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domFog_coord_src::create(daeInt bytes)
{
	domGl_pipeline_settings::domFog_coord_srcRef ref = new(bytes) domGl_pipeline_settings::domFog_coord_src;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domFog_coord_src::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fog_coord_src" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domFog_coord_src::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domFog_coord_src::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_fog_coord_src_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domFog_coord_src , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "FOG_COORDINATE");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domFog_coord_src , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domFog_coord_src));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domFront_face::create(daeInt bytes)
{
	domGl_pipeline_settings::domFront_faceRef ref = new(bytes) domGl_pipeline_settings::domFront_face;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domFront_face::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "front_face" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domFront_face::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domFront_face::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_front_face_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domFront_face , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domFront_face , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domFront_face));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domLight_model_color_control::create(daeInt bytes)
{
	domGl_pipeline_settings::domLight_model_color_controlRef ref = new(bytes) domGl_pipeline_settings::domLight_model_color_control;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domLight_model_color_control::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_model_color_control" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domLight_model_color_control::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domLight_model_color_control::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_light_model_color_control_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_model_color_control , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "SINGLE_COLOR");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_model_color_control , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domLight_model_color_control));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domLogic_op::create(daeInt bytes)
{
	domGl_pipeline_settings::domLogic_opRef ref = new(bytes) domGl_pipeline_settings::domLogic_op;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domLogic_op::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "logic_op" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domLogic_op::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domLogic_op::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_logic_op_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLogic_op , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLogic_op , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domLogic_op));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domPolygon_mode::create(daeInt bytes)
{
	domGl_pipeline_settings::domPolygon_modeRef ref = new(bytes) domGl_pipeline_settings::domPolygon_mode;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domPolygon_mode::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "polygon_mode" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domPolygon_mode::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domPolygon_mode::create);

	// Add elements: face, mode
    _Meta->appendElement(domGl_pipeline_settings::domPolygon_mode::domFace::registerElement(),daeOffsetOf(domGl_pipeline_settings::domPolygon_mode,elemFace));
    _Meta->appendElement(domGl_pipeline_settings::domPolygon_mode::domMode::registerElement(),daeOffsetOf(domGl_pipeline_settings::domPolygon_mode,elemMode));
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domPolygon_mode));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domPolygon_mode::domFace::create(daeInt bytes)
{
	domGl_pipeline_settings::domPolygon_mode::domFaceRef ref = new(bytes) domGl_pipeline_settings::domPolygon_mode::domFace;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domPolygon_mode::domFace::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "face" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domPolygon_mode::domFace::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domPolygon_mode::domFace::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_face_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPolygon_mode::domFace , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "FRONT_AND_BACK");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPolygon_mode::domFace , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domPolygon_mode::domFace));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domPolygon_mode::domMode::create(daeInt bytes)
{
	domGl_pipeline_settings::domPolygon_mode::domModeRef ref = new(bytes) domGl_pipeline_settings::domPolygon_mode::domMode;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domPolygon_mode::domMode::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "mode" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domPolygon_mode::domMode::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domPolygon_mode::domMode::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_polygon_mode_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPolygon_mode::domMode , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "FILL");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPolygon_mode::domMode , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domPolygon_mode::domMode));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domShade_model::create(daeInt bytes)
{
	domGl_pipeline_settings::domShade_modelRef ref = new(bytes) domGl_pipeline_settings::domShade_model;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domShade_model::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "shade_model" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domShade_model::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domShade_model::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_shade_model_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domShade_model , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domShade_model , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domShade_model));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domStencil_func::create(daeInt bytes)
{
	domGl_pipeline_settings::domStencil_funcRef ref = new(bytes) domGl_pipeline_settings::domStencil_func;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domStencil_func::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "stencil_func" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domStencil_func::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domStencil_func::create);

	// Add elements: func, ref, mask
    _Meta->appendElement(domGl_pipeline_settings::domStencil_func::domFunc::registerElement(),daeOffsetOf(domGl_pipeline_settings::domStencil_func,elemFunc));
    _Meta->appendElement(domGl_pipeline_settings::domStencil_func::domRef::registerElement(),daeOffsetOf(domGl_pipeline_settings::domStencil_func,elemRef));
    _Meta->appendElement(domGl_pipeline_settings::domStencil_func::domMask::registerElement(),daeOffsetOf(domGl_pipeline_settings::domStencil_func,elemMask));
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domStencil_func));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domStencil_func::domFunc::create(daeInt bytes)
{
	domGl_pipeline_settings::domStencil_func::domFuncRef ref = new(bytes) domGl_pipeline_settings::domStencil_func::domFunc;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domStencil_func::domFunc::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "func" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domStencil_func::domFunc::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domStencil_func::domFunc::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_func_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_func::domFunc , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_func::domFunc , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domStencil_func::domFunc));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domStencil_func::domRef::create(daeInt bytes)
{
	domGl_pipeline_settings::domStencil_func::domRefRef ref = new(bytes) domGl_pipeline_settings::domStencil_func::domRef;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domStencil_func::domRef::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "ref" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domStencil_func::domRef::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domStencil_func::domRef::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("xsUnsignedByte"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_func::domRef , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_func::domRef , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domStencil_func::domRef));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domStencil_func::domMask::create(daeInt bytes)
{
	domGl_pipeline_settings::domStencil_func::domMaskRef ref = new(bytes) domGl_pipeline_settings::domStencil_func::domMask;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domStencil_func::domMask::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "mask" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domStencil_func::domMask::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domStencil_func::domMask::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("xsUnsignedByte"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_func::domMask , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_func::domMask , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domStencil_func::domMask));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domStencil_op::create(daeInt bytes)
{
	domGl_pipeline_settings::domStencil_opRef ref = new(bytes) domGl_pipeline_settings::domStencil_op;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domStencil_op::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "stencil_op" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domStencil_op::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domStencil_op::create);

	// Add elements: fail, zfail, zpass
    _Meta->appendElement(domGl_pipeline_settings::domStencil_op::domFail::registerElement(),daeOffsetOf(domGl_pipeline_settings::domStencil_op,elemFail));
    _Meta->appendElement(domGl_pipeline_settings::domStencil_op::domZfail::registerElement(),daeOffsetOf(domGl_pipeline_settings::domStencil_op,elemZfail));
    _Meta->appendElement(domGl_pipeline_settings::domStencil_op::domZpass::registerElement(),daeOffsetOf(domGl_pipeline_settings::domStencil_op,elemZpass));
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domStencil_op));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domStencil_op::domFail::create(daeInt bytes)
{
	domGl_pipeline_settings::domStencil_op::domFailRef ref = new(bytes) domGl_pipeline_settings::domStencil_op::domFail;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domStencil_op::domFail::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fail" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domStencil_op::domFail::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domStencil_op::domFail::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_stencil_op_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_op::domFail , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_op::domFail , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domStencil_op::domFail));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domStencil_op::domZfail::create(daeInt bytes)
{
	domGl_pipeline_settings::domStencil_op::domZfailRef ref = new(bytes) domGl_pipeline_settings::domStencil_op::domZfail;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domStencil_op::domZfail::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "zfail" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domStencil_op::domZfail::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domStencil_op::domZfail::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_stencil_op_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_op::domZfail , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_op::domZfail , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domStencil_op::domZfail));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domStencil_op::domZpass::create(daeInt bytes)
{
	domGl_pipeline_settings::domStencil_op::domZpassRef ref = new(bytes) domGl_pipeline_settings::domStencil_op::domZpass;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domStencil_op::domZpass::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "zpass" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domStencil_op::domZpass::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domStencil_op::domZpass::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_stencil_op_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_op::domZpass , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_op::domZpass , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domStencil_op::domZpass));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domStencil_func_separate::create(daeInt bytes)
{
	domGl_pipeline_settings::domStencil_func_separateRef ref = new(bytes) domGl_pipeline_settings::domStencil_func_separate;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domStencil_func_separate::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "stencil_func_separate" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domStencil_func_separate::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domStencil_func_separate::create);

	// Add elements: front, back, ref, mask
    _Meta->appendElement(domGl_pipeline_settings::domStencil_func_separate::domFront::registerElement(),daeOffsetOf(domGl_pipeline_settings::domStencil_func_separate,elemFront));
    _Meta->appendElement(domGl_pipeline_settings::domStencil_func_separate::domBack::registerElement(),daeOffsetOf(domGl_pipeline_settings::domStencil_func_separate,elemBack));
    _Meta->appendElement(domGl_pipeline_settings::domStencil_func_separate::domRef::registerElement(),daeOffsetOf(domGl_pipeline_settings::domStencil_func_separate,elemRef));
    _Meta->appendElement(domGl_pipeline_settings::domStencil_func_separate::domMask::registerElement(),daeOffsetOf(domGl_pipeline_settings::domStencil_func_separate,elemMask));
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domStencil_func_separate));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domStencil_func_separate::domFront::create(daeInt bytes)
{
	domGl_pipeline_settings::domStencil_func_separate::domFrontRef ref = new(bytes) domGl_pipeline_settings::domStencil_func_separate::domFront;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domStencil_func_separate::domFront::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "front" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domStencil_func_separate::domFront::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domStencil_func_separate::domFront::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_func_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_func_separate::domFront , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_func_separate::domFront , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domStencil_func_separate::domFront));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domStencil_func_separate::domBack::create(daeInt bytes)
{
	domGl_pipeline_settings::domStencil_func_separate::domBackRef ref = new(bytes) domGl_pipeline_settings::domStencil_func_separate::domBack;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domStencil_func_separate::domBack::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "back" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domStencil_func_separate::domBack::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domStencil_func_separate::domBack::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_func_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_func_separate::domBack , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_func_separate::domBack , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domStencil_func_separate::domBack));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domStencil_func_separate::domRef::create(daeInt bytes)
{
	domGl_pipeline_settings::domStencil_func_separate::domRefRef ref = new(bytes) domGl_pipeline_settings::domStencil_func_separate::domRef;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domStencil_func_separate::domRef::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "ref" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domStencil_func_separate::domRef::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domStencil_func_separate::domRef::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("xsUnsignedByte"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_func_separate::domRef , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_func_separate::domRef , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domStencil_func_separate::domRef));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domStencil_func_separate::domMask::create(daeInt bytes)
{
	domGl_pipeline_settings::domStencil_func_separate::domMaskRef ref = new(bytes) domGl_pipeline_settings::domStencil_func_separate::domMask;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domStencil_func_separate::domMask::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "mask" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domStencil_func_separate::domMask::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domStencil_func_separate::domMask::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("xsUnsignedByte"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_func_separate::domMask , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_func_separate::domMask , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domStencil_func_separate::domMask));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domStencil_op_separate::create(daeInt bytes)
{
	domGl_pipeline_settings::domStencil_op_separateRef ref = new(bytes) domGl_pipeline_settings::domStencil_op_separate;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domStencil_op_separate::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "stencil_op_separate" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domStencil_op_separate::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domStencil_op_separate::create);

	// Add elements: face, fail, zfail, zpass
    _Meta->appendElement(domGl_pipeline_settings::domStencil_op_separate::domFace::registerElement(),daeOffsetOf(domGl_pipeline_settings::domStencil_op_separate,elemFace));
    _Meta->appendElement(domGl_pipeline_settings::domStencil_op_separate::domFail::registerElement(),daeOffsetOf(domGl_pipeline_settings::domStencil_op_separate,elemFail));
    _Meta->appendElement(domGl_pipeline_settings::domStencil_op_separate::domZfail::registerElement(),daeOffsetOf(domGl_pipeline_settings::domStencil_op_separate,elemZfail));
    _Meta->appendElement(domGl_pipeline_settings::domStencil_op_separate::domZpass::registerElement(),daeOffsetOf(domGl_pipeline_settings::domStencil_op_separate,elemZpass));
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domStencil_op_separate));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domStencil_op_separate::domFace::create(daeInt bytes)
{
	domGl_pipeline_settings::domStencil_op_separate::domFaceRef ref = new(bytes) domGl_pipeline_settings::domStencil_op_separate::domFace;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domStencil_op_separate::domFace::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "face" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domStencil_op_separate::domFace::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domStencil_op_separate::domFace::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_face_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_op_separate::domFace , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "FRONT_AND_BACK");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_op_separate::domFace , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domStencil_op_separate::domFace));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domStencil_op_separate::domFail::create(daeInt bytes)
{
	domGl_pipeline_settings::domStencil_op_separate::domFailRef ref = new(bytes) domGl_pipeline_settings::domStencil_op_separate::domFail;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domStencil_op_separate::domFail::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fail" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domStencil_op_separate::domFail::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domStencil_op_separate::domFail::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_stencil_op_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_op_separate::domFail , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_op_separate::domFail , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domStencil_op_separate::domFail));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domStencil_op_separate::domZfail::create(daeInt bytes)
{
	domGl_pipeline_settings::domStencil_op_separate::domZfailRef ref = new(bytes) domGl_pipeline_settings::domStencil_op_separate::domZfail;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domStencil_op_separate::domZfail::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "zfail" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domStencil_op_separate::domZfail::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domStencil_op_separate::domZfail::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_stencil_op_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_op_separate::domZfail , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_op_separate::domZfail , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domStencil_op_separate::domZfail));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domStencil_op_separate::domZpass::create(daeInt bytes)
{
	domGl_pipeline_settings::domStencil_op_separate::domZpassRef ref = new(bytes) domGl_pipeline_settings::domStencil_op_separate::domZpass;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domStencil_op_separate::domZpass::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "zpass" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domStencil_op_separate::domZpass::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domStencil_op_separate::domZpass::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_stencil_op_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_op_separate::domZpass , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_op_separate::domZpass , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domStencil_op_separate::domZpass));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domStencil_mask_separate::create(daeInt bytes)
{
	domGl_pipeline_settings::domStencil_mask_separateRef ref = new(bytes) domGl_pipeline_settings::domStencil_mask_separate;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domStencil_mask_separate::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "stencil_mask_separate" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domStencil_mask_separate::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domStencil_mask_separate::create);

	// Add elements: face, mask
    _Meta->appendElement(domGl_pipeline_settings::domStencil_mask_separate::domFace::registerElement(),daeOffsetOf(domGl_pipeline_settings::domStencil_mask_separate,elemFace));
    _Meta->appendElement(domGl_pipeline_settings::domStencil_mask_separate::domMask::registerElement(),daeOffsetOf(domGl_pipeline_settings::domStencil_mask_separate,elemMask));
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domStencil_mask_separate));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domStencil_mask_separate::domFace::create(daeInt bytes)
{
	domGl_pipeline_settings::domStencil_mask_separate::domFaceRef ref = new(bytes) domGl_pipeline_settings::domStencil_mask_separate::domFace;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domStencil_mask_separate::domFace::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "face" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domStencil_mask_separate::domFace::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domStencil_mask_separate::domFace::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Gl_face_type"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_mask_separate::domFace , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "FRONT_AND_BACK");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_mask_separate::domFace , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domStencil_mask_separate::domFace));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domStencil_mask_separate::domMask::create(daeInt bytes)
{
	domGl_pipeline_settings::domStencil_mask_separate::domMaskRef ref = new(bytes) domGl_pipeline_settings::domStencil_mask_separate::domMask;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domStencil_mask_separate::domMask::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "mask" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domStencil_mask_separate::domMask::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domStencil_mask_separate::domMask::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("xsUnsignedByte"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_mask_separate::domMask , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_mask_separate::domMask , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domStencil_mask_separate::domMask));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domLight_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domLight_enableRef ref = new(bytes) domGl_pipeline_settings::domLight_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domLight_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domLight_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domLight_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_LIGHTS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_enable , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domLight_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domLight_ambient::create(daeInt bytes)
{
	domGl_pipeline_settings::domLight_ambientRef ref = new(bytes) domGl_pipeline_settings::domLight_ambient;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domLight_ambient::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_ambient" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domLight_ambient::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domLight_ambient::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_ambient , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_ambient , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_LIGHTS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_ambient , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domLight_ambient));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domLight_diffuse::create(daeInt bytes)
{
	domGl_pipeline_settings::domLight_diffuseRef ref = new(bytes) domGl_pipeline_settings::domLight_diffuse;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domLight_diffuse::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_diffuse" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domLight_diffuse::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domLight_diffuse::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_diffuse , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_diffuse , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_LIGHTS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_diffuse , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domLight_diffuse));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domLight_specular::create(daeInt bytes)
{
	domGl_pipeline_settings::domLight_specularRef ref = new(bytes) domGl_pipeline_settings::domLight_specular;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domLight_specular::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_specular" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domLight_specular::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domLight_specular::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_specular , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_specular , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_LIGHTS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_specular , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domLight_specular));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domLight_position::create(daeInt bytes)
{
	domGl_pipeline_settings::domLight_positionRef ref = new(bytes) domGl_pipeline_settings::domLight_position;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domLight_position::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_position" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domLight_position::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domLight_position::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_position , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_position , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_LIGHTS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_position , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domLight_position));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domLight_constant_attenuation::create(daeInt bytes)
{
	domGl_pipeline_settings::domLight_constant_attenuationRef ref = new(bytes) domGl_pipeline_settings::domLight_constant_attenuation;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domLight_constant_attenuation::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_constant_attenuation" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domLight_constant_attenuation::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domLight_constant_attenuation::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_constant_attenuation , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_constant_attenuation , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_LIGHTS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_constant_attenuation , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domLight_constant_attenuation));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domLight_linear_attenuation::create(daeInt bytes)
{
	domGl_pipeline_settings::domLight_linear_attenuationRef ref = new(bytes) domGl_pipeline_settings::domLight_linear_attenuation;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domLight_linear_attenuation::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_linear_attenuation" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domLight_linear_attenuation::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domLight_linear_attenuation::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_linear_attenuation , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_linear_attenuation , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_LIGHTS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_linear_attenuation , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domLight_linear_attenuation));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domLight_quadratic_attenuation::create(daeInt bytes)
{
	domGl_pipeline_settings::domLight_quadratic_attenuationRef ref = new(bytes) domGl_pipeline_settings::domLight_quadratic_attenuation;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domLight_quadratic_attenuation::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_quadratic_attenuation" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domLight_quadratic_attenuation::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domLight_quadratic_attenuation::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_quadratic_attenuation , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_quadratic_attenuation , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_LIGHTS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_quadratic_attenuation , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domLight_quadratic_attenuation));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domLight_spot_cutoff::create(daeInt bytes)
{
	domGl_pipeline_settings::domLight_spot_cutoffRef ref = new(bytes) domGl_pipeline_settings::domLight_spot_cutoff;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domLight_spot_cutoff::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_spot_cutoff" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domLight_spot_cutoff::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domLight_spot_cutoff::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_spot_cutoff , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_spot_cutoff , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_LIGHTS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_spot_cutoff , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domLight_spot_cutoff));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domLight_spot_direction::create(daeInt bytes)
{
	domGl_pipeline_settings::domLight_spot_directionRef ref = new(bytes) domGl_pipeline_settings::domLight_spot_direction;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domLight_spot_direction::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_spot_direction" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domLight_spot_direction::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domLight_spot_direction::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float3"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_spot_direction , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_spot_direction , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_LIGHTS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_spot_direction , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domLight_spot_direction));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domLight_spot_exponent::create(daeInt bytes)
{
	domGl_pipeline_settings::domLight_spot_exponentRef ref = new(bytes) domGl_pipeline_settings::domLight_spot_exponent;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domLight_spot_exponent::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_spot_exponent" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domLight_spot_exponent::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domLight_spot_exponent::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_spot_exponent , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_spot_exponent , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_LIGHTS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_spot_exponent , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domLight_spot_exponent));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domTexture1D::create(daeInt bytes)
{
	domGl_pipeline_settings::domTexture1DRef ref = new(bytes) domGl_pipeline_settings::domTexture1D;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domTexture1D::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "texture1D" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domTexture1D::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domTexture1D::create);

	// Add elements: value, param
    _Meta->appendElement(domGl_sampler1D::registerElement(),daeOffsetOf(domGl_pipeline_settings::domTexture1D,elemValue),"value"); 
    _Meta->appendElement(domGl_pipeline_settings::domTexture1D::domParam::registerElement(),daeOffsetOf(domGl_pipeline_settings::domTexture1D,elemParam));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domGl_pipeline_settings::domTexture1D,_contents));


	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_TEXTURE_IMAGE_UNITS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTexture1D , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domTexture1D));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domTexture1D::domParam::create(daeInt bytes)
{
	domGl_pipeline_settings::domTexture1D::domParamRef ref = new(bytes) domGl_pipeline_settings::domTexture1D::domParam;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domTexture1D::domParam::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "param" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domTexture1D::domParam::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domTexture1D::domParam::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTexture1D::domParam , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domTexture1D::domParam));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domTexture2D::create(daeInt bytes)
{
	domGl_pipeline_settings::domTexture2DRef ref = new(bytes) domGl_pipeline_settings::domTexture2D;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domTexture2D::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "texture2D" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domTexture2D::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domTexture2D::create);

	// Add elements: value, param
    _Meta->appendElement(domGl_sampler2D::registerElement(),daeOffsetOf(domGl_pipeline_settings::domTexture2D,elemValue),"value"); 
    _Meta->appendElement(domGl_pipeline_settings::domTexture2D::domParam::registerElement(),daeOffsetOf(domGl_pipeline_settings::domTexture2D,elemParam));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domGl_pipeline_settings::domTexture2D,_contents));


	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_TEXTURE_IMAGE_UNITS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTexture2D , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domTexture2D));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domTexture2D::domParam::create(daeInt bytes)
{
	domGl_pipeline_settings::domTexture2D::domParamRef ref = new(bytes) domGl_pipeline_settings::domTexture2D::domParam;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domTexture2D::domParam::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "param" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domTexture2D::domParam::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domTexture2D::domParam::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTexture2D::domParam , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domTexture2D::domParam));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domTexture3D::create(daeInt bytes)
{
	domGl_pipeline_settings::domTexture3DRef ref = new(bytes) domGl_pipeline_settings::domTexture3D;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domTexture3D::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "texture3D" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domTexture3D::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domTexture3D::create);

	// Add elements: value, param
    _Meta->appendElement(domGl_sampler3D::registerElement(),daeOffsetOf(domGl_pipeline_settings::domTexture3D,elemValue),"value"); 
    _Meta->appendElement(domGl_pipeline_settings::domTexture3D::domParam::registerElement(),daeOffsetOf(domGl_pipeline_settings::domTexture3D,elemParam));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domGl_pipeline_settings::domTexture3D,_contents));


	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_TEXTURE_IMAGE_UNITS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTexture3D , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domTexture3D));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domTexture3D::domParam::create(daeInt bytes)
{
	domGl_pipeline_settings::domTexture3D::domParamRef ref = new(bytes) domGl_pipeline_settings::domTexture3D::domParam;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domTexture3D::domParam::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "param" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domTexture3D::domParam::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domTexture3D::domParam::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTexture3D::domParam , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domTexture3D::domParam));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domTextureCUBE::create(daeInt bytes)
{
	domGl_pipeline_settings::domTextureCUBERef ref = new(bytes) domGl_pipeline_settings::domTextureCUBE;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domTextureCUBE::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "textureCUBE" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domTextureCUBE::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domTextureCUBE::create);

	// Add elements: value, param
    _Meta->appendElement(domGl_samplerCUBE::registerElement(),daeOffsetOf(domGl_pipeline_settings::domTextureCUBE,elemValue),"value"); 
    _Meta->appendElement(domGl_pipeline_settings::domTextureCUBE::domParam::registerElement(),daeOffsetOf(domGl_pipeline_settings::domTextureCUBE,elemParam));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domGl_pipeline_settings::domTextureCUBE,_contents));


	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_TEXTURE_IMAGE_UNITS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTextureCUBE , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domTextureCUBE));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domTextureCUBE::domParam::create(daeInt bytes)
{
	domGl_pipeline_settings::domTextureCUBE::domParamRef ref = new(bytes) domGl_pipeline_settings::domTextureCUBE::domParam;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domTextureCUBE::domParam::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "param" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domTextureCUBE::domParam::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domTextureCUBE::domParam::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTextureCUBE::domParam , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domTextureCUBE::domParam));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domTextureRECT::create(daeInt bytes)
{
	domGl_pipeline_settings::domTextureRECTRef ref = new(bytes) domGl_pipeline_settings::domTextureRECT;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domTextureRECT::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "textureRECT" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domTextureRECT::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domTextureRECT::create);

	// Add elements: value, param
    _Meta->appendElement(domGl_samplerRECT::registerElement(),daeOffsetOf(domGl_pipeline_settings::domTextureRECT,elemValue),"value"); 
    _Meta->appendElement(domGl_pipeline_settings::domTextureRECT::domParam::registerElement(),daeOffsetOf(domGl_pipeline_settings::domTextureRECT,elemParam));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domGl_pipeline_settings::domTextureRECT,_contents));


	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_TEXTURE_IMAGE_UNITS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTextureRECT , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domTextureRECT));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domTextureRECT::domParam::create(daeInt bytes)
{
	domGl_pipeline_settings::domTextureRECT::domParamRef ref = new(bytes) domGl_pipeline_settings::domTextureRECT::domParam;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domTextureRECT::domParam::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "param" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domTextureRECT::domParam::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domTextureRECT::domParam::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTextureRECT::domParam , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domTextureRECT::domParam));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domTextureDEPTH::create(daeInt bytes)
{
	domGl_pipeline_settings::domTextureDEPTHRef ref = new(bytes) domGl_pipeline_settings::domTextureDEPTH;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domTextureDEPTH::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "textureDEPTH" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domTextureDEPTH::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domTextureDEPTH::create);

	// Add elements: value, param
    _Meta->appendElement(domGl_samplerDEPTH::registerElement(),daeOffsetOf(domGl_pipeline_settings::domTextureDEPTH,elemValue),"value"); 
    _Meta->appendElement(domGl_pipeline_settings::domTextureDEPTH::domParam::registerElement(),daeOffsetOf(domGl_pipeline_settings::domTextureDEPTH,elemParam));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domGl_pipeline_settings::domTextureDEPTH,_contents));


	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_TEXTURE_IMAGE_UNITS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTextureDEPTH , attrIndex ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domTextureDEPTH));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domTextureDEPTH::domParam::create(daeInt bytes)
{
	domGl_pipeline_settings::domTextureDEPTH::domParamRef ref = new(bytes) domGl_pipeline_settings::domTextureDEPTH::domParam;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domTextureDEPTH::domParam::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "param" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domTextureDEPTH::domParam::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domTextureDEPTH::domParam::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTextureDEPTH::domParam , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domTextureDEPTH::domParam));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domTexture1D_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domTexture1D_enableRef ref = new(bytes) domGl_pipeline_settings::domTexture1D_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domTexture1D_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "texture1D_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domTexture1D_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domTexture1D_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTexture1D_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTexture1D_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_TEXTURE_IMAGE_UNITS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTexture1D_enable , attrIndex ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domTexture1D_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domTexture2D_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domTexture2D_enableRef ref = new(bytes) domGl_pipeline_settings::domTexture2D_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domTexture2D_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "texture2D_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domTexture2D_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domTexture2D_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTexture2D_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTexture2D_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_TEXTURE_IMAGE_UNITS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTexture2D_enable , attrIndex ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domTexture2D_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domTexture3D_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domTexture3D_enableRef ref = new(bytes) domGl_pipeline_settings::domTexture3D_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domTexture3D_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "texture3D_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domTexture3D_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domTexture3D_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTexture3D_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTexture3D_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_TEXTURE_IMAGE_UNITS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTexture3D_enable , attrIndex ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domTexture3D_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domTextureCUBE_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domTextureCUBE_enableRef ref = new(bytes) domGl_pipeline_settings::domTextureCUBE_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domTextureCUBE_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "textureCUBE_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domTextureCUBE_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domTextureCUBE_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTextureCUBE_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTextureCUBE_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_TEXTURE_IMAGE_UNITS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTextureCUBE_enable , attrIndex ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domTextureCUBE_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domTextureRECT_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domTextureRECT_enableRef ref = new(bytes) domGl_pipeline_settings::domTextureRECT_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domTextureRECT_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "textureRECT_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domTextureRECT_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domTextureRECT_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTextureRECT_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTextureRECT_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_TEXTURE_IMAGE_UNITS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTextureRECT_enable , attrIndex ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domTextureRECT_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domTextureDEPTH_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domTextureDEPTH_enableRef ref = new(bytes) domGl_pipeline_settings::domTextureDEPTH_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domTextureDEPTH_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "textureDEPTH_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domTextureDEPTH_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domTextureDEPTH_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTextureDEPTH_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTextureDEPTH_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_TEXTURE_IMAGE_UNITS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTextureDEPTH_enable , attrIndex ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domTextureDEPTH_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domTexture_env_color::create(daeInt bytes)
{
	domGl_pipeline_settings::domTexture_env_colorRef ref = new(bytes) domGl_pipeline_settings::domTexture_env_color;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domTexture_env_color::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "texture_env_color" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domTexture_env_color::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domTexture_env_color::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTexture_env_color , attrValue ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTexture_env_color , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_TEXTURE_IMAGE_UNITS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTexture_env_color , attrIndex ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domTexture_env_color));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domTexture_env_mode::create(daeInt bytes)
{
	domGl_pipeline_settings::domTexture_env_modeRef ref = new(bytes) domGl_pipeline_settings::domTexture_env_mode;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domTexture_env_mode::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "texture_env_mode" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domTexture_env_mode::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domTexture_env_mode::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("String"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTexture_env_mode , attrValue ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTexture_env_mode , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_TEXTURE_IMAGE_UNITS_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domTexture_env_mode , attrIndex ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domTexture_env_mode));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domClip_plane::create(daeInt bytes)
{
	domGl_pipeline_settings::domClip_planeRef ref = new(bytes) domGl_pipeline_settings::domClip_plane;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domClip_plane::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "clip_plane" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domClip_plane::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domClip_plane::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domClip_plane , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domClip_plane , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_CLIP_PLANES_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domClip_plane , attrIndex ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domClip_plane));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domClip_plane_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domClip_plane_enableRef ref = new(bytes) domGl_pipeline_settings::domClip_plane_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domClip_plane_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "clip_plane_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domClip_plane_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domClip_plane_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domClip_plane_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domClip_plane_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: index
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "index" );
		ma->setType( daeAtomicType::get("GL_MAX_CLIP_PLANES_index"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domClip_plane_enable , attrIndex ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domClip_plane_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domBlend_color::create(daeInt bytes)
{
	domGl_pipeline_settings::domBlend_colorRef ref = new(bytes) domGl_pipeline_settings::domBlend_color;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domBlend_color::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "blend_color" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domBlend_color::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domBlend_color::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domBlend_color , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domBlend_color , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domBlend_color));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domClear_color::create(daeInt bytes)
{
	domGl_pipeline_settings::domClear_colorRef ref = new(bytes) domGl_pipeline_settings::domClear_color;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domClear_color::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "clear_color" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domClear_color::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domClear_color::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domClear_color , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domClear_color , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domClear_color));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domClear_stencil::create(daeInt bytes)
{
	domGl_pipeline_settings::domClear_stencilRef ref = new(bytes) domGl_pipeline_settings::domClear_stencil;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domClear_stencil::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "clear_stencil" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domClear_stencil::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domClear_stencil::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Int"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domClear_stencil , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domClear_stencil , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domClear_stencil));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domClear_depth::create(daeInt bytes)
{
	domGl_pipeline_settings::domClear_depthRef ref = new(bytes) domGl_pipeline_settings::domClear_depth;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domClear_depth::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "clear_depth" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domClear_depth::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domClear_depth::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domClear_depth , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domClear_depth , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domClear_depth));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domColor_mask::create(daeInt bytes)
{
	domGl_pipeline_settings::domColor_maskRef ref = new(bytes) domGl_pipeline_settings::domColor_mask;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domColor_mask::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "color_mask" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domColor_mask::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domColor_mask::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool4"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domColor_mask , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "true true true true");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domColor_mask , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domColor_mask));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domDepth_bounds::create(daeInt bytes)
{
	domGl_pipeline_settings::domDepth_boundsRef ref = new(bytes) domGl_pipeline_settings::domDepth_bounds;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domDepth_bounds::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "depth_bounds" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domDepth_bounds::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domDepth_bounds::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float2"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domDepth_bounds , attrValue ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domDepth_bounds , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domDepth_bounds));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domDepth_mask::create(daeInt bytes)
{
	domGl_pipeline_settings::domDepth_maskRef ref = new(bytes) domGl_pipeline_settings::domDepth_mask;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domDepth_mask::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "depth_mask" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domDepth_mask::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domDepth_mask::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domDepth_mask , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domDepth_mask , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domDepth_mask));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domDepth_range::create(daeInt bytes)
{
	domGl_pipeline_settings::domDepth_rangeRef ref = new(bytes) domGl_pipeline_settings::domDepth_range;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domDepth_range::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "depth_range" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domDepth_range::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domDepth_range::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float2"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domDepth_range , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domDepth_range , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domDepth_range));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domFog_density::create(daeInt bytes)
{
	domGl_pipeline_settings::domFog_densityRef ref = new(bytes) domGl_pipeline_settings::domFog_density;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domFog_density::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fog_density" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domFog_density::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domFog_density::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domFog_density , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domFog_density , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domFog_density));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domFog_start::create(daeInt bytes)
{
	domGl_pipeline_settings::domFog_startRef ref = new(bytes) domGl_pipeline_settings::domFog_start;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domFog_start::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fog_start" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domFog_start::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domFog_start::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domFog_start , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domFog_start , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domFog_start));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domFog_end::create(daeInt bytes)
{
	domGl_pipeline_settings::domFog_endRef ref = new(bytes) domGl_pipeline_settings::domFog_end;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domFog_end::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fog_end" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domFog_end::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domFog_end::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domFog_end , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domFog_end , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domFog_end));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domFog_color::create(daeInt bytes)
{
	domGl_pipeline_settings::domFog_colorRef ref = new(bytes) domGl_pipeline_settings::domFog_color;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domFog_color::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fog_color" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domFog_color::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domFog_color::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domFog_color , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domFog_color , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domFog_color));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domLight_model_ambient::create(daeInt bytes)
{
	domGl_pipeline_settings::domLight_model_ambientRef ref = new(bytes) domGl_pipeline_settings::domLight_model_ambient;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domLight_model_ambient::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_model_ambient" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domLight_model_ambient::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domLight_model_ambient::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_model_ambient , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_model_ambient , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domLight_model_ambient));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domLighting_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domLighting_enableRef ref = new(bytes) domGl_pipeline_settings::domLighting_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domLighting_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "lighting_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domLighting_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domLighting_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLighting_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLighting_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domLighting_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domLine_stipple::create(daeInt bytes)
{
	domGl_pipeline_settings::domLine_stippleRef ref = new(bytes) domGl_pipeline_settings::domLine_stipple;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domLine_stipple::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "line_stipple" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domLine_stipple::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domLine_stipple::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Int2"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLine_stipple , attrValue ));
		ma->setContainer( _Meta );
		ma->setDefault( "1 65536");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLine_stipple , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domLine_stipple));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domLine_width::create(daeInt bytes)
{
	domGl_pipeline_settings::domLine_widthRef ref = new(bytes) domGl_pipeline_settings::domLine_width;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domLine_width::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "line_width" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domLine_width::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domLine_width::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLine_width , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLine_width , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domLine_width));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domMaterial_ambient::create(daeInt bytes)
{
	domGl_pipeline_settings::domMaterial_ambientRef ref = new(bytes) domGl_pipeline_settings::domMaterial_ambient;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domMaterial_ambient::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "material_ambient" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domMaterial_ambient::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domMaterial_ambient::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domMaterial_ambient , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domMaterial_ambient , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domMaterial_ambient));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domMaterial_diffuse::create(daeInt bytes)
{
	domGl_pipeline_settings::domMaterial_diffuseRef ref = new(bytes) domGl_pipeline_settings::domMaterial_diffuse;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domMaterial_diffuse::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "material_diffuse" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domMaterial_diffuse::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domMaterial_diffuse::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domMaterial_diffuse , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domMaterial_diffuse , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domMaterial_diffuse));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domMaterial_emission::create(daeInt bytes)
{
	domGl_pipeline_settings::domMaterial_emissionRef ref = new(bytes) domGl_pipeline_settings::domMaterial_emission;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domMaterial_emission::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "material_emission" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domMaterial_emission::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domMaterial_emission::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domMaterial_emission , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domMaterial_emission , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domMaterial_emission));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domMaterial_shininess::create(daeInt bytes)
{
	domGl_pipeline_settings::domMaterial_shininessRef ref = new(bytes) domGl_pipeline_settings::domMaterial_shininess;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domMaterial_shininess::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "material_shininess" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domMaterial_shininess::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domMaterial_shininess::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domMaterial_shininess , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domMaterial_shininess , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domMaterial_shininess));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domMaterial_specular::create(daeInt bytes)
{
	domGl_pipeline_settings::domMaterial_specularRef ref = new(bytes) domGl_pipeline_settings::domMaterial_specular;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domMaterial_specular::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "material_specular" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domMaterial_specular::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domMaterial_specular::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domMaterial_specular , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domMaterial_specular , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domMaterial_specular));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domModel_view_matrix::create(daeInt bytes)
{
	domGl_pipeline_settings::domModel_view_matrixRef ref = new(bytes) domGl_pipeline_settings::domModel_view_matrix;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domModel_view_matrix::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "model_view_matrix" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domModel_view_matrix::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domModel_view_matrix::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4x4"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domModel_view_matrix , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domModel_view_matrix , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domModel_view_matrix));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domPoint_distance_attenuation::create(daeInt bytes)
{
	domGl_pipeline_settings::domPoint_distance_attenuationRef ref = new(bytes) domGl_pipeline_settings::domPoint_distance_attenuation;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domPoint_distance_attenuation::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "point_distance_attenuation" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domPoint_distance_attenuation::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domPoint_distance_attenuation::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float3"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPoint_distance_attenuation , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPoint_distance_attenuation , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domPoint_distance_attenuation));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domPoint_fade_threshold_size::create(daeInt bytes)
{
	domGl_pipeline_settings::domPoint_fade_threshold_sizeRef ref = new(bytes) domGl_pipeline_settings::domPoint_fade_threshold_size;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domPoint_fade_threshold_size::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "point_fade_threshold_size" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domPoint_fade_threshold_size::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domPoint_fade_threshold_size::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPoint_fade_threshold_size , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPoint_fade_threshold_size , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domPoint_fade_threshold_size));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domPoint_size::create(daeInt bytes)
{
	domGl_pipeline_settings::domPoint_sizeRef ref = new(bytes) domGl_pipeline_settings::domPoint_size;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domPoint_size::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "point_size" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domPoint_size::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domPoint_size::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPoint_size , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPoint_size , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domPoint_size));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domPoint_size_min::create(daeInt bytes)
{
	domGl_pipeline_settings::domPoint_size_minRef ref = new(bytes) domGl_pipeline_settings::domPoint_size_min;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domPoint_size_min::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "point_size_min" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domPoint_size_min::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domPoint_size_min::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPoint_size_min , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPoint_size_min , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domPoint_size_min));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domPoint_size_max::create(daeInt bytes)
{
	domGl_pipeline_settings::domPoint_size_maxRef ref = new(bytes) domGl_pipeline_settings::domPoint_size_max;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domPoint_size_max::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "point_size_max" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domPoint_size_max::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domPoint_size_max::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPoint_size_max , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPoint_size_max , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domPoint_size_max));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domPolygon_offset::create(daeInt bytes)
{
	domGl_pipeline_settings::domPolygon_offsetRef ref = new(bytes) domGl_pipeline_settings::domPolygon_offset;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domPolygon_offset::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "polygon_offset" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domPolygon_offset::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domPolygon_offset::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float2"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPolygon_offset , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPolygon_offset , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domPolygon_offset));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domProjection_matrix::create(daeInt bytes)
{
	domGl_pipeline_settings::domProjection_matrixRef ref = new(bytes) domGl_pipeline_settings::domProjection_matrix;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domProjection_matrix::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "projection_matrix" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domProjection_matrix::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domProjection_matrix::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Float4x4"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domProjection_matrix , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domProjection_matrix , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domProjection_matrix));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domScissor::create(daeInt bytes)
{
	domGl_pipeline_settings::domScissorRef ref = new(bytes) domGl_pipeline_settings::domScissor;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domScissor::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "scissor" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domScissor::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domScissor::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Int4"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domScissor , attrValue ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: param
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "param" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domScissor , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domScissor));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domStencil_mask::create(daeInt bytes)
{
	domGl_pipeline_settings::domStencil_maskRef ref = new(bytes) domGl_pipeline_settings::domStencil_mask;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domStencil_mask::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "stencil_mask" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domStencil_mask::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domStencil_mask::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Int"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_mask , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_mask , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domStencil_mask));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domAlpha_test_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domAlpha_test_enableRef ref = new(bytes) domGl_pipeline_settings::domAlpha_test_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domAlpha_test_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "alpha_test_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domAlpha_test_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domAlpha_test_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domAlpha_test_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domAlpha_test_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domAlpha_test_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domAuto_normal_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domAuto_normal_enableRef ref = new(bytes) domGl_pipeline_settings::domAuto_normal_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domAuto_normal_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "auto_normal_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domAuto_normal_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domAuto_normal_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domAuto_normal_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domAuto_normal_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domAuto_normal_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domBlend_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domBlend_enableRef ref = new(bytes) domGl_pipeline_settings::domBlend_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domBlend_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "blend_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domBlend_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domBlend_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domBlend_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domBlend_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domBlend_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domColor_logic_op_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domColor_logic_op_enableRef ref = new(bytes) domGl_pipeline_settings::domColor_logic_op_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domColor_logic_op_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "color_logic_op_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domColor_logic_op_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domColor_logic_op_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domColor_logic_op_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domColor_logic_op_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domColor_logic_op_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domCull_face_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domCull_face_enableRef ref = new(bytes) domGl_pipeline_settings::domCull_face_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domCull_face_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "cull_face_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domCull_face_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domCull_face_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domCull_face_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domCull_face_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domCull_face_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domDepth_bounds_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domDepth_bounds_enableRef ref = new(bytes) domGl_pipeline_settings::domDepth_bounds_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domDepth_bounds_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "depth_bounds_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domDepth_bounds_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domDepth_bounds_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domDepth_bounds_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domDepth_bounds_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domDepth_bounds_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domDepth_clamp_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domDepth_clamp_enableRef ref = new(bytes) domGl_pipeline_settings::domDepth_clamp_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domDepth_clamp_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "depth_clamp_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domDepth_clamp_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domDepth_clamp_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domDepth_clamp_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domDepth_clamp_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domDepth_clamp_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domDepth_test_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domDepth_test_enableRef ref = new(bytes) domGl_pipeline_settings::domDepth_test_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domDepth_test_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "depth_test_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domDepth_test_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domDepth_test_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domDepth_test_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domDepth_test_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domDepth_test_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domDither_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domDither_enableRef ref = new(bytes) domGl_pipeline_settings::domDither_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domDither_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "dither_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domDither_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domDither_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domDither_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domDither_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domDither_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domFog_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domFog_enableRef ref = new(bytes) domGl_pipeline_settings::domFog_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domFog_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "fog_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domFog_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domFog_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domFog_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domFog_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domFog_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domLight_model_local_viewer_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domLight_model_local_viewer_enableRef ref = new(bytes) domGl_pipeline_settings::domLight_model_local_viewer_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domLight_model_local_viewer_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_model_local_viewer_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domLight_model_local_viewer_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domLight_model_local_viewer_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_model_local_viewer_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_model_local_viewer_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domLight_model_local_viewer_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domLight_model_two_side_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domLight_model_two_side_enableRef ref = new(bytes) domGl_pipeline_settings::domLight_model_two_side_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domLight_model_two_side_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "light_model_two_side_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domLight_model_two_side_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domLight_model_two_side_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_model_two_side_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLight_model_two_side_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domLight_model_two_side_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domLine_smooth_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domLine_smooth_enableRef ref = new(bytes) domGl_pipeline_settings::domLine_smooth_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domLine_smooth_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "line_smooth_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domLine_smooth_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domLine_smooth_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLine_smooth_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLine_smooth_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domLine_smooth_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domLine_stipple_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domLine_stipple_enableRef ref = new(bytes) domGl_pipeline_settings::domLine_stipple_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domLine_stipple_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "line_stipple_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domLine_stipple_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domLine_stipple_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLine_stipple_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLine_stipple_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domLine_stipple_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domLogic_op_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domLogic_op_enableRef ref = new(bytes) domGl_pipeline_settings::domLogic_op_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domLogic_op_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "logic_op_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domLogic_op_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domLogic_op_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLogic_op_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domLogic_op_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domLogic_op_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domMultisample_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domMultisample_enableRef ref = new(bytes) domGl_pipeline_settings::domMultisample_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domMultisample_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "multisample_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domMultisample_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domMultisample_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domMultisample_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domMultisample_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domMultisample_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domNormalize_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domNormalize_enableRef ref = new(bytes) domGl_pipeline_settings::domNormalize_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domNormalize_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "normalize_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domNormalize_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domNormalize_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domNormalize_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domNormalize_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domNormalize_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domPoint_smooth_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domPoint_smooth_enableRef ref = new(bytes) domGl_pipeline_settings::domPoint_smooth_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domPoint_smooth_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "point_smooth_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domPoint_smooth_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domPoint_smooth_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPoint_smooth_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPoint_smooth_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domPoint_smooth_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domPolygon_offset_fill_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domPolygon_offset_fill_enableRef ref = new(bytes) domGl_pipeline_settings::domPolygon_offset_fill_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domPolygon_offset_fill_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "polygon_offset_fill_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domPolygon_offset_fill_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domPolygon_offset_fill_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPolygon_offset_fill_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPolygon_offset_fill_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domPolygon_offset_fill_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domPolygon_offset_line_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domPolygon_offset_line_enableRef ref = new(bytes) domGl_pipeline_settings::domPolygon_offset_line_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domPolygon_offset_line_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "polygon_offset_line_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domPolygon_offset_line_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domPolygon_offset_line_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPolygon_offset_line_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPolygon_offset_line_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domPolygon_offset_line_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domPolygon_offset_point_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domPolygon_offset_point_enableRef ref = new(bytes) domGl_pipeline_settings::domPolygon_offset_point_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domPolygon_offset_point_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "polygon_offset_point_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domPolygon_offset_point_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domPolygon_offset_point_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPolygon_offset_point_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPolygon_offset_point_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domPolygon_offset_point_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domPolygon_smooth_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domPolygon_smooth_enableRef ref = new(bytes) domGl_pipeline_settings::domPolygon_smooth_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domPolygon_smooth_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "polygon_smooth_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domPolygon_smooth_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domPolygon_smooth_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPolygon_smooth_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPolygon_smooth_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domPolygon_smooth_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domPolygon_stipple_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domPolygon_stipple_enableRef ref = new(bytes) domGl_pipeline_settings::domPolygon_stipple_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domPolygon_stipple_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "polygon_stipple_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domPolygon_stipple_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domPolygon_stipple_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPolygon_stipple_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domPolygon_stipple_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domPolygon_stipple_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domRescale_normal_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domRescale_normal_enableRef ref = new(bytes) domGl_pipeline_settings::domRescale_normal_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domRescale_normal_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "rescale_normal_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domRescale_normal_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domRescale_normal_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domRescale_normal_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domRescale_normal_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domRescale_normal_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domSample_alpha_to_coverage_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domSample_alpha_to_coverage_enableRef ref = new(bytes) domGl_pipeline_settings::domSample_alpha_to_coverage_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domSample_alpha_to_coverage_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "sample_alpha_to_coverage_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domSample_alpha_to_coverage_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domSample_alpha_to_coverage_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domSample_alpha_to_coverage_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domSample_alpha_to_coverage_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domSample_alpha_to_coverage_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domSample_alpha_to_one_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domSample_alpha_to_one_enableRef ref = new(bytes) domGl_pipeline_settings::domSample_alpha_to_one_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domSample_alpha_to_one_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "sample_alpha_to_one_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domSample_alpha_to_one_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domSample_alpha_to_one_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domSample_alpha_to_one_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domSample_alpha_to_one_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domSample_alpha_to_one_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domSample_coverage_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domSample_coverage_enableRef ref = new(bytes) domGl_pipeline_settings::domSample_coverage_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domSample_coverage_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "sample_coverage_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domSample_coverage_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domSample_coverage_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domSample_coverage_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domSample_coverage_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domSample_coverage_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domScissor_test_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domScissor_test_enableRef ref = new(bytes) domGl_pipeline_settings::domScissor_test_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domScissor_test_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "scissor_test_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domScissor_test_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domScissor_test_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domScissor_test_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domScissor_test_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domScissor_test_enable));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domGl_pipeline_settings::domStencil_test_enable::create(daeInt bytes)
{
	domGl_pipeline_settings::domStencil_test_enableRef ref = new(bytes) domGl_pipeline_settings::domStencil_test_enable;
	return ref;
}


daeMetaElement *
domGl_pipeline_settings::domStencil_test_enable::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "stencil_test_enable" );
	_Meta->setStaticPointerAddress(&domGl_pipeline_settings::domStencil_test_enable::_Meta);
	_Meta->registerConstructor(domGl_pipeline_settings::domStencil_test_enable::create);


	//	Add attribute: value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "value" );
		ma->setType( daeAtomicType::get("Bool"));
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_test_enable , attrValue ));
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
		ma->setOffset( daeOffsetOf( domGl_pipeline_settings::domStencil_test_enable , attrParam ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domGl_pipeline_settings::domStencil_test_enable));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domGl_pipeline_settings::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domAlpha_func::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domAlpha_func::domFunc::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domAlpha_func::domValue::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domBlend_func::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domBlend_func::domSrc::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domBlend_func::domDest::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domBlend_func_separate::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domBlend_func_separate::domSrc_rgb::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domBlend_func_separate::domDest_rgb::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domBlend_func_separate::domSrc_alpha::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domBlend_func_separate::domDest_alpha::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domBlend_equation::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domBlend_equation_separate::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domBlend_equation_separate::domRgb::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domBlend_equation_separate::domAlpha::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domColor_material::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domColor_material::domFace::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domColor_material::domMode::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domCull_face::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domDepth_func::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domFog_mode::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domFog_coord_src::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domFront_face::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domLight_model_color_control::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domLogic_op::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domPolygon_mode::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domPolygon_mode::domFace::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domPolygon_mode::domMode::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domShade_model::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domStencil_func::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domStencil_func::domFunc::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domStencil_func::domRef::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domStencil_func::domMask::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domStencil_op::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domStencil_op::domFail::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domStencil_op::domZfail::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domStencil_op::domZpass::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domStencil_func_separate::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domStencil_func_separate::domFront::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domStencil_func_separate::domBack::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domStencil_func_separate::domRef::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domStencil_func_separate::domMask::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domStencil_op_separate::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domStencil_op_separate::domFace::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domStencil_op_separate::domFail::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domStencil_op_separate::domZfail::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domStencil_op_separate::domZpass::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domStencil_mask_separate::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domStencil_mask_separate::domFace::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domStencil_mask_separate::domMask::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domLight_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domLight_ambient::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domLight_diffuse::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domLight_specular::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domLight_position::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domLight_constant_attenuation::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domLight_linear_attenuation::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domLight_quadratic_attenuation::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domLight_spot_cutoff::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domLight_spot_direction::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domLight_spot_exponent::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domTexture1D::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domTexture1D::domParam::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domTexture2D::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domTexture2D::domParam::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domTexture3D::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domTexture3D::domParam::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domTextureCUBE::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domTextureCUBE::domParam::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domTextureRECT::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domTextureRECT::domParam::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domTextureDEPTH::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domTextureDEPTH::domParam::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domTexture1D_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domTexture2D_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domTexture3D_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domTextureCUBE_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domTextureRECT_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domTextureDEPTH_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domTexture_env_color::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domTexture_env_mode::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domClip_plane::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domClip_plane_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domBlend_color::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domClear_color::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domClear_stencil::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domClear_depth::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domColor_mask::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domDepth_bounds::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domDepth_mask::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domDepth_range::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domFog_density::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domFog_start::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domFog_end::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domFog_color::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domLight_model_ambient::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domLighting_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domLine_stipple::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domLine_width::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domMaterial_ambient::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domMaterial_diffuse::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domMaterial_emission::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domMaterial_shininess::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domMaterial_specular::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domModel_view_matrix::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domPoint_distance_attenuation::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domPoint_fade_threshold_size::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domPoint_size::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domPoint_size_min::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domPoint_size_max::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domPolygon_offset::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domProjection_matrix::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domScissor::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domStencil_mask::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domAlpha_test_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domAuto_normal_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domBlend_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domColor_logic_op_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domCull_face_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domDepth_bounds_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domDepth_clamp_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domDepth_test_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domDither_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domFog_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domLight_model_local_viewer_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domLight_model_two_side_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domLine_smooth_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domLine_stipple_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domLogic_op_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domMultisample_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domNormalize_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domPoint_smooth_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domPolygon_offset_fill_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domPolygon_offset_line_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domPolygon_offset_point_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domPolygon_smooth_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domPolygon_stipple_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domRescale_normal_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domSample_alpha_to_coverage_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domSample_alpha_to_one_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domSample_coverage_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domScissor_test_enable::_Meta = NULL;
daeMetaElement * domGl_pipeline_settings::domStencil_test_enable::_Meta = NULL;


