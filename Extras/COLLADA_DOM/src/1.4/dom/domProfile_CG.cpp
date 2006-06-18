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
#include <dom/domProfile_CG.h>

daeElementRef
domProfile_CG::create(daeInt bytes)
{
	domProfile_CGRef ref = new(bytes) domProfile_CG;
	return ref;
}


daeMetaElement *
domProfile_CG::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "profile_CG" );
	_Meta->setStaticPointerAddress(&domProfile_CG::_Meta);
	_Meta->registerConstructor(domProfile_CG::create);

	// Add elements: code, include, image, newparam, technique
    _Meta->appendArrayElement(domFx_code_profile::registerElement(),daeOffsetOf(domProfile_CG,elemCode_array),"code"); 
    _Meta->appendArrayElement(domFx_include_common::registerElement(),daeOffsetOf(domProfile_CG,elemInclude_array),"include"); 
    _Meta->appendArrayElement(domImage::registerElement(),daeOffsetOf(domProfile_CG,elemImage_array));
    _Meta->appendArrayElement(domCg_newparam::registerElement(),daeOffsetOf(domProfile_CG,elemNewparam_array),"newparam"); 
    _Meta->appendArrayElement(domProfile_CG::domTechnique::registerElement(),daeOffsetOf(domProfile_CG,elemTechnique_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domProfile_CG,_contents));


	//	Add attribute: platform
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "platform" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domProfile_CG , attrPlatform ));
		ma->setContainer( _Meta );
		ma->setDefault( "PC");
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_CG));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_CG::domTechnique::create(daeInt bytes)
{
	domProfile_CG::domTechniqueRef ref = new(bytes) domProfile_CG::domTechnique;
	return ref;
}


daeMetaElement *
domProfile_CG::domTechnique::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "technique" );
	_Meta->setStaticPointerAddress(&domProfile_CG::domTechnique::_Meta);
	_Meta->registerConstructor(domProfile_CG::domTechnique::create);

	// Add elements: asset, annotate, code, include, image, newparam, setparam, pass
    _Meta->appendElement(domAsset::registerElement(),daeOffsetOf(domProfile_CG::domTechnique,elemAsset));
    _Meta->appendArrayElement(domFx_annotate_common::registerElement(),daeOffsetOf(domProfile_CG::domTechnique,elemAnnotate_array),"annotate"); 
    _Meta->appendArrayElement(domFx_code_profile::registerElement(),daeOffsetOf(domProfile_CG::domTechnique,elemCode_array),"code"); 
    _Meta->appendArrayElement(domFx_include_common::registerElement(),daeOffsetOf(domProfile_CG::domTechnique,elemInclude_array),"include"); 
    _Meta->appendArrayElement(domImage::registerElement(),daeOffsetOf(domProfile_CG::domTechnique,elemImage_array));
    _Meta->appendArrayElement(domCg_newparam::registerElement(),daeOffsetOf(domProfile_CG::domTechnique,elemNewparam_array),"newparam"); 
    _Meta->appendArrayElement(domCg_setparam::registerElement(),daeOffsetOf(domProfile_CG::domTechnique,elemSetparam_array),"setparam"); 
    _Meta->appendArrayElement(domProfile_CG::domTechnique::domPass::registerElement(),daeOffsetOf(domProfile_CG::domTechnique,elemPass_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domProfile_CG::domTechnique,_contents));


	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domProfile_CG::domTechnique , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domProfile_CG::domTechnique , attrSid ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_CG::domTechnique));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_CG::domTechnique::domPass::create(daeInt bytes)
{
	domProfile_CG::domTechnique::domPassRef ref = new(bytes) domProfile_CG::domTechnique::domPass;
	return ref;
}


daeMetaElement *
domProfile_CG::domTechnique::domPass::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "pass" );
	_Meta->setStaticPointerAddress(&domProfile_CG::domTechnique::domPass::_Meta);
	_Meta->registerConstructor(domProfile_CG::domTechnique::domPass::create);

	// Add elements: annotate, color_target, depth_target, stencil_target, color_clear, depth_clear, stencil_clear, draw, gl_pipeline_settings, shader
    _Meta->appendArrayElement(domFx_annotate_common::registerElement(),daeOffsetOf(domProfile_CG::domTechnique::domPass,elemAnnotate_array),"annotate"); 
    _Meta->appendArrayElement(domFx_colortarget_common::registerElement(),daeOffsetOf(domProfile_CG::domTechnique::domPass,elemColor_target_array),"color_target"); 
    _Meta->appendArrayElement(domFx_depthtarget_common::registerElement(),daeOffsetOf(domProfile_CG::domTechnique::domPass,elemDepth_target_array),"depth_target"); 
    _Meta->appendArrayElement(domFx_stenciltarget_common::registerElement(),daeOffsetOf(domProfile_CG::domTechnique::domPass,elemStencil_target_array),"stencil_target"); 
    _Meta->appendArrayElement(domFx_clearcolor_common::registerElement(),daeOffsetOf(domProfile_CG::domTechnique::domPass,elemColor_clear_array),"color_clear"); 
    _Meta->appendArrayElement(domFx_cleardepth_common::registerElement(),daeOffsetOf(domProfile_CG::domTechnique::domPass,elemDepth_clear_array),"depth_clear"); 
    _Meta->appendArrayElement(domFx_clearstencil_common::registerElement(),daeOffsetOf(domProfile_CG::domTechnique::domPass,elemStencil_clear_array),"stencil_clear"); 
    _Meta->appendElement(domProfile_CG::domTechnique::domPass::domDraw::registerElement(),daeOffsetOf(domProfile_CG::domTechnique::domPass,elemDraw));
    _Meta->appendArrayElement(domGl_pipeline_settings::registerElement(),daeOffsetOf(domProfile_CG::domTechnique::domPass,elemGl_pipeline_settings_array));
	_Meta->appendPossibleChild( "alpha_func", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "blend_func", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "blend_func_separate", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "blend_equation", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "blend_equation_separate", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "color_material", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "cull_face", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "depth_func", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "fog_mode", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "fog_coord_src", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "front_face", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_model_color_control", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "logic_op", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "polygon_mode", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "shade_model", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "stencil_func", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "stencil_op", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "stencil_func_separate", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "stencil_op_separate", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "stencil_mask_separate", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_ambient", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_diffuse", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_specular", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_position", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_constant_attenuation", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_linear_attenuation", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_quadratic_attenuation", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_spot_cutoff", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_spot_direction", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_spot_exponent", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "texture1D", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "texture2D", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "texture3D", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "textureCUBE", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "textureRECT", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "textureDEPTH", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "texture1D_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "texture2D_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "texture3D_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "textureCUBE_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "textureRECT_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "textureDEPTH_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "texture_env_color", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "texture_env_mode", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "clip_plane", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "clip_plane_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "blend_color", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "clear_color", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "clear_stencil", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "clear_depth", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "color_mask", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "depth_bounds", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "depth_mask", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "depth_range", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "fog_density", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "fog_start", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "fog_end", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "fog_color", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_model_ambient", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "lighting_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "line_stipple", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "line_width", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "material_ambient", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "material_diffuse", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "material_emission", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "material_shininess", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "material_specular", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "model_view_matrix", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "point_distance_attenuation", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "point_fade_threshold_size", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "point_size", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "point_size_min", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "point_size_max", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "polygon_offset", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "projection_matrix", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "scissor", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "stencil_mask", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "alpha_test_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "auto_normal_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "blend_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "color_logic_op_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "cull_face_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "depth_bounds_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "depth_clamp_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "depth_test_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "dither_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "fog_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_model_local_viewer_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_model_two_side_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "line_smooth_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "line_stipple_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "logic_op_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "multisample_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "normalize_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "point_smooth_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "polygon_offset_fill_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "polygon_offset_line_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "polygon_offset_point_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "polygon_smooth_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "polygon_stipple_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "rescale_normal_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "sample_alpha_to_coverage_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "sample_alpha_to_one_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "sample_coverage_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "scissor_test_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "stencil_test_enable", _Meta->getMetaElements()[8]);
    _Meta->appendArrayElement(domProfile_CG::domTechnique::domPass::domShader::registerElement(),daeOffsetOf(domProfile_CG::domTechnique::domPass,elemShader_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domProfile_CG::domTechnique::domPass,_contents));


	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domProfile_CG::domTechnique::domPass , attrSid ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_CG::domTechnique::domPass));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_CG::domTechnique::domPass::domDraw::create(daeInt bytes)
{
	domProfile_CG::domTechnique::domPass::domDrawRef ref = new(bytes) domProfile_CG::domTechnique::domPass::domDraw;
	return ref;
}


daeMetaElement *
domProfile_CG::domTechnique::domPass::domDraw::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "draw" );
	_Meta->setStaticPointerAddress(&domProfile_CG::domTechnique::domPass::domDraw::_Meta);
	_Meta->registerConstructor(domProfile_CG::domTechnique::domPass::domDraw::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_draw_common"));
		ma->setOffset( daeOffsetOf( domProfile_CG::domTechnique::domPass::domDraw , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_CG::domTechnique::domPass::domDraw));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_CG::domTechnique::domPass::domShader::create(daeInt bytes)
{
	domProfile_CG::domTechnique::domPass::domShaderRef ref = new(bytes) domProfile_CG::domTechnique::domPass::domShader;
	return ref;
}


daeMetaElement *
domProfile_CG::domTechnique::domPass::domShader::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "shader" );
	_Meta->setStaticPointerAddress(&domProfile_CG::domTechnique::domPass::domShader::_Meta);
	_Meta->registerConstructor(domProfile_CG::domTechnique::domPass::domShader::create);

	// Add elements: annotate, compiler_target, compiler_options, name, bind
    _Meta->appendArrayElement(domFx_annotate_common::registerElement(),daeOffsetOf(domProfile_CG::domTechnique::domPass::domShader,elemAnnotate_array),"annotate"); 
    _Meta->appendElement(domProfile_CG::domTechnique::domPass::domShader::domCompiler_target::registerElement(),daeOffsetOf(domProfile_CG::domTechnique::domPass::domShader,elemCompiler_target));
    _Meta->appendElement(domProfile_CG::domTechnique::domPass::domShader::domCompiler_options::registerElement(),daeOffsetOf(domProfile_CG::domTechnique::domPass::domShader,elemCompiler_options));
    _Meta->appendElement(domProfile_CG::domTechnique::domPass::domShader::domName::registerElement(),daeOffsetOf(domProfile_CG::domTechnique::domPass::domShader,elemName));
    _Meta->appendArrayElement(domProfile_CG::domTechnique::domPass::domShader::domBind::registerElement(),daeOffsetOf(domProfile_CG::domTechnique::domPass::domShader,elemBind_array));

	//	Add attribute: stage
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "stage" );
		ma->setType( daeAtomicType::get("Cg_pipeline_stage"));
		ma->setOffset( daeOffsetOf( domProfile_CG::domTechnique::domPass::domShader , attrStage ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_CG::domTechnique::domPass::domShader));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_CG::domTechnique::domPass::domShader::domCompiler_target::create(daeInt bytes)
{
	domProfile_CG::domTechnique::domPass::domShader::domCompiler_targetRef ref = new(bytes) domProfile_CG::domTechnique::domPass::domShader::domCompiler_target;
	return ref;
}


daeMetaElement *
domProfile_CG::domTechnique::domPass::domShader::domCompiler_target::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "compiler_target" );
	_Meta->setStaticPointerAddress(&domProfile_CG::domTechnique::domPass::domShader::domCompiler_target::_Meta);
	_Meta->registerConstructor(domProfile_CG::domTechnique::domPass::domShader::domCompiler_target::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNMTOKEN"));
		ma->setOffset( daeOffsetOf( domProfile_CG::domTechnique::domPass::domShader::domCompiler_target , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_CG::domTechnique::domPass::domShader::domCompiler_target));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_CG::domTechnique::domPass::domShader::domCompiler_options::create(daeInt bytes)
{
	domProfile_CG::domTechnique::domPass::domShader::domCompiler_optionsRef ref = new(bytes) domProfile_CG::domTechnique::domPass::domShader::domCompiler_options;
	return ref;
}


daeMetaElement *
domProfile_CG::domTechnique::domPass::domShader::domCompiler_options::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "compiler_options" );
	_Meta->setStaticPointerAddress(&domProfile_CG::domTechnique::domPass::domShader::domCompiler_options::_Meta);
	_Meta->registerConstructor(domProfile_CG::domTechnique::domPass::domShader::domCompiler_options::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsString"));
		ma->setOffset( daeOffsetOf( domProfile_CG::domTechnique::domPass::domShader::domCompiler_options , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_CG::domTechnique::domPass::domShader::domCompiler_options));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_CG::domTechnique::domPass::domShader::domName::create(daeInt bytes)
{
	domProfile_CG::domTechnique::domPass::domShader::domNameRef ref = new(bytes) domProfile_CG::domTechnique::domPass::domShader::domName;
	return ref;
}


daeMetaElement *
domProfile_CG::domTechnique::domPass::domShader::domName::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "name" );
	_Meta->setStaticPointerAddress(&domProfile_CG::domTechnique::domPass::domShader::domName::_Meta);
	_Meta->registerConstructor(domProfile_CG::domTechnique::domPass::domShader::domName::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domProfile_CG::domTechnique::domPass::domShader::domName , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: source
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "source" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domProfile_CG::domTechnique::domPass::domShader::domName , attrSource ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_CG::domTechnique::domPass::domShader::domName));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_CG::domTechnique::domPass::domShader::domBind::create(daeInt bytes)
{
	domProfile_CG::domTechnique::domPass::domShader::domBindRef ref = new(bytes) domProfile_CG::domTechnique::domPass::domShader::domBind;
	return ref;
}


daeMetaElement *
domProfile_CG::domTechnique::domPass::domShader::domBind::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "bind" );
	_Meta->setStaticPointerAddress(&domProfile_CG::domTechnique::domPass::domShader::domBind::_Meta);
	_Meta->registerConstructor(domProfile_CG::domTechnique::domPass::domShader::domBind::create);

	// Add elements: cg_param_type, param
    _Meta->appendElement(domCg_param_type::registerElement(),daeOffsetOf(domProfile_CG::domTechnique::domPass::domShader::domBind,elemCg_param_type));
	_Meta->appendPossibleChild( "bool", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool1x1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool1x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool1x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool1x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool2x1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool2x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool2x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool2x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool3x1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool3x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool3x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool3x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool4x1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool4x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool4x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "bool4x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float1x1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float1x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float1x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float1x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float2x1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float2x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float2x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float2x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float3x1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float3x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float3x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float3x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float4x1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float4x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float4x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "float4x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int1x1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int1x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int1x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int1x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int2x1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int2x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int2x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int2x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int3x1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int3x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int3x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int3x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int4x1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int4x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int4x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "int4x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "half", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "half1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "half2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "half3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "half4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "half1x1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "half1x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "half1x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "half1x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "half2x1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "half2x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "half2x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "half2x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "half3x1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "half3x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "half3x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "half3x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "half4x1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "half4x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "half4x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "half4x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "fixed", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "fixed1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "fixed2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "fixed3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "fixed4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "fixed1x1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "fixed1x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "fixed1x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "fixed1x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "fixed2x1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "fixed2x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "fixed2x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "fixed2x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "fixed3x1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "fixed3x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "fixed3x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "fixed3x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "fixed4x1", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "fixed4x2", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "fixed4x3", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "fixed4x4", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "surface", _Meta->getMetaElements()[0], "cg_surface_type");
	_Meta->appendPossibleChild( "sampler1D", _Meta->getMetaElements()[0], "cg_sampler1D");
	_Meta->appendPossibleChild( "sampler2D", _Meta->getMetaElements()[0], "cg_sampler2D");
	_Meta->appendPossibleChild( "sampler3D", _Meta->getMetaElements()[0], "cg_sampler3D");
	_Meta->appendPossibleChild( "samplerRECT", _Meta->getMetaElements()[0], "cg_samplerRECT");
	_Meta->appendPossibleChild( "samplerCUBE", _Meta->getMetaElements()[0], "cg_samplerCUBE");
	_Meta->appendPossibleChild( "samplerDEPTH", _Meta->getMetaElements()[0], "cg_samplerDEPTH");
	_Meta->appendPossibleChild( "string", _Meta->getMetaElements()[0]);
	_Meta->appendPossibleChild( "enum", _Meta->getMetaElements()[0]);
    _Meta->appendElement(domProfile_CG::domTechnique::domPass::domShader::domBind::domParam::registerElement(),daeOffsetOf(domProfile_CG::domTechnique::domPass::domShader::domBind,elemParam));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domProfile_CG::domTechnique::domPass::domShader::domBind,_contents));


	//	Add attribute: symbol
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "symbol" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domProfile_CG::domTechnique::domPass::domShader::domBind , attrSymbol ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_CG::domTechnique::domPass::domShader::domBind));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_CG::domTechnique::domPass::domShader::domBind::domParam::create(daeInt bytes)
{
	domProfile_CG::domTechnique::domPass::domShader::domBind::domParamRef ref = new(bytes) domProfile_CG::domTechnique::domPass::domShader::domBind::domParam;
	return ref;
}


daeMetaElement *
domProfile_CG::domTechnique::domPass::domShader::domBind::domParam::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "param" );
	_Meta->setStaticPointerAddress(&domProfile_CG::domTechnique::domPass::domShader::domBind::domParam::_Meta);
	_Meta->registerConstructor(domProfile_CG::domTechnique::domPass::domShader::domBind::domParam::create);


	//	Add attribute: ref
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domProfile_CG::domTechnique::domPass::domShader::domBind::domParam , attrRef ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_CG::domTechnique::domPass::domShader::domBind::domParam));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domProfile_CG::_Meta = NULL;
daeMetaElement * domProfile_CG::domTechnique::_Meta = NULL;
daeMetaElement * domProfile_CG::domTechnique::domPass::_Meta = NULL;
daeMetaElement * domProfile_CG::domTechnique::domPass::domDraw::_Meta = NULL;
daeMetaElement * domProfile_CG::domTechnique::domPass::domShader::_Meta = NULL;
daeMetaElement * domProfile_CG::domTechnique::domPass::domShader::domCompiler_target::_Meta = NULL;
daeMetaElement * domProfile_CG::domTechnique::domPass::domShader::domCompiler_options::_Meta = NULL;
daeMetaElement * domProfile_CG::domTechnique::domPass::domShader::domName::_Meta = NULL;
daeMetaElement * domProfile_CG::domTechnique::domPass::domShader::domBind::_Meta = NULL;
daeMetaElement * domProfile_CG::domTechnique::domPass::domShader::domBind::domParam::_Meta = NULL;


