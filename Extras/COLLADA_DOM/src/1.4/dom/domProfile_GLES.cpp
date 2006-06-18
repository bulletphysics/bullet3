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
#include <dom/domProfile_GLES.h>

daeElementRef
domProfile_GLES::create(daeInt bytes)
{
	domProfile_GLESRef ref = new(bytes) domProfile_GLES;
	return ref;
}


daeMetaElement *
domProfile_GLES::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "profile_GLES" );
	_Meta->setStaticPointerAddress(&domProfile_GLES::_Meta);
	_Meta->registerConstructor(domProfile_GLES::create);

	// Add elements: image, newparam, technique
    _Meta->appendArrayElement(domImage::registerElement(),daeOffsetOf(domProfile_GLES,elemImage_array));
    _Meta->appendArrayElement(domGles_newparam::registerElement(),daeOffsetOf(domProfile_GLES,elemNewparam_array),"newparam"); 
    _Meta->appendArrayElement(domProfile_GLES::domTechnique::registerElement(),daeOffsetOf(domProfile_GLES,elemTechnique_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domProfile_GLES,_contents));

	
	
	_Meta->setElementSize(sizeof(domProfile_GLES));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_GLES::domTechnique::create(daeInt bytes)
{
	domProfile_GLES::domTechniqueRef ref = new(bytes) domProfile_GLES::domTechnique;
	return ref;
}


daeMetaElement *
domProfile_GLES::domTechnique::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "technique" );
	_Meta->setStaticPointerAddress(&domProfile_GLES::domTechnique::_Meta);
	_Meta->registerConstructor(domProfile_GLES::domTechnique::create);

	// Add elements: asset, annotate, image, newparam, setparam, pass
    _Meta->appendElement(domAsset::registerElement(),daeOffsetOf(domProfile_GLES::domTechnique,elemAsset));
    _Meta->appendElement(domFx_annotate_common::registerElement(),daeOffsetOf(domProfile_GLES::domTechnique,elemAnnotate),"annotate"); 
    _Meta->appendArrayElement(domImage::registerElement(),daeOffsetOf(domProfile_GLES::domTechnique,elemImage_array));
    _Meta->appendArrayElement(domGles_newparam::registerElement(),daeOffsetOf(domProfile_GLES::domTechnique,elemNewparam_array),"newparam"); 
    _Meta->appendArrayElement(domProfile_GLES::domTechnique::domSetparam::registerElement(),daeOffsetOf(domProfile_GLES::domTechnique,elemSetparam_array));
    _Meta->appendArrayElement(domProfile_GLES::domTechnique::domPass::registerElement(),daeOffsetOf(domProfile_GLES::domTechnique,elemPass_array));
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domProfile_GLES::domTechnique,_contents));


	//	Add attribute: id
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "id" );
		ma->setType( daeAtomicType::get("xsID"));
		ma->setOffset( daeOffsetOf( domProfile_GLES::domTechnique , attrId ));
		ma->setContainer( _Meta );
	
		_Meta->appendAttribute(ma);
	}

	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domProfile_GLES::domTechnique , attrSid ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_GLES::domTechnique));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_GLES::domTechnique::domSetparam::create(daeInt bytes)
{
	domProfile_GLES::domTechnique::domSetparamRef ref = new(bytes) domProfile_GLES::domTechnique::domSetparam;
	return ref;
}


daeMetaElement *
domProfile_GLES::domTechnique::domSetparam::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "setparam" );
	_Meta->setStaticPointerAddress(&domProfile_GLES::domTechnique::domSetparam::_Meta);
	_Meta->registerConstructor(domProfile_GLES::domTechnique::domSetparam::create);

	// Add elements: annotate, gles_basic_type_common
    _Meta->appendArrayElement(domFx_annotate_common::registerElement(),daeOffsetOf(domProfile_GLES::domTechnique::domSetparam,elemAnnotate_array),"annotate"); 
    _Meta->appendElement(domGles_basic_type_common::registerElement(),daeOffsetOf(domProfile_GLES::domTechnique::domSetparam,elemGles_basic_type_common));
	_Meta->appendPossibleChild( "bool", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "bool4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "int4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float1x1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float1x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float1x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float1x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float2x1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float2x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float2x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float2x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float3x1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float3x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float3x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float3x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float4x1", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float4x2", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float4x3", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "float4x4", _Meta->getMetaElements()[1]);
	_Meta->appendPossibleChild( "surface", _Meta->getMetaElements()[1], "fx_surface_common");
	_Meta->appendPossibleChild( "texture_pipeline", _Meta->getMetaElements()[1], "gles_texture_pipeline");
	_Meta->appendPossibleChild( "sampler_state", _Meta->getMetaElements()[1], "gles_sampler_state");
	_Meta->appendPossibleChild( "texture_unit", _Meta->getMetaElements()[1], "gles_texture_unit");
	_Meta->appendPossibleChild( "enum", _Meta->getMetaElements()[1]);

	//	Add attribute: ref
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "ref" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domProfile_GLES::domTechnique::domSetparam , attrRef ));
		ma->setContainer( _Meta );
		ma->setIsRequired( true );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_GLES::domTechnique::domSetparam));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_GLES::domTechnique::domPass::create(daeInt bytes)
{
	domProfile_GLES::domTechnique::domPassRef ref = new(bytes) domProfile_GLES::domTechnique::domPass;
	return ref;
}


daeMetaElement *
domProfile_GLES::domTechnique::domPass::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "pass" );
	_Meta->setStaticPointerAddress(&domProfile_GLES::domTechnique::domPass::_Meta);
	_Meta->registerConstructor(domProfile_GLES::domTechnique::domPass::create);

	// Add elements: annotate, color_target, depth_target, stencil_target, color_clear, depth_clear, stencil_clear, draw, gles_pipeline_settings
    _Meta->appendArrayElement(domFx_annotate_common::registerElement(),daeOffsetOf(domProfile_GLES::domTechnique::domPass,elemAnnotate_array),"annotate"); 
    _Meta->appendElement(domProfile_GLES::domTechnique::domPass::domColor_target::registerElement(),daeOffsetOf(domProfile_GLES::domTechnique::domPass,elemColor_target));
    _Meta->appendElement(domProfile_GLES::domTechnique::domPass::domDepth_target::registerElement(),daeOffsetOf(domProfile_GLES::domTechnique::domPass,elemDepth_target));
    _Meta->appendElement(domProfile_GLES::domTechnique::domPass::domStencil_target::registerElement(),daeOffsetOf(domProfile_GLES::domTechnique::domPass,elemStencil_target));
    _Meta->appendElement(domProfile_GLES::domTechnique::domPass::domColor_clear::registerElement(),daeOffsetOf(domProfile_GLES::domTechnique::domPass,elemColor_clear));
    _Meta->appendElement(domProfile_GLES::domTechnique::domPass::domDepth_clear::registerElement(),daeOffsetOf(domProfile_GLES::domTechnique::domPass,elemDepth_clear));
    _Meta->appendElement(domProfile_GLES::domTechnique::domPass::domStencil_clear::registerElement(),daeOffsetOf(domProfile_GLES::domTechnique::domPass,elemStencil_clear));
    _Meta->appendElement(domProfile_GLES::domTechnique::domPass::domDraw::registerElement(),daeOffsetOf(domProfile_GLES::domTechnique::domPass,elemDraw));
    _Meta->appendArrayElement(domGles_pipeline_settings::registerElement(),daeOffsetOf(domProfile_GLES::domTechnique::domPass,elemGles_pipeline_settings_array));
	_Meta->appendPossibleChild( "alpha_func", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "blend_func", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "clear_color", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "clear_stencil", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "clear_depth", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "clip_plane", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "color_mask", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "cull_face", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "depth_func", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "depth_mask", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "depth_range", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "fog_color", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "fog_density", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "fog_mode", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "fog_start", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "fog_end", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "front_face", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "texture_pipeline", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "logic_op", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_ambient", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_diffuse", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_specular", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_position", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_constant_attenuation", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_linear_attenutation", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_quadratic_attenuation", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_spot_cutoff", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_spot_direction", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_spot_exponent", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_model_ambient", _Meta->getMetaElements()[8]);
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
	_Meta->appendPossibleChild( "shade_model", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "stencil_func", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "stencil_mask", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "stencil_op", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "alpha_test_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "blend_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "clip_plane_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "color_logic_op_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "color_material_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "cull_face_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "depth_test_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "dither_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "fog_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "texture_pipeline_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "lighting_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "light_model_two_side_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "line_smooth_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "multisample_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "normalize_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "point_smooth_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "polygon_offset_fill_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "rescale_normal_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "sample_alpha_to_coverage_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "sample_alpha_to_one_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "sample_coverage_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "scissor_test_enable", _Meta->getMetaElements()[8]);
	_Meta->appendPossibleChild( "stencil_test_enable", _Meta->getMetaElements()[8]);
	// Ordered list of sub-elements
    _Meta->addContents(daeOffsetOf(domProfile_GLES::domTechnique::domPass,_contents));


	//	Add attribute: sid
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "sid" );
		ma->setType( daeAtomicType::get("xsNCName"));
		ma->setOffset( daeOffsetOf( domProfile_GLES::domTechnique::domPass , attrSid ));
		ma->setContainer( _Meta );
		ma->setIsRequired( false );
	
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_GLES::domTechnique::domPass));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_GLES::domTechnique::domPass::domColor_target::create(daeInt bytes)
{
	domProfile_GLES::domTechnique::domPass::domColor_targetRef ref = new(bytes) domProfile_GLES::domTechnique::domPass::domColor_target;
	return ref;
}


daeMetaElement *
domProfile_GLES::domTechnique::domPass::domColor_target::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "color_target" );
	_Meta->setStaticPointerAddress(&domProfile_GLES::domTechnique::domPass::domColor_target::_Meta);
	_Meta->registerConstructor(domProfile_GLES::domTechnique::domPass::domColor_target::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Gles_rendertarget_common"));
		ma->setOffset( daeOffsetOf( domProfile_GLES::domTechnique::domPass::domColor_target , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_GLES::domTechnique::domPass::domColor_target));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_GLES::domTechnique::domPass::domDepth_target::create(daeInt bytes)
{
	domProfile_GLES::domTechnique::domPass::domDepth_targetRef ref = new(bytes) domProfile_GLES::domTechnique::domPass::domDepth_target;
	return ref;
}


daeMetaElement *
domProfile_GLES::domTechnique::domPass::domDepth_target::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "depth_target" );
	_Meta->setStaticPointerAddress(&domProfile_GLES::domTechnique::domPass::domDepth_target::_Meta);
	_Meta->registerConstructor(domProfile_GLES::domTechnique::domPass::domDepth_target::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Gles_rendertarget_common"));
		ma->setOffset( daeOffsetOf( domProfile_GLES::domTechnique::domPass::domDepth_target , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_GLES::domTechnique::domPass::domDepth_target));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_GLES::domTechnique::domPass::domStencil_target::create(daeInt bytes)
{
	domProfile_GLES::domTechnique::domPass::domStencil_targetRef ref = new(bytes) domProfile_GLES::domTechnique::domPass::domStencil_target;
	return ref;
}


daeMetaElement *
domProfile_GLES::domTechnique::domPass::domStencil_target::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "stencil_target" );
	_Meta->setStaticPointerAddress(&domProfile_GLES::domTechnique::domPass::domStencil_target::_Meta);
	_Meta->registerConstructor(domProfile_GLES::domTechnique::domPass::domStencil_target::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Gles_rendertarget_common"));
		ma->setOffset( daeOffsetOf( domProfile_GLES::domTechnique::domPass::domStencil_target , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_GLES::domTechnique::domPass::domStencil_target));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_GLES::domTechnique::domPass::domColor_clear::create(daeInt bytes)
{
	domProfile_GLES::domTechnique::domPass::domColor_clearRef ref = new(bytes) domProfile_GLES::domTechnique::domPass::domColor_clear;
	return ref;
}


daeMetaElement *
domProfile_GLES::domTechnique::domPass::domColor_clear::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "color_clear" );
	_Meta->setStaticPointerAddress(&domProfile_GLES::domTechnique::domPass::domColor_clear::_Meta);
	_Meta->registerConstructor(domProfile_GLES::domTechnique::domPass::domColor_clear::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaArrayAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_color_common"));
		ma->setOffset( daeOffsetOf( domProfile_GLES::domTechnique::domPass::domColor_clear , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_GLES::domTechnique::domPass::domColor_clear));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_GLES::domTechnique::domPass::domDepth_clear::create(daeInt bytes)
{
	domProfile_GLES::domTechnique::domPass::domDepth_clearRef ref = new(bytes) domProfile_GLES::domTechnique::domPass::domDepth_clear;
	return ref;
}


daeMetaElement *
domProfile_GLES::domTechnique::domPass::domDepth_clear::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "depth_clear" );
	_Meta->setStaticPointerAddress(&domProfile_GLES::domTechnique::domPass::domDepth_clear::_Meta);
	_Meta->registerConstructor(domProfile_GLES::domTechnique::domPass::domDepth_clear::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Float"));
		ma->setOffset( daeOffsetOf( domProfile_GLES::domTechnique::domPass::domDepth_clear , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_GLES::domTechnique::domPass::domDepth_clear));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_GLES::domTechnique::domPass::domStencil_clear::create(daeInt bytes)
{
	domProfile_GLES::domTechnique::domPass::domStencil_clearRef ref = new(bytes) domProfile_GLES::domTechnique::domPass::domStencil_clear;
	return ref;
}


daeMetaElement *
domProfile_GLES::domTechnique::domPass::domStencil_clear::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "stencil_clear" );
	_Meta->setStaticPointerAddress(&domProfile_GLES::domTechnique::domPass::domStencil_clear::_Meta);
	_Meta->registerConstructor(domProfile_GLES::domTechnique::domPass::domStencil_clear::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("xsByte"));
		ma->setOffset( daeOffsetOf( domProfile_GLES::domTechnique::domPass::domStencil_clear , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_GLES::domTechnique::domPass::domStencil_clear));
	_Meta->validate();

	return _Meta;
}

daeElementRef
domProfile_GLES::domTechnique::domPass::domDraw::create(daeInt bytes)
{
	domProfile_GLES::domTechnique::domPass::domDrawRef ref = new(bytes) domProfile_GLES::domTechnique::domPass::domDraw;
	return ref;
}


daeMetaElement *
domProfile_GLES::domTechnique::domPass::domDraw::registerElement()
{
    if ( _Meta != NULL ) return _Meta;
    
    _Meta = new daeMetaElement;
    _Meta->setName( "draw" );
	_Meta->setStaticPointerAddress(&domProfile_GLES::domTechnique::domPass::domDraw::_Meta);
	_Meta->registerConstructor(domProfile_GLES::domTechnique::domPass::domDraw::create);

	//	Add attribute: _value
 	{
		daeMetaAttribute *ma = new daeMetaAttribute;
		ma->setName( "_value" );
		ma->setType( daeAtomicType::get("Fx_draw_common"));
		ma->setOffset( daeOffsetOf( domProfile_GLES::domTechnique::domPass::domDraw , _value ));
		ma->setContainer( _Meta );
		_Meta->appendAttribute(ma);
	}
	
	
	_Meta->setElementSize(sizeof(domProfile_GLES::domTechnique::domPass::domDraw));
	_Meta->validate();

	return _Meta;
}


daeMetaElement * domProfile_GLES::_Meta = NULL;
daeMetaElement * domProfile_GLES::domTechnique::_Meta = NULL;
daeMetaElement * domProfile_GLES::domTechnique::domSetparam::_Meta = NULL;
daeMetaElement * domProfile_GLES::domTechnique::domPass::_Meta = NULL;
daeMetaElement * domProfile_GLES::domTechnique::domPass::domColor_target::_Meta = NULL;
daeMetaElement * domProfile_GLES::domTechnique::domPass::domDepth_target::_Meta = NULL;
daeMetaElement * domProfile_GLES::domTechnique::domPass::domStencil_target::_Meta = NULL;
daeMetaElement * domProfile_GLES::domTechnique::domPass::domColor_clear::_Meta = NULL;
daeMetaElement * domProfile_GLES::domTechnique::domPass::domDepth_clear::_Meta = NULL;
daeMetaElement * domProfile_GLES::domTechnique::domPass::domStencil_clear::_Meta = NULL;
daeMetaElement * domProfile_GLES::domTechnique::domPass::domDraw::_Meta = NULL;


