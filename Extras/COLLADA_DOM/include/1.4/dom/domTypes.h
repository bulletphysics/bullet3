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

#ifndef __DOM_TYPES_H__
#define __DOM_TYPES_H__

#include <dae/daeDomTypes.h>

// Register all types
extern void registerDomTypes();

typedef xsBoolean		domBool;
typedef xsDateTime		domDateTime;
typedef xsDouble		domFloat;
typedef xsLong		domInt;
typedef xsName		domName;
typedef xsString		domString;
typedef xsToken		domToken;
typedef xsUnsignedLong		domUint;
typedef daeTArray<domBool>		domListOfBools;
typedef daeTArray<domFloat>		domListOfFloats;
typedef xsHexBinaryArray		domListOfHexBinary;
typedef daeTArray<domInt>		domListOfInts;
typedef daeTArray<domName>		domListOfNames;
typedef daeTArray<domToken>		domListOfTokens;
typedef daeTArray<domUint>		domListOfUInts;
typedef domListOfBools		domBool2;
typedef domListOfBools		domBool3;
typedef domListOfBools		domBool4;
typedef domListOfFloats		domFloat2;
typedef domListOfFloats		domFloat3;
typedef domListOfFloats		domFloat4;
typedef domListOfFloats		domFloat7;
typedef domListOfFloats		domFloat2x2;
typedef domListOfFloats		domFloat3x3;
typedef domListOfFloats		domFloat4x4;
typedef domListOfFloats		domFloat2x3;
typedef domListOfFloats		domFloat2x4;
typedef domListOfFloats		domFloat3x2;
typedef domListOfFloats		domFloat3x4;
typedef domListOfFloats		domFloat4x2;
typedef domListOfFloats		domFloat4x3;
typedef domListOfInts		domInt2;
typedef domListOfInts		domInt3;
typedef domListOfInts		domInt4;
typedef domListOfInts		domInt2x2;
typedef domListOfInts		domInt3x3;
typedef domListOfInts		domInt4x4;
/**
 * This type is used for URI reference which can only reference a resource
 * declared within it's same document.
 */
typedef xsAnyURI		domURIFragmentType;
typedef domFloat4		domFx_color_common;
typedef xsString		domFx_draw_common;
typedef xsNonNegativeInteger		domGL_MAX_LIGHTS_index;
typedef xsNonNegativeInteger		domGL_MAX_CLIP_PLANES_index;
typedef xsNonNegativeInteger		domGL_MAX_TEXTURE_IMAGE_UNITS_index;
typedef xsFloat		domGl_alpha_value_type;
typedef xsFloat		domGlsl_float;
typedef xsInt		domGlsl_int;
typedef xsBoolean		domGlsl_bool;
typedef daeTArray<domGlsl_bool>		domGlsl_ListOfBool;
typedef daeTArray<domGlsl_float>		domGlsl_ListOfFloat;
typedef daeTArray<domGlsl_int>		domGlsl_ListOfInt;
typedef domGlsl_ListOfBool		domGlsl_bool2;
typedef domGlsl_ListOfBool		domGlsl_bool3;
typedef domGlsl_ListOfBool		domGlsl_bool4;
typedef domGlsl_ListOfFloat		domGlsl_float2;
typedef domGlsl_ListOfFloat		domGlsl_float3;
typedef domGlsl_ListOfFloat		domGlsl_float4;
typedef domGlsl_ListOfFloat		domGlsl_float2x2;
typedef domGlsl_ListOfFloat		domGlsl_float3x3;
typedef domGlsl_ListOfFloat		domGlsl_float4x4;
typedef domGlsl_ListOfInt		domGlsl_int2;
typedef domGlsl_ListOfInt		domGlsl_int3;
typedef domGlsl_ListOfInt		domGlsl_int4;
typedef xsString		domGlsl_identifier;
typedef xsBoolean		domCg_bool;
typedef xsFloat		domCg_float;
typedef xsInt		domCg_int;
typedef xsFloat		domCg_half;
typedef xsFloat		domCg_fixed;
typedef xsBoolean		domCg_bool1;
typedef xsFloat		domCg_float1;
typedef xsInt		domCg_int1;
typedef xsFloat		domCg_half1;
typedef xsFloat		domCg_fixed1;
typedef daeTArray<domCg_bool>		domCg_ListOfBool;
typedef daeTArray<domCg_float>		domCg_ListOfFloat;
typedef daeTArray<domCg_int>		domCg_ListOfInt;
typedef daeTArray<domCg_half>		domCg_ListOfHalf;
typedef daeTArray<domCg_fixed>		domCg_ListOfFixed;
typedef domCg_ListOfBool		domCg_bool2;
typedef domCg_ListOfBool		domCg_bool3;
typedef domCg_ListOfBool		domCg_bool4;
typedef domCg_ListOfBool		domCg_bool1x1;
typedef domCg_ListOfBool		domCg_bool1x2;
typedef domCg_ListOfBool		domCg_bool1x3;
typedef domCg_ListOfBool		domCg_bool1x4;
typedef domCg_ListOfBool		domCg_bool2x1;
typedef domCg_ListOfBool		domCg_bool2x2;
typedef domCg_ListOfBool		domCg_bool2x3;
typedef domCg_ListOfBool		domCg_bool2x4;
typedef domCg_ListOfBool		domCg_bool3x1;
typedef domCg_ListOfBool		domCg_bool3x2;
typedef domCg_ListOfBool		domCg_bool3x3;
typedef domCg_ListOfBool		domCg_bool3x4;
typedef domCg_ListOfBool		domCg_bool4x1;
typedef domCg_ListOfBool		domCg_bool4x2;
typedef domCg_ListOfBool		domCg_bool4x3;
typedef domCg_ListOfBool		domCg_bool4x4;
typedef domCg_ListOfFloat		domCg_float2;
typedef domCg_ListOfFloat		domCg_float3;
typedef domCg_ListOfFloat		domCg_float4;
typedef domCg_ListOfFloat		domCg_float1x1;
typedef domCg_ListOfFloat		domCg_float1x2;
typedef domCg_ListOfFloat		domCg_float1x3;
typedef domCg_ListOfFloat		domCg_float1x4;
typedef domCg_ListOfFloat		domCg_float2x1;
typedef domCg_ListOfFloat		domCg_float2x2;
typedef domCg_ListOfFloat		domCg_float2x3;
typedef domCg_ListOfFloat		domCg_float2x4;
typedef domCg_ListOfFloat		domCg_float3x1;
typedef domCg_ListOfFloat		domCg_float3x2;
typedef domCg_ListOfFloat		domCg_float3x3;
typedef domCg_ListOfFloat		domCg_float3x4;
typedef domCg_ListOfFloat		domCg_float4x1;
typedef domCg_ListOfFloat		domCg_float4x2;
typedef domCg_ListOfFloat		domCg_float4x3;
typedef domCg_ListOfFloat		domCg_float4x4;
typedef domCg_ListOfInt		domCg_int2;
typedef domCg_ListOfInt		domCg_int3;
typedef domCg_ListOfInt		domCg_int4;
typedef domCg_ListOfInt		domCg_int1x1;
typedef domCg_ListOfInt		domCg_int1x2;
typedef domCg_ListOfInt		domCg_int1x3;
typedef domCg_ListOfInt		domCg_int1x4;
typedef domCg_ListOfInt		domCg_int2x1;
typedef domCg_ListOfInt		domCg_int2x2;
typedef domCg_ListOfInt		domCg_int2x3;
typedef domCg_ListOfInt		domCg_int2x4;
typedef domCg_ListOfInt		domCg_int3x1;
typedef domCg_ListOfInt		domCg_int3x2;
typedef domCg_ListOfInt		domCg_int3x3;
typedef domCg_ListOfInt		domCg_int3x4;
typedef domCg_ListOfInt		domCg_int4x1;
typedef domCg_ListOfInt		domCg_int4x2;
typedef domCg_ListOfInt		domCg_int4x3;
typedef domCg_ListOfInt		domCg_int4x4;
typedef domCg_ListOfHalf		domCg_half2;
typedef domCg_ListOfHalf		domCg_half3;
typedef domCg_ListOfHalf		domCg_half4;
typedef domCg_ListOfHalf		domCg_half1x1;
typedef domCg_ListOfHalf		domCg_half1x2;
typedef domCg_ListOfHalf		domCg_half1x3;
typedef domCg_ListOfHalf		domCg_half1x4;
typedef domCg_ListOfHalf		domCg_half2x1;
typedef domCg_ListOfHalf		domCg_half2x2;
typedef domCg_ListOfHalf		domCg_half2x3;
typedef domCg_ListOfHalf		domCg_half2x4;
typedef domCg_ListOfHalf		domCg_half3x1;
typedef domCg_ListOfHalf		domCg_half3x2;
typedef domCg_ListOfHalf		domCg_half3x3;
typedef domCg_ListOfHalf		domCg_half3x4;
typedef domCg_ListOfHalf		domCg_half4x1;
typedef domCg_ListOfHalf		domCg_half4x2;
typedef domCg_ListOfHalf		domCg_half4x3;
typedef domCg_ListOfHalf		domCg_half4x4;
typedef domCg_ListOfFixed		domCg_fixed2;
typedef domCg_ListOfFixed		domCg_fixed3;
typedef domCg_ListOfFixed		domCg_fixed4;
typedef domCg_ListOfFixed		domCg_fixed1x1;
typedef domCg_ListOfFixed		domCg_fixed1x2;
typedef domCg_ListOfFixed		domCg_fixed1x3;
typedef domCg_ListOfFixed		domCg_fixed1x4;
typedef domCg_ListOfFixed		domCg_fixed2x1;
typedef domCg_ListOfFixed		domCg_fixed2x2;
typedef domCg_ListOfFixed		domCg_fixed2x3;
typedef domCg_ListOfFixed		domCg_fixed2x4;
typedef domCg_ListOfFixed		domCg_fixed3x1;
typedef domCg_ListOfFixed		domCg_fixed3x2;
typedef domCg_ListOfFixed		domCg_fixed3x3;
typedef domCg_ListOfFixed		domCg_fixed3x4;
typedef domCg_ListOfFixed		domCg_fixed4x1;
typedef domCg_ListOfFixed		domCg_fixed4x2;
typedef domCg_ListOfFixed		domCg_fixed4x3;
typedef domCg_ListOfFixed		domCg_fixed4x4;
typedef xsString		domCg_identifier;
typedef xsNonNegativeInteger		domGLES_MAX_LIGHTS_index;
typedef xsNonNegativeInteger		domGLES_MAX_CLIP_PLANES_index;
typedef xsNonNegativeInteger		domGLES_MAX_TEXTURE_COORDS_index;
typedef xsNonNegativeInteger		domGLES_MAX_TEXTURE_IMAGE_UNITS_index;
typedef xsNonNegativeInteger		domGles_texcombiner_argument_index_type;
typedef xsNCName		domGles_rendertarget_common;

/**
 * An enumuerated type specifying the acceptable morph methods.
 */
enum domMorphMethodType {
	MORPHMETHODTYPE_NORMALIZED,
	MORPHMETHODTYPE_RELATIVE,
	MORPHMETHODTYPE_COUNT = 2
};

/**
 * An enumerated type specifying the acceptable node types.
 */
enum domNodeType {
	NODETYPE_JOINT,
	NODETYPE_NODE,
	NODETYPE_COUNT = 2
};

/**
 * An enumerated type specifying the acceptable up-axis values.
 */
enum domUpAxisType {
	UPAXISTYPE_X_UP,
	UPAXISTYPE_Y_UP,
	UPAXISTYPE_Z_UP,
	UPAXISTYPE_COUNT = 3
};

/**
 * An enumerated type specifying the acceptable document versions.
 */
enum domVersionType {
	VERSIONTYPE_1_4_0,
	VERSIONTYPE_COUNT = 1
};

enum domFx_surface_type_enum {
	FX_SURFACE_TYPE_ENUM_UNTYPED,
	FX_SURFACE_TYPE_ENUM_1D,
	FX_SURFACE_TYPE_ENUM_2D,
	FX_SURFACE_TYPE_ENUM_3D,
	FX_SURFACE_TYPE_ENUM_RECT,
	FX_SURFACE_TYPE_ENUM_CUBE,
	FX_SURFACE_TYPE_ENUM_DEPTH,
	FX_SURFACE_TYPE_ENUM_COUNT = 7
};

enum domFx_surface_face_enum {
	FX_SURFACE_FACE_ENUM_POSITIVE_X,
	FX_SURFACE_FACE_ENUM_NEGATIVE_X,
	FX_SURFACE_FACE_ENUM_POSITIVE_Y,
	FX_SURFACE_FACE_ENUM_NEGATIVE_Y,
	FX_SURFACE_FACE_ENUM_POSITIVE_Z,
	FX_SURFACE_FACE_ENUM_NEGATIVE_Z,
	FX_SURFACE_FACE_ENUM_COUNT = 6
};

enum domFx_sampler_wrap_common {
	FX_SAMPLER_WRAP_COMMON_NONE,
	FX_SAMPLER_WRAP_COMMON_WRAP,
	FX_SAMPLER_WRAP_COMMON_MIRROR,
	FX_SAMPLER_WRAP_COMMON_CLAMP,
	FX_SAMPLER_WRAP_COMMON_BORDER,
	FX_SAMPLER_WRAP_COMMON_COUNT = 5
};

enum domFx_sampler_filter_common {
	FX_SAMPLER_FILTER_COMMON_NONE,
	FX_SAMPLER_FILTER_COMMON_NEAREST,
	FX_SAMPLER_FILTER_COMMON_LINEAR,
	FX_SAMPLER_FILTER_COMMON_NEAREST_MIPMAP_NEAREST,
	FX_SAMPLER_FILTER_COMMON_LINEAR_MIPMAP_NEAREST,
	FX_SAMPLER_FILTER_COMMON_NEAREST_MIPMAP_LINEAR,
	FX_SAMPLER_FILTER_COMMON_LINEAR_MIPMAP_LINEAR,
	FX_SAMPLER_FILTER_COMMON_COUNT = 7
};

enum domFx_modifier_enum_common {
	FX_MODIFIER_ENUM_COMMON_CONST,
	FX_MODIFIER_ENUM_COMMON_UNIFORM,
	FX_MODIFIER_ENUM_COMMON_VARYING,
	FX_MODIFIER_ENUM_COMMON_STATIC,
	FX_MODIFIER_ENUM_COMMON_VOLATILE,
	FX_MODIFIER_ENUM_COMMON_EXTERN,
	FX_MODIFIER_ENUM_COMMON_SHARED,
	FX_MODIFIER_ENUM_COMMON_COUNT = 7
};

enum domFx_pipeline_stage_common {
	FX_PIPELINE_STAGE_COMMON_VERTEXPROGRAM,
	FX_PIPELINE_STAGE_COMMON_FRAGMENTPROGRAM,
	FX_PIPELINE_STAGE_COMMON_VERTEXSHADER,
	FX_PIPELINE_STAGE_COMMON_PIXELSHADER,
	FX_PIPELINE_STAGE_COMMON_COUNT = 4
};

enum domGl_blend_type {
	GL_BLEND_TYPE_ZERO = 0x0,
	GL_BLEND_TYPE_ONE = 0x1,
	GL_BLEND_TYPE_SRC_COLOR = 0x0300,
	GL_BLEND_TYPE_ONE_MINUS_SRC_COLOR = 0x0301,
	GL_BLEND_TYPE_DEST_COLOR = 0x0306,
	GL_BLEND_TYPE_ONE_MINUS_DEST_COLOR = 0x0307,
	GL_BLEND_TYPE_SRC_ALPHA = 0x0302,
	GL_BLEND_TYPE_ONE_MINUS_SRC_ALPHA = 0x0303,
	GL_BLEND_TYPE_DST_ALPHA = 0x0304,
	GL_BLEND_TYPE_ONE_MINUS_DST_ALPHA = 0x0305,
	GL_BLEND_TYPE_CONSTANT_COLOR = 0x8001,
	GL_BLEND_TYPE_ONE_MINUS_CONSTANT_COLOR = 0x8002,
	GL_BLEND_TYPE_CONSTANT_ALPHA = 0x8003,
	GL_BLEND_TYPE_ONE_MINUS_CONSTANT_ALPHA = 0x8004,
	GL_BLEND_TYPE_SRC_ALPHA_SATURATE = 0x0308,
	GL_BLEND_TYPE_COUNT = 15
};

enum domGl_face_type {
	GL_FACE_TYPE_FRONT = 0x0404,
	GL_FACE_TYPE_BACK = 0x0405,
	GL_FACE_TYPE_FRONT_AND_BACK = 0x0408,
	GL_FACE_TYPE_COUNT = 3
};

enum domGl_blend_equation_type {
	GL_BLEND_EQUATION_TYPE_FUNC_ADD = 0x8006,
	GL_BLEND_EQUATION_TYPE_FUNC_SUBTRACT = 0x800A,
	GL_BLEND_EQUATION_TYPE_FUNC_REVERSE_SUBTRACT = 0x800B,
	GL_BLEND_EQUATION_TYPE_MIN = 0x8007,
	GL_BLEND_EQUATION_TYPE_MAX = 0x8008,
	GL_BLEND_EQUATION_TYPE_COUNT = 5
};

enum domGl_func_type {
	GL_FUNC_TYPE_NEVER = 0x0200,
	GL_FUNC_TYPE_LESS = 0x0201,
	GL_FUNC_TYPE_LEQUAL = 0x0203,
	GL_FUNC_TYPE_EQUAL = 0x0202,
	GL_FUNC_TYPE_GREATER = 0x0204,
	GL_FUNC_TYPE_NOTEQUAL = 0x0205,
	GL_FUNC_TYPE_GEQUAL = 0x0206,
	GL_FUNC_TYPE_ALWAYS = 0x0207,
	GL_FUNC_TYPE_COUNT = 8
};

enum domGl_stencil_op_type {
	GL_STENCIL_OP_TYPE_KEEP = 0x1E00,
	GL_STENCIL_OP_TYPE_ZERO = 0x0,
	GL_STENCIL_OP_TYPE_REPLACE = 0x1E01,
	GL_STENCIL_OP_TYPE_INCR = 0x1E02,
	GL_STENCIL_OP_TYPE_DECR = 0x1E03,
	GL_STENCIL_OP_TYPE_INVERT = 0x150A,
	GL_STENCIL_OP_TYPE_INCR_WRAP = 0x8507,
	GL_STENCIL_OP_TYPE_DECR_WRAP = 0x8508,
	GL_STENCIL_OP_TYPE_COUNT = 8
};

enum domGl_material_type {
	GL_MATERIAL_TYPE_EMISSION = 0x1600,
	GL_MATERIAL_TYPE_AMBIENT = 0x1200,
	GL_MATERIAL_TYPE_DIFFUSE = 0x1201,
	GL_MATERIAL_TYPE_SPECULAR = 0x1202,
	GL_MATERIAL_TYPE_AMBIENT_AND_DIFFUSE = 0x1602,
	GL_MATERIAL_TYPE_COUNT = 5
};

enum domGl_fog_type {
	GL_FOG_TYPE_LINEAR = 0x2601,
	GL_FOG_TYPE_EXP = 0x0800,
	GL_FOG_TYPE_EXP2 = 0x0801,
	GL_FOG_TYPE_COUNT = 3
};

enum domGl_fog_coord_src_type {
	GL_FOG_COORD_SRC_TYPE_FOG_COORDINATE = 0x8451,
	GL_FOG_COORD_SRC_TYPE_FRAGMENT_DEPTH = 0x8452,
	GL_FOG_COORD_SRC_TYPE_COUNT = 2
};

enum domGl_front_face_type {
	GL_FRONT_FACE_TYPE_CW = 0x0900,
	GL_FRONT_FACE_TYPE_CCW = 0x0901,
	GL_FRONT_FACE_TYPE_COUNT = 2
};

enum domGl_light_model_color_control_type {
	GL_LIGHT_MODEL_COLOR_CONTROL_TYPE_SINGLE_COLOR = 0x81F9,
	GL_LIGHT_MODEL_COLOR_CONTROL_TYPE_SEPARATE_SPECULAR_COLOR = 0x81FA,
	GL_LIGHT_MODEL_COLOR_CONTROL_TYPE_COUNT = 2
};

enum domGl_logic_op_type {
	GL_LOGIC_OP_TYPE_CLEAR = 0x1500,
	GL_LOGIC_OP_TYPE_AND = 0x1501,
	GL_LOGIC_OP_TYPE_AND_REVERSE = 0x1502,
	GL_LOGIC_OP_TYPE_COPY = 0x1503,
	GL_LOGIC_OP_TYPE_AND_INVERTED = 0x1504,
	GL_LOGIC_OP_TYPE_NOOP = 0x1505,
	GL_LOGIC_OP_TYPE_XOR = 0x1506,
	GL_LOGIC_OP_TYPE_OR = 0x1507,
	GL_LOGIC_OP_TYPE_NOR = 0x1508,
	GL_LOGIC_OP_TYPE_EQUIV = 0x1509,
	GL_LOGIC_OP_TYPE_INVERT = 0x150A,
	GL_LOGIC_OP_TYPE_OR_REVERSE = 0x150B,
	GL_LOGIC_OP_TYPE_COPY_INVERTED = 0x150C,
	GL_LOGIC_OP_TYPE_NAND = 0x150E,
	GL_LOGIC_OP_TYPE_SET = 0x150F,
	GL_LOGIC_OP_TYPE_COUNT = 15
};

enum domGl_polygon_mode_type {
	GL_POLYGON_MODE_TYPE_POINT = 0x1B00,
	GL_POLYGON_MODE_TYPE_LINE = 0x1B01,
	GL_POLYGON_MODE_TYPE_FILL = 0x1B02,
	GL_POLYGON_MODE_TYPE_COUNT = 3
};

enum domGl_shade_model_type {
	GL_SHADE_MODEL_TYPE_FLAT = 0x1D00,
	GL_SHADE_MODEL_TYPE_SMOOTH = 0x1D01,
	GL_SHADE_MODEL_TYPE_COUNT = 2
};

enum domGlsl_pipeline_stage {
	GLSL_PIPELINE_STAGE_VERTEXPROGRAM,
	GLSL_PIPELINE_STAGE_FRAGMENTPROGRAM,
	GLSL_PIPELINE_STAGE_COUNT = 2
};

enum domCg_pipeline_stage {
	CG_PIPELINE_STAGE_VERTEX,
	CG_PIPELINE_STAGE_FRAGMENT,
	CG_PIPELINE_STAGE_COUNT = 2
};

enum domGles_texenv_mode_enums {
	GLES_TEXENV_MODE_ENUMS_REPLACE = 0x1E01,
	GLES_TEXENV_MODE_ENUMS_MODULATE = 0x2100,
	GLES_TEXENV_MODE_ENUMS_DECAL = 0x2101,
	GLES_TEXENV_MODE_ENUMS_BLEND = 0x0BE2,
	GLES_TEXENV_MODE_ENUMS_ADD = 0x0104,
	GLES_TEXENV_MODE_ENUMS_COUNT = 5
};

enum domGles_texcombiner_operatorRGB_enums {
	GLES_TEXCOMBINER_OPERATORRGB_ENUMS_REPLACE = 0x1E01,
	GLES_TEXCOMBINER_OPERATORRGB_ENUMS_MODULATE = 0x2100,
	GLES_TEXCOMBINER_OPERATORRGB_ENUMS_ADD = 0x0104,
	GLES_TEXCOMBINER_OPERATORRGB_ENUMS_ADD_SIGNED = 0x8574,
	GLES_TEXCOMBINER_OPERATORRGB_ENUMS_INTERPOLATE = 0x8575,
	GLES_TEXCOMBINER_OPERATORRGB_ENUMS_SUBTRACT = 0x84E7,
	GLES_TEXCOMBINER_OPERATORRGB_ENUMS_DOT3_RGB = 0x86AE,
	GLES_TEXCOMBINER_OPERATORRGB_ENUMS_DOT3_RGBA = 0x86AF,
	GLES_TEXCOMBINER_OPERATORRGB_ENUMS_COUNT = 8
};

enum domGles_texcombiner_operatorAlpha_enums {
	GLES_TEXCOMBINER_OPERATORALPHA_ENUMS_REPLACE = 0x1E01,
	GLES_TEXCOMBINER_OPERATORALPHA_ENUMS_MODULATE = 0x2100,
	GLES_TEXCOMBINER_OPERATORALPHA_ENUMS_ADD = 0x0104,
	GLES_TEXCOMBINER_OPERATORALPHA_ENUMS_ADD_SIGNED = 0x8574,
	GLES_TEXCOMBINER_OPERATORALPHA_ENUMS_INTERPOLATE = 0x8575,
	GLES_TEXCOMBINER_OPERATORALPHA_ENUMS_SUBTRACT = 0x84E7,
	GLES_TEXCOMBINER_OPERATORALPHA_ENUMS_COUNT = 6
};

enum domGles_texcombiner_source_enums {
	GLES_TEXCOMBINER_SOURCE_ENUMS_TEXTURE = 0x1702,
	GLES_TEXCOMBINER_SOURCE_ENUMS_CONSTANT = 0x8576,
	GLES_TEXCOMBINER_SOURCE_ENUMS_PRIMARY = 0x8577,
	GLES_TEXCOMBINER_SOURCE_ENUMS_PREVIOUS = 0x8578,
	GLES_TEXCOMBINER_SOURCE_ENUMS_COUNT = 4
};

enum domGles_texcombiner_operandRGB_enums {
	GLES_TEXCOMBINER_OPERANDRGB_ENUMS_SRC_COLOR = 0x0300,
	GLES_TEXCOMBINER_OPERANDRGB_ENUMS_ONE_MINUS_SRC_COLOR = 0x0301,
	GLES_TEXCOMBINER_OPERANDRGB_ENUMS_SRC_ALPHA = 0x0302,
	GLES_TEXCOMBINER_OPERANDRGB_ENUMS_ONE_MINUS_SRC_ALPHA = 0x0303,
	GLES_TEXCOMBINER_OPERANDRGB_ENUMS_COUNT = 4
};

enum domGles_texcombiner_operandAlpha_enums {
	GLES_TEXCOMBINER_OPERANDALPHA_ENUMS_SRC_ALPHA = 0x0302,
	GLES_TEXCOMBINER_OPERANDALPHA_ENUMS_ONE_MINUS_SRC_ALPHA = 0x0303,
	GLES_TEXCOMBINER_OPERANDALPHA_ENUMS_COUNT = 2
};

enum domGles_sampler_wrap {
	GLES_SAMPLER_WRAP_REPEAT,
	GLES_SAMPLER_WRAP_CLAMP,
	GLES_SAMPLER_WRAP_CLAMP_TO_EDGE,
	GLES_SAMPLER_WRAP_MIRRORED_REPEAT,		/**< supported by GLES 1.1 only */
	GLES_SAMPLER_WRAP_COUNT = 4
};

enum domGles_stencil_op_type {
	GLES_STENCIL_OP_TYPE_KEEP = 0x1E00,
	GLES_STENCIL_OP_TYPE_ZERO = 0x0,
	GLES_STENCIL_OP_TYPE_REPLACE = 0x1E01,
	GLES_STENCIL_OP_TYPE_INCR = 0x1E02,
	GLES_STENCIL_OP_TYPE_DECR = 0x1E03,
	GLES_STENCIL_OP_TYPE_INVERT = 0x150A,
	GLES_STENCIL_OP_TYPE_COUNT = 6
};

enum domSpringType {
	SPRINGTYPE_LINEAR,
	SPRINGTYPE_ANGULAR,
	SPRINGTYPE_COUNT = 2
};

enum domGl_enumeration {
	GL_ENUMERATION_ZERO = 0x0,
	GL_ENUMERATION_ONE = 0x1,
	GL_ENUMERATION_SRC_COLOR = 0x0300,
	GL_ENUMERATION_ONE_MINUS_SRC_COLOR = 0x0301,
	GL_ENUMERATION_DEST_COLOR = 0x0306,
	GL_ENUMERATION_ONE_MINUS_DEST_COLOR = 0x0307,
	GL_ENUMERATION_SRC_ALPHA = 0x0302,
	GL_ENUMERATION_ONE_MINUS_SRC_ALPHA = 0x0303,
	GL_ENUMERATION_DST_ALPHA = 0x0304,
	GL_ENUMERATION_ONE_MINUS_DST_ALPHA = 0x0305,
	GL_ENUMERATION_CONSTANT_COLOR = 0x8001,
	GL_ENUMERATION_ONE_MINUS_CONSTANT_COLOR = 0x8002,
	GL_ENUMERATION_CONSTANT_ALPHA = 0x8003,
	GL_ENUMERATION_ONE_MINUS_CONSTANT_ALPHA = 0x8004,
	GL_ENUMERATION_SRC_ALPHA_SATURATE = 0x0308,
	GL_ENUMERATION_FRONT = 0x0404,
	GL_ENUMERATION_BACK = 0x0405,
	GL_ENUMERATION_FRONT_AND_BACK = 0x0408,
	GL_ENUMERATION_FUNC_ADD = 0x8006,
	GL_ENUMERATION_FUNC_SUBTRACT = 0x800A,
	GL_ENUMERATION_FUNC_REVERSE_SUBTRACT = 0x800B,
	GL_ENUMERATION_MIN = 0x8007,
	GL_ENUMERATION_MAX = 0x8008,
	GL_ENUMERATION_NEVER = 0x0200,
	GL_ENUMERATION_LESS = 0x0201,
	GL_ENUMERATION_LEQUAL = 0x0203,
	GL_ENUMERATION_EQUAL = 0x0202,
	GL_ENUMERATION_GREATER = 0x0204,
	GL_ENUMERATION_NOTEQUAL = 0x0205,
	GL_ENUMERATION_GEQUAL = 0x0206,
	GL_ENUMERATION_ALWAYS = 0x0207,
	GL_ENUMERATION_KEEP = 0x1E00,
	GL_ENUMERATION_REPLACE = 0x1E01,
	GL_ENUMERATION_INCR = 0x1E02,
	GL_ENUMERATION_DECR = 0x1E03,
	GL_ENUMERATION_INVERT = 0x150A,
	GL_ENUMERATION_INCR_WRAP = 0x8507,
	GL_ENUMERATION_DECR_WRAP = 0x8508,
	GL_ENUMERATION_EMISSION = 0x1600,
	GL_ENUMERATION_AMBIENT = 0x1200,
	GL_ENUMERATION_DIFFUSE = 0x1201,
	GL_ENUMERATION_SPECULAR = 0x1202,
	GL_ENUMERATION_AMBIENT_AND_DIFFUSE = 0x1602,
	GL_ENUMERATION_LINEAR = 0x2601,
	GL_ENUMERATION_EXP = 0x0800,
	GL_ENUMERATION_EXP2 = 0x0801,
	GL_ENUMERATION_FOG_COORDINATE = 0x8451,
	GL_ENUMERATION_FRAGMENT_DEPTH = 0x8452,
	GL_ENUMERATION_CW = 0x0900,
	GL_ENUMERATION_CCW = 0x0901,
	GL_ENUMERATION_SINGLE_COLOR = 0x81F9,
	GL_ENUMERATION_SEPARATE_SPECULAR_COLOR = 0x81FA,
	GL_ENUMERATION_CLEAR = 0x1500,
	GL_ENUMERATION_AND = 0x1501,
	GL_ENUMERATION_AND_REVERSE = 0x1502,
	GL_ENUMERATION_COPY = 0x1503,
	GL_ENUMERATION_AND_INVERTED = 0x1504,
	GL_ENUMERATION_NOOP = 0x1505,
	GL_ENUMERATION_XOR = 0x1506,
	GL_ENUMERATION_OR = 0x1507,
	GL_ENUMERATION_NOR = 0x1508,
	GL_ENUMERATION_EQUIV = 0x1509,
	GL_ENUMERATION_OR_REVERSE = 0x150B,
	GL_ENUMERATION_COPY_INVERTED = 0x150C,
	GL_ENUMERATION_NAND = 0x150E,
	GL_ENUMERATION_SET = 0x150F,
	GL_ENUMERATION_POINT = 0x1B00,
	GL_ENUMERATION_LINE = 0x1B01,
	GL_ENUMERATION_FILL = 0x1B02,
	GL_ENUMERATION_FLAT = 0x1D00,
	GL_ENUMERATION_SMOOTH = 0x1D01,
	GL_ENUMERATION_COUNT = 72
};

enum domGles_enumeration {
	GLES_ENUMERATION_ZERO = 0x0,
	GLES_ENUMERATION_ONE = 0x1,
	GLES_ENUMERATION_SRC_COLOR = 0x0300,
	GLES_ENUMERATION_ONE_MINUS_SRC_COLOR = 0x0301,
	GLES_ENUMERATION_DEST_COLOR = 0x0306,
	GLES_ENUMERATION_ONE_MINUS_DEST_COLOR = 0x0307,
	GLES_ENUMERATION_SRC_ALPHA = 0x0302,
	GLES_ENUMERATION_ONE_MINUS_SRC_ALPHA = 0x0303,
	GLES_ENUMERATION_DST_ALPHA = 0x0304,
	GLES_ENUMERATION_ONE_MINUS_DST_ALPHA = 0x0305,
	GLES_ENUMERATION_CONSTANT_COLOR = 0x8001,
	GLES_ENUMERATION_ONE_MINUS_CONSTANT_COLOR = 0x8002,
	GLES_ENUMERATION_CONSTANT_ALPHA = 0x8003,
	GLES_ENUMERATION_ONE_MINUS_CONSTANT_ALPHA = 0x8004,
	GLES_ENUMERATION_SRC_ALPHA_SATURATE = 0x0308,
	GLES_ENUMERATION_FRONT = 0x0404,
	GLES_ENUMERATION_BACK = 0x0405,
	GLES_ENUMERATION_FRONT_AND_BACK = 0x0408,
	GLES_ENUMERATION_NEVER = 0x0200,
	GLES_ENUMERATION_LESS = 0x0201,
	GLES_ENUMERATION_LEQUAL = 0x0203,
	GLES_ENUMERATION_EQUAL = 0x0202,
	GLES_ENUMERATION_GREATER = 0x0204,
	GLES_ENUMERATION_NOTEQUAL = 0x0205,
	GLES_ENUMERATION_GEQUAL = 0x0206,
	GLES_ENUMERATION_ALWAYS = 0x0207,
	GLES_ENUMERATION_KEEP = 0x1E00,
	GLES_ENUMERATION_REPLACE = 0x1E01,
	GLES_ENUMERATION_INCR = 0x1E02,
	GLES_ENUMERATION_DECR = 0x1E03,
	GLES_ENUMERATION_INVERT = 0x150A,
	GLES_ENUMERATION_INCR_WRAP = 0x8507,
	GLES_ENUMERATION_DECR_WRAP = 0x8508,
	GLES_ENUMERATION_EMISSION = 0x1600,
	GLES_ENUMERATION_AMBIENT = 0x1200,
	GLES_ENUMERATION_DIFFUSE = 0x1201,
	GLES_ENUMERATION_SPECULAR = 0x1202,
	GLES_ENUMERATION_AMBIENT_AND_DIFFUSE = 0x1602,
	GLES_ENUMERATION_LINEAR = 0x2601,
	GLES_ENUMERATION_EXP = 0x0800,
	GLES_ENUMERATION_EXP2 = 0x0801,
	GLES_ENUMERATION_CW = 0x0900,
	GLES_ENUMERATION_CCW = 0x0901,
	GLES_ENUMERATION_SINGLE_COLOR = 0x81F9,
	GLES_ENUMERATION_SEPARATE_SPECULAR_COLOR = 0x81FA,
	GLES_ENUMERATION_CLEAR = 0x1500,
	GLES_ENUMERATION_AND = 0x1501,
	GLES_ENUMERATION_AND_REVERSE = 0x1502,
	GLES_ENUMERATION_COPY = 0x1503,
	GLES_ENUMERATION_AND_INVERTED = 0x1504,
	GLES_ENUMERATION_NOOP = 0x1505,
	GLES_ENUMERATION_XOR = 0x1506,
	GLES_ENUMERATION_OR = 0x1507,
	GLES_ENUMERATION_NOR = 0x1508,
	GLES_ENUMERATION_EQUIV = 0x1509,
	GLES_ENUMERATION_OR_REVERSE = 0x150B,
	GLES_ENUMERATION_COPY_INVERTED = 0x150C,
	GLES_ENUMERATION_NAND = 0x150E,
	GLES_ENUMERATION_SET = 0x150F,
	GLES_ENUMERATION_POINT = 0x1B00,
	GLES_ENUMERATION_LINE = 0x1B01,
	GLES_ENUMERATION_FILL = 0x1B02,
	GLES_ENUMERATION_FLAT = 0x1D00,
	GLES_ENUMERATION_SMOOTH = 0x1D01,
	GLES_ENUMERATION_COUNT = 65
};


#endif
