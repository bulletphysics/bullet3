/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/
/*
	Based on the FS Import classes:
	Copyright (C) 2005-2006 Feeling Software Inc
	Copyright (C) 2005-2006 Autodesk Media Entertainment
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FUtils/FUDaeEnum.h"
#include "FUtils/FUDaeSyntax.h"

namespace FUDaeInterpolation
{
	Interpolation FromString(const string& value)
	{
		if (value == DAE_NONE_INTERPOLATION) return STEP;
		else if (value == DAE_STEP_INTERPOLATION) return STEP;
		else if (value == DAE_LINEAR_INTERPOLATION) return LINEAR;
		else if (value == DAE_BEZIER_INTERPOLATION) return BEZIER;
		else return UNKNOWN;
	}

	const char* ToString(const Interpolation& value)
	{
		switch (value)
		{
		case STEP: return DAE_STEP_INTERPOLATION;
		case LINEAR: return DAE_LINEAR_INTERPOLATION;
		case BEZIER: return DAE_BEZIER_INTERPOLATION;

		case UNKNOWN:
		default: return DAEERR_UNKNOWN_ELEMENT;
		}
	}
};

namespace FUDaeFunction
{
	Function FromString(const char* value)
	{
		if (IsEquivalent(value, DAE_CONSTANT_FUNCTION)) return CONSTANT;
		else if (IsEquivalent(value, DAE_LINEAR_FUNCTION)) return LINEAR;
		else if (IsEquivalent(value, DAE_QUADRATIC_FUNCTION)) return QUADRATIC;
		else return UNKNOWN;
	}
};

namespace FUDaeTextureChannel
{
	Channel FromString(const string& value)
	{
		if (value == DAE_AMBIENT_TEXTURE_CHANNEL) return AMBIENT;
		else if (value == DAE_BUMP_TEXTURE_CHANNEL) return BUMP;
		else if (value == DAE_DIFFUSE_TEXTURE_CHANNEL) return DIFFUSE;
		else if (value == DAE_DISPLACEMENT_TEXTURE_CHANNEL) return DISPLACEMENT;
		else if (value == DAE_EMISSION_TEXTURE_CHANNEL) return EMISSION;
		else if (value == DAE_FILTER_TEXTURE_CHANNEL) return FILTER;
		else if (value == DAE_OPACITY_TEXTURE_CHANNEL) return OPACITY;
		else if (value == DAE_REFLECTION_TEXTURE_CHANNEL) return REFLECTION;
		else if (value == DAE_REFRACTION_TEXTURE_CHANNEL) return REFRACTION;
		else if (value == DAE_SHININESS_TEXTURE_CHANNEL) return SHININESS;
		else if (value == DAE_SPECULAR_TEXTURE_CHANNEL) return SPECULAR;
		else if (value == DAE_SPECULARLEVEL_TEXTURE_CHANNEL) return SPECULAR_LEVEL;
		else if (value == DAE_TRANSPARENT_TEXTURE_CHANNEL) return TRANSPARENT;
		else return UNKNOWN;
	}
};

namespace FUDaeMorphMethod
{
	Method FromString(const char* value)
	{
		if (IsEquivalent(value, DAE_NORMALIZED_MORPH_METHOD)) return NORMALIZED;
		else if (IsEquivalent(value, DAE_RELATIVE_MORPH_METHOD)) return RELATIVE;
		else return DEFAULT;
	}

	const char* ToString(Method method)
	{
		switch (method)
		{
		case NORMALIZED: return DAE_NORMALIZED_MORPH_METHOD;
		case RELATIVE: return DAE_RELATIVE_MORPH_METHOD;
		case UNKNOWN:
		default: return DAEERR_UNKNOWN_MORPH_METHOD;
		}
	}
};

namespace FUDaeInfinity
{
	Infinity FromString(const char* value)
	{
		if (IsEquivalent(value, DAEMAYA_CONSTANT_INFINITY)) return CONSTANT;
		else if (IsEquivalent(value, DAEMAYA_LINEAR_INFINITY)) return LINEAR;
		else if (IsEquivalent(value, DAEMAYA_CYCLE_INFINITY)) return CYCLE;
		else if (IsEquivalent(value, DAEMAYA_CYCLE_RELATIVE_INFINITY)) return CYCLE_RELATIVE;
		else if (IsEquivalent(value, DAEMAYA_OSCILLATE_INFINITY)) return OSCILLATE;
		else return DEFAULT;
	}

	const char* ToString(Infinity type)
	{
		switch (type)
		{
		case CONSTANT: return DAEMAYA_CONSTANT_INFINITY;
		case LINEAR: return DAEMAYA_LINEAR_INFINITY;
		case CYCLE: return DAEMAYA_CYCLE_INFINITY;
		case CYCLE_RELATIVE: return DAEMAYA_CYCLE_RELATIVE_INFINITY;
		case OSCILLATE: return DAEMAYA_OSCILLATE_INFINITY;
		default: return DAEMAYA_CONSTANT_INFINITY;
		}
	}
};

namespace FUDaeBlendMode
{
	Mode FromString(const char* value)
	{
		if (IsEquivalent(value, DAEMAYA_NONE_BLENDMODE)) return NONE;
		else if (IsEquivalent(value, DAEMAYA_OVER_BLENDMODE)) return OVER;
		else if (IsEquivalent(value, DAEMAYA_IN_BLENDMODE)) return IN;
		else if (IsEquivalent(value, DAEMAYA_OUT_BLENDMODE)) return OUT;
		else if (IsEquivalent(value, DAEMAYA_ADD_BLENDMODE)) return ADD;
		else if (IsEquivalent(value, DAEMAYA_SUBSTRACT_BLENDMODE)) return SUBSTRACT;
		else if (IsEquivalent(value, DAEMAYA_MULTIPLY_BLENDMODE)) return MULTIPLY;
		else if (IsEquivalent(value, DAEMAYA_DIFFERENCE_BLENDMODE)) return DIFFERENCE;
		else if (IsEquivalent(value, DAEMAYA_LIGHTEN_BLENDMODE)) return LIGHTEN;
		else if (IsEquivalent(value, DAEMAYA_DARKEN_BLENDMODE)) return DARKEN;
		else if (IsEquivalent(value, DAEMAYA_SATURATE_BLENDMODE)) return SATURATE;
		else if (IsEquivalent(value, DAEMAYA_DESATURATE_BLENDMODE)) return DESATURATE;
		else if (IsEquivalent(value, DAEMAYA_ILLUMINATE_BLENDMODE)) return ILLUMINATE;
		else return UNKNOWN;
	}

	const char* ToString(Mode mode)
	{
		switch (mode)
		{
		case NONE: return DAEMAYA_NONE_BLENDMODE;
		case OVER: return DAEMAYA_OVER_BLENDMODE;
		case IN: return DAEMAYA_IN_BLENDMODE;
		case OUT: return DAEMAYA_OUT_BLENDMODE;
		case ADD: return DAEMAYA_ADD_BLENDMODE;
		case SUBSTRACT: return DAEMAYA_SUBSTRACT_BLENDMODE;
		case MULTIPLY: return DAEMAYA_MULTIPLY_BLENDMODE;
		case DIFFERENCE: return DAEMAYA_DIFFERENCE_BLENDMODE;
		case LIGHTEN: return DAEMAYA_LIGHTEN_BLENDMODE;
		case DARKEN: return DAEMAYA_DARKEN_BLENDMODE;
		case SATURATE: return DAEMAYA_SATURATE_BLENDMODE;
		case DESATURATE: return DAEMAYA_DESATURATE_BLENDMODE;
		case ILLUMINATE: return DAEMAYA_ILLUMINATE_BLENDMODE;
		default: return DAEMAYA_NONE_BLENDMODE;
		}
	}
}

namespace FUDaeGeometryInput
{
	Semantic FromString(const char* value)
	{
		if (IsEquivalent(value, DAE_POSITION_INPUT)) return POSITION;
		else if (IsEquivalent(value, DAE_VERTEX_INPUT)) return VERTEX;
		else if (IsEquivalent(value, DAE_NORMAL_INPUT)) return NORMAL;
		else if (IsEquivalent(value, DAE_GEOTANGENT_INPUT)) return GEOTANGENT;
		else if (IsEquivalent(value, DAE_GEOBINORMAL_INPUT)) return GEOBINORMAL;
		else if (IsEquivalent(value, DAE_TEXCOORD_INPUT)) return TEXCOORD;
		else if (IsEquivalent(value, DAE_TEXTANGENT_INPUT)) return TEXTANGENT;
		else if (IsEquivalent(value, DAE_TEXBINORMAL_INPUT)) return TEXBINORMAL;
		else if (IsEquivalent(value, DAE_MAPPING_INPUT)) return UV;
		else if (IsEquivalent(value, DAE_COLOR_INPUT)) return COLOR;
		else if (IsEquivalent(value, DAEMAYA_EXTRA_INPUT)) return EXTRA;
		else return UNKNOWN;
	}

    const char* ToString(Semantic semantic)
	{
		switch(semantic)
		{
		case POSITION: return DAE_POSITION_INPUT;
		case VERTEX: return DAE_VERTEX_INPUT;
		case NORMAL: return DAE_NORMAL_INPUT;
		case GEOTANGENT: return DAE_GEOTANGENT_INPUT;
		case GEOBINORMAL: return DAE_GEOBINORMAL_INPUT;
		case TEXCOORD: return DAE_TEXCOORD_INPUT;
		case TEXTANGENT: return DAE_TEXTANGENT_INPUT;
		case TEXBINORMAL: return DAE_TEXBINORMAL_INPUT;
		case UV: return DAE_MAPPING_INPUT;
		case COLOR: return DAE_COLOR_INPUT;
		case EXTRA: return DAEMAYA_EXTRA_INPUT;

		case UNKNOWN:
		default: return DAEERR_UNKNOWN_INPUT;
		}
	}
}

namespace FUDaeProfileType
{
	Type FromString(const char* value)
	{
		if (IsEquivalent(value, DAE_FX_PROFILE_COMMON_ELEMENT)) return COMMON;
		else if (IsEquivalent(value, DAE_FX_PROFILE_CG_ELEMENT)) return CG;
		else if (IsEquivalent(value, DAE_FX_PROFILE_HLSL_ELEMENT)) return HLSL;
		else if (IsEquivalent(value, DAE_FX_PROFILE_GLSL_ELEMENT)) return GLSL;
		else if (IsEquivalent(value, DAE_FX_PROFILE_GLES_ELEMENT)) return GLES;
		else return UNKNOWN;
	}

    const char* ToString(Type type)
	{
		switch(type)
		{
		case COMMON: return DAE_FX_PROFILE_COMMON_ELEMENT;
		case CG: return DAE_FX_PROFILE_CG_ELEMENT;
		case HLSL: return DAE_FX_PROFILE_HLSL_ELEMENT;
		case GLSL: return DAE_FX_PROFILE_GLSL_ELEMENT;
		case GLES: return DAE_FX_PROFILE_GLES_ELEMENT;

		case UNKNOWN:
		default: return DAEERR_UNKNOWN_INPUT;
		}
	}
}
