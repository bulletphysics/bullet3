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

#ifndef _FU_DAE_ENUM_H_
#define _FU_DAE_ENUM_H_

// Animation curve interpolation function
// Defaults to the STEP interpolation by definition in COLLADA.
// BEZIER is the more common interpolation type
namespace FUDaeInterpolation
{
	enum Interpolation
	{
		STEP = 0, //equivalent to no interpolation
		LINEAR,
		BEZIER,

		UNKNOWN,
		DEFAULT = STEP,
	};

	FCOLLADA_EXPORT Interpolation FromString(const string& value);
	const char* ToString(const Interpolation& value);
};

typedef vector<FUDaeInterpolation::Interpolation> FUDaeInterpolationList;

// COLLADA generic degree function. Used by lights and the profile_COMMON materials.
namespace FUDaeFunction
{
	enum Function
	{
		CONSTANT = 0,
		LINEAR,
		QUADRATIC,

		UNKNOWN,
		DEFAULT = CONSTANT,
	};

	FCOLLADA_EXPORT Function FromString(const char* value);
	inline Function FromString(const string& value) { return FromString(value.c_str()); }
};


// Material texture channels. Used by profile_common materials to assign textures to channels/slots
// Multi-texturing is done by assigning more than one texture per slot.
// Defaults to diffuse texture slot
#undef TRANSPARENT // Win32: GDI stupidely defines this in the global namespace
namespace FUDaeTextureChannel
{
	enum Channel
	{
		AMBIENT = 0,
		BUMP,
		DIFFUSE,
		DISPLACEMENT,
		EMISSION,
		FILTER,
		OPACITY,
		REFLECTION,
		REFRACTION,
		SHININESS,
		SPECULAR,
		SPECULAR_LEVEL,
		TRANSPARENT,

		COUNT,
		UNKNOWN,
		DEFAULT = DIFFUSE,
	};

	FCOLLADA_EXPORT Channel FromString(const string& value);
};

// Morph controller method
// NORMALIZED implies that the morph targets all have absolute vertex positions
// RELATIVE implies that the morph targets have relative vertex positions
//
// Whether the vertex position is relative or absolute is irrelevant,
// as long as you use the correct weight generation function:
// NORMALIZED: base_weight = 1.0f - SUM(weight[t])
// RELATIVE: base_weight = 1.0f
// and position[k] = SUM(weight[t][k] * position[t][k])
#undef RELATIVE // Win32: GDI stupidely defines this in the global namespace
namespace FUDaeMorphMethod
{
	enum Method
	{
		NORMALIZED = 0,
		RELATIVE,

		UNKNOWN,
		DEFAULT = NORMALIZED,
	};

	FCOLLADA_EXPORT Method FromString(const char* value);
	FCOLLADA_EXPORT const char* ToString(Method method);
	inline Method FromString(const string& value) { return FromString(value.c_str()); }
};

// Maya uses the infinity to determine what happens outside an animation curve
// Intentionally matches the MFnAnimCurve::InfinityType enum
namespace FUDaeInfinity
{
	enum Infinity
	{
		CONSTANT = 0,
		LINEAR,
		CYCLE,
		CYCLE_RELATIVE,
		OSCILLATE,

		UNKNOWN,
		DEFAULT = CONSTANT
	};

	FCOLLADA_EXPORT Infinity FromString(const char* value);
	FCOLLADA_EXPORT const char* ToString(Infinity infinity);
	inline Infinity FromString(const string& value) { return FromString(value.c_str()); }
};

// Maya uses the blend mode for texturing purposes
// Intentionally matches the equivalent Maya enum
#undef IN
#undef OUT
#undef DIFFERENCE
namespace FUDaeBlendMode
{
	enum Mode
	{
		NONE,
		OVER,
		IN,
		OUT,
		ADD,
		SUBSTRACT,
		MULTIPLY,
		DIFFERENCE,
		LIGHTEN,
		DARKEN,
		SATURATE,
		DESATURATE,
		ILLUMINATE,

		UNKNOWN,
		DEFAULT = NONE,
	};

	FCOLLADA_EXPORT Mode FromString(const char* value);
	FCOLLADA_EXPORT const char* ToString(Mode mode);
	inline Mode FromString(const string& value) { return FromString(value.c_str()); }
}

// Geometry Input Semantics
// Found in the <mesh><vertices>, <mesh><polygons>...
namespace FUDaeGeometryInput
{
	enum Semantic
	{
		POSITION = 0,
		VERTEX,
		NORMAL,
		GEOTANGENT,
		GEOBINORMAL,
		TEXCOORD,
		TEXTANGENT,
		TEXBINORMAL,
		UV,
		COLOR,
		EXTRA, // Maya-specific, used for blind data

		UNKNOWN = -1,
	};
	typedef vector<Semantic> SemanticList;

	FCOLLADA_EXPORT Semantic FromString(const char* value);
    FCOLLADA_EXPORT const char* ToString(Semantic semantic);
	inline Semantic FromString(const string& value) { return FromString(value.c_str()); }
}

/** The types of effect profiles. */
namespace FUDaeProfileType
{
	/** Enumerates the types of effect profiles. */
	enum Type
	{
		CG, /**< The CG profile. */
		HLSL, /**< The HLSL profile for DirectX. */
		GLSL, /**< The GLSL profile for OpenGL. */
		GLES, /**< The GLES profile for OpenGL ES. */
		COMMON, /**< The common profile which encapsulates the standard lighting equations: Lambert, Phong, Blinn. */

		UNKNOWN /**< Not a valid profile type. */
	};

	/** Converts the COLLADA profile name string to a profile type.
		Examples of COLLADA profile name strings are 'profile_CG' and 'profile_COMMON'.
		@param value The profile name string.
		@return The profile type. */
	FCOLLADA_EXPORT Type FromString(const char* value);
	inline Type FromString(const string& value) { return FromString(value.c_str()); } /**< See above. */

	/** Converts the profile type to its COLLADA profile name string.
		Examples of COLLADA profile name strings are 'profile_CG' and 'profile_COMMON'.
		@param type The profile type.
		@return The name string for the profile type. */
	FCOLLADA_EXPORT const char* ToString(Type type);
}

#endif // _FU_DAE_ENUM_H_

