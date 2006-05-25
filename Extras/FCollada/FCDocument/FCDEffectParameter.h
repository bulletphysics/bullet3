/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FCDEffectParameter.h
	This file contains the FCDEffectParameter interface and most of its derivate classes:
	FCDEffectParameterSampler, FCDEffectParameterFloat, FCDEffectParameterVector...
*/

#ifndef _FCD_EFFECT_PARAMETER_H_
#define _FCD_EFFECT_PARAMETER_H_

#include "FCDocument/FCDObject.h"

class FCDEffectPass;
class FCDocument;
class FCDEffectParameterSurface;

/**
	A COLLADA effect parameter.

	This interface class is used to define all the valid
	ColladaFX parameter types. There are many types of
	parameters: integers, booleans, floating-point
	values, 2D, 3D and 4D vectors of floating-point values,
	matrices, strings, surfaces and their samplers.

	A COLLADA effect parameter may generate a new
	effect parameter, in which case it will declare a semantic
	and a reference: to represent it within the COLLADA document.

	@ingroup FCDEffect
*/
class FCOLLADA_EXPORT FCDEffectParameter : public FCDObject
{
public:
	/** The type of the effect parameter class. */
	enum Type
	{
		SAMPLER, /**< A sampler effect parameter. Points towards a surface parameter and adds extra texturing parameters. */
		INTEGER, /**< A single integer effect parameter. */
		BOOLEAN, /**< A single boolean effect parameter. */
		FLOAT, /**< A single floating-pointer value effect parameter. */
		FLOAT2, /**< A 2D vector of floating-pointer values. */
		FLOAT3, /**< A 3D vector of floating-pointer values. */
		VECTOR, /**< A 4D vector of floating-pointer values. */
		MATRIX, /**< A 4x4 matrix. */
		STRING, /**< A string effect parameter. */
		SURFACE /**< A surface effect parameter. Contains a COLLADA image pointer. */
	};

private:
	bool isGenerator; // whether this effect parameter structure generates a new value or modifies an existing value (is <newparam>?)
	string reference;
	string semantic; // this is a Collada Semantic, not a Cg semantic
	
	// [glaforte] These two members should be somewhere else
	string bindSymbol; // this can be used in Cg to bind to the correct variable
	bool isFragment; // parameter bound to the fragment program or the vertex one

public:
	/** Constructor: do not use directly.
		Instead, use the FCDEffectParameterList::AddParameter function.
		@param document The COLLADA document that owns the effect parameter. */
	FCDEffectParameter(FCDocument* document);

	/** Destructor: do not use directly.
		Instead, use the FCDEffectParameterList::ReleaseParameter function.
		When released, the effect parameter list will also release all
		its parameters, if it owns them. */
	virtual ~FCDEffectParameter();

	/** Retrieves the type of effect parameter class.
		@return The type of the effect parameter class.*/
	virtual Type GetType() const = 0;

	/** Retrieves the reference for this effect parameter.
		In the case of generators, the reference string contains the sub-id.
		@return The reference. */
	const string& GetReference() const { return reference; } 

	/** Retrieves the semantic for this effect parameter.
		@return The semantic. */
	const string& GetSemantic() const { return semantic; }

	/** Sets the semantic for this effect parameter.
		@param _semantic The semantic. */
	void SetSemantic(const string& _semantic) { semantic = _semantic; } 

	/** Retrieves whether this effect parameter is a parameter generator.
		A ColladaFX parameter must be generated to be modified or bound at
		higher abstraction levels.
		@return Whether this is a generator. */
	bool IsGenerator() const { return isGenerator; }

	/** Retrieves whether this effect parameter is a parameter modifier.
		A ColladaFX parameter must be generated to be modified or bound at
		higher abstraction levels.
		@return Whether this is a modifier. */
	bool IsModifier() const { return !isGenerator; }

	/** @deprecated Retrieves the program bind symbol for
		this parameter. This information should be available
		per-shader, in the FCDEffectPassShader class.
		@return The program bind symbol. */
	const string& GetBindSymbol() const { return bindSymbol; }

	/** @deprecated Sets the program bind symbol for this parameter.
		This information is available per-shader, in the FCDEffectPassShader class.
		@param _bindSymbol The program bind symbol. */
	void SetBindSymbol(const string& _bindSymbol) { bindSymbol = _bindSymbol; }

	/** @deprecated Retrieves whether the program bind symbol attached
		to this parameter belongs to a fragment/pixel shader.
		This information is available per-shader, in the FCDEffectPassShader class.
		@return Whether it belongs to a fragment/pixel shader. */
	bool IsFragment() const { return isFragment; }

	/** @deprecated Sets whether the program bind symbol attached to this
		parameter belongs to a fragment/pixel shader.
		This information is available per-shader, in the FCDEffectPassShader class.
		@param _isFragment Whether it belongs to a fragment/pixel shader. */
	void SetFragment(bool _isFragment) { isFragment = _isFragment;}

	/** Creates a full copy of the effect parameter.
		@return The cloned effect parameter. You will need to delete this pointer. */
	virtual FCDEffectParameter* Clone() = 0;

	/** [INTERNAL] Overwrites the target parameter with this parameter.
		This function is used during the flattening of materials.
		@param target The target parameter to overwrite. */
	virtual void Overwrite(FCDEffectParameter* target);

	/** [INTERNAL] Reads in the effect parameter from a given COLLADA XML tree node.
		@param parameterNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the parameter.*/
	virtual FUStatus LoadFromXML(xmlNode* parameterNode);

	/** [INTERNAL] Writes out the effect parameter to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the parameter.
		@return The created element XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;

protected:
	/** [INTERNAL] Copies into the given effect parameters, the variables
		held by the FCDEffectParameter interface. This function is used by the classes
		based on this interface during the cloning process.
		@param clone The parameter to clone. */
	void Clone(FCDEffectParameter* clone);
};

/**
	A COLLADA sampler effect parameter.
	A sampler parameter provides the extra texturing information necessary
	to correctly sample a surface parameter.
	There are four types of samplers supported: 1D, 2D, 3D and cube.

	@ingroup FCDEffect
*/
class FCOLLADA_EXPORT FCDEffectParameterSampler : public FCDEffectParameter
{
public:
	/** The type of sampling to execute. */
	enum SamplerType
	{
		SAMPLER1D, /** 1D sampling. */
		SAMPLER2D, /** 2D sampling. */
		SAMPLER3D, /** 3D sampling. */
		SAMPLERCUBE /** Cube-map sampling. */
	};

private:
	SamplerType samplerType;
	string surfaceSid;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDEffectParameterList::AddParameter function.
		@param document The COLLADA document that owns the effect parameter. */
	FCDEffectParameterSampler(FCDocument* document);

	/** Destructor: do not use directly.
		Instead, use the FCDEffectParameterList::ReleaseParameter function.
		When released, the effect parameter list will also release all
		its parameters, if it owns them. */
	virtual ~FCDEffectParameterSampler();

	/** Retrieves the type of effect parameter class.
		@return The parameter class type: SAMPLER. */
	virtual Type GetType() const { return SAMPLER; }

	/** Retrieves the sub-id of the surface parameter.
		You will want to search for that sub-id within the parameters to find the
		FCDEffectParameterSurface object.
		@return The sub-id. */
	const char* GetSurfaceSid() const { return surfaceSid.c_str(); }

	/** Sets the sub-id of the surface parameter to sample.
		@param sid The surface parameter sub-id. */
	void SetSurfaceSid(const char* sid) { surfaceSid = sid; }

	/** Retrieves the type of sampling to do.
		@return The sampling type. */
	SamplerType GetSamplerType() const { return samplerType; }

	/** Sets the type of sampling to do.
		@param type The sampling type. */
	void SetSamplerType(SamplerType type) { samplerType = type; }

	/** Creates a full copy of the effect parameter.
		@return The cloned effect parameter. You will need to delete this pointer. */
	virtual FCDEffectParameter* Clone();

	/** [INTERNAL] Overwrites the target parameter with this parameter.
		This function is used during the flattening of materials.
		@param target The target parameter to overwrite. */
	virtual void Overwrite(FCDEffectParameter* target);

	/** [INTERNAL] Reads in the effect parameter from a given COLLADA XML tree node.
		@param parameterNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the parameter.*/
	virtual FUStatus LoadFromXML(xmlNode* parameterNode);

	/** [INTERNAL] Writes out the effect parameter to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the parameter.
		@return The created element XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

/**
	A COLLADA integer effect parameter.
	Contains a single, unanimated integer.
*/
class FCOLLADA_EXPORT FCDEffectParameterInt : public FCDEffectParameter
{
private:
	int value;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDEffectParameterList::AddParameter function.
		@param document The COLLADA document that owns the effect parameter. */
	FCDEffectParameterInt(FCDocument* document);

	/** Destructor: do not use directly.
		Instead, use the FCDEffectParameterList::ReleaseParameter function.
		When released, the effect parameter list will also release all
		its parameters, if it owns them. */
	virtual ~FCDEffectParameterInt();

	/** Retrieves the type of effect parameter class.
		@return The parameter class type: INTEGER. */
	virtual Type GetType() const { return INTEGER; }

	/** Retrieves the value of the effect parameter.
		@return The integer value. */
	int GetValue() const { return value; }

	/** Sets the integer value of the effect parameter.
		@param _value The integer value. */
	void SetValue(int _value) { value = _value; }

	/** Creates a full copy of the effect parameter.
		@return The cloned effect parameter. You will need to delete this pointer. */
	virtual FCDEffectParameter* Clone();

	/** [INTERNAL] Overwrites the target parameter with this parameter.
		This function is used during the flattening of materials.
		@param target The target parameter to overwrite. */
	virtual void Overwrite(FCDEffectParameter* target);

	/** [INTERNAL] Reads in the effect parameter from a given COLLADA XML tree node.
		@param parameterNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the parameter.*/
	virtual FUStatus LoadFromXML(xmlNode* parameterNode);

	/** [INTERNAL] Writes out the effect parameter to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the parameter.
		@return The created element XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

/**
	A COLLADA boolean effect parameter.
	Contains a single, unanimated boolean.
*/
class FCOLLADA_EXPORT FCDEffectParameterBool : public FCDEffectParameter
{
private:
	bool value;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDEffectParameterList::AddParameter function.
		@param document The COLLADA document that owns the effect parameter. */
	FCDEffectParameterBool(FCDocument* document);

	/** Destructor: do not use directly.
		Instead, use the FCDEffectParameterList::ReleaseParameter function.
		When released, the effect parameter list will also release all
		its parameters, if it owns them. */
	virtual ~FCDEffectParameterBool();

	/** Retrieves the type of effect parameter class.
		@return The parameter class type: BOOLEAN. */
	virtual Type GetType() const { return BOOLEAN; }

	/** Retrieves the boolean value of the effect parameter.
		@return The boolean value. */
	bool GetValue() const { return value; }

	/** Sets the boolean value of the effect parameter.
		@param _value The boolean value. */
	void SetValue(bool _value) { value = _value; }

	/** Creates a full copy of the effect parameter.
		@return The cloned effect parameter. You will need to delete this pointer. */
	virtual FCDEffectParameter* Clone();

	/** [INTERNAL] Overwrites the target parameter with this parameter.
		This function is used during the flattening of materials.
		@param target The target parameter to overwrite. */
	virtual void Overwrite(FCDEffectParameter* target);

	/** [INTERNAL] Reads in the effect parameter from a given COLLADA XML tree node.
		@param parameterNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the parameter.*/
	virtual FUStatus LoadFromXML(xmlNode* parameterNode);

	/** [INTERNAL] Writes out the effect parameter to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the parameter.
		@return The created element XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

/**
	A COLLADA string effect parameter.
	Contains a single, unanimated string.
*/
class FCOLLADA_EXPORT FCDEffectParameterString : public FCDEffectParameter
{
private:
	string value;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDEffectParameterList::AddParameter function.
		@param document The COLLADA document that owns the effect parameter. */
	FCDEffectParameterString(FCDocument* document);

	/** Destructor: do not use directly.
		Instead, use the FCDEffectParameterList::ReleaseParameter function.
		When released, the effect parameter list will also release all
		its parameters, if it owns them. */
	virtual ~FCDEffectParameterString();

	/** Retrieves the type of effect parameter class.
		@return The parameter class type: STRING. */
	virtual Type GetType() const { return STRING; }

	/** Retrieves the string contained in the effect parameter.
		@return The string. */
	const string& GetValue() const { return value; }

	/** Sets the string contained in the effect parameter.
		@param _value The string. */
	void SetValue(const string& _value) { value = _value; }

	/** Creates a full copy of the effect parameter.
		@return The cloned effect parameter. You will need to delete this pointer. */
	virtual FCDEffectParameter* Clone();

	/** [INTERNAL] Overwrites the target parameter with this parameter.
		This function is used during the flattening of materials.
		@param target The target parameter to overwrite. */
	virtual void Overwrite(FCDEffectParameter* target);

	/** [INTERNAL] Reads in the effect parameter from a given COLLADA XML tree node.
		@param parameterNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the parameter.*/
	virtual FUStatus LoadFromXML(xmlNode* parameterNode);

	/** [INTERNAL] Writes out the effect parameter to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the parameter.
		@return The created element XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

/**
	A COLLADA float effect parameter.
	Contains a single, possibly animated, floating-point value.
	The type of the floating-point value may be HALF or FLOAT.
*/
class FCOLLADA_EXPORT FCDEffectParameterFloat : public FCDEffectParameter
{
public:
	/** The supported types of float-point values. */
	enum FloatType
	{
		FLOAT, /** A full-fledged floating-point value. This is the default. */
		HALF /** Probably implies a 16-bit floating-point value. */
	};

private:
	FloatType floatType;
	float value;
	float min;
	float max;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDEffectParameterList::AddParameter function.
		@param document The COLLADA document that owns the effect parameter. */
	FCDEffectParameterFloat(FCDocument* document);

	/** Destructor: do not use directly.
		Instead, use the FCDEffectParameterList::ReleaseParameter function.
		When released, the effect parameter list will also release all
		its parameters, if it owns them. */
	virtual ~FCDEffectParameterFloat();

	/** Retrieves the type of effect parameter class.
		@return The parameter class type: FLOAT. */
	virtual FCDEffectParameter::Type GetType() const { return FCDEffectParameter::FLOAT; }

	/** Retrieves the type of floating-point value held by this effect parameter.
		@return The type of floating-point value. */
	FloatType GetFloatType() const { return floatType; }

	/** Sets the type of floating-point value held by this effect parameter.
		@param type The type of floating-point value. */
	void SetFloatType(FloatType type) { floatType = type; }

	/** Retrieves the floating-point value of the effect parameter.
		@return The floating-point value. */
	float& GetValue() { return value; }
	const float& GetValue() const { return value; } /**< See above. */

	/** Sets the floating-point value of the effect parameter.
		@param _value The floating-point value. */
	void SetValue(float _value) { value = _value; }

	/** Retrieves the minimum value for the UI widget created for this effect parameter.
		This value is for UI purposes only and has no real impact on the value.
		@return The minimum value. */
	float GetMin() const { return min; }

	/** Sets the minimum value for the UI widget created for this effect parameter.
		This value is for UI purposes only and has no real impact on the value.
		@param _min The minimum value. */
	void SetMin(float _min) { min = _min; }

	/** Retrieves the maximum value for the UI widget created for this effect parameter.
		This value is for UI purposes only and has no real impact on the value.
		@return The maximum value. */
	float GetMax() const { return max; }

	/** Sets the maximum value for the UI widget created for this effect parameter.
		This value is for UI purposes only and has no real impact on the value.
		@param _max The maximum value. */
	void SetMax(float _max) { max = _max; }

	/** Creates a full copy of the effect parameter.
		@return The cloned effect parameter. You will need to delete this pointer. */
	virtual FCDEffectParameter* Clone();

	/** [INTERNAL] Overwrites the target parameter with this parameter.
		This function is used during the flattening of materials.
		@param target The target parameter to overwrite. */
	virtual void Overwrite(FCDEffectParameter* target);

	/** [INTERNAL] Reads in the effect parameter from a given COLLADA XML tree node.
		@param parameterNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the parameter.*/
	virtual FUStatus LoadFromXML(xmlNode* parameterNode);

	/** [INTERNAL] Writes out the effect parameter to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the parameter.
		@return The created element XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

/**
	A COLLADA 2D vector of floats.
	Contains two, possibly animated, floating-point values.
	The type of the floating-point values may be HALF or FLOAT.
*/
class FCOLLADA_EXPORT FCDEffectParameterFloat2 : public FCDEffectParameter
{
public:
	/** The supported types of float-point values. */
	enum FloatType
	{
		FLOAT, /** A full-fledged floating-point value. This is the default. */
		HALF /** Probably implies a 16-bit floating-point value. */
	};

private:
	FloatType floatType;
	float value_x;
	float value_y;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDEffectParameterList::AddParameter function.
		@param document The COLLADA document that owns the effect parameter. */
	FCDEffectParameterFloat2(FCDocument* document);

	/** Destructor: do not use directly.
		Instead, use the FCDEffectParameterList::ReleaseParameter function.
		When released, the effect parameter list will also release all
		its parameters, if it owns them. */
	virtual ~FCDEffectParameterFloat2();

	/** Retrieves the type of effect parameter class.
		@return The parameter class type: FLOAT2. */
	virtual Type GetType() const { return FLOAT2; }

	/** Retrieves the type of floating-point value held by this effect parameter.
		@return The type of floating-point value. */
	FloatType GetFloatType() const { return floatType; }

	/** Sets the type of floating-point value held by this effect parameter.
		@param type The type of floating-point value. */
	void SetFloatType(FloatType type) { floatType = type; }

	/** Retrieves the first floating-point value of the effect parameter.
		@return The first floating-point value. */
	float& GetValueX() { return value_x; }
	const float& GetValueX() const { return value_x; } /**< See above. */

	/** Sets the first floating-point value of the effect parameter.
		@param value The first floating-point value. */
	void SetValueX(float value) { value_x = value; }

	/** Retrieves the second floating-point value of the effect parameter.
		@return The second floating-point value. */
	float& GetValueY() { return value_y; }
	const float& GetValueY() const { return value_y; } /**< See above. */

	/** Sets the second floating-point value of the effect parameter.
		@param value The second floating-point value. */
	void SetValueY(float value) { value_y = value; }

	/** Creates a full copy of the effect parameter.
		@return The cloned effect parameter. You will need to delete this pointer. */
	virtual FCDEffectParameter* Clone();

	/** [INTERNAL] Overwrites the target parameter with this parameter.
		This function is used during the flattening of materials.
		@param target The target parameter to overwrite. */
	virtual void Overwrite(FCDEffectParameter* target);

	/** [INTERNAL] Reads in the effect parameter from a given COLLADA XML tree node.
		@param parameterNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the parameter.*/
	virtual FUStatus LoadFromXML(xmlNode* parameterNode);

	/** [INTERNAL] Writes out the effect parameter to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the parameter.
		@return The created element XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

/**
	A COLLADA 3D vector of floats.
	Contains three, possibly animated, floating-point values.
	The type of the floating-point values may be HALF or FLOAT.
*/
class FCOLLADA_EXPORT FCDEffectParameterFloat3 : public FCDEffectParameter
{
public:
	/** The supported types of float-point values. */
	enum FloatType
	{
		FLOAT, /** A full-fledged floating-point value. This is the default. */
		HALF /** Probably implies a 16-bit floating-point value. */
	};

private:
	FloatType floatType;
	FMVector3 value;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDEffectParameterList::AddParameter function.
		@param document The COLLADA document that owns the effect parameter. */
	FCDEffectParameterFloat3(FCDocument* document);

	/** Destructor: do not use directly.
		Instead, use the FCDEffectParameterList::ReleaseParameter function.
		When released, the effect parameter list will also release all
		its parameters, if it owns them. */
	virtual ~FCDEffectParameterFloat3();

	/** Retrieves the type of effect parameter class.
		@return The parameter class type: FLOAT3. */
	virtual Type GetType() const { return FLOAT3; }

	/** Retrieves the type of floating-point value held by this effect parameter.
		@return The type of floating-point value. */
	FloatType GetFloatType() const { return floatType; }

	/** Sets the type of floating-point value held by this effect parameter.
		@param type The type of floating-point value. */
	void SetFloatType(FloatType type) { floatType = type; }

	/** Retrieves the first floating-point value of the effect parameter.
		@return The first floating-point value. */
	float& GetValueX() { return value.x; }
	const float& GetValueX() const { return value.x; } /**< See above. */

	/** Sets the first floating-point value of the effect parameter.
		@param _value The first floating-point value. */
	void SetValueX(float _value) { value.x = _value; }

	/** Retrieves the second floating-point value of the effect parameter.
		@return The second floating-point value. */
	float& GetValueY() { return value.y; }
	const float& GetValueY() const { return value.y; } /**< See above. */

	/** Sets the second floating-point value of the effect parameter.
		@param _value The second floating-point value. */
	void SetValueY(float _value) { value.y = _value; }

	/** Retrieves the third floating-point value of the effect parameter.
		@return The third floating-point value. */
	float& GetValueZ() { return value.z; }
	const float& GetValueZ() const { return value.z; } /**< See above. */

	/** Sets the third floating-point value of the effect parameter.
		@param _value The third floating-point value. */
	void SetValueZ(float _value) { value.z = _value; }

	/** Creates a full copy of the effect parameter.
		@return The cloned effect parameter. You will need to delete this pointer. */
	virtual FCDEffectParameter* Clone();

	/** [INTERNAL] Overwrites the target parameter with this parameter.
		This function is used during the flattening of materials.
		@param target The target parameter to overwrite. */
	virtual void Overwrite(FCDEffectParameter* target);

	/** [INTERNAL] Reads in the effect parameter from a given COLLADA XML tree node.
		@param parameterNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the parameter.*/
	virtual FUStatus LoadFromXML(xmlNode* parameterNode);

	/** [INTERNAL] Writes out the effect parameter to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the parameter.
		@return The created element XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

/**
	A COLLADA 4D vector of floats.
	Contains four, possibly animated, floating-point values.
	The type of the floating-point values may be HALF or FLOAT.
*/
class FCOLLADA_EXPORT FCDEffectParameterVector : public FCDEffectParameter
{
public:
	/** The supported types of float-point values. */
	enum FloatType
	{
		FLOAT, /** A full-fledged floating-point value. This is the default. */
		HALF /** Probably implies a 16-bit floating-point value. */
	};

private:
	FloatType floatType;
	float vector[4];

public:
	/** Constructor: do not use directly.
		Instead, use the FCDEffectParameterList::AddParameter function.
		@param document The COLLADA document that owns the effect parameter. */
	FCDEffectParameterVector(FCDocument* document);

	/** Destructor: do not use directly.
		Instead, use the FCDEffectParameterList::ReleaseParameter function.
		When released, the effect parameter list will also release all
		its parameters, if it owns them. */
	virtual ~FCDEffectParameterVector();

	/** Retrieves the type of effect parameter class.
		@return The parameter class type: VECTOR. */
	virtual Type GetType() const { return VECTOR; }

	/** Retrieves the type of floating-point value held by this effect parameter.
		@return The type of floating-point value. */
	FloatType GetFloatType() const { return floatType; }

	/** Sets the type of floating-point value held by this effect parameter.
		@param type The type of floating-point value. */
	void SetFloatType(FloatType type) { floatType = type; }

	/** Sets the vector value of the effect parameter.
		@return The vector value. */
	float* GetVector() { return vector; }
	const float* GetVector() const { return vector; } /**< See above. */

	/** Retrieves the first floating-point value of the effect parameter.
		@return The first floating-point value. */
	float& GetValueX() { return vector[0]; }
	const float& GetValueX() const { return vector[0]; } /**< See above. */

	/** Sets the first floating-point value of the effect parameter.
		@param _value The first floating-point value. */
	void SetValueX(float _value) { vector[0] = _value; }

	/** Retrieves the second floating-point value of the effect parameter.
		@return The second floating-point value. */
	float& GetValueY() { return vector[1]; }
	const float& GetValueY() const { return vector[1]; } /**< See above. */

	/** Sets the second floating-point value of the effect parameter.
		@param _value The second floating-point value. */
	void SetValueY(float _value) { vector[1] = _value; }

	/** Retrieves the third floating-point value of the effect parameter.
		@return The third floating-point value. */
	float& GetValueZ() { return vector[2]; }
	const float& GetValueZ() const { return vector[2]; } /**< See above. */

	/** Sets the third floating-point value of the effect parameter.
		@param _value The third floating-point value. */
	void SetValueZ(float _value) { vector[2] = _value; }

	/** Retrieves the fourth floating-point value of the effect parameter.
		@return The fourth floating-point value. */
	float& GetValueW() { return vector[3]; }
	const float& GetValueW() const { return vector[3]; } /**< See above. */

	/** Sets the fourth floating-point value of the effect parameter.
		@param _value The fourth floating-point value. */
	void SetValueW(float _value) { vector[3] = _value; }

	/** Creates a full copy of the effect parameter.
		@return The cloned effect parameter. You will need to delete this pointer. */
	virtual FCDEffectParameter* Clone();

	/** [INTERNAL] Overwrites the target parameter with this parameter.
		This function is used during the flattening of materials.
		@param target The target parameter to overwrite. */
	virtual void Overwrite(FCDEffectParameter* target);

	/** [INTERNAL] Reads in the effect parameter from a given COLLADA XML tree node.
		@param parameterNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the parameter.*/
	virtual FUStatus LoadFromXML(xmlNode* parameterNode);

	/** [INTERNAL] Writes out the effect parameter to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the parameter.
		@return The created element XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

/**
	A COLLADA 4x4 matrix.
	Contains 16 floating-point values that represent a COLLADA column-major 4x4 matrix.
	The type of the floating-point values may be HALF or FLOAT.
*/
class FCOLLADA_EXPORT FCDEffectParameterMatrix : public FCDEffectParameter
{
public:
	/** The supported types of float-point values. */
	enum FloatType
	{
		FLOAT, /** A full-fledged floating-point value. This is the default. */
		HALF /** Probably implies a 16-bit floating-point value. */
	};

private:
	FloatType floatType;
	FMMatrix44 matrix;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDEffectParameterList::AddParameter function.
		@param document The COLLADA document that owns the effect parameter. */
	FCDEffectParameterMatrix(FCDocument* document);

	/** Destructor: do not use directly.
		Instead, use the FCDEffectParameterList::ReleaseParameter function.
		When released, the effect parameter list will also release all
		its parameters, if it owns them. */
	virtual ~FCDEffectParameterMatrix();

	/** Retrieves the type of effect parameter class.
		@return The parameter class type: MATRIX. */
	virtual Type GetType() const { return MATRIX; }

	/** Retrieves the type of floating-point value held by this effect parameter.
		@return The type of floating-point value. */
	FloatType GetFloatType() const { return floatType; }

	/** Sets the type of floating-point value held by this effect parameter.
		@param type The type of floating-point value. */
	void SetFloatType(FloatType type) { floatType = type; }

	/** Retrieves the matrix contained within this effect parameter.
		@return The matrix. */
	FMMatrix44& GetMatrix() { return matrix; }
	const FMMatrix44& GetMatrix() const { return matrix; } /**< See above. */

	/** Sets the matrix contained within this effect parameter.
		@param mx The matrix. */
	void SetMatrix(const FMMatrix44& mx) { matrix = mx; }

	/** Creates a full copy of the effect parameter.
		@return The cloned effect parameter. You will need to delete this pointer. */
	virtual FCDEffectParameter* Clone();

	/** [INTERNAL] Overwrites the target parameter with this parameter.
		This function is used during the flattening of materials.
		@param target The target parameter to overwrite. */
	virtual void Overwrite(FCDEffectParameter* target);

	/** [INTERNAL] Reads in the effect parameter from a given COLLADA XML tree node.
		@param parameterNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the parameter.*/
	virtual FUStatus LoadFromXML(xmlNode* parameterNode);

	/** [INTERNAL] Writes out the effect parameter to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the parameter.
		@return The created element XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

#endif // _FCD_EFFECT_PARAMETER_H_

