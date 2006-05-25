/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FCDEffectPassShader.h
	This file contains the FCDEffectPassShader and the FCDEffectPassBind classes.
*/

#ifndef _FCD_EFFECT_PASS_SHADER_H_
#define _FCD_EFFECT_PASS_SHADER_H_

#include "FCDocument/FCDObject.h"

class FCDocument;
class FCDEffectCode;

/**
	A COLLADA shader binding.

	Binds an external symbol to a COLLADA effect parameter, by reference.

	@ingroup FCDEffect
*/
class FCOLLADA_EXPORT FCDEffectPassBind
{
public:
	string reference; /**< A COLLADA effect parameter reference. */
	string symbol; /**< An external symbol, used within the shader code. */
};

typedef vector<FCDEffectPassBind> FCDEffectPassBindList; /**< A dynamically-sized array of shader bindings. */

/**
	A COLLADA shader.

	The shader abstraction level in ColladaFX is contained within the effect passes.
	There are two types of shaders: vertex shaders and fragment/pixel shaders.
	A COLLADA shader contains a list of bindings to attach the effect parameters to the
	shader input parameters.

	The shader object also contains the compiler information necessary to build
	the shader: its code, the compiler target and the compiler options.
*/
class FCOLLADA_EXPORT FCDEffectPassShader : public FCDObject
{
private:
	FCDEffectPass* parent;

	FCDEffectPassBindList bindings;
	fstring compilerTarget;
	fstring compilerOptions;
	string name;
	bool isFragment;
	FCDEffectCode* code;

public:
	/** Constructor: do not use directly. Instead, use the FCDEffectPass::AddShader,
		FCDEffectPass::AddVertexShader or FCDEffectPass::AddFragmentShader functions.
		@param document The COLLADA document that owns this shader.
		@param parent The effect pass that contains this shader. */
	FCDEffectPassShader(FCDocument* document, FCDEffectPass* parent);

	/** Destructor: do not use directly.
		Instead, use the FCDEffectPass::ReleaseShader function. */
	virtual ~FCDEffectPassShader();

	/** Retrieves the effect pass that contains this shader.
		@return The effect pass. */
	inline FCDEffectPass* GetParent() { return parent; }
	inline const FCDEffectPass* GetParent() const { return parent; } /**< See above. */

	/** Sets this shader as affecting vertices.
		This sets the stage of the shader to the vertex pipeline. */
	inline void AffectsVertices() { isFragment = false; }

	/** Sets this shader as affecting fragments/pixels.
		This sets the stage of the shader to the fragment/pixel pipeline. */
	inline void AffectsFragments() { isFragment = true; }

	/** Retrieves whether this shader affects fragments/pixels.
		@return Whether this shader affects fragments/pixels. */
	inline bool IsFragmentShader() const { return isFragment; }

	/** Retrieves whether this shader affects vertices.
		@return Whether this shader affects vertices. */
	inline bool IsVertexShader() const { return !isFragment; }

	/** Retrieves the list of bindings for this shader.
		@return The list of bindings. */
	inline FCDEffectPassBindList& GetBindings() { return bindings; }
	inline const FCDEffectPassBindList& GetBindings() const { return bindings; } /**< See above. */

	/** Retrieves the number of bindings for this shader.
		@return The number of bindings. */
	inline size_t GetBindingCount() const { return bindings.size(); }
	
	/** Retrieves a binding contained in this shader.
		@param index The index of the binding.
		@return The binding. This pointer will be NULL if the index is out-of-bounds. */
	inline FCDEffectPassBind* GetBinding(size_t index) { FUAssert(index < GetBindingCount(), return NULL); return &bindings.at(index); }
	inline const FCDEffectPassBind* GetBinding(size_t index) const { FUAssert(index < GetBindingCount(), return NULL); return &bindings.at(index); } /**< See above. */

	/** Adds a new binding to this shader.
		@return The new binding. */
	FCDEffectPassBind* AddBinding();

	/** Releases a binding contained within this shader.
		@param binding The binding to release. */
	void ReleaseBinding(FCDEffectPassBind* binding);

	/** Retrieves the compiler target information.
		The validity of this string depends on the type of the profile that contains this shader.
		@return The compiler target information string. */
	inline const fstring& GetCompilerTarget() const { return compilerTarget; }

	/** Sets the compiler target information string.
		The validity of this string depends on the type of the profile that contains this shader.
		@param _compilerTarget The compiler target information. */
	inline void SetCompilerTarget(const fstring& _compilerTarget) { compilerTarget = _compilerTarget; }

	/** Retrieves the compiler option string.
		The validity of this string depends on the type of the profile that contains this shader.
		@return The compiler option string. */
	inline const fstring& GetCompilerOptions() const { return compilerOptions; }

	/** Sets the compiler option string.
		The validity of this string depends on the type of the profile that contains this shader.
		@param _compilerOptions The compiler option string. */
	inline void SetCompilerOptions(const fstring& _compilerOptions) { compilerOptions = _compilerOptions; }

	/** Retrieves the sub-id of the shader.
		@return The sub-id. */
	inline const string& GetName() const { return name; }

	/** Sets the sub-id of the shader.
		@param _name The sub-id. */
	inline void SetName(const string& _name) { name = _name; }

	/** Retrieves the code inclusion that contains the code for this shader.
		@return The code inclusion. This pointer will be NULL if this shader
			is not yet attached to any code. */
	inline FCDEffectCode* GetCode() { return code; }
	inline const FCDEffectCode* GetCode() const { return code; } /**< See above. */

	/** Sets the code inclusion that contains the code for this shader.
		@param _code The code inclusion. This pointer will be NULL to detach
			 a shader from its code. */
	inline void SetCode(FCDEffectCode* _code) { code = _code; }

	/** [INTERNAL] Clones this shader. You must manually delete the clone.
		@param newParent The effect pass that will contain the clone.
		@return The cloned shader. */
	FCDEffectPassShader* Clone(FCDEffectPass* newParent) const;

	/** [INTERNAL] Reads in the pass shader from a given COLLADA XML tree node.
		@param shaderNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the shader.*/
	FUStatus LoadFromXML(xmlNode* shaderNode);

	/** [INTERNAL] Writes out the pass shader to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the effect profile.
		@return The created element XML tree node. */
	xmlNode* WriteToXML(xmlNode* parentNode) const;
};

#endif // _FCD_EFFECT_PASS_SHADER_H_
