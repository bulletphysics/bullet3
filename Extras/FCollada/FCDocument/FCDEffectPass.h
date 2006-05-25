/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FCDEffectPass.h
	This file contains the FCDEffectPass class.
*/

#ifndef _FCD_EFFECT_PASS_H_
#define _FCD_EFFECT_PASS_H_

#include "FCDocument/FCDObject.h"

class FCDEffectTechnique;
class FCDEffectParameter;
class FCDEffectParameterList;
class FCDEffectPassShader;

typedef vector<FCDEffectPassShader*> FCDEffectPassShaderList; /**< A dynamically-sized array of shaders. */
typedef vector<string> MeshDataList; /**< @deprecated A dynamically-sized array of mesh bindings. These should be bound using the \<bind\> element, at the instantiation level! */

/**
	A COLLADA effect pass.

	The effect pass contains a list of effect shaders. While they
	may be missing, it does not make sense for the effect pass to
	contain more than two shaders: a vertex shader and a fragment/pixel shader.

	For this reason, we provide the GetVertexShader and the GetFragmentShader
	which we expect will be used for most applications, rather than looking
	through the list of shader objects.
	
	@ingroup FCDEffect
*/
class FCOLLADA_EXPORT FCDEffectPass : public FCDObject
{
private:
	fstring name;
	FCDEffectTechnique* parent;
	FCDEffectPassShaderList shaders;
	MeshDataList meshdata;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDEffectTechnique::AddPass function.
		@param document The COLLADA document that owns this effect pass.
		@param _parent The effect technique that contains this effect pass. */
	FCDEffectPass(FCDocument* document, FCDEffectTechnique *_parent);

	/** Destructor: do not use directly.
		Instead, use the FCDEffectTechnique::ReleasePass function. */
	virtual ~FCDEffectPass();

	/** Retrieves the effect techniques which contains this effect pass.
		@return The parent technique. */
	FCDEffectTechnique* GetParent() { return parent; }
	const FCDEffectTechnique* GetParent() const { return parent; } /**< See above. */

	/** Retrieves the COLLADA id of the parent effect.
		This function is mostly useful as a shortcut for debugging and reporting.
		@return The COLLADA id of the parent effect. */
	const string& GetDaeId() const;

	/** Retrieves the sub-id of the effect pass.
		This sub-id is optional.
		@return The sub-id. */
	const fstring& GetPassName() const { return name; }

	/** Sets the optional sub-id for the effect pass.
		This sub-id is optional.
		@param _name The sub-id. */
	void SetPassName(const fstring& _name) { name = _name; }

	/** @deprecated Retrieves the list of mesh data bindings.
		This patches bad export data in ColladaMaya and will be removed soon.
		@return The list of mesh data bindings. */
	const MeshDataList& GetMeshData() const { return meshdata; }

	/** Retrieves the number of shaders contained within the effect pass.
		@return The number of shaders. */
	size_t GetShaderCount() const { return shaders.size(); }

	/** Retrieves a specific shader.
		@param index The index of the shader.
		@return The shader. This pointer will be NULL if the index is out-of-bounds. */
	FCDEffectPassShader* GetShader(size_t index) { FUAssert(index < GetShaderCount(), return NULL); return shaders.at(index); }
	const FCDEffectPassShader* GetShader(size_t index) const { FUAssert(index < GetShaderCount(), return NULL); return shaders.at(index); } /**< See above. */

	/** Adds a new shader to the pass.
		@return The new shader. */
	FCDEffectPassShader* AddShader();

	/** Releases a shader contained within the pass.
		@param shader The shader to release. */
	void ReleaseShader(FCDEffectPassShader* shader);

	/** Retrieves the vertex shader for this effect pass.
		@return The vertex shader. This pointer will be NULL if no
			shader within the pass affects vertices. */
	FCDEffectPassShader* GetVertexShader();
	const FCDEffectPassShader* GetVertexShader() const; /**< See above. */

	/** Retrieves the fragment shader for this effect pass.
		@return The fragment shader. This pointer will be NULL if no
			shader within the pass affects pixels/fragments. */
	FCDEffectPassShader* GetFragmentShader();
	const FCDEffectPassShader* GetFragmentShader() const; /**< See above. */

	/** Adds a new vertex shader to the pass.
		If a vertex shader already exists within the pass, it will be released.
		@return The new vertex shader. */
	FCDEffectPassShader* AddVertexShader();

	/** Adds a new fragment shader to the pass.
		If a fragment shader already exists within the pass, it will be released.
		@return The new fragment shader. */
	FCDEffectPassShader* AddFragmentShader();

	/** [INTERNAL] Clones the full effect pass.
		@param newParent The effect technique that will contain the cloned profile.
		@return The cloned pass. This pointer will never be NULL. */
	FCDEffectPass* Clone(FCDEffectTechnique* newParent) const;

	/** [INTERNAL] Reads in the effect pass from a given COLLADA XML tree node.
		@param passNode The COLLADA XML tree node.
		@param techniqueNode X @deprecated bad interface : this dependency must be taken out[3]
		@param profileNode X @deprecated bad interface : this dependency must be taken out[2]
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the effect pass.*/
	FUStatus LoadFromXML(xmlNode* passNode, xmlNode* techniqueNode, xmlNode* profileNode);

	/** [INTERNAL] Writes out the effect pass to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the effect pass.
		@return The created element XML tree node. */
	xmlNode* WriteToXML(xmlNode* parentNode) const;
};

#endif
