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

/**
	@file FCDMaterial.h
	This file contains the FCDMaterail class and the FCDMaterialTechniqueHint structure.
*/

#ifndef _FCD_MATERIAL_H_
#define _FCD_MATERIAL_H_

#include "FCDocument/FCDEntity.h"

class FCDocument;
class FCDEffect;
class FCDEffectParameter;
class FCDEffectParameterList;

/**
	A technique usage hint for a material.
	This structure contains two strings to help applications
	choose a technique within the material's instantiated effect
	according to their application platform.
*/
class FCOLLADA_EXPORT FCDMaterialTechniqueHint
{
public:
	fstring platform; /**< A platform semantic. COLLADA defines no platform semantics. */
	string technique; /**< The sid for the technique to choose for the platform. */
};

/** A dynamically-sized list of material platform-technique hints. */
typedef vector<FCDMaterialTechniqueHint> FCDMaterialTechniqueHintList; 

/**
	A COLLADA material.

	A COLLADA material is one of many abstraction level that defines how
	to render mesh polygon sets. It instantiates an effect and may
	overrides some of the effect parameters with its own values.

	Unless you care about the construction history or memory, you should probably
	use the FCDMaterialInstance::FlattenMaterial function.

	@ingroup FCDocument
*/
class FCOLLADA_EXPORT FCDMaterial : public FCDEntity
{
private:
	bool ownsEffect;
	FCDEffect* effect;
	FCDEffectParameterList* parameters;
	FCDMaterialTechniqueHintList techniqueHints;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDMaterialLibrary::AddMaterial function.
		@param document The COLLADA document that owns the material. */
	FCDMaterial(FCDocument* document);

	/** Destructor: do not use directly.
		The material library will release all the materials when it is
		released. If you want to remove a material from the material library:
		use the FCDMaterialLibrary::RemoveMaterial function. */
	virtual ~FCDMaterial();

	/** Retrieves the entity type for this class. This function is part
		of the FCDEntity class interface.
		@return The entity type: MATERIAL. */
	virtual Type GetType() const { return FCDEntity::MATERIAL; }

	/** Retrieves the effect instantiated for this material.
		The parameters of the effect may be overwritten by this material.
		You should either flatten the material using the FlattenMaterial function
		or verify the parameter values manually using the parameter list accessors.
		@return The instantiated effect. This pointer will be NULL if the material has no rendering. */
	FCDEffect* GetEffect() { return effect; }
	const FCDEffect* GetEffect() const { return effect; } /**< See above. */

	/** Sets the effect instantiated for this material.
		@param _effect The effect instantiated for this material. */
	void SetEffect(FCDEffect* _effect) { effect = _effect; }

	/** Retrieves the list of the material platform-technique hints.
		@return The list of material platform-technique hints. */
	FCDMaterialTechniqueHintList& GetTechniqueHints() { return techniqueHints; }
	const FCDMaterialTechniqueHintList& GetTechniqueHints() const { return techniqueHints; } /**< See above. */

	/** Retrieves the list of effect parameter overrides.
		@return The list of effect parameter overrides. */
	FCDEffectParameterList* GetParameters() { return parameters; }
	const FCDEffectParameterList* GetParameters() const { return parameters; } /**< See above. */

	/** Retrieves an effect parameter override. Looks for the effect parameter override with the correct
		semantic, in order to bind or set its value. This function searches through the material and the
		level of abstractions below.
		@param semantic The effect parameter semantic to match.
		@return The effect parameter override that matches the semantic.
			This pointer will be NULL if no effect parameter override matches
			the given semantic. */
	FCDEffectParameter* FindParameterBySemantic(const string& semantic);

	/** Retrieves a subset of the effect parameter override list.
		Look for the effect parameter overrides with the correct semantic.
		This function searches through the material and the level of abstractions below.
		@param semantic The effect parameter semantic to match.
		@param parameters The list of parameters to fill in. This list is not cleared. */
	void FindParametersBySemantic(const string& semantic, FCDEffectParameterList& parameters);

	/** Retrieves a subset of the effect parameter override list.
		Look for the effect parameter overrides with the correct reference.
		This function searches through the material and the level of abstractions below.
		@param reference The effect parameter reference to match. In the case of effect
			parameter generators, the reference is replaced by the sub-id.
		@param parameters The list of parameters to fill in. This list is not cleared. */
	void FindParametersByReference(const string& reference, FCDEffectParameterList& parameters);

	/** [INTERNAL] Clones the material object. Everything is cloned, including the effect parameter.
		You will need release the cloned material directly, by deleting the pointer.
		@return The cloned material object. You will must delete this pointer. */
	FCDMaterial* Clone();

	/** [INTERNAL] Flattens the material, pushing all the effect parameter overrides
		into the effect parameter generators and moving all the parameters to the 
		effect technique level of abstraction. To flatten the material, use the
		FCDMaterialInstance::FlattenMaterial function. */
	void Flatten();

	/** [INTERNAL] Reads in the \<material\> element from a given COLLADA XML tree node.
		@param materialNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the material.*/
	virtual FUStatus LoadFromXML(xmlNode* materialNode);

	/** [INTERNAL] Writes out the \<material\> element to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the material declaration.
		@return The created element XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;

private:
	void AddParameter(FCDEffectParameter* parameter);
};

#endif // _FCD_MATERIAL_H_
