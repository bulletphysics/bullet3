/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

/**
	@file FCDEffectProfile.h
	This file contains the FCDEffectProfile abstract class.
*/

#ifndef _FCD_EFFECT_PROFILE_H_
#define _FCD_EFFECT_PROFILE_H_

#include "FUtils/FUDaeEnum.h"
#include "FCDocument/FCDObject.h"

class FCDocument;
class FCDEffect;

/**
	The base for a COLLADA effect profile.

	COLLADA has multiple effect profiles: CG, HLSL, GLSL, GLES and the COMMON profile.
	For each profile, there is a class which implements this abstract class.
	This abstract class solely holds the parent effect and allows access to the
	profile type.

	@see FCDEffectProfileFX FCDEffectStandard
	
	@ingroup FCDEffect
*/
class FCOLLADA_EXPORT FCDEffectProfile : public FCDObject
{
private:
	FCDEffect* parent;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDEffect::AddProfile function.
		@param document The COLLADA document that owns this effect profile.
		@param parent The effect which contains this profile. */
	FCDEffectProfile(FCDocument* document, FCDEffect* parent);

	/** Destructor: do not use directly.
		Instead, use the FCDEffect::ReleaseProfile function. */
	virtual ~FCDEffectProfile();

	/** Retrieves the profile type for this effect.
		This function allows you to up-cast the pointer safely to a more specific
		effect profile class.
		@return The profile type. */
	virtual FUDaeProfileType::Type GetType() const = 0;

	/** Retrieves the parent effect.
		This is the effect which contains this profile.
		@return The parent effect. This pointer will never be NULL. */
	FCDEffect* GetParent() { return parent; }
	const FCDEffect* GetParent() const { return parent; } /**< See above. */

	/** [INTERNAL] Retrieves the COLLADA id of the parent effect.
		This function is useful when reporting errors and warnings.
		@return The COLLADA id of the parent effect. */
	const string& GetDaeId() const;

	/** Retrieves an effect parameter. Looks for the effect parameter with the correct
		semantic, in order to bind or override its value.
		This function searches through the effect profile and the level of abstractions below.
		@param semantic The effect parameter semantic to match.
		@return The effect parameter that matches the semantic. This pointer will be NULL
			if no effect parameter matches the given semantic. */
	virtual FCDEffectParameter* FindParameterBySemantic(const string& semantic) = 0;

	/** Retrieves a subset of the effect parameter list.
		Look for effect parameters with the correct semantic.
		This function searches through the effect profile and the level of abstractions below.
		@param semantic The effect parameter semantic to match.
		@param parameters The list of parameters to fill in. This list is not cleared. */
	virtual void FindParametersBySemantic(const string& semantic, FCDEffectParameterList& parameters) = 0;

	/** Retrieves a subset of the effect parameter list.
		Look for effect parameters with the correct reference.
		This function searches through the effect profile and the level of abstractions below.
		@param reference The effect parameter reference to match. In the case of effect
			parameter generators, the reference is replaced by the sub-id.
		@param parameters The list of parameters to fill in. This list is not cleared. */
	virtual void FindParametersByReference(const string& reference, FCDEffectParameterList& parameters) = 0;

	/** [INTERNAL] Clones the profile effect and its parameters.
		@param newParent The effect that will contain the cloned profile.
		@return The cloned profile. This pointer will never be NULL. */
	virtual FCDEffectProfile* Clone(FCDEffect* newParent) = 0;

	/** [INTERNAL] Flattens this effect profile, pushing all the effect parameter overrides
		into the effect parameter generators and moving all the parameters to the 
		effect technique level of abstraction. To flatten the material, use the
		FCDMaterialInstance::FlattenMaterial function. */
	virtual void Flatten() = 0;

	/** [INTERNAL] Reads in the effect profile from a given COLLADA XML tree node.
		@param profileNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the effect profile.*/
	virtual FUStatus LoadFromXML(xmlNode* profileNode) = 0;

	/** [INTERNAL] Writes out the effect profile to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the effect profile.
		@return The created element XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const = 0;
};

#endif // _FCD_EFFECT_PROFILE_H_
