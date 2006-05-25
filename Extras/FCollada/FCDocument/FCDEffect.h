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
	@file FCDEffect.h
	This file contains the FCDEffect class.
*/

#ifndef _FCD_EFFECT_H_
#define _FCD_EFFECT_H_

#include "FUtils/FUDaeEnum.h"
#include "FCDocument/FCDEntity.h"

class FCDocument;
class FCDEffectStandard;
class FCDEffectParameter;
class FCDEffectProfile;
class FCDEffectParameterList;

/**	
	@defgroup FCDEffect COLLADA Effect Classes [ColladaFX]
*/

class FCDEffectProfile;
class FCDEffectParameterList;
class FCDImage;

/** A dynamically-sized array of effect profiles. */
typedef vector<FCDEffectProfile*> FCDEffectProfileList;
typedef vector<FCDImage*> FCDEffectImageList;

/**
	A COLLADA effect.
	
	A COLLADA effect is one of many abstraction level that defines how
	to render mesh polygon sets. It contains one or more rendering profile
	that the application can choose to support. In theory, all the rendering
	profiles should reach the same render output, using different
	rendering technologies.

	An effect may also declare new general purpose parameters that are common
	to all the profiles.

	@ingroup FCDEffect
*/
class FCOLLADA_EXPORT FCDEffect : public FCDEntity
{
private:
	FCDEffectProfileList profiles;
	FCDEffectParameterList* parameters;
	FCDEffectImageList images;

public:
	/** Constructor: do not use directly.
		Instead use the FCDMaterialLibrary::AddEffect function.
		@param document The COLLADA document that owns this effect. */
	FCDEffect(FCDocument* document);

	/** Destructor: do not use directly.
		Instead use the FCDMaterialLibrary::ReleaseEffect function. */
	virtual ~FCDEffect();

	/** Retrieves the type for this entity class.
		This function is a part of the FCDEntity interface.
		@return The entity type: EFFECT. */
	virtual Type GetType() const { return FCDEntity::EFFECT; }

	/** Retrieves the number of profiles contained within the effect.
		@return The number of profiles within the effect. */
	size_t GetProfileCount() const { return profiles.size(); }

	/** Retrieves a profile contained within the effect.
		@param index The index of the profile.
		@return The profile. This pointer will be NULL, if the given index is out-of-bounds. */
	FCDEffectProfile* GetProfile(size_t index) { FUAssert(index < GetProfileCount(), return NULL); return profiles.at(index); }
	const FCDEffectProfile* GetProfile(size_t index) const { FUAssert(index < GetProfileCount(), return NULL); return profiles.at(index); } /**< See above. */

	/** Retrieves the list of the profiles contained within the effect.
		@return The list of effect profiles. */
	FCDEffectProfileList& GetProfiles() { return profiles; }
	const FCDEffectProfileList& GetProfiles() const { return profiles; } /**< See above. */

	/** Retrieves the profile for a specific profile type.
		There should only be one profile of each type within an effect. This
		function allows you to retrieve the profile for a given type.
		@param type The profile type.
		@return The profile of this type. This pointer will be NULL if the effect
			does not have any profile of this type. */
	FCDEffectProfile* FindProfile(FUDaeProfileType::Type type);
	const FCDEffectProfile* FindProfile(FUDaeProfileType::Type type) const; /**< See above. */

	/** Retrieves whether the effect contains a profile of the given type.
		@param type The profile type.
		@return Whether the effect has a profile of this type. */
	inline bool HasProfile(FUDaeProfileType::Type type) const { return FindProfile(type) != NULL; }

	/** Creates a profile of the given type.
		If a profile of this type already exists, it will be released, as
		a COLLADA effect should only contain one profile of each type.
		@param type The profile type.
		@return The new effect profile. */
	FCDEffectProfile* AddProfile(FUDaeProfileType::Type type);

	/** Releases the given effect profile.
		@param profile The effect profile. */
	void ReleaseProfile(FCDEffectProfile* profile);

	/** Retrieves the list of common effect parameters declared at the effect level.
		According to the COLLADA 1.4 schema, you should expect only parameter generators
		at this abstraction level.
		@return The list of effect parameters. */
	FCDEffectParameterList* GetParameters() { return parameters; }
	const FCDEffectParameterList* GetParameters() const { return parameters; } /**< See above. */

	/** [INTERNAL] Inserts an existing parameter into the list of common effect parameters
		at this abstraction level. This function is used during the flattening of a material.
		@param parameter The effect parameter to insert. */
	void AddParameter(FCDEffectParameter* parameter);

	/** Retrieves a common effect parameter. Looks for the common effect parameter with the correct
		semantic, in order to bind or override its value.
		This function searches through the effect and the level of abstractions below.
		@param semantic The effect parameter semantic to match.
		@return The first effect parameter that matches the semantic.
			This pointer will be NULL if no effect parameter matches the given semantic. */
	FCDEffectParameter* FindParameterBySemantic(const string& semantic);

	/** Retrieves a subset of the common effect parameter list.
		Look for the effect parameter generators with the correct semantic.
		This function searches through the effect and the level of abstractions below.
		@param semantic The effect parameter semantic to match.
		@param parameters The list of parameters to fill in. This list is not cleared. */
	void FindParametersBySemantic(const string& semantic, FCDEffectParameterList& parameters);

	/** Retrieves a subset of the common effect parameter list.
		Look for the effect parameter generators with the correct reference.
		This function searches through the effect and the level of abstractions below.
		@param reference The effect parameter reference to match. In the case of effect
			parameter generators, the reference is replaced by the sub-id.
		@param parameters The list of parameters to fill in. This list is not cleared. */
	void FindParametersByReference(const string& reference, FCDEffectParameterList& parameters);

	/** Clones the effect object. Everything is cloned, including the effect parameter
		and their animations. You will need release the cloned effect directly, by deleting the pointer.
		@return The cloned effect object. You will must delete this pointer. */
	FCDEffect* Clone();

	/** [INTERNAL] Flattens the effect, pushing all the common effect parameters
		into to the effect technique level of abstraction. To correctly flatten a
		material, use the FCDMaterialInstance::FlattenMaterial function. */
	void Flatten();

	/** [INTERNAL] Reads in the \<effect\> element from a given COLLADA XML tree node.
		@param effectNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the effect.*/
	virtual FUStatus LoadFromXML(xmlNode* effectNode);

	/** [INTERNAL] Writes out the \<effect\> element to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the effect.
		@return The created element XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
	
	/** Retrieves the list of the images contained within the effect.
		@return The list of effect images. */
	FCDEffectImageList& GetImages() { return images; }
	const FCDEffectImageList& GetImages() const { return images; } /**< See above. */
};

#endif // _FCD_MATERIAL_H_
