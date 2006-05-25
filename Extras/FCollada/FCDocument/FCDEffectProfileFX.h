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
	@file FCDEffectProfileFX.h
	This file declares the FCDEffectProfileFX class.
*/

#ifndef _FCD_EFFECT_PROFILE_FX_H_
#define _FCD_EFFECT_PROFILE_FX_H_

#include "FUtils/FUDaeEnum.h"
#include "FCDocument/FCDEffectProfile.h"

class FCDocument;
class FCDEffect;
class FCDEffectCode;
class FCDEffectParameter;
class FCDEffectParameterSurface;
class FCDEffectTechnique;
class FCDEffectParameterList;

typedef vector<FCDEffectTechnique*> FCDEffectTechniqueList; /**< A dynamically-sized array of effect techniques. */
typedef vector<FCDEffectCode*> FCDEffectCodeList; /**< A dynamically-sized array of effect code inclusion. */

/**
	A general effect profile description.

	The general effect profile contains all the information necessary
	to implement the advanced effect profiles, such as CG, HLSL, GLSL and GLES.
	Since these effect profiles contains extremely similar information, they
	use the same description structure. For the COMMON profile,
	see the FCDEffectStandard class.

	You should use the GetType function to figure out which profile this structure
	addresses. You can then retrieve one or many of the FCDEffectTechnique objects
	that describe how to render for this profile. You may want to check the
	FCDEffectMaterialTechniqueHint objects at the FCDMaterial level, in order to
	determine which technique(s) to use for your platform. At the profile
	level of abstraction, parameters may be generated within the FCDEffectParamterList.

	@ingroup FCDEffect
*/
class FCOLLADA_EXPORT FCDEffectProfileFX : public FCDEffectProfile
{
private:
	FUDaeProfileType::Type type;
	string includeFilename;
	fstring platform;

	FCDEffectCodeList codes;
	FCDEffectTechniqueList techniques;
	FCDEffectParameterList* parameters;

public:
	/** Constructor: do not use directly. Instead, use the FCDEffect::AddProfile function.
		@param document The COLLADA document which owns the effect profile.
		@param parent The effect which contains this profile.
		@param type The type of profile. */
	FCDEffectProfileFX(FCDocument* document, FCDEffect* parent, FUDaeProfileType::Type type);

	/** Destructor: do not use directly. Instead, use the FCDEffect:RemoveProfile function. */
	virtual ~FCDEffectProfileFX();

	/** Retrieves the profile type for this effect.
		This function is a part of the FCDEffectProfile interface and allows you
		to up-cast an effect profile pointer safely to this class.
		@return The profile type. This should never be the value: 'COMMON',
			but all other profiles currently derive from this class. */
	virtual FUDaeProfileType::Type GetType() const { return type; }

	/** @deprecated Retrieves the filename of the file that contains the code for this effect profile.
		Instead, look through the FCDEffectCode object, using the GetCodeList function and retrieve
		the correct object and its filename string.
		@return The filename of the file to import. */
	const string& GetIncludeFilename() const { return includeFilename; }

	/** Retrieves the name of the platform in which to use the effect profile.
		This parameter is very optional.
		@return The platform name. */
	const fstring& GetPlatform() const { return platform; }

	/** Sets the name of the platform in which to use the effect profile.
		This parameter is very optional.
		@param _platform The platform name. */
	void SetPlatform(fstring& _platform) { platform = _platform; }

	/** Retrieves the list of techniques contained within this effect profile.
		You may want to check the FCDEffectMaterialTechniqueHint objects at the FCDMaterial level,
		in order to determine which technique(s) to use for your platform.
		@return The list of inner techniques. */
	FCDEffectTechniqueList& GetTechniqueList() { return techniques; }
	const FCDEffectTechniqueList& GetTechniqueList() const { return techniques; } /**< See above. */

	/** Retrieves the number of techniques contained within this effect profile.
		@return The number of inner techniques. */
	size_t GetTechniqueCount() const { return techniques.size(); }

	/** Retrieves a technique contained within this effect profile.
		You may want to check the FCDEffectMaterialTechniqueHint objects at the FCDMaterial level,
		in order to determine which technique(s) to use for your platform.
		@param index The index of the technique.
		@return The inner technique. This pointer will be NULL if the index is out-of-bounds. */
	FCDEffectTechnique* GetTechnique(size_t index) { FUAssert(index < GetTechniqueCount(), return NULL); return techniques.at(index); }
	const FCDEffectTechnique* GetTechnique(size_t index) const { FUAssert(index < GetTechniqueCount(), return NULL); return techniques.at(index); } /**< See above. */

	/** Adds a new technique to this effect profile.
		@return The new technique object. */
	FCDEffectTechnique* AddTechnique();

	/** Releases a technique contained within this effect profile.
		@param technique The technique to release. */
	void ReleaseTechnique(FCDEffectTechnique* technique);

	/** Retrieves the list of code inclusions.
		@return The list of code inclusions. */		
	FCDEffectCodeList& GetCodeList() { return codes; }
	const FCDEffectCodeList& GetCodeList() const { return codes; } /**< See above. */

	/** Retrieves the number of code inclusions contained within the effect profile.
		@return The number of code inclusions. */
	size_t GetCodeCount() const { return codes.size(); }

	/** Retrieves a code inclusion contained within the effect profile.
		@param index The index of the code inclusion.
		@return The code inclusion. This pointer will be NULL if the index is out-of-bounds. */
	FCDEffectCode* GetCode(size_t index) { FUAssert(index < GetCodeCount(), return NULL); return codes.at(index); }
	const FCDEffectCode* GetCode(size_t index) const { FUAssert(index < GetCodeCount(), return NULL); return codes.at(index); } /**< See above. */

	/** Retrieves the code inclusion with the given sub-id.
		@param sid A COLLADA sub-id.
		@return The code inclusion with the given sub-id. This pointer will be NULL,
			if there are no code inclusions that match the given sub-id. */
	FCDEffectCode* FindCode(const string& sid);
	const FCDEffectCode* FindCode(const string& sid) const; /**< See above. */

	/** Adds a new code inclusion to this effect profile.
		@return The new code inclusion. */
	FCDEffectCode* AddCode();

	/** Releases a code inclusion contained within this effect profile.
		@param code The code inclusion to release. */
	void ReleaseCode(FCDEffectCode* code);

	/** Retrieves the list of effect parameters contained within the effect profile.
		At this level of abstraction, there should be only effect parameter generators.
		@return The list of effect parameters. */
	FCDEffectParameterList* GetParameters() { return parameters; }
	const FCDEffectParameterList* GetParameters() const { return parameters; } /**< See above. */

	/** [INTERNAL] Inserts an existing parameter into the list of effect parameters
		at this abstraction level. This function is used during the flattening of a material.
		@param parameter The effect parameter to insert. */
	void AddParameter(FCDEffectParameter* parameter);

	/** Retrieves an effect parameter.
		Looks for the effect parameter with the correct reference, in order to bind or override its value.
		This function searches through the effect profile and the level of abstractions below.
		@param reference The reference to match. In the case of effect parameter generators,
			the sub-id is used to match.
		@return The first effect parameter that matches the reference.
			This pointer will be NULL if no effect parameter matches the given semantic. */
	const FCDEffectParameter* FindParameter(const char* reference) const;

	/** Retrieves an effect parameter.
		Looks for the effect parameter with the correct semantic, in order to bind or override its value.
		This function searches through the effect profile and the level of abstractions below.
		@param semantic The effect parameter semantic to match.
		@return The first effect parameter that matches the semantic.
			This pointer will be NULL if no effect parameter matches the given semantic. */
	virtual FCDEffectParameter* FindParameterBySemantic(const string& semantic);

	/** Retrieves a subset of the effect parameter list.
		Look for the effect parameter generators with the correct semantic.
		This function searches through the effect profile and the level of abstractions below.
		@param semantic The effect parameter semantic to match.
		@param parameters The list of parameters to fill in. This list is not cleared. */
	virtual void FindParametersBySemantic(const string& semantic, FCDEffectParameterList& parameters);

	/** Retrieves a subset of the effect parameter list.
		Look for the effect parameter generators with the correct reference.
		This function searches through the effect profile and the level of abstractions below.
		@param reference The effect parameter reference to match. In the case of effect
			parameter generators, the reference is replaced by the sub-id.
		@param parameters The list of parameters to fill in. This list is not cleared. */
	virtual void FindParametersByReference(const string& reference, FCDEffectParameterList& parameters);

	/** [INTERNAL] Clones the full effect profile.
		@param newParent The effect that will contain the cloned profile.
		@return The cloned profile. This pointer will never be NULL. */
	virtual FCDEffectProfile* Clone(FCDEffect* newParent);

	/** [INTERNAL] Flattens this effect profile. Pushes all the effect parameter overrides
		into the effect parameter generators and moves all the parameters to the 
		effect technique level of abstraction. To flatten the material, use the
		FCDMaterialInstance::FlattenMaterial function. */
	virtual void Flatten();

	/** [INTERNAL] Reads in the effect profile from a given COLLADA XML tree node.
		@param profileNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the effect profile.*/
	virtual FUStatus LoadFromXML(xmlNode* profileNode);

	/** [INTERNAL] Writes out the effect profile to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the material declaration.
		@return The created element XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode)  const;
};

#endif // _FCD_EFFECT_PROFILE_H_
