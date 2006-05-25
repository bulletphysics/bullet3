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
	@file FCDEffectStandard.h
	This file contains the FCDEffectStandard class.
*/

#ifndef _FCD_MATERIAL_STANDARD_H_
#define _FCD_MATERIAL_STANDARD_H_

#include "FUtils/FUDaeEnum.h"
#include "FCDocument/FCDEffectProfile.h"

class FCDocument;
class FCDEffect;
class FCDEffectParameter;
class FCDTexture;
class FCDEffectParameterList;

/** A dynamically-sized array of texture objects. */
typedef vector<FCDTexture*> FCDTextureList;

/**
	A COMMON profile effect description.
	
	The COMMON effect profile holds the information necessary
	to render your polygon sets using the well-defined lighting models.

	COLLADA supports four lighting models: constant, Lambert, Phong and Blinn.

	@ingroup FCDEffect
*/
class FCOLLADA_EXPORT FCDEffectStandard : public FCDEffectProfile
{
public:
	/** The list of the lighting models supported by the COMMON profile of COLLADA. */
	enum LightingType
	{
		/** The constant lighting model.
			This lighting model uses the emissive color everywhere, without
			any complex lighting calculations. It also uses the translucency
			factor and the translucency color, by multiplying them together
			and applying them to your standard alpha channel according to the
			final lighting color.*/
		CONSTANT, 

		/** The Lambert lighting model.
			This lighting model improves on the constant lighting model by
			using the dot-product between the normalized light vectors and the
			polygon normals to determine how much light should affect each polygon.
			This value is multiplied to the diffuse color and (1 + the ambient color). */
		LAMBERT,

		/** The Phong lighting model.
			This lighting model improves on the Lambert lighting model by
			calculating how much light is reflected by the polygons into the viewer's eye.
			For this calculation, the shininess, the specular color and the reflectivity is used. */
		PHONG,

		/** The Blinn lighting model.
			This lighting model improves on the Lambert lighting model by
			calculating how much light is reflected by the polygons into the viewer's eye.
			For this calculation, the shininess, the specular color and the reflectivity is used. */
		BLINN,

		/** Not a valid lighting model. */
		UNKNOWN
	};

private:
	LightingType type;
	FCDTextureList* textureBuckets;

	// Common material parameter: Emission
	FMVector3 emissionColor;
	float emissionFactor; // Max-specific
	
	// Common material parameter: Translucency
	FMVector3 translucencyColor;
	float translucencyFactor;

	// Lambert material parameters
	FMVector3 diffuseColor;
	FMVector3 ambientColor;

	// Phong material parameters: Specular
	FMVector3 specularColor;
	float specularFactor; // Max-specific
	float shininess;

	// Phong material parameter: Reflectivity
	FMVector3 reflectivityColor; // Maya-specific
	float reflectivityFactor; // Maya-specific

	// Geometry modifier
	bool isFaceted; // Max-specific
	bool isDoubleSided; // Max-specific for now
	bool isWireframe; // Max-specific
	bool isFaceMap; // Max-specific
	bool isEmissionFactor; // Max-specific

public:
	/** Constructor: do not use directly. Instead, use the FCDEffect::AddProfile function
		with the FUDaeProfileType::COMMON parameter.
		@param document The COLLADA document that owns this effect profile.
		@param parent The effect that contains this profile. */
	FCDEffectStandard(FCDocument* document, FCDEffect* parent);

	/** Destructor: do not use directly.
		Instead, use the FCDEffect::ReleaseProfile function. */
	virtual ~FCDEffectStandard();

	/** Retrieves the lighting model to be used for this profile.
		@return The lighting model. */
	inline LightingType GetLightingType() const { return type; }

	/** Sets the lighting model to be used for this profile.
		Note that which parameters are exported depends on the lighting model.
		@param _type The lighting model. */
	inline void SetLightingType(LightingType _type) { type = _type; }

	/** Retrieves the profile type for this effect.
		This function is a part of the FCDEffectProfile interface and allows you
		to up-cast an effect profile pointer safely to this class.
		@return The profile type: COMMON. */
	virtual FUDaeProfileType::Type GetType() const { return FUDaeProfileType::COMMON; }

	/** Retrieves the list of textures belonging to a specific channel.
		@param bucket A texture channel index. This index should match one
			of the values in the FUDaeTextureChannel enum.
		@return The list of textures for this channel. */
	const FCDTextureList& GetTextureBucket(uint32 bucket) const;

	/** Retrieves the number of textures belonging to a specific channel.
		@param bucket A texture channel index. This index should match one
			of the values in the FUDaeTextureChannel enum.
		@return The number of textures in that channel. */
	size_t GetTextureCount(uint32 bucket) const { FUAssert(bucket < FUDaeTextureChannel::COUNT, return 0); return textureBuckets[bucket].size(); }

	/** Retrieves a texture
		@param bucket A texture channel index. This index should match one
			of the values in the FUDaeTextureChannel enum.
		@param index The index of a texture within this channel.
		@return The texture. This pointer will be NULL if either the bucket or the index is out-of-bounds. */
	inline FCDTexture* GetTexture(uint32 bucket, size_t index) { FUAssert(index < GetTextureCount(bucket), return NULL); return textureBuckets[bucket].at(index); }
	inline const FCDTexture* GetTexture(uint32 bucket, size_t index) const { FUAssert(index < GetTextureCount(bucket), return NULL); return textureBuckets[bucket].at(index); } /**< See above. */

	/** Adds a texture to a specific channel.
		@param bucket A texture channel index. This index should match one
			of the values in the FUDaeTextureChannel enum.
		@return The new texture. This pointer will be NULL if the bucket is out-of-bounds. */
	FCDTexture* AddTexture(uint32 bucket);

	/** Releases a texture contained within this effect profile.
		@param texture The texture to release. */
	void ReleaseTexture(FCDTexture* texture);

	/** Retrieves the base translucency color.
		This value must be multiplied with the translucency factor
		to get the real translucency color.
		This value is used in all lighting models.
		@return The base translucency color. */
	inline const FMVector3& GetTranslucencyColor() const { return translucencyColor; }

	/** Sets the base translucency color.
		@param color The base translucency color. */
	inline void SetTranslucencyColor(const FMVector3& color) { translucencyColor = color; }

	/** Retrieves the translucency factor.
		This value must be multiplied with the translucency color
		to get the real translucency color.
		This value is used in all lighting models.
		@return The translucency factor. */
	inline const float& GetTranslucencyFactor() const { return translucencyFactor; }

	/** Sets the translucency factor.
		@param factor The translucency factor. */
	inline void SetTranslucencyFactor(float factor) { translucencyFactor = factor; }

	/** Retrieves the flat opacity.
		This is a calculated value and will not take into consideration any animations
		that affect either the base translucency color or the translucency factor.
		This value can be used in all lighting models.
		@return The flat opacity. */
	float GetOpacity() const;

	/** Retrieves the base emission/self-illumination color.
		This value must be multiplied with the emission factor to get the real emission color.
		This value is used in all lighting models.
		@return The base emission color. */
	inline const FMVector3& GetEmissionColor() const { return emissionColor; }

	/** Sets the base emission/self-illumination color.
		@param color The base emission color. */
	inline void SetEmissionColor(const FMVector3& color) { emissionColor = color; }

    /** Retrieves the emission/self-illumination factor.
		This value must be multiplied with the base emission color to get the real emission color.
		@return The emission factor. */
	inline const float& GetEmissionFactor() const { return emissionFactor; }

	/** Sets the emission/self-illumination factor.
		@param factor The emission factor. */
	inline void SetEmissionFactor(float factor) { emissionFactor = factor; }

	/** Retrieves whether the emission factor was used, rather than the emission color.
		This value is used in conjunction with 3dsMax, in which the self-illumination color
		and the self-illumination factor are mutually exclusive.
		@return Whether the emission factor is to be used. */
	inline bool IsEmissionFactor() const { return isEmissionFactor; }

	/** Sets whether the emission factor is to be used, rather than the emission color.
		This value is used in conjunction with 3dsMax, in which the self-illumination color
		and the self-illumination factor are mutually exclusive.
		@param useFactor Whether the emission factor should be used. */
	inline void SetIsEmissionFactor(bool useFactor) { isEmissionFactor = useFactor; }

	/** Retrieves the diffuse color.
		This value is used in the Lambert lighting model.
		@return The diffuse color. */
	inline const FMVector3& GetDiffuseColor() const { return diffuseColor; }

	/** Sets the diffuse color.
		@param color The diffuse color. */
	inline void SetDiffuseColor(const FMVector3& color) { diffuseColor = color; }

	/** Retrieves the ambient color.
		This value is used in the Lambert lighting model.
		@return The ambient color. */
	inline const FMVector3& GetAmbientColor() const { return ambientColor; }

	/** Sets the ambient color.
		@param color The ambient color. */
	inline void SetAmbientColor(const FMVector3& color) { ambientColor = color; }

	/** Retrieves the base specular color.
		This value must be multiplied with the specular factor
		to get the real specular color.
		This value is used in the Phong and Blinn lighting models.
		@return The specular color. */
	inline const FMVector3& GetSpecularColor() const { return specularColor; }

	/** Sets the specular color.
		@param color The specular color. */
	inline void SetSpecularColor(const FMVector3& color) { specularColor = color; }

	/** Retrieves the specular factor.
		This value must be multiplied with the base specular color
		to get the real specular color.
		This value is used in the Phong and Blinn lighting models.
		@return The specular factor. */
	inline const float& GetSpecularFactor() const { return specularFactor; }

	/** Sets the specular factor.
		@param factor The specular factor. */
	inline void SetSpecularFactor(float factor) { specularFactor = factor; }

	/** Retrieves the specular shininess.
		This value represents the exponent to which you must raise
		the dot-product between the view vector and reflected light vectors:
		as such, it is usually a number greater than 1.
		This value is used in the Phong and Blinn lighting models.
		@return The specular shininess. */
	inline const float& GetShininess() const { return shininess; }

	/** Sets the specular shininess.
		This value represents the exponent to which you must raise
		the dot-product between the view vector and reflected light vectors:
		as such, it is usually a number greater than 1.
		@param _shininess The specular shininess. */
	inline void SetShininess(float _shininess) { shininess = _shininess; }

	/** Retrieves the base reflectivity color.
		This value must be multiplied to the reflectivity factor to
		get the real reflectivity color.
		This value is used in the Phong and Blinn lighting models.
		@return The base reflectivity color. */
    inline const FMVector3& GetReflectivityColor() const { return reflectivityColor; }

	/** Sets the base reflectivity color.
		@param color The base reflectivity color. */
	inline void SetReflectivityColor(const FMVector3& color) { reflectivityColor = color; }
	
	/** Retrieves the reflectivity factor.
		This value must be multiplied to the base reflectivity color
		to get the real reflectivity color.
		This value is used in the Phong and Blinn lighting models.
		@return The reflectivity factor. */
	inline const float& GetReflectivityFactor() const { return reflectivityFactor; }

	/** Sets the reflectivity factor.
		@param factor The reflectivity factor. */
	inline void SetReflectivityFactor(float factor) { reflectivityFactor = factor; }

	/** Retrieves the flat reflectivity.
		This is a calculated value and will not take into consideration any animations
		that affect either the base reflectivity color or the reflectivity factor.
		This value can be used in the Phong and Blinn lighting models.
		@return The flat reflectivity. */
	float GetReflectivity() const;

	/** Retrieves the 'faceted' flag.
		This flag is used in conjunction with 3dsMax. It represents whether all the edges
		of the polygon sets using this effect profile should be hard. The final result
		of using this flag is a mesh where all the faces stand out.
		@return The status of the 'faceted' flag. */
    inline bool GetFacetedFlag() const { return isFaceted; }

	/** Sets the 'faceted' flag.
		This flag is used in conjunction with 3dsMax. It represents whether all the edges
		of the polygon sets using this effect profile should be hard. The final result
		of using this flag is a mesh where all the faces stand out.
		@param flag The status of the 'faceted' flag. */
	inline void SetFacetedFlag(bool flag) { isFaceted = flag; }

	/** Retrieves the 'double-sided' flag.
		This flag is used in conjunction with 3dsMax. It represents whether all the faces
		of the polygon sets should be treated as two-sided and have normals in both directions.
		@return The status of the 'double-sided' flag. */
	inline bool GetDoubleSidedFlag() const { return isDoubleSided; }

	/** Sets the 'double-sided' flag.
		This flag is used in conjunction with 3dsMax. It represents whether all the faces
		of the polygon sets should be treated as two-sided and have normals in both directions.
		@param flag The status of the 'double-sided' flag. */
	inline bool SetDoubleSidedFlag(bool flag) { isDoubleSided = flag; }

	/** Retrieves the 'wireframe' flag.
		This flag is used in conjunction with 3dsMax. It represents whether all the edges
		of the polygon sets should be rendered, rather than the faces.
		@return The status of the 'wireframe' flag. */
	inline bool GetWireframeFlag() const { return isWireframe; }

	/** Sets the 'wireframe' flag.
		This flag is used in conjunction with 3dsMax. It represents whether all the edges
		of the polygon sets should be rendered, rather than the faces.
		@param flag The status of the 'wireframe' flag. */
	inline bool SetWireframeFlag(bool flag) { isWireframe = flag; }

	/** Retrieves the 'face-map' flag.
		This is a pure 3dsMax flag and I have no idea what it does.
		@return The status of the 'face-map' flag. */
	inline bool GetFaceMapFlag() const { return isFaceMap; }

	/** Sets the 'face-map' flag.
		This is a pure 3dsMax flag and I have no idea what it does.
		@param flag The status of the 'face-map' flag. */
	inline void SetFaceMapFlag(bool flag) { isFaceMap = flag; }

	/** Retrieves an effect parameter.
		Looks for the effect parameter with the correct semantic, in order to bind or override its value.
		This function searches through the effect profile and the level of abstractions below.
		@param semantic The effect parameter semantic to match.
		@return The effect parameter that matches the semantic. This pointer may be
			NULL if no effect parameter matches the given semantic. */
	virtual FCDEffectParameter* FindParameterBySemantic(const string& semantic);

	/** Retrieves a subset of the effect parameter list.
		Look for effect parameters with the correct semantic.
		This function searches through the effect profile and the level of abstractions below.
		@param semantic The effect parameter semantic to match.
		@param parameters The list of parameters to fill in. This list is not cleared. */
	virtual void FindParametersBySemantic(const string& semantic, FCDEffectParameterList& parameters);

	/** Retrieves a subset of the effect parameter list.
		Look for effect parameters with the correct reference.
		This function searches through the effect profile and the level of abstractions below.
		@param reference The effect parameter reference to match. In the case of effect
			parameter, the reference is replaced by the sub-id.
		@param parameters The list of parameters to fill in. This list is not cleared. */
	virtual void FindParametersByReference(const string& reference, FCDEffectParameterList& parameters);

	/** [INTERNAL] Clones the COMMON profile effect.
		You will need release the cloned effect directly, by deleting the pointer.
		@param newParent The effect that contains the cloned effect profile.
		@return The cloned effect profile. You will must delete this pointer. */
	virtual FCDEffectProfile* Clone(FCDEffect* newParent);

	/** [INTERNAL] Flattens the profile.
		Does nothing on the common profile. */
	virtual void Flatten() {}

	/** [INTERNAL] Reads in the \<profile_COMMON\> element from a given COLLADA XML tree node.
		For COLLADA 1.3 backward-compatibility, this function can also read in \<material\> elements.
		@param baseNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the effect profile.*/
	virtual FUStatus LoadFromXML(xmlNode* baseNode);

	/** [INTERNAL] Writes out the \<profile_COMMON\> element to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the effect profile.
		@return The created element XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;

private:
	xmlNode* WriteColorTextureParameterToXML(xmlNode* parentNode, const char* parameterNodeName, const FMVector3& value, const FCDTextureList& textureBucket) const;
	xmlNode* WriteFloatTextureParameterToXML(xmlNode* parentNode, const char* parameterNodeName, const float& value, const FCDTextureList& textureBucket) const;
	xmlNode* WriteTextureParameterToXML(xmlNode* parentNode, const FCDTextureList& textureBucket) const;

	FUStatus ParseColorTextureParameter(xmlNode* parameterNode, FMVector3& value, FCDTextureList& textureBucket);
	FUStatus ParseFloatTextureParameter(xmlNode* parameterNode, float& value, FCDTextureList& textureBucket);
	FUStatus ParseSimpleTextureParameter(xmlNode* parameterNode, FCDTextureList& textureBucket);
};

#endif //_FCD_MATERIAL_STANDARD_H_

