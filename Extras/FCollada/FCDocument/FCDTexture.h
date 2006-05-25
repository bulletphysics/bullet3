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
	@file FCDTexture.h
	This file contains the FCDTexture class.
*/

#ifndef _FCD_TEXTURE_H_
#define _FCD_TEXTURE_H_

#include "FCDocument/FCDEntity.h"
#include "FUtils/FUDaeEnum.h"

class FCDocument;
class FCDEffectParameter;
class FCDEffectParameterInt;
class FCDEffectParameterList;
class FCDImage;

/**
	A COLLADA texture.
	
	Textures are used by the COMMON profile materials. For COLLADA 1.3 backward
	compatibility, the textures may also be entities contained within
	the material library. The ColladaFX profiles use the FCDEffectParameterSampler
	and the FCDEffectParameterSurface classes, instead, to hold the texturing information.

	Textures hold the information necessary to place an image onto polygon sets.
	This information includes: which texture coordinate set to use, which image
	contains the texels, the blend mode, the components of the texture matrix, etc.

	Most of the texturing information does not belong to COLLADA and is specific to
	either Maya or 3dsMax.

	@ingroup FCDEffect
*/
class FCOLLADA_EXPORT FCDTexture : public FCDEntity
{
private:
	FUDaeTextureChannel::Channel textureChannel; // COLLADA 1.3 backward compatibility
	FCDImage* image;
	string subId;

	// Always preset, this parameter hold the map channel/uv set index
	FCDEffectParameterInt* set;

	// Texture color multiplier
	float multiplier; // Max-specific

	// Placement parameters. Yes, there's a ton of them [Maya-only]
	bool hasProjection3D;
	bool hasPlacement2D;
	FUDaeBlendMode::Mode blendMode;

	// 2D placement [Maya-only]
	float wrapU, wrapV, mirrorU, mirrorV, stagger, fast; // animated, but should really be booleans
	float coverageU, coverageV, translateFrameU, translateFrameV, rotateFrame;
	float repeatU, repeatV, offsetU, offsetV, rotateUV, noiseU, noiseV;

	// 3D projection [Maya-only]
	FMMatrix44 projectionMatrix;
	fstring projectionType;

public:
	/** Constructor: do not use directly.
		Instead, use the FCDEffectStandard::AddTexture function.
		@param document The COLLADA document that owns this texture. */
	FCDTexture(FCDocument* document);

	/** Destructor: do not use directly.
		Instead, use the FCDEffectStandard::ReleaseTexture function. */
	virtual ~FCDTexture();

	/** Retrieves the type of the entity class.
		This function is part of the FCDEntity interface and is used
		for COLLADA 1.3 backward compatibility.
		@return The entity class type: TEXTURE. */
	virtual Type GetType() const { return TEXTURE; }

	/** Retrieves which texture channel to use for this texture.
		As it is directly tied in with which buckets holds this texture, you
		cannot modify this value. Instead, create a new texture in the correct
		bucket of the FCDEffectStandard object.
		@return The texture channel. */
	FUDaeTextureChannel::Channel GetTextureChannel() const { return textureChannel; }

	/** Retrieves the image information for this texture.
		@return The image. This pointer will be NULL if this texture is not yet
			tied to a valid image. */
	FCDImage* GetImage() { return image; }
	const FCDImage* GetImage() const { return image; } /**< See above. */

	/** Sets the image to be used with this texture.
		@param _image An image. */
	void SetImage(FCDImage* _image) { image = _image; }

	/** Retrieves the texture coordinate set to use with this texture.
		This information is duplicated from the material instance abstraction level.
		@return The effect parameter containing the set. */
	FCDEffectParameterInt* GetSet() { return set; }
	const FCDEffectParameterInt* GetSet() const { return set; } /**< See above. */

	/** Retrieves the blend mode to use if this texture is not the first one
		within its channel's bucket.
		@return The blend mode. */
	FUDaeBlendMode::Mode GetBlendMode() const { return blendMode; }

	/** Sets the blend mode to use if this texture is not the first one
		within its channel's bucket.
		@param mode The blend mode. */
	void SetBlendMode(FUDaeBlendMode::Mode mode) { blendMode = mode; }

	/** @deprecated Retrieves the sub-id of the texture.
		@return The sub-id. */
	const string& GetSubId() const { return subId; }

	/** @deprecated Sets the sub-id of the texture.
		@param _subId The sub-id. */
	void SetSubId(const string& _subId) { subId = _subId; }

	/** Retrieves the generic multiplier to apply on the texture.
		This parameter is specific to ColladaMax. The meaning of this parameter
		depends on the channel that the texture belongs to.
		This value is animatable.
		@return The generic multiplier. */
	float& GetMultiplier() { return multiplier; }
	const float& GetMultiplier() const { return multiplier; } /**< See above. */

	/** Sets the generic multiplier.
		This parameter is specific to ColladaMax. The meaning of this parameter
		depends on the channel that the texture belongs to.
		@param _multiplier The generic multiplier. */
	void SetMultiplier(float _multiplier) { multiplier = _multiplier; }

	/** Retrieves whether the texturing information includes the
		ColladaMaya-specific 2D placement parameters.
		@return Whether the ColladaMaya-specific 2D placement parameters are valid. */
	bool HasPlacement2D() const { return hasPlacement2D; }

	/** Removes the ColladaMaya-specific 2D placement parameters and resets them. */
	void ClearPlacement2D();

	/** Retrieves whether to wrap the U texture coordinates.
		This parameter is specific to ColladaMaya. This value is a float because
		it is animatable: it should be interpreted as a boolean.
		@return Whether to wrap the U texture coordinates. */
	float& GetWrapU() { return wrapU; }
	const float& GetWrapU() const { return wrapU; } /**< See above. */

	/** Sets whether to wrap the U texture coordinates.
		This parameter is specific to ColladaMaya.
		@param _wrapU Whether to wrap the U texture coordinate. */
	void SetWrapU(bool _wrapU) { wrapU = _wrapU ? 1.0f : 0.0f; hasPlacement2D = true; }

	/** Retrieves whether to wrap the V texture coordinates.
		This parameter is specific to ColladaMaya. This value is a float because
		it is animatable: it should be interpreted as a boolean.
		@return Whether to wrap the V texture coordinates. */
	float& GetWrapV() { return wrapV; }
	const float& GetWrapV() const { return wrapV; } /**< See above. */

	/** Sets whether to wrap the V texture coordinates.
		This parameter is specific to ColladaMaya.
		@param _wrapV Whether to wrap the V texture coordinate. */
	void SetWrapV(bool _wrapV) { wrapV = _wrapV ? 1.0f : 0.0f; hasPlacement2D = true; }

	/** Retrieves whether to mirror the U texture coordinates.
		This parameter is specific to ColladaMaya. This value is a float
		because it is animatable: it should be interpreted as a boolean.
		@return Whether to mirror the U texture coordinate. */
	float& GetMirrorU() { return mirrorU; }
	const float& GetMirrorU() const { return mirrorU; } /**< See above. */

	/** Sets whether to mirror the U texture coordinates.
		This parameter is specific to ColladaMaya.
		@param _mirrorU Whether to mirror the U texture coordinates. */
	void SetMirrorU(bool _mirrorU) { mirrorU = _mirrorU ? 1.0f : 0.0f; hasPlacement2D = true; }

	/** Retrieves whether to mirror the V texture coordinates.
		This parameter is specific to ColladaMaya. This value is a float
		because it is animatable: it should be interpreted as a boolean.
		@return Whether to mirror the V texture coordinate. */
	float& GetMirrorV() { return mirrorV; }
	const float& GetMirrorV() const { return mirrorV; } /**< See above. */

	/** Sets whether to mirror the V texture coordinates.
		This parameter is specific to ColladaMaya.
		@param _mirrorV Whether to mirror the V texture coordinates. */
	void SetMirrorV(bool _mirrorV) { mirrorV = _mirrorV ? 1.0f : 0.0f; hasPlacement2D = true; }

	/** Retrieves whether to stagger the texture.
		This parameter is specific to ColladaMaya. This value is a float
		because it is animatable: it should be interpreted as a boolean.
		@return Whether to stagger the texture. */
	float& GetStagger() { return stagger; }
	const float& GetStagger() const { return stagger; } /**< See above. */

	/** Sets whether to stagger the texture.
		This parameter is specific to ColladaMaya. 
		@param _stagger Whether to stagger the texture. */
	void SetStagger(bool _stagger) { stagger = _stagger ? 1.0f : 0.0f; hasPlacement2D = true; }

	/** Retrieves the 'fast' flag.
		This parameter is specific to ColladaMaya. This value is a float
		because it is animatable: it should be interpreted as a boolean.
		This parameter has no meaning outside of Maya.
		@return The 'fast' flag. */
	float& GetFast() { return fast; }
	const float& GetFast() const { return fast; } /**< See above. */

	/** Sets the 'fast' flag.
		This parameter is specific to ColladaMaya and has no meaning
		outside of Maya.
		@param _fast The 'fast' flag. */
	void SetFast(bool _fast) { fast = _fast ? 1.0f : 0.0f; hasPlacement2D = true; }

	/** Retrieves the scale factor of the texture frame, in the U coordinates.
		This parameter is specific to ColladaMaya.
		This value is animatable. For more information on this parameter,
		check the Maya documentation.
		@return The scale factor. */
	float& GetCoverageU() { return coverageU; }
	const float& GetCoverageU() const { return coverageU; } /**< See above. */

	/** Sets the scale factor of the texture frame, in the U coordinates.
		This parameter is specific to ColladaMaya.
		@param _coverageU The scale factor. */
	void SetCoverageU(float _coverageU) { coverageU = _coverageU; hasPlacement2D = true; }

	/** Retrieves the scale factor of the texture frame, in the V coordinates.
		This parameter is specific to ColladaMaya.
		This value is animatable. For more information on this parameter,
		check the Maya documentation.
		@return The scale factor. */
	float& GetCoverageV() { return coverageV; }
	const float& GetCoverageV() const { return coverageV; } /**< See above. */

	/** Sets the scale factor of the texture frame, in the V coordinates.
		This parameter is specific to ColladaMaya.
		@param _coverageV The scale factor. */
	void SetCoverageV(float _coverageV) { coverageV = _coverageV; hasPlacement2D = true; }

	/** Retrieves the translation of the texture frame, in the U coordinate.
		This parameter is specific to ColladaMaya.
		This value is animatable. For more information on this parameter,
		check the Maya documentation.
		@return The translation offset. */
	float& GetTranslateFrameU() { return translateFrameU; }
	const float& GetTranslateFrameU() const { return translateFrameU; } /**< See above. */

	/** Sets the translation of the texture frame, in the U coordinate.
		This parameter is specific to ColladaMaya.
		For more information on this parameter, check the Maya documentation.
		@param _translateFrameU The translation offset. */
	void SetTranslateFrameU(float _translateFrameU) { translateFrameU = _translateFrameU; hasPlacement2D = true; }

	/** Retrieves the translation of the texture frame, in the V coordinate.
		This parameter is specific to ColladaMaya.
		This value is animatable. For more information on this parameter,
		check the Maya documentation.
		@return The translation offset. */
	float& GetTranslateFrameV() { return translateFrameV; }
	const float& GetTranslateFrameV() const { return translateFrameV; } /**< See above. */

	/** Sets the translation of the texture frame, in the V coordinate.
		This parameter is specific to ColladaMaya.
		For more information on this parameter, check the Maya documentation.
		@param _translateFrameV The translation offset. */
	void SetTranslateFrameV(float _translateFrameV) { translateFrameV = _translateFrameV; hasPlacement2D = true; }

	/** Retrieves the angle of rotation of the texture frame.
		This parameter is specific to ColladaMaya.
		This value is animatable. For more information on this parameter,
		check the Maya documentation.
		@return The angle of rotation (in degrees). */
	float& GetRotateFrame() { return rotateFrame; }
	const float& GetRotateFrame() const { return rotateFrame; } /**< See above. */

	/** Sets the angle of rotation of the texture frame.
		This parameter is specific to ColladaMaya.
		For more information on this parameter, check the Maya documentation.
		@param _rotateFrame The angle of rotation (in degrees). */
	void SetRotateFrame(float _rotateFrame) { rotateFrame = _rotateFrame; hasPlacement2D = true; }

	/** Retrieves the scale factor applied on the U coordinates of the rendered polygons.
		This parameter is specific to ColladaMaya. This value is animatable.
		@return The scale factor. */
	float& GetRepeatU() { return repeatU; }
	const float& GetRepeatU() const { return repeatU; } /**< See above. */

	/** Sets the scale factor applied on the U coordinates of the rendered polygons.
		This parameter is specific to ColladaMaya.
		@param _repeatU The scale factor. */
	void SetRepeatU(float _repeatU) { repeatU = _repeatU; hasPlacement2D = true; }

	/** Retrieves the scale factor applied on the V coordinates of the rendered polygons.
		This parameter is specific to ColladaMaya. This value is animatable.
		@return The scale factor. */
	float& GetRepeatV() { return repeatV; }
	const float& GetRepeatV() const { return repeatV; } /**< See above. */

	/** Sets the scale factor applied on the V coordinates of the rendered polygons.
		This parameter is specific to ColladaMaya.
		@param _repeatV The scale factor. */
	void SetRepeatV(float _repeatV) { repeatV = _repeatV; hasPlacement2D = true; }

	/** Retrieves the translation offset applied on the U coordinates of the rendered polygons.
		This parameter is specific to ColladaMaya. This value is animatable.
		@return The translation offset. */
	float& GetOffsetU() { return offsetU; }
	const float& GetOffsetU() const { return offsetU; } /**< See above. */

	/** Sets the translation offset applied on the U coordinates of the rendered polygons.
		This parameter is specific to ColladaMaya.
		@param _offsetU The translation offset. */
	void SetOffsetU(float _offsetU) { offsetU = _offsetU; hasPlacement2D = true; }

	/** Retrieves the translation offset applied on the V coordinates of the rendered polygons.
		This parameter is specific to ColladaMaya. This value is animatable.
		@return The translation offset. */
	float& GetOffsetV() { return offsetV; }
	const float& GetOffsetV() const { return offsetV; } /**< See above. */

	/** Sets the translation offset applied on the V coordinates of the rendered polygons.
		This parameter is specific to ColladaMaya.
		@param _offsetV The translation offset. */
	void SetOffsetV(float _offsetV) { offsetV = _offsetV; hasPlacement2D = true; }

	/** Retrieves the rotation angle applied on the texture coordinates of the rendered polygons.
		This parameter is specific to ColladaMaya. This value is animatable.
		@return The rotation angle (in degrees). */
	float& GetRotateUV() { return rotateUV; }
	const float& GetRotateUV() const { return rotateUV; } /**< See above. */

	/** Sets the rotation angle applied on the texture coordinates of the rendered polygons.
		This parameter is specific to ColladaMaya. This value is animatable.
		@param _rotateUV The rotation angle (in degrees). */
	void SetRotateUV(float _rotateUV) { rotateUV = _rotateUV; hasPlacement2D = true; }

	/** Retrieves the maxmimum amount of noise to add to each of the U coordinates of the rendered polygons.
		This parameter is specific to ColladaMaya. This value is animatable.
		@return The maxmimum amount of noise. */
	float& GetNoiseU() { return noiseU; }
	const float& GetNoiseU() const { return noiseU; } /**< See above. */

	/** Sets the maximum amount of noise to add to each of the U coordinates of the rendered polygons.
		This parameter is specific to ColladaMaya.
		@param _noiseU The maximum amount of noise. */
	void SetNoiseU(float _noiseU) { noiseU = _noiseU; hasPlacement2D = true; }

	/** Retrieves the maxmimum amount of noise to add to each of the V coordinates of the rendered polygons.
		This parameter is specific to ColladaMaya. This value is animatable.
		@return The maxmimum amount of noise. */
	float& GetNoiseV() { return noiseV; }
	const float& GetNoiseV() const { return noiseV; } /**< See above. */

	/** Sets the maximum amount of noise to add to each of the V coordinates of the rendered polygons.
		This parameter is specific to ColladaMaya.
		@param _noiseV The maximum amount of noise. */
	void SetNoiseV(float _noiseV) { noiseV = _noiseV; hasPlacement2D = true; }

	/** Retrieves whether the texturing information includes the
		ColladaMaya-specific 3D projection parameters.
		@return Whether the ColladaMaya-specific 3D projection parameters are valid. */
	bool HasProjection3D() const { return hasProjection3D; }

	/** Removes the ColladaMaya-specific 3D projection parameters and resets them. */
	void ClearProjection3D();

	/** Retrieves the texture projection matrix.
		This matrix transforms the 3D texture coordinates into valid 2D texture coordinates.
		This parameter is specific to ColladaMaya.
		@return The texture projection matrix. */
	FMMatrix44& GetProjectionMatrix() { return projectionMatrix; }
	const FMMatrix44& GetProjectionMatrix() const { return projectionMatrix; } /**< See above. */

	/** Sets the texture projection matrix.
		This matrix transforms the 3D texture coordinates into valid 2D texture coordinates.
		This parameter is specific to ColladaMaya.
		@param matrix The texture projection matrix. */
	void SetProjectionMatrix(const FMMatrix44& matrix) { projectionMatrix = matrix; hasProjection3D = true; }
	
	/** Retrieves the type of projection to use on the 3D texture coordinates.
		This parameter is specific to ColladaMaya.
		@return The type string. */
	const fstring& GetProjectionType() const { return projectionType; }

	/** Sets the type of projection to use on the 3D texture coordinates.
		This paramter is specific to ColladaMaya.
		@param type The type string. */
	void SetProjectionType(const fstring& type) { projectionType = type; hasProjection3D = true; } 

	/** Retrieves an effect parameter.
		Looks for the effect parameter with the correct semantic, in order to bind or override its value.
		This function compares the local effect parameters with the given semantic.
		@param semantic The effect parameter semantic to match.
		@return The effect parameter that matches the semantic. This pointer may be
			NULL if no effect parameter matches the given semantic. */
	FCDEffectParameter* FindParameterBySemantic(const string& semantic);

	/** Retrieves a subset of the local effect parameter list.
		Look for local effect parameters with the correct semantic.
		This function searches through the effect profile and the level of abstractions below.
		@param semantic The effect parameter semantic to match.
		@param parameters The list of parameters to fill in. This list is not cleared. */
	void FindParametersBySemantic(const string& semantic, FCDEffectParameterList& parameters);

	/** Retrieves a subset of the local effect parameter list.
		Look for the local effect parameter with the correct reference.
		@param reference The effect parameter reference to match.
		@param parameters The list of parameters to fill in. This list is not cleared. */
	void FindParametersByReference(const string& reference, FCDEffectParameterList& parameters);

	/** [INTERNAL] Clones the texture.
		Does not clone the image: both texture objects will point to the same image object.
		@return The cloned texture. This pointer will never be NULL. */
	FCDTexture* Clone();

	/** [INTERNAL] Reads in the texture from a given COLLADA XML tree node.
		@param textureNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the texture.*/
	FUStatus LoadFromTextureXML(xmlNode* textureNode);

	/** [INTERNAL] Reads in the texture from a given COLLADA XML tree node.
		This function is useful only for COLLADA 1.3 backward compatibility.
		@param textureNode The COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the texture.*/
	virtual FUStatus LoadFromXML(xmlNode* textureNode);

	/** [INTERNAL] Writes out the texture to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the texture.
		@return The created element XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode);

private:
	// Read in the Maya-specific texture placement parameters and look for a texture projection program
	FUStatus LoadPlacementXML(xmlNode* techniqueNode);
};

#endif // _FCD_TEXTURE_H_
