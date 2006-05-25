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
	@file FCDLight.h
	This file contains the FCDLight class.
*/

#ifndef _FCD_LIGHT_H_
#define _FCD_LIGHT_H_

#include "FCDocument/FCDTargetedEntity.h"
#include "FUtils/FUDaeEnum.h"

class FCDocument;
class FCDSceneNode;

/**
	A COLLADA light.
	Based on the FCDTargetedEntity class to supported aimed lights.
	COLLADA defines four types of native lights: point, spot, ambient and directional.
	These four types are fully handled by this class: make sure to check the type flag
	as well as which information to expect for each light type.

	A COLLADA ambient light has a global color, which should be added to
	all other lighting on all geometry.

	A COLLADA directional light has a global color, which should be multiplied
	to the cosine of the angle between the normal vector of a triangle
	and the direction of the light. Note that the direction will be calculated
	from the transforms, for each instance, and is not provided by this class.

	A COLLADA point light has a color which attenuates as the distance increases
	between the light position and the vertex being shaded. Note that the position
	will be calculated from the transforms, for each instance,
	and is not provided by this class.

	A COLLADA spot light is a point light which lights only the objects that
	appear within a specific angle, with respect to the direction of the light.
	Note that the position and the direction will be calculated from the
	transforms, for each instance, and is not provided by this class.

	@ingroup FCDocument
*/
class FCOLLADA_EXPORT FCDLight : public FCDTargetedEntity
{
public:
	/** The types of lights supported by this class. */
	enum LightType
	{
		POINT, /**< A point light. This is the default type. */
		SPOT, /**< A spot light. */
		AMBIENT, /**< An ambient light. */
		DIRECTIONAL /**< A directional light. */
	};

private:
	// Common Light parameters
	FMVector3 color;	
	float intensity; // Max and Maya
	LightType lightType;

	// Point and spot light parameters
	float constantAttenuationFactor;
	float linearAttenuationFactor;
	float quadracticAttenuationFactor;

	// Spot-specific light parameters
	float fallOffExponent;
	float fallOffAngle;
	float outerAngle; // Max-specific
	float penumbraAngle; // Maya-specifc: these last two are are related as 'penumbra = outerAngle - fallOffAngle'.
	float aspectRatio; // Max-specific, for rectangle lights
	float dropoff; // Maya-specific

	// Directional light parameters
	// Overshoot is a Max-specific flag that sets a directional light to cover everything,
	// rather than to be restricted to a cylinder defined by the fallOffAngle/outerAngle.
	bool overshoots; // Max-specific

public:
	/** Constructor: do not use directly. Create new lights using the FCDLibrary::AddEntity function.
		@param document The COLLADA document that contains this light entity. */
	FCDLight(FCDocument* document);

	/** Destructor: do not release directly. Release lights using the FCDLibrary::ReleaseEntity function.
		All lights are also released with the document that they belong to. */
	virtual ~FCDLight();

	/** Retrieves the entity type for this class. This function is part of the FCDEntity interface.
		@return The entity type: LIGHT. */
	virtual Type GetType() const { return LIGHT; }

	/** Retrieves the base color for the light. To calculate the light color,
		multiply the base color with the intensity.
		@return The base color for the light. */
	FMVector3& GetColor() { return color; }
	const FMVector3& GetColor() const { return color; } /**< See above. */

	/** Sets the base color for the light. To calculate the light color,
		multiply the base color with the intensity.
		@param col The base color for the light. */
	void SetColor(const FMVector3& col) { color = col; }

	/** Retrieves the intensity of the light. To calculate the light color,
		multiply the base color with the intensity.
		@return The intensity of the light. */
	float& GetIntensity() { return intensity; }
	const float& GetIntensity() const { return intensity; } /**< See above. */

	/** Sets the intensity of the light. To calculate the light color,
		multiply the base color with the intensity.
		@param _intensity The intensity of the light. */
	void SetIntensity(float _intensity) { intensity = _intensity; }

	/** Retrieves the type of the light.
		Make sure to check the type of light before using the values, as some values
		may not make sense with some types of light.
		@return The light type. */
	LightType GetLightType() const { return lightType; }

	/** Sets the type of the light. The default type of a new light is POINT.
		@param type The light type. */
	void SetLightType(LightType type) { lightType = type; }

	/** Retrieves the constant attenuation factor for the light.
		This value is valid only for point and spot lights.
		@return The constant attenuation factor. */
	float& GetConstantAttenuationFactor() { return constantAttenuationFactor; }
	const float& GetConstantAttenuationFactor() const { return constantAttenuationFactor; } /**< See above. */

	/** Sets the constant attenuation factor for the light.
		This value is valid only for point and spot lights.
		@param factor The constant attenuation factor. */
	void SetConstantAttenuationFactor(float factor) { constantAttenuationFactor = factor; }

	/** Retrieves the linear attenuation factor for the light.
		This value is valid only for point and spot lights.
		@return The linear attenuation factor. */
	float& GetLinearAttenuationFactor() { return linearAttenuationFactor; }
	const float& GetLinearAttenuationFactor() const { return linearAttenuationFactor; } /**< See above. */

	/** Sets the linear attenuation factor for the light.
		This value is valid only for point and spot lights.
		@param factor The linear attenuation factor. */
	void SetLinearAttenuationFactor(float factor) { linearAttenuationFactor = factor; }

	/** Retrieves the quadratic attenuation factor for the light.
		This value is valid only for point and spot lights.
		@return The quadratic attenuation factor. */
	float& GetQuadraticAttenuationFactor() { return quadracticAttenuationFactor; }
	const float& GetQuadraticAttenuationFactor() const { return quadracticAttenuationFactor; } /**< See above. */

	/** Sets the quadratic attenuation factor for the light.
		This value is valid only for point and spot lights.
		@param factor The quadratic attenuation factor. */
	void SetQuadraticAttenuationFactor(float factor) { quadracticAttenuationFactor = factor; }

	/** Retrieves the fall-off exponent for the light.
		This value is valid only for spot lights. It determines
		how fast the lighting turns off, with respect to
		angles greater than the fall-off angle. This results in a smooth
		lighting at the spot light's edges.
		
		IMPORTANT NOTE: Neither ColladaMaya or ColladaMax use this value
		as neither Maya or 3dsMax use this technique for soft lighting.

		@return The spot light fall-off exponent. */
	float& GetFallOffExponent() { return fallOffExponent; }
	const float& GetFallOffExponent() const { return fallOffExponent; } /**< See above. */

	/** Sets the fall-off exponent for the light.
		@see GetFallOffExponent
		@param exponent The spot light fall-off exponent. */
	void SetFallOffExponent(float exponent) { fallOffExponent = exponent; }

	/** Retrieves the fall-off angle for the light.
		This value is valid only for spot lights. It defines
		the cone of the spot light.
		@return The spot light fall-off angle. */
	float& GetFallOffAngle() { return fallOffAngle; }
	const float& GetFallOffAngle() const { return fallOffAngle; } /**< See above. */

	/** Sets the fall-off angle for the light.
		@see GetFallOffAngle
		@param angle The spot light fall-off angle. */
	void SetFallOffAngle(float angle) { fallOffAngle = angle; }

	/** Retrieves the outer angle for the light.
		This value is valid only for spot lights. This value is only used
		by documents exported by ColladaMax. This value should always be
		greater than the fall-off angle. It represents the angle at which
		the lighting is black. All lighting between the fall-off angle and
		the outer angle is a linear interpolation between the light color
		and black.
		@return The spot light outer angle. */
	float& GetOuterAngle() { return outerAngle; }
	const float& GetOuterAngle() const { return outerAngle; } /**< See above. */

	/** Sets the outer angle for the light.
		@see GetOuterAngle
		@param angle The spot light outer angle. */
	void SetOuterAngle(float angle) { outerAngle = angle; }

	/** Retrieves the penumbra angle for the light.
		This value is valid only for spot lights. The value is only used
		by documents exported by ColladaMaya. This value is relative to
		the fall-off angle and may be negative. If this value is positive,
		it determines the outer angle, as described above. If this value
		is negative, the fall-off angle is used as the outer angle and the
		fall-off angle + the penumbra angle is used as the full-lighting
		angle.
		@see GetOuterAngle
		@return The spot light penumbra angle. */
	float& GetPenumbraAngle() { return penumbraAngle; }
	const float& GetPenumbraAngle() const { return penumbraAngle; } /**< See above. */

	/** Sets the penumbra angle for the light.
		@see GetPenumbraAngle
		@param angle The spot light penumbra angle. */
	void SetPenumbraAngle(float angle) { penumbraAngle = angle; }

	/** Retrieves the aspect ratio for the light.
		This value is only used by documents exported by ColladaMax.
		This value is valid only for spot lights and directional lights
		which project a rectangle (for pyramidal projection). It represents the ratio
		of the projection's height to the projection's width and defines
		the projection's rectangle. For pyramidal projections, the fall-off and outer angles
		represent the width of the projection.
		Note that there is no way to know if the projection is conic or pyramidal.
		@return The aspect ratio of the light pyramidal projection. */
	float& GetAspectRatio() { return aspectRatio; }
	const float& GetAspectRatio() const { return aspectRatio; } /**< See above. */

	/** Sets the aspect ratio for the light.
		@see GetAspectRatio
		@param ratio The aspect ratio of the light pyramidal projection. */
	void SetAspectRatio(float ratio) { aspectRatio = ratio; }
	
	/** Retrieves the drop-off for the light.
		This value is only used by documents exported by ColladaMaya.
		@return The drop-off for the light. */
	float& GetDropoff() { return dropoff; }
	const float& GetDropoff() const { return dropoff; } /**< See above. */

	/** Sets the drop-off for the light.
		This value is only used by documents exported by ColladaMaya.
		@param factor The drop-off for the light. */
	void SetDropoff(float factor) { dropoff = factor; }

	/** Retrieves whether the directional light overshoots.
		This value is only used by documents exported by ColladaMax.
		This value is valid only for directional lights. This flag
		represents whether the directional light has a global projection,
		as defined in COLLADA, or a cylinder/prism projection.
		Note that there is no way to know if the projection is conic or pyramidal.
		@return Whether the directional light overshoots. */
	bool DoesOvershoot() const { return overshoots; }

	/** Sets whether the directional light overshoots.
		@see DoesOvershoot
		@param _overshoots The overshoot flag for the directional light. */
	void SetOvershoot(bool _overshoots) { overshoots = _overshoots; }

	/** [INTERNAL] Reads in the \<light\> element from a given COLLADA XML tree node.
		@param lightNode A COLLADA XML tree node.
		@return The status of the import. If the status is not successful,
			it may be dangerous to extract information from the light.*/
	FUStatus LoadFromXML(xmlNode* lightNode);

	/** [INTERNAL] Writes out the \<light\> element to the given COLLADA XML tree node.
		@param parentNode The COLLADA XML parent node in which to insert the geometry information.
		@return The created XML tree node. */
	virtual xmlNode* WriteToXML(xmlNode* parentNode) const;
};

#endif // _FCD_LIGHT_H_

