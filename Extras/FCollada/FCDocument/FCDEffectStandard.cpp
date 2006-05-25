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

#include "StdAfx.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDAnimated.h"
#include "FCDocument/FCDEffect.h"
#include "FCDocument/FCDEffectStandard.h"
#include "FCDocument/FCDMaterial.h"
#include "FCDocument/FCDTexture.h"
#include "FUtils/FUDaeEnum.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
#include "FUtils/FUStringConversion.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDEffectStandard::FCDEffectStandard(FCDocument* document, FCDEffect* _parent) : FCDEffectProfile(document, _parent)
{
	emissionColor = translucencyColor = diffuseColor = ambientColor = specularColor = FMVector3::Origin;
	reflectivityFactor = translucencyFactor = specularFactor = emissionFactor = 1.0f;
	reflectivityColor = translucencyColor = FMVector3(1.0f, 1.0f, 1.0f);
	shininess = 20.0f;
	type = CONSTANT;
	isWireframe = isFaceMap = isDoubleSided = isFaceted = isEmissionFactor = false;
	textureBuckets = new FCDTextureList[FUDaeTextureChannel::COUNT];
}

FCDEffectStandard::~FCDEffectStandard()
{
	for (uint32 i = 0; i < FUDaeTextureChannel::COUNT; ++i)
	{
		FCDTextureList& t = textureBuckets[i];
		CLEAR_POINTER_VECTOR(t);
	}
	SAFE_DELETE_ARRAY(textureBuckets);
}

// Retrieve one of the buckets
const FCDTextureList& FCDEffectStandard::GetTextureBucket(uint32 bucket) const
{
	if (bucket < FUDaeTextureChannel::COUNT) return textureBuckets[bucket];
	else return textureBuckets[FUDaeTextureChannel::FILTER]; // Because I think this one will almost always be empty. ;)
}

// Adds a texture to a specific channel.
FCDTexture* FCDEffectStandard::AddTexture(uint32 bucket)
{
	FUAssert(bucket < FUDaeTextureChannel::COUNT, return NULL);
	FCDTexture* texture = new FCDTexture(GetDocument());
	textureBuckets[bucket].push_back(texture);
	return texture;
}

// Releases a texture contained within this effect profile.
void FCDEffectStandard::ReleaseTexture(FCDTexture* texture)
{
	for (uint32 i = 0; i < FUDaeTextureChannel::COUNT; ++i)
	{
		FCDTextureList::iterator it = std::find(textureBuckets[i].begin(), textureBuckets[i].end(), texture);
		if (it != textureBuckets[i].end())
		{
			delete *it;
			textureBuckets[i].erase(it);
			break;
		}
	}
}

// Calculate the opacity for this material
float FCDEffectStandard::GetOpacity() const
{
	return 1.0f - (translucencyColor.x + translucencyColor.y + translucencyColor.z) / 3.0f * translucencyFactor;
}

// Calculate the overall reflectivity for this material
float FCDEffectStandard::GetReflectivity() const
{
	return (reflectivityColor.x + reflectivityColor.y + reflectivityColor.z) / 3.0f * reflectivityFactor;
}

// Look for the effect parameter with the correct semantic, in order to bind/set its value
FCDEffectParameter* FCDEffectStandard::FindParameterBySemantic(const string& semantic)
{
	// Check only the texture buckets, for now.
	for (uint32 i = 0; i < FUDaeTextureChannel::COUNT; ++i)
	{
		for (FCDTextureList::iterator itT = textureBuckets[i].begin(); itT != textureBuckets[i].end(); ++itT)
		{
			FCDEffectParameter* p = (*itT)->FindParameterBySemantic(semantic);
			if (p != NULL) return p;
		}
	}

	return NULL;
}

void FCDEffectStandard::FindParametersBySemantic(const string& semantic, FCDEffectParameterList& parameters)
{
	// Check only the texture buckets, for now.
	for (uint32 i = 0; i < FUDaeTextureChannel::COUNT; ++i)
	{
		for (FCDTextureList::iterator itT = textureBuckets[i].begin(); itT != textureBuckets[i].end(); ++itT)
		{
			(*itT)->FindParametersBySemantic(semantic, parameters);
		}
	}
}

void FCDEffectStandard::FindParametersByReference(const string& reference, FCDEffectParameterList& parameters)
{
	// Check only the texture buckets, for now.
	for (uint32 i = 0; i < FUDaeTextureChannel::COUNT; ++i)
	{
		for (FCDTextureList::iterator itT = textureBuckets[i].begin(); itT != textureBuckets[i].end(); ++itT)
		{
			(*itT)->FindParametersByReference(reference, parameters);
		}
	}
}

// Clone the standard effect
FCDEffectProfile* FCDEffectStandard::Clone(FCDEffect* newParent)
{
	FCDEffectStandard* clone = new FCDEffectStandard(GetDocument(), newParent);
	clone->type = type;
	for (uint32 i = 0; i < FUDaeTextureChannel::COUNT; ++i)
	{
		for (FCDTextureList::iterator itT = textureBuckets[i].begin(); itT != textureBuckets[i].end(); ++itT)
		{
			clone->textureBuckets[i].push_back((*itT)->Clone());
		}
	}

#	define CLONE_ANIMATED_F(flt) clone->flt = flt; FCDAnimatedFloat::Clone(GetDocument(), &flt, &clone->flt);
#	define CLONE_ANIMATED_C(flt) clone->flt = flt; FCDAnimatedColor::Clone(GetDocument(), &flt, &clone->flt);

	CLONE_ANIMATED_C(emissionColor); CLONE_ANIMATED_F(emissionFactor); clone->isEmissionFactor = isEmissionFactor;
	CLONE_ANIMATED_C(translucencyColor); CLONE_ANIMATED_F(translucencyFactor);
	CLONE_ANIMATED_C(diffuseColor); CLONE_ANIMATED_C(ambientColor);
	CLONE_ANIMATED_C(specularColor); CLONE_ANIMATED_F(specularFactor); CLONE_ANIMATED_F(shininess);
	CLONE_ANIMATED_C(reflectivityColor); CLONE_ANIMATED_F(reflectivityFactor);
	clone->isFaceted = isFaceted;
	clone->isDoubleSided = isDoubleSided;
	clone->isWireframe = isWireframe;
	clone->isFaceMap = isFaceMap;

#	undef CLONE_ANIMATED_F
#	undef CLONE_ANIMATED_C

	return clone;
}

// Read in a <material> node from the COLLADA document
FUStatus FCDEffectStandard::LoadFromXML(xmlNode* baseNode)
{
	FUStatus status;

	// <shader> support is for COLLADA 1.3 backward compatibility
	bool isCollada1_3 = IsEquivalent(baseNode->name, DAE_SHADER_ELEMENT);
	if (!isCollada1_3 && !IsEquivalent(baseNode->name, DAE_FX_PROFILE_COMMON_ELEMENT))
	{
		return status.Warning(FS("Unknown element as standard material base."), baseNode->line);
	}

	// For COLLADA 1.3 backward compatibility: find the correct base node for the profile.
	// While we're digging, find the node with the Max-specific parameters
	xmlNode* maxParameterNode = NULL;
	xmlNode* mayaParameterNode = NULL;
	if (isCollada1_3)
	{
		// COLLADA 1.3 backward compatibility: the base node is <shader><technique><pass>
		// and the Max-specific parameters are in <shader><technique><pass><program>.
		xmlNode* commonTechniqueNode = FindTechnique(baseNode, DAE_COMMON_PROFILE);
		xmlNode* maxTechniqueNode = FindTechnique(baseNode, DAEMAX_MAX_PROFILE);
		baseNode = FindChildByType(commonTechniqueNode, DAE_PASS_ELEMENT);
		xmlNode* maxPassNode = FindChildByType(maxTechniqueNode, DAE_PASS_ELEMENT);
		maxParameterNode = FindChildByType(maxPassNode, DAE_PROGRAM_ELEMENT);
	}
	else
	{
		// Bump the base node up the first <technique> element
		xmlNode* techniqueNode = FindChildByType(baseNode, DAE_TECHNIQUE_ELEMENT);
		if (techniqueNode == NULL)
		{
			return status.Warning(FS("Expecting <technique> within the <profile_COMMON> element for effect: ") + TO_FSTRING(GetDaeId()), baseNode->line);
		}
		baseNode = techniqueNode;

		// Look for an <extra><technique> node for Max-specific parameter
		xmlNode* extraNode = FindChildByType(baseNode, DAE_EXTRA_ELEMENT);
		maxParameterNode = FindTechnique(extraNode, DAEMAX_MAX_PROFILE);
		mayaParameterNode = FindTechnique(extraNode, DAEMAYA_MAYA_PROFILE);
	}

	if (isCollada1_3)
	{
		// COLLADA 1.3 backward compatibility: look for <input> elements pointing to textures
		xmlNodeList textureInputNodes;
		FindChildrenByType(baseNode, DAE_INPUT_ELEMENT, textureInputNodes);
		for (xmlNodeList::iterator it = textureInputNodes.begin(); it != textureInputNodes.end(); ++it)
		{
			string semantic = ReadNodeSemantic(*it);
			if (semantic != DAE_TEXTURE_INPUT)
			{
				status.Warning(FS("Unknown input semantic in material: ") + TO_FSTRING(GetDaeId()), (*it)->line);
				continue;
			}

			// Retrieve the texture and bucket it
			string textureId = ReadNodeSource(*it);
			FCDTexture* texture = GetDocument()->FindTexture(textureId);
			if (texture != NULL)
			{
				uint32 channel = (uint32) texture->GetTextureChannel();
				textureBuckets[(uint32) channel].push_back(texture->Clone());
			}
			else
			{
				status.Warning(FS("Unknown input texture '") + TO_FSTRING(textureId) + FS("' in material: ") + TO_FSTRING(GetDaeId()), (*it)->line);
			}
		}
	}

	// Parse the material's program node and figure out the correct shader type
	xmlNode* commonParameterNode = NULL;
	if (isCollada1_3)
	{
		// COLLADA 1.3 backward compatibility: read in the type attribute of the <program> node
		commonParameterNode = FindChildByType(baseNode, DAE_PROGRAM_ELEMENT);
		FUUri programUrl = ReadNodeUrl(commonParameterNode);
		string materialType = FUStringConversion::ToString(programUrl.prefix);
		if (materialType == DAE_CONSTANT_MATERIAL_PROGRAM) type = CONSTANT;
		else if (materialType == DAE_LAMBERT_MATERIAL_PROGRAM) type = LAMBERT;
		else if (materialType == DAE_PHONG_MATERIAL_PROGRAM) type = PHONG;
		else
		{
			return status.Warning(FS("Unsupported shader program type: '") + programUrl.prefix + FS("' in material: ") + TO_FSTRING(GetDaeId()), commonParameterNode->line);
		}
	}
	else
	{
		// Either <phong>, <lambert> or <constant> are expected
		for (commonParameterNode = baseNode->children; commonParameterNode != NULL; commonParameterNode = commonParameterNode->next)
		{
			if (commonParameterNode->type != XML_ELEMENT_NODE) continue;
			if (IsEquivalent(commonParameterNode->name, DAE_FXSTD_CONSTANT_ELEMENT)) { type = CONSTANT; break; }
			else if (IsEquivalent(commonParameterNode->name, DAE_FXSTD_LAMBERT_ELEMENT)) { type = LAMBERT; break; }
			else if (IsEquivalent(commonParameterNode->name, DAE_FXSTD_PHONG_ELEMENT)) { type = PHONG; break; }
			else if (IsEquivalent(commonParameterNode->name, DAE_FXSTD_BLINN_ELEMENT)) { type = BLINN; break; }
		}
	}
	if (commonParameterNode == NULL)
	{
		return status.Fail(FS("Unable to find the program node for standard effect: ") + TO_FSTRING(GetDaeId()), baseNode->line);
	}

	bool hasTranslucency = false, hasReflectivity = false;
	FCDTextureList emptyBucket;

	// Read in the parameters for the common program types and apply them to the shader
	StringList parameterNames;
	xmlNodeList parameterNodes;
	FindParameters(commonParameterNode, parameterNames, parameterNodes);
	FindParameters(maxParameterNode, parameterNames, parameterNodes);
	FindParameters(mayaParameterNode, parameterNames, parameterNodes);
	size_t parameterCount = parameterNodes.size();
	for (size_t i = 0; i < parameterCount; ++i)
	{
		xmlNode* parameterNode = parameterNodes[i];
		const string& parameterName = parameterNames[i];
		const char* parameterContent = ReadNodeContentDirect(parameterNode);
		if (parameterName == DAE_EMISSION_MATERIAL_PARAMETER || parameterName == DAE_EMISSION_MATERIAL_PARAMETER1_3)
		{
			status.AppendStatus(ParseColorTextureParameter(parameterNode, emissionColor, textureBuckets[FUDaeTextureChannel::EMISSION]));
		}
		else if (parameterName == DAE_DIFFUSE_MATERIAL_PARAMETER || parameterName == DAE_DIFFUSE_MATERIAL_PARAMETER1_3)
		{
			status.AppendStatus(ParseColorTextureParameter(parameterNode, diffuseColor, textureBuckets[FUDaeTextureChannel::DIFFUSE]));
		}
		else if (parameterName == DAE_AMBIENT_MATERIAL_PARAMETER || parameterName == DAE_AMBIENT_MATERIAL_PARAMETER1_3)
		{
			status.AppendStatus(ParseColorTextureParameter(parameterNode, ambientColor, textureBuckets[FUDaeTextureChannel::AMBIENT]));
		}
		else if (parameterName == DAE_TRANSPARENT_MATERIAL_PARAMETER || parameterName == DAE_TRANSPARENT_MATERIAL_PARAMETER1_3)
		{
			status.AppendStatus(ParseColorTextureParameter(parameterNode, translucencyColor, textureBuckets[FUDaeTextureChannel::TRANSPARENT]));
			hasTranslucency = true;
		}
		else if (parameterName == DAE_TRANSPARENCY_MATERIAL_PARAMETER || parameterName == DAE_TRANSPARENCY_MATERIAL_PARAMETER1_3)
		{
			status.AppendStatus(ParseFloatTextureParameter(parameterNode, translucencyFactor, textureBuckets[FUDaeTextureChannel::OPACITY]));
			hasTranslucency = true;
		}
		else if (parameterName == DAE_SPECULAR_MATERIAL_PARAMETER || parameterName == DAE_SPECULAR_MATERIAL_PARAMETER1_3)
		{
			status.AppendStatus(ParseColorTextureParameter(parameterNode, specularColor, textureBuckets[FUDaeTextureChannel::SPECULAR]));
		}
		else if (parameterName == DAE_SPECULAR_MATERIAL_PARAMETER || parameterName == DAE_SPECULAR_MATERIAL_PARAMETER1_3)
		{
			status.AppendStatus(ParseColorTextureParameter(parameterNode, specularColor, textureBuckets[FUDaeTextureChannel::SPECULAR]));
		}
		else if (parameterName == DAE_SHININESS_MATERIAL_PARAMETER || parameterName == DAE_SHININESS_MATERIAL_PARAMETER1_3)
		{
			status.AppendStatus(ParseFloatTextureParameter(parameterNode, shininess, textureBuckets[FUDaeTextureChannel::SHININESS]));
		}
		else if (parameterName == DAE_REFLECTIVE_MATERIAL_PARAMETER || parameterName == DAE_REFLECTIVE_MATERIAL_PARAMETER1_3)
		{
			status.AppendStatus(ParseColorTextureParameter(parameterNode, reflectivityColor, textureBuckets[FUDaeTextureChannel::REFLECTION]));
			hasReflectivity = true;
		}
		else if (parameterName == DAE_REFLECTIVITY_MATERIAL_PARAMETER || parameterName == DAE_REFLECTIVITY_MATERIAL_PARAMETER1_3)
		{
			status.AppendStatus(ParseFloatTextureParameter(parameterNode, reflectivityFactor, emptyBucket));
			hasReflectivity = true;
		}
		else if (parameterName == DAE_BUMP_MATERIAL_PARAMETER)
		{
			status.AppendStatus(ParseSimpleTextureParameter(parameterNode, textureBuckets[FUDaeTextureChannel::BUMP]));
		}
		else if (parameterName == DAEMAX_SPECLEVEL_MATERIAL_PARAMETER || parameterName == DAEMAX_SPECLEVEL_MATERIAL_PARAMETER1_3)
		{
			status.AppendStatus(ParseFloatTextureParameter(parameterNode, specularFactor, textureBuckets[FUDaeTextureChannel::SPECULAR_LEVEL]));
		}
		else if (parameterName == DAEMAX_EMISSIONLEVEL_MATERIAL_PARAMETER || parameterName == DAEMAX_EMISSIONLEVEL_MATERIAL_PARAMETER1_3)
		{
			status.AppendStatus(ParseFloatTextureParameter(parameterNode, emissionFactor, textureBuckets[FUDaeTextureChannel::EMISSION]));
			isEmissionFactor = true;
		}
		else if (parameterName == DAEMAX_FACETED_MATERIAL_PARAMETER || parameterName == DAEMAX_FACETED_MATERIAL_PARAMETER1_3)
		{
			isFaceted = FUStringConversion::ToBoolean(parameterContent);
		}
		else if (parameterName == DAESHD_DOUBLESIDED_PARAMETER)
		{
			isDoubleSided = FUStringConversion::ToBoolean(parameterContent);
		}
		else if (parameterName == DAEMAX_WIREFRAME_MATERIAL_PARAMETER)
		{
			isWireframe = FUStringConversion::ToBoolean(parameterContent);
		}
		else if (parameterName == DAEMAX_FACEMAP_MATERIAL_PARAMETER)
		{
			isFaceMap = FUStringConversion::ToBoolean(parameterContent);
		}
		else if (parameterName == DAEMAX_INDEXOFREFRACTION_MATERIAL_PARAMETER)
		{
			status.AppendStatus(ParseSimpleTextureParameter(parameterNode, textureBuckets[FUDaeTextureChannel::REFRACTION]));
		}
		else if (parameterName == DAEMAX_DISPLACEMENT_MATERIAL_PARAMETER)
		{
			status.AppendStatus(ParseSimpleTextureParameter(parameterNode, textureBuckets[FUDaeTextureChannel::DISPLACEMENT]));
		}
		else if (parameterName == DAEMAX_FILTERCOLOR_MATERIAL_PARAMETER)
		{
			status.AppendStatus(ParseSimpleTextureParameter(parameterNode, textureBuckets[FUDaeTextureChannel::FILTER]));
		}
		else
		{
			status.Warning(FS("Unknown parameter name for material ") + TO_FSTRING(GetDaeId()), parameterNode->line);
		}
	}

	bool isEmptyBucketEmpty = emptyBucket.empty();
	CLEAR_POINTER_VECTOR(emptyBucket);
	if (!isEmptyBucketEmpty)
	{
		return status.Fail(FS("Unexpected texture sampler on some parameters for material ") + TO_FSTRING(GetDaeId()), baseNode->line);
	}

	// Although the default COLLADA materials gives, wrongly, a transparent material,
	// when neither the TRANSPARENT or TRANSPARENCY parameters are set, assume an opaque material.
	// Similarly for reflectivity
	if (!hasTranslucency)
	{
		translucencyColor = FMVector3::Origin;
		translucencyFactor = 0.0f;
	}
	if (!hasReflectivity)
	{
		reflectivityColor = FMVector3::Origin;
		reflectivityFactor = 0.0f;
	}

	// Convert some of the values that may appear in different formats
	if (!isEmissionFactor)
	{
		emissionFactor = (emissionColor.x + emissionColor.y + emissionColor.z) / 3.0f;
	}

	return status;
}

xmlNode* FCDEffectStandard::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* profileCommonNode = AddChild(parentNode, DAE_FX_PROFILE_COMMON_ELEMENT);
	xmlNode* techniqueCommonNode = AddChild(profileCommonNode, DAE_TECHNIQUE_ELEMENT);
	AddNodeSid(techniqueCommonNode, "common");

	const char* materialName;
	switch (type)
	{
	case CONSTANT: materialName = DAE_FXSTD_CONSTANT_ELEMENT; break;
	case LAMBERT: materialName = DAE_FXSTD_LAMBERT_ELEMENT; break;
	case PHONG: materialName = DAE_FXSTD_PHONG_ELEMENT; break;
	case BLINN: materialName = DAE_FXSTD_BLINN_ELEMENT; break;
	case UNKNOWN:
	default: materialName = DAEERR_UNKNOWN_ELEMENT; break;
	}
	xmlNode* materialNode = AddChild(techniqueCommonNode, materialName);
	xmlNode* techniqueNode = AddExtraTechniqueChild(techniqueCommonNode, DAEMAYA_MAYA_PROFILE);

	// Export the color/float parameters
	FCDTextureList emptyBucket; float emptyValue = 0.0f; FMVector3 emptyColor;
	WriteColorTextureParameterToXML(materialNode, DAE_EMISSION_MATERIAL_PARAMETER, emissionColor, textureBuckets[FUDaeTextureChannel::EMISSION]);
	if (type != CONSTANT)
	{
		WriteColorTextureParameterToXML(materialNode, DAE_AMBIENT_MATERIAL_PARAMETER, emissionColor, textureBuckets[FUDaeTextureChannel::AMBIENT]);
		WriteColorTextureParameterToXML(materialNode, DAE_DIFFUSE_MATERIAL_PARAMETER, diffuseColor, textureBuckets[FUDaeTextureChannel::DIFFUSE]);
		if (type != LAMBERT)
		{
			WriteColorTextureParameterToXML(materialNode, DAE_SPECULAR_MATERIAL_PARAMETER, specularColor, textureBuckets[FUDaeTextureChannel::SPECULAR]);
			WriteFloatTextureParameterToXML(materialNode, DAE_SHININESS_MATERIAL_PARAMETER, shininess, emptyBucket);
			if (!textureBuckets[FUDaeTextureChannel::SHININESS].empty())
			{
				WriteFloatTextureParameterToXML(techniqueNode, DAE_SHININESS_MATERIAL_PARAMETER, shininess, textureBuckets[FUDaeTextureChannel::SHININESS]);
				WriteFloatTextureParameterToXML(techniqueNode, DAEMAX_SPECLEVEL_MATERIAL_PARAMETER, specularFactor, textureBuckets[FUDaeTextureChannel::SPECULAR_LEVEL]);
			}
		}
	}
	WriteColorTextureParameterToXML(materialNode, DAE_REFLECTIVE_MATERIAL_PARAMETER, reflectivityColor, textureBuckets[FUDaeTextureChannel::REFLECTION]);
	WriteFloatTextureParameterToXML(materialNode, DAE_REFLECTIVITY_MATERIAL_PARAMETER, reflectivityFactor, emptyBucket);
	
	// Translucency includes both transparent and opacity textures
	FCDTextureList translucencyBucket = textureBuckets[FUDaeTextureChannel::TRANSPARENT];
	translucencyBucket.insert(translucencyBucket.end(), textureBuckets[FUDaeTextureChannel::OPACITY].begin(), textureBuckets[FUDaeTextureChannel::OPACITY].end());
	WriteColorTextureParameterToXML(materialNode, DAE_TRANSPARENT_MATERIAL_PARAMETER, translucencyColor, translucencyBucket);
	WriteFloatTextureParameterToXML(materialNode, DAE_TRANSPARENCY_MATERIAL_PARAMETER, translucencyFactor, emptyBucket);
	WriteFloatTextureParameterToXML(materialNode, DAEMAX_INDEXOFREFRACTION_MATERIAL_PARAMETER, emptyValue, textureBuckets[FUDaeTextureChannel::REFRACTION]);

	// Non-COLLADA parameters
	if (!textureBuckets[FUDaeTextureChannel::BUMP].empty())
	{
		WriteFloatTextureParameterToXML(techniqueNode, DAE_BUMP_MATERIAL_PARAMETER, emptyValue, textureBuckets[FUDaeTextureChannel::BUMP]);
	}
	if (!textureBuckets[FUDaeTextureChannel::DISPLACEMENT].empty())
	{
		WriteFloatTextureParameterToXML(techniqueNode, DAEMAX_DISPLACEMENT_MATERIAL_PARAMETER, emptyValue, textureBuckets[FUDaeTextureChannel::DISPLACEMENT]);
	}
	if (!textureBuckets[FUDaeTextureChannel::FILTER].empty())
	{
		WriteColorTextureParameterToXML(techniqueNode, DAEMAX_FILTERCOLOR_MATERIAL_PARAMETER, emptyColor, textureBuckets[FUDaeTextureChannel::FILTER]);
	}

	return profileCommonNode;
}

xmlNode* FCDEffectStandard::WriteColorTextureParameterToXML(xmlNode* parentNode, const char* parameterNodeName, const FMVector3& value, const FCDTextureList& textureBucket) const
{
	xmlNode* parameterNode = AddChild(parentNode, parameterNodeName);
	if (WriteTextureParameterToXML(parameterNode, textureBucket) == NULL)
	{
		// The color value expected by the profile_COMMON <color> element has four floating-point values.
		string colorValue = FUStringConversion::ToString(value) + " 1.0";
		xmlNode* valueNode = AddChild(parameterNode, DAE_FXSTD_COLOR_ELEMENT, colorValue);
		GetDocument()->WriteAnimatedValueToXML(&value.x, valueNode, parameterNodeName);
	}
	return parameterNode;
}

xmlNode* FCDEffectStandard::WriteFloatTextureParameterToXML(xmlNode* parentNode, const char* parameterNodeName, const float& value, const FCDTextureList& textureBucket) const
{
	xmlNode* parameterNode = AddChild(parentNode, parameterNodeName);
	if (WriteTextureParameterToXML(parameterNode, textureBucket) == NULL)
	{
		xmlNode* valueNode = AddChild(parameterNode, DAE_FXSTD_FLOAT_ELEMENT, value);
		GetDocument()->WriteAnimatedValueToXML(&value, valueNode, parameterNodeName);
	}
	return parameterNode;
}

xmlNode* FCDEffectStandard::WriteTextureParameterToXML(xmlNode* parentNode, const FCDTextureList& textureBucket) const
{
	xmlNode* textureNode = NULL;
	for (FCDTextureList::const_iterator itT = textureBucket.begin(); itT != textureBucket.end(); ++itT)
	{
		xmlNode* newTextureNode = (*itT)->WriteToXML(parentNode);
		if (newTextureNode != NULL && textureNode == NULL) textureNode = newTextureNode;
	}
	return textureNode;
}

// Parse in the different standard effect parameters, bucketing the textures
FUStatus FCDEffectStandard::ParseColorTextureParameter(xmlNode* parameterNode, FMVector3& value, FCDTextureList& textureBucket)
{
	FUStatus status;

	// Look for <texture> elements, they pre-empt everything else
	size_t originalSize = textureBucket.size();
	ParseSimpleTextureParameter(parameterNode, textureBucket);
	if (originalSize < textureBucket.size()) { value = FMVector3(1.0f, 1.0f, 1.0f); return status; }

	// Next, look for a <color> element
	// COLLADA 1.3 backward compatibility: also look for the color value directly inside the parameter node.
	xmlNode* colorNode = FindChildByType(parameterNode, DAE_FXSTD_COLOR_ELEMENT);
	const char* content = ReadNodeContentDirect(colorNode);
	if (content == NULL || *content == 0) content = ReadNodeContentDirect(parameterNode);

	// Parse the color value and allow for an animation of it
	value = FUStringConversion::ToPoint(content);
	if (HasNodeProperty(colorNode, DAE_ID_ATTRIBUTE) || HasNodeProperty(colorNode, DAE_SID_ATTRIBUTE))
		FCDAnimatedColor::Create(GetDocument(), colorNode, &value);
	else
		FCDAnimatedColor::Create(GetDocument(), parameterNode, &value);

	return status;
}

FUStatus FCDEffectStandard::ParseFloatTextureParameter(xmlNode* parameterNode, float& value, FCDTextureList& textureBucket)
{
	FUStatus status;

	// Look for <texture> elements, they pre-empt everything else
	size_t originalSize = textureBucket.size();
	ParseSimpleTextureParameter(parameterNode, textureBucket);
	if (originalSize < textureBucket.size()) { value = 1.0f; return status; }

	// Next, look for a <float> element
	// COLLADA 1.3 backward compatibility: also look for the value directly inside the parameter node.
	xmlNode* floatNode = FindChildByType(parameterNode, DAE_FXSTD_FLOAT_ELEMENT);
	const char* content = ReadNodeContentDirect(floatNode);
	if (content == NULL || *content == 0) content = ReadNodeContentDirect(parameterNode);

	// Parse the value and register it for an animation.
	value = FUStringConversion::ToFloat(content);
	if (HasNodeProperty(floatNode, DAE_ID_ATTRIBUTE) || HasNodeProperty(floatNode, DAE_SID_ATTRIBUTE))
		FCDAnimatedFloat::Create(GetDocument(), floatNode, &value);
	else
		FCDAnimatedFloat::Create(GetDocument(), parameterNode, &value);


	return status;
}

FUStatus FCDEffectStandard::ParseSimpleTextureParameter(xmlNode* parameterNode, FCDTextureList& textureBucket)
{
	FUStatus status;

	// Parse in all the <texture> elements as standard effect samplers
	xmlNodeList samplerNodes;
	FindChildrenByType(parameterNode, DAE_FXSTD_SAMPLER_ELEMENT, samplerNodes);
	if (!samplerNodes.empty())
	{
		for (xmlNodeList::iterator itS = samplerNodes.begin(); itS != samplerNodes.end(); ++itS)
		{
			// Parse in the texture element and bucket them
			FCDTexture* texture = new FCDTexture(GetDocument());
			status.AppendStatus(texture->LoadFromTextureXML(*itS));
			if (status) textureBucket.push_back(texture);
			else { SAFE_DELETE(texture); }
		}
	}
	return status;
}
