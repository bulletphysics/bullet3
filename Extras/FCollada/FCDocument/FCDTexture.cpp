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
#include "FCDocument/FCDEffectParameter.h"
#include "FCDocument/FCDEffectParameterList.h"
#include "FCDocument/FCDImage.h"
#include "FCDocument/FCDTexture.h"
#include "FUtils/FUStringConversion.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDTexture::FCDTexture(FCDocument* document) : FCDEntity(document, "Texture")
{
	textureChannel = FUDaeTextureChannel::DEFAULT;
	image = NULL;
	set = new FCDEffectParameterInt(document);
	set->SetValue(-1);
	multiplier = 1.0f;
	blendMode = FUDaeBlendMode::DEFAULT;
	ClearPlacement2D();
	hasProjection3D = false;
	hasPlacement2D = false;
}

FCDTexture::~FCDTexture()
{
	image = NULL;
	SAFE_DELETE(set);
}

void FCDTexture::ClearPlacement2D()
{
	hasPlacement2D = false;
	wrapU = wrapV = mirrorU = mirrorV = stagger = fast = 0.0f; // false
	translateFrameU = translateFrameV = rotateFrame = 0.0f;
	offsetU = offsetV = rotateUV = noiseU = noiseV = 0.0f;
	coverageU = coverageV = repeatU = repeatV = 1.0f;
}

void FCDTexture::ClearProjection3D()
{
	hasProjection3D = false;
	projectionMatrix = FMMatrix44::Identity;
	projectionType.clear();
}

// Look for the effect parameter with the correct semantic, in order to bind/set its value
FCDEffectParameter* FCDTexture::FindParameterBySemantic(const string& semantic)
{
	if (set->GetSemantic() == semantic) return set;
	else return NULL;
}

void FCDTexture::FindParametersBySemantic(const string& semantic, FCDEffectParameterList& parameters)
{
	if (set->GetSemantic() == semantic) parameters.push_back(set);
}

void FCDTexture::FindParametersByReference(const string& reference, FCDEffectParameterList& parameters)
{
	if (set->GetReference() == reference) parameters.push_back(set);
}

// Returns a copy of the texture/sampler, with all the animations attached
FCDTexture* FCDTexture::Clone()
{
	FCDTexture* clone = new FCDTexture(GetDocument());
	FCDEntity::Clone(clone);
	clone->textureChannel = textureChannel;
	clone->image = image; // copy only the pointer.
	SAFE_DELETE(clone->set);
	clone->set = (FCDEffectParameterInt*) set->Clone();

	clone->hasProjection3D = hasProjection3D;
	clone->hasPlacement2D = hasPlacement2D;
	clone->blendMode = blendMode;
	clone->projectionMatrix = projectionMatrix;

#	define CLONE_ANIMATED(flt) clone->flt = flt; FCDAnimatedFloat::Clone(GetDocument(), &flt, &clone->flt);
	CLONE_ANIMATED(multiplier);
	CLONE_ANIMATED(wrapU); CLONE_ANIMATED(wrapV);
	CLONE_ANIMATED(mirrorU); CLONE_ANIMATED(mirrorV);
	CLONE_ANIMATED(stagger); CLONE_ANIMATED(fast);
	CLONE_ANIMATED(translateFrameU); CLONE_ANIMATED(translateFrameV);
	CLONE_ANIMATED(offsetU); CLONE_ANIMATED(offsetV);
	CLONE_ANIMATED(rotateUV);
	CLONE_ANIMATED(noiseU); CLONE_ANIMATED(noiseV);
	CLONE_ANIMATED(coverageU); CLONE_ANIMATED(coverageV);
	CLONE_ANIMATED(repeatU); CLONE_ANIMATED(repeatV);
#	undef CLONE_ANIMATED

	return clone;
}

// Read in the texture element for a standard effect COLLADA texture
FUStatus FCDTexture::LoadFromTextureXML(xmlNode* textureNode)
{
	FUStatus status;

	// Verify that this is a sampler node
	if (!IsEquivalent(textureNode->name, DAE_FXSTD_SAMPLER_ELEMENT))
	{
		return status.Fail(FS("Unknown texture sampler element."), textureNode->line);
	}
	
	// Read in the 'texture' attribute: points to an image
	string imageId = ReadNodeProperty(textureNode, DAE_FXSTD_TEXTURE_ATTRIBUTE);
	image = GetDocument()->FindImage(imageId);
	if (image == NULL)
	{
		status.Warning(FS("Unable to find image source for sampler."), textureNode->line);
	}

	// Read in the 'texcoord' attribute: a texture coordinate set identifier
	string semantic = ReadNodeProperty(textureNode, DAE_FXSTD_TEXTURESET_ATTRIBUTE);
	if (!semantic.empty())
	{
		set->SetSemantic(semantic);

		// [GLaforte 06-01-06] Also attempt to convert the value to a signed integer
		// since that was done quite a bit in COLLADA 1.4 preview exporters.
		set->SetValue(FUStringConversion::ToInt32(semantic));
	}

	// Look for/parse in the texture placement parameters
	xmlNode* extraNode = FindChildByType(textureNode, DAE_EXTRA_ELEMENT);
	xmlNode* mayaTechniqueNode = FindTechnique(extraNode, DAEMAYA_MAYA_PROFILE);
	status.AppendStatus(LoadPlacementXML(mayaTechniqueNode));
	xmlNode* maxTechniqueNode = FindTechnique(extraNode, DAEMAX_MAX_PROFILE);
	status.AppendStatus(LoadPlacementXML(maxTechniqueNode));

	return status;
}

// COLLADA 1.3 Backward compatibility: Read in a <texture> node from the COLLADA document
FUStatus FCDTexture::LoadFromXML(xmlNode* textureNode)
{
	FUStatus status = FCDEntity::LoadFromXML(textureNode);
	if (!status) return status;
	if (!IsEquivalent(textureNode->name, DAE_TEXTURE_ELEMENT))
	{
		return status.Warning(FS("Texture library contains unknown element."), textureNode->line);
	}

	// Read the channel usage for this texture
	xmlNode* usageParameterNode = FindChildByType(textureNode, DAE_PARAMETER_ELEMENT);
	string channelUsage = ReadNodeName(usageParameterNode);
	textureChannel = FUDaeTextureChannel::FromString(channelUsage);
	if (textureChannel == FUDaeTextureChannel::UNKNOWN)
	{
		textureChannel = FUDaeTextureChannel::DEFAULT;
		status.Warning(FS("Unknown channel usage for texture: ") + TO_FSTRING(GetDaeId()), textureNode->line);
	}

	// Retrieve the common technique and verify that the input node inside points to an image
	xmlNode* commonTechniqueNode = FindTechnique(textureNode, DAE_COMMON_PROFILE);
	xmlNode* imageInputNode = FindChildByType(commonTechniqueNode, DAE_INPUT_ELEMENT);
	string inputSemantic = ReadNodeSemantic(imageInputNode);
	if (inputSemantic != DAE_IMAGE_INPUT)
	{
		status.Warning(FS("Unknown input semantic for texture: ") + TO_FSTRING(GetDaeId()), imageInputNode->line);
	}
	else
	{
		// Link to the image input
		string imageId = ReadNodeSource(imageInputNode);
		if (imageId.empty() || imageId[0] != '#')
		{
			status.Warning(FS("Unknown or external image source for texture: ") + TO_FSTRING(GetDaeId()), imageInputNode->line);
		}
		else
		{
			image = GetDocument()->FindImage(imageId);
			if (image == NULL)
			{
				status.Warning(FS("Unable to find image source: ") + TO_FSTRING(imageId), imageInputNode->line);
			}
		}
	}

	// Read in the Maya-specific texture placement parameters
	// and look for a texture program
	xmlNode* mayaTechniqueNode = FindTechnique(textureNode, DAEMAYA_MAYA_PROFILE);
	status.AppendStatus(LoadPlacementXML(mayaTechniqueNode));
	return status;
}

// Read in the Maya-specific texture placement parameters and look for a texture program
FUStatus FCDTexture::LoadPlacementXML(xmlNode* techniqueNode)
{
	// This function loads extra functionality, so it should NEVER fail.
	// It can throw all the warnings it wants, though!
	FUStatus status;

	if (techniqueNode == NULL) return status;

	// Look for some placement 2D parameters
	StringList parameterNames;
	xmlNodeList parameterNodes;
	FindParameters(techniqueNode, parameterNames, parameterNodes);
	size_t parameterCount = parameterNames.size();
	for (size_t i = 0; i < parameterCount; ++i)
	{
		const string& parameterName = parameterNames[i];
		xmlNode* parameterNode = parameterNodes[i];
		const char* content = ReadNodeContentDirect(parameterNode);
		if (content == NULL || *content == 0 || parameterName.empty()) continue;

		#define CHECKBOOLPARAM(memberName, paramName2) \
		if (parameterName == paramName2) { \
			memberName = (float) FUStringConversion::ToBoolean(content); \
			FCDAnimatedFloat::Create(GetDocument(), parameterNode, &memberName); \
			hasPlacement2D = true; } else

		#define CHECKFLOATPARAM(memberName, paramName2) \
		if (parameterName == paramName2) { \
			memberName = FUStringConversion::ToFloat(content); \
			FCDAnimatedFloat::Create(GetDocument(), parameterNode, &memberName); \
			hasPlacement2D = true; } else

		CHECKBOOLPARAM(wrapU, DAEMAYA_TEXTURE_WRAPU_PARAMETER)
		CHECKBOOLPARAM(wrapV, DAEMAYA_TEXTURE_WRAPV_PARAMETER)
		CHECKBOOLPARAM(mirrorU, DAEMAYA_TEXTURE_MIRRORU_PARAMETER)
		CHECKBOOLPARAM(mirrorV, DAEMAYA_TEXTURE_MIRRORV_PARAMETER)
		CHECKFLOATPARAM(coverageU, DAEMAYA_TEXTURE_COVERAGEU_PARAMETER)
		CHECKFLOATPARAM(coverageV, DAEMAYA_TEXTURE_COVERAGEV_PARAMETER)
		CHECKFLOATPARAM(translateFrameU, DAEMAYA_TEXTURE_TRANSFRAMEU_PARAMETER)
		CHECKFLOATPARAM(translateFrameV, DAEMAYA_TEXTURE_TRANSFRAMEV_PARAMETER)
		CHECKFLOATPARAM(rotateFrame, DAEMAYA_TEXTURE_ROTFRAME_PARAMETER)
		CHECKBOOLPARAM(stagger, DAEMAYA_TEXTURE_STAGGER_PARAMETER)
		CHECKBOOLPARAM(fast, DAEMAYA_TEXTURE_FAST_PARAMETER)
		CHECKFLOATPARAM(repeatU, DAEMAYA_TEXTURE_REPEATU_PARAMETER)
		CHECKFLOATPARAM(repeatV, DAEMAYA_TEXTURE_REPEATV_PARAMETER)
		CHECKFLOATPARAM(offsetU, DAEMAYA_TEXTURE_OFFSETU_PARAMETER)
		CHECKFLOATPARAM(offsetV, DAEMAYA_TEXTURE_OFFSETV_PARAMETER)
		CHECKFLOATPARAM(rotateUV, DAEMAYA_TEXTURE_ROTATEUV_PARAMETER)
		CHECKFLOATPARAM(noiseU, DAEMAYA_TEXTURE_NOISEU_PARAMETER)
		CHECKFLOATPARAM(noiseV, DAEMAYA_TEXTURE_NOISEV_PARAMETER)

		CHECKFLOATPARAM(multiplier, DAEMAX_AMOUNT_TEXTURE_PARAMETER)

		#undef CHECKFLOATPARAM
		#undef CHECKBOOLPARAM

		if (parameterName == DAEMAYA_TEXTURE_BLENDMODE_PARAMETER || parameterName == DAEMAYA_TEXTURE_BLENDMODE_PARAMETER1_3)
		{
			blendMode = FUDaeBlendMode::FromString(content);
		}
		else
		{
			status.Warning(FS("Unknown texture parameter: ") + TO_FSTRING(parameterName) + FS(" in texture: ") + TO_FSTRING(GetDaeId()), parameterNode->line);
		}
	}

	xmlNode* mayaProjectionNode = FindChildByType(techniqueNode, DAEMAYA_PROJECTION_ELEMENT);

	// COLLADA 1.3 backward compatibility: check for a <program URL="PROJECTION">, also.
	if (mayaProjectionNode == NULL) mayaProjectionNode = FindChildByType(techniqueNode, DAE_PROGRAM_ELEMENT);
	if (mayaProjectionNode != NULL)
	{
		xmlNodeList parameters;
		FindChildrenByType(mayaProjectionNode, DAE_PARAMETER_ELEMENT, parameters);
		for (xmlNodeList::iterator itP = parameters.begin(); itP != parameters.end(); ++itP)
		{
			xmlNode* param = *itP;
			string paramName = ReadNodeName(param);
			const char* content = ReadNodeContentDirect(param);

			if (paramName == DAEMAYA_PROJECTION_TYPE_PARAMETER || paramName == DAEMAYA_PROJECTION_TYPE_PARAMETER1_3)
			{
				projectionType = TO_FSTRING(content);
			}
			else if (paramName == DAEMAYA_PROJECTION_MATRIX_PARAMETER || paramName == DAEMAYA_PROJECTION_MATRIX_PARAMETER1_3)
			{
				FUStringConversion::ToMatrix(&content, projectionMatrix, GetDocument()->GetLengthUnitConversion());
			}
			else
			{
				status.Warning(FS("Unknown projection program parameter: '") + TO_FSTRING(paramName) + FS("' in texture : ") + TO_FSTRING(GetDaeId()), param->line);
			}
		}
		hasProjection3D = true;
	}

	return status;
}

// Write out the texture to the COLLADA xml node tree
xmlNode* FCDTexture::WriteToXML(xmlNode* parentNode)
{
	// Create the <texture> element
	xmlNode* textureNode = AddChild(parentNode, DAE_TEXTURE_ELEMENT);
	AddAttribute(textureNode, DAE_FXSTD_TEXTURE_ATTRIBUTE, image != NULL ? image->GetDaeId() : "");
	AddAttribute(textureNode, DAE_FXSTD_TEXTURESET_ATTRIBUTE, set->GetValue());

	if (hasPlacement2D || hasProjection3D)
	{
		// Create the <extra> element with the texture placement parameters and their animations
		xmlNode* techniqueNode = AddExtraTechniqueChild(textureNode, DAEMAYA_MAYA_PROFILE);
		if (hasPlacement2D)
		{
#define ADD_PARAM_BOOL(memberName, paramName) { xmlNode* x = AddChild(techniqueNode, paramName, !IsEquivalent(memberName, 0.0f)); GetDocument()->WriteAnimatedValueToXML(&memberName, x, paramName); }
#define ADD_PARAM_FLOAT(memberName, paramName) { xmlNode* x = AddChild(techniqueNode, paramName, memberName); GetDocument()->WriteAnimatedValueToXML(&memberName, x, paramName); }

			ADD_PARAM_BOOL(wrapU, DAEMAYA_TEXTURE_WRAPU_PARAMETER)
			ADD_PARAM_BOOL(wrapV, DAEMAYA_TEXTURE_WRAPV_PARAMETER)
			ADD_PARAM_BOOL(mirrorU, DAEMAYA_TEXTURE_MIRRORU_PARAMETER)
			ADD_PARAM_BOOL(mirrorV, DAEMAYA_TEXTURE_MIRRORV_PARAMETER)
			ADD_PARAM_FLOAT(coverageU, DAEMAYA_TEXTURE_COVERAGEU_PARAMETER)
			ADD_PARAM_FLOAT(coverageV, DAEMAYA_TEXTURE_COVERAGEV_PARAMETER)
			ADD_PARAM_FLOAT(translateFrameU, DAEMAYA_TEXTURE_TRANSFRAMEU_PARAMETER)
			ADD_PARAM_FLOAT(translateFrameV, DAEMAYA_TEXTURE_TRANSFRAMEV_PARAMETER)
			ADD_PARAM_FLOAT(rotateFrame, DAEMAYA_TEXTURE_ROTFRAME_PARAMETER)
			ADD_PARAM_BOOL(stagger, DAEMAYA_TEXTURE_STAGGER_PARAMETER)
			ADD_PARAM_BOOL(fast, DAEMAYA_TEXTURE_FAST_PARAMETER)
			ADD_PARAM_FLOAT(repeatU, DAEMAYA_TEXTURE_REPEATU_PARAMETER)
			ADD_PARAM_FLOAT(repeatV, DAEMAYA_TEXTURE_REPEATV_PARAMETER)
			ADD_PARAM_FLOAT(offsetU, DAEMAYA_TEXTURE_OFFSETU_PARAMETER)
			ADD_PARAM_FLOAT(offsetV, DAEMAYA_TEXTURE_OFFSETV_PARAMETER)
			ADD_PARAM_FLOAT(rotateUV, DAEMAYA_TEXTURE_ROTATEUV_PARAMETER)
			ADD_PARAM_FLOAT(noiseU, DAEMAYA_TEXTURE_NOISEU_PARAMETER)
			ADD_PARAM_FLOAT(noiseV, DAEMAYA_TEXTURE_NOISEV_PARAMETER)
			ADD_PARAM_FLOAT(multiplier, DAEMAX_AMOUNT_TEXTURE_PARAMETER)

#undef ADD_PARAM_BOOL
#undef ADD_PARAM_FLOAT

			string blendModeValue = FUDaeBlendMode::ToString(blendMode);
			AddChild(techniqueNode, DAEMAYA_TEXTURE_BLENDMODE_PARAMETER, blendModeValue);
		}

		if (hasProjection3D)
		{
			xmlNode* projectionNode = AddChild(techniqueNode, DAEMAYA_PROJECTION_ELEMENT);
			AddChild(projectionNode, DAEMAYA_PROJECTION_TYPE_PARAMETER, projectionType);
			AddChild(projectionNode, DAEMAYA_PROJECTION_MATRIX_PARAMETER, FUStringConversion::ToString(projectionMatrix, GetDocument()->GetLengthUnitConversion()));
		}
	}
	return textureNode;
}
