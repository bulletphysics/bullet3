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
#include "FCDocument/FCDLight.h"
#include "FCDocument/FCDSceneNode.h"
#include "FUtils/FUStringConversion.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDLight::FCDLight(FCDocument* document) : FCDTargetedEntity(document, "Light")
{
	color = FMVector3(1.0f, 1.0f, 1.0f);
	intensity = 1.0f;
	lightType = POINT;
	constantAttenuationFactor = linearAttenuationFactor = quadracticAttenuationFactor = 0.0f;
	fallOffExponent = 1.0f;
	outerAngle = fallOffAngle = 30.0f;
	penumbraAngle = 0.0f;
	dropoff = 10.0f;
	overshoots = true;
}

FCDLight::~FCDLight()
{
}

// Load this light from the given COLLADA document's node
FUStatus FCDLight::LoadFromXML(xmlNode* lightNode)
{
	FUStatus status = FCDEntity::LoadFromXML(lightNode);
	if (!status) return status;
	if (!IsEquivalent(lightNode->name, DAE_LIGHT_ELEMENT))
	{
		return status.Warning(FS("Light library contains unknown element."), lightNode->line);
	}

	// Grab the <technique_common> element. If not present, assume COLLADA 1.3
	xmlNode* commonTechniqueNode = FindChildByType(lightNode, DAE_TECHNIQUE_COMMON_ELEMENT);
	bool isCollada1_3 = commonTechniqueNode == NULL;
	xmlNode* maxTechniqueNode = FindTechnique(lightNode, DAEMAX_MAX_PROFILE);
	xmlNode* mayaTechniqueNode = FindTechnique(lightNode, DAEMAYA_MAYA_PROFILE);

	// Create the correct light source
	xmlNode* lightParameterNode = NULL;
	if (isCollada1_3)
	{
		// COLLADA 1.3 backward-compatibility: read the 'type' attribute of the <light> element
		string type = ReadNodeProperty(lightNode, DAE_TYPE_ATTRIBUTE);
		if (type == DAE_AMBIENT_LIGHT_TYPE) lightType = AMBIENT;
		else if (type == DAE_DIRECTIONAL_LIGHT_TYPE) lightType = DIRECTIONAL;
		else if (type == DAE_POINT_LIGHT_TYPE) lightType = POINT;
		else if (type == DAE_SPOT_LIGHT_TYPE) lightType = SPOT;
		else
		{
			status.Warning(FS("Unknown light type value for light: ") + TO_FSTRING(GetDaeId()), lightNode->line);
		}

		// COLLADA 1.3.2 backward-compatibility: look for this hack for the parameters under a common-profile technique
		lightParameterNode = FindTechnique(lightNode, DAE_COMMON_PROFILE);
		if (lightParameterNode == NULL) lightParameterNode = lightNode;
	}
	else
	{
		// Look for the <point>, <directional>, <spot> or <ambient> element under the common-profile technique
		for (xmlNode* child = commonTechniqueNode->children; child != NULL; child = child->next)
		{
			if (child->type != XML_ELEMENT_NODE) continue;
			if (IsEquivalent(child->name, DAE_LIGHT_POINT_ELEMENT)) { lightParameterNode = child; lightType = POINT; break; }
			else if (IsEquivalent(child->name, DAE_LIGHT_SPOT_ELEMENT)) { lightParameterNode = child; lightType = SPOT; break; }
			else if (IsEquivalent(child->name, DAE_LIGHT_AMBIENT_ELEMENT)) { lightParameterNode = child; lightType = AMBIENT; break; }
			else if (IsEquivalent(child->name, DAE_LIGHT_DIRECTIONAL_ELEMENT)) { lightParameterNode = child; lightType = DIRECTIONAL; break; }
			else
			{
				status.Warning(FS("Unknown element under <light><technique_common> for light: ") + TO_FSTRING(GetDaeId()), child->line);
			}
		}
	}

	// Verify the light's basic structures are found
	if (lightParameterNode == NULL)
	{
		return status.Fail(FS("Unable to find the parameter root node for light: ") + TO_FSTRING(GetDaeId()), lightNode->line);
	}

	// Retrieve the light parameters
	StringList parameterNames;
	xmlNodeList parameterNodes;
	FindParameters(lightParameterNode, parameterNames, parameterNodes);
	FindParameters(maxTechniqueNode, parameterNames, parameterNodes);
	FindParameters(mayaTechniqueNode, parameterNames, parameterNodes);
	xmlNode* extraNode = FindChildByType(lightNode, DAE_EXTRA_ELEMENT);
	xmlNode* maxExtraNode = FindTechnique(extraNode, DAEMAX_MAX_PROFILE);
	FindParameters(maxExtraNode, parameterNames, parameterNodes);

	// Parse the light parameters
	FUDaeFunction::Function attenuationFunction = FUDaeFunction::CONSTANT;
	float attenuationFactor = 0.0f;
	xmlNode* attenuationFactorNode = NULL;

	size_t parameterCount = parameterNodes.size();
	for (size_t i = 0; i < parameterCount; ++i)
	{
		xmlNode* parameterNode = parameterNodes[i];
		const string& parameterName = parameterNames[i];
		const char* content = ReadNodeContentDirect(parameterNode);
		if (parameterName == DAE_COLOR_LIGHT_PARAMETER || parameterName == DAE_COLOR_LIGHT_PARAMETER1_3)
		{
			color = FUStringConversion::ToPoint(content);
			FCDAnimatedColor::Create(GetDocument(), parameterNode, &color);
		}
		else if (parameterName == DAE_CONST_ATTENUATION_LIGHT_PARAMETER)
		{
			constantAttenuationFactor = FUStringConversion::ToFloat(content);
			FCDAnimatedFloat::Create(GetDocument(), parameterNode, &constantAttenuationFactor);
		}
		else if (parameterName == DAE_LIN_ATTENUATION_LIGHT_PARAMETER)
		{
			linearAttenuationFactor = FUStringConversion::ToFloat(content);
			FCDAnimatedFloat::Create(GetDocument(), parameterNode, &linearAttenuationFactor);
		}
		else if (parameterName == DAE_QUAD_ATTENUATION_LIGHT_PARAMETER)
		{
			quadracticAttenuationFactor = FUStringConversion::ToFloat(content);
			FCDAnimatedFloat::Create(GetDocument(), parameterNode, &quadracticAttenuationFactor);
		}
		else if (parameterName == DAE_ATTENUATION_LIGHT_PARAMETER1_3)
		{
			attenuationFunction = FUDaeFunction::FromString(content);
		}
		else if (parameterName == DAE_ATTENUATIONSCALE_LIGHT_PARAMETER1_3)
		{
			attenuationFactor = FUStringConversion::ToFloat(content);
			attenuationFactorNode = parameterNode;
		}
		else if (parameterName == DAE_FALLOFFEXPONENT_LIGHT_PARAMETER)
		{
			fallOffExponent = FUStringConversion::ToFloat(content);
			FCDAnimatedFloat::Create(GetDocument(), parameterNode, &fallOffExponent);
		}
		else if (parameterName == DAE_FALLOFF_LIGHT_PARAMETER1_3)
		{
			fallOffExponent = (float) FUDaeFunction::FromString(content);
		}
		else if (parameterName == DAE_FALLOFFANGLE_LIGHT_PARAMETER || parameterName == DAE_ANGLE_LIGHT_PARAMETER1_3)
		{
			fallOffAngle = FUStringConversion::ToFloat(content);
			FCDAnimatedFloat::Create(GetDocument(), parameterNode, &fallOffAngle);
		}
		else if (parameterName == DAESHD_INTENSITY_LIGHT_PARAMETER || parameterName == DAESHD_INTENSITY_LIGHT_PARAMETER1_3)
		{
			intensity = FUStringConversion::ToFloat(content);
			FCDAnimatedFloat::Create(GetDocument(), parameterNode, &intensity);
		}
		else if (parameterName == DAEMAX_OUTERCONE_LIGHT_PARAMETER || parameterName == DAEMAX_OUTERCONE_LIGHT_PARAMETER1_3)
		{
			outerAngle = FUStringConversion::ToFloat(content);
			FCDAnimatedFloat::Create(GetDocument(), parameterNode, &outerAngle);
		}
		else if (parameterName == DAEMAX_OVERSHOOT_LIGHT_PARAMETER || parameterName == DAEMAX_OVERSHOOT_LIGHT_PARAMETER1_3)
		{
			overshoots = FUStringConversion::ToBoolean(content);
		}
		else if (parameterName == DAEMAX_TARGET_LIGHT_PARAMETER || parameterName == DAEMAX_TARGET_LIGHT_PARAMETER1_3)
		{
			SetTargetId(content);
		}
		else if (parameterName == DAEMAYA_PENUMBRA_LIGHT_PARAMETER || parameterName == DAEMAYA_PENUMBRA_LIGHT_PARAMETER1_3)
		{
			penumbraAngle = FUStringConversion::ToFloat(content);
			FCDAnimatedFloat::Create(GetDocument(), parameterNode, &penumbraAngle);
		}
		else if (parameterName == DAEMAYA_DROPOFF_LIGHT_PARAMETER || parameterName == DAE_FALLOFFSCALE_LIGHT_PARAMETER1_3)
		{
			dropoff = FUStringConversion::ToFloat(content);
			FCDAnimatedFloat::Create(GetDocument(), parameterNode, &dropoff);
		}
		else if (parameterName == DAEMAX_ASPECTRATIO_LIGHT_PARAMETER || parameterName == DAEMAX_ASPECTRATIO_LIGHT_PARAMETER1_3)
		{
			aspectRatio = FUStringConversion::ToFloat(content);
			FCDAnimatedFloat::Create(GetDocument(), parameterNode, &aspectRatio);
		}
		else
		{
			status.Warning(FS("Unknown program parameter for light: ") + TO_FSTRING(GetDaeId()), parameterNode->line);
		}
	}

	if (attenuationFactorNode != NULL)
	{
		// Match the deprecated COLLADA 1.3 parameter's animation to the correct, new attenuation factor
		switch (attenuationFunction)
		{
		case FUDaeFunction::CONSTANT:
			constantAttenuationFactor = attenuationFactor;
			FCDAnimatedFloat::Create(GetDocument(), attenuationFactorNode, &constantAttenuationFactor);
			break; 
		case FUDaeFunction::LINEAR:
			linearAttenuationFactor = attenuationFactor;
			FCDAnimatedFloat::Create(GetDocument(), attenuationFactorNode, &linearAttenuationFactor);
			break; 
		case FUDaeFunction::QUADRATIC:
			quadracticAttenuationFactor = attenuationFactor;
			FCDAnimatedFloat::Create(GetDocument(), attenuationFactorNode, &quadracticAttenuationFactor);
			break; 
		default: break;
		}
	}

	return status;
}

// Write out this light to the COLLADA XML document
xmlNode* FCDLight::WriteToXML(xmlNode* parentNode) const
{
	// Create the base light node
	xmlNode* lightNode = WriteToEntityXML(parentNode, DAE_LIGHT_ELEMENT);
	xmlNode* baseNode = AddChild(lightNode, DAE_TECHNIQUE_COMMON_ELEMENT);
	const char* baseNodeName;
	switch (lightType)
	{
	case POINT: baseNodeName = DAE_LIGHT_POINT_ELEMENT; break;
	case SPOT: baseNodeName = DAE_LIGHT_SPOT_ELEMENT; break;
	case AMBIENT: baseNodeName = DAE_LIGHT_AMBIENT_ELEMENT; break;
	case DIRECTIONAL: baseNodeName = DAE_LIGHT_DIRECTIONAL_ELEMENT; break;
	default: baseNodeName = DAEERR_UNKNOWN_INPUT; break;
	}
	baseNode = AddChild(baseNode, baseNodeName);

	// Add the application-specific technique
	xmlNode* techniqueMayaNode = AddTechniqueChild(lightNode, DAEMAYA_MAYA_PROFILE);
	xmlNode* techniqueMaxNode = AddTechniqueChild(lightNode, DAEMAX_MAX_PROFILE);

	// Write out the light parameters
	string colorValue = FUStringConversion::ToString(color);
	xmlNode* colorNode = AddChild(baseNode, DAE_COLOR_LIGHT_PARAMETER, colorValue);
	GetDocument()->WriteAnimatedValueToXML(&color.x, colorNode, "color");
	
	xmlNode* attenuationBaseNode = (lightType == POINT || lightType == SPOT) ? baseNode : techniqueMaxNode;
	xmlNode* attenuationNode = AddChild(attenuationBaseNode, DAE_CONST_ATTENUATION_LIGHT_PARAMETER, constantAttenuationFactor);
	GetDocument()->WriteAnimatedValueToXML(&constantAttenuationFactor, attenuationNode, "constant_attenuation");
	attenuationNode = AddChild(attenuationBaseNode, DAE_LIN_ATTENUATION_LIGHT_PARAMETER, linearAttenuationFactor);
	GetDocument()->WriteAnimatedValueToXML(&linearAttenuationFactor, attenuationNode, "linear_attenuation");
	attenuationNode = AddChild(attenuationBaseNode, DAE_QUAD_ATTENUATION_LIGHT_PARAMETER, quadracticAttenuationFactor);
	GetDocument()->WriteAnimatedValueToXML(&quadracticAttenuationFactor, attenuationNode, "quadratic_attenuation");
	
	xmlNode* falloffBaseNode = (lightType == SPOT) ? baseNode : techniqueMaxNode;
	xmlNode* falloffNode = AddChild(falloffBaseNode, DAE_FALLOFFANGLE_LIGHT_PARAMETER, fallOffAngle);
	GetDocument()->WriteAnimatedValueToXML(&fallOffAngle, falloffNode, "falloff_angle");
	falloffNode = AddChild(falloffBaseNode, DAE_FALLOFFEXPONENT_LIGHT_PARAMETER, fallOffExponent);
	GetDocument()->WriteAnimatedValueToXML(&fallOffExponent, falloffNode, "falloff_exponent");

	xmlNode* intensityNode = AddChild(techniqueMaxNode, DAESHD_INTENSITY_LIGHT_PARAMETER, intensity);
	AddChild(techniqueMayaNode, DAESHD_INTENSITY_LIGHT_PARAMETER, intensity);
	GetDocument()->WriteAnimatedValueToXML(&intensity, intensityNode, "intensity");
	xmlNode* outerAngleNode = AddChild(techniqueMaxNode, DAEMAX_OUTERCONE_LIGHT_PARAMETER, outerAngle);
	GetDocument()->WriteAnimatedValueToXML(&outerAngle, outerAngleNode, "outer_angle");
	xmlNode* penumbraAngleNode = AddChild(techniqueMayaNode, DAEMAYA_PENUMBRA_LIGHT_PARAMETER, penumbraAngle);
	GetDocument()->WriteAnimatedValueToXML(&penumbraAngle, penumbraAngleNode, "penumbra_angle");
	xmlNode* aspectRatioNode = AddChild(techniqueMaxNode, DAEMAX_ASPECTRATIO_LIGHT_PARAMETER, aspectRatio);
	GetDocument()->WriteAnimatedValueToXML(&aspectRatio, aspectRatioNode, "aspect_ratio");
	xmlNode* dropoffNode = AddChild(techniqueMayaNode, DAEMAYA_DROPOFF_LIGHT_PARAMETER, dropoff);
	GetDocument()->WriteAnimatedValueToXML(&dropoff, dropoffNode, "dropoff");
	AddChild(techniqueMaxNode, DAEMAX_OVERSHOOT_LIGHT_PARAMETER, overshoots);

	if (GetTargetNode() != NULL) AddChild(techniqueMaxNode, DAEMAX_TARGET_LIGHT_PARAMETER, GetTargetNode()->GetDaeId());

	FCDEntity::WriteToExtraXML(lightNode);
	return lightNode;
}
