/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDEffectParameter.h"
#include "FCDocument/FCDEffectParameterFactory.h"
#include "FCDocument/FCDEffectParameterSurface.h"
#include "FUtils/FUDaeParser.h"

// Creates a new effect parameter, given a type.
FCDEffectParameter* FCDEffectParameterFactory::Create(FCDocument* document, uint32 type)
{
	FCDEffectParameter* parameter = NULL;

	switch (type)
	{
	case FCDEffectParameter::SAMPLER: parameter = new FCDEffectParameterSampler(document); break;
	case FCDEffectParameter::INTEGER: parameter = new FCDEffectParameterInt(document); break;
	case FCDEffectParameter::BOOLEAN: parameter = new FCDEffectParameterBool(document); break;
	case FCDEffectParameter::FLOAT: parameter = new FCDEffectParameterFloat(document); break;
	case FCDEffectParameter::FLOAT2: parameter = new FCDEffectParameterFloat2(document); break;
	case FCDEffectParameter::FLOAT3: parameter = new FCDEffectParameterFloat3(document); break;
	case FCDEffectParameter::VECTOR: parameter = new FCDEffectParameterVector(document); break;
	case FCDEffectParameter::MATRIX: parameter = new FCDEffectParameterMatrix(document); break;
	case FCDEffectParameter::STRING: parameter = new FCDEffectParameterString(document); break;
	case FCDEffectParameter::SURFACE: parameter = new FCDEffectParameterSurface(document); break;
	default: break;
	}

	return parameter;
}

// Generates the effect parameter object for the given XML node tree
FCDEffectParameter* FCDEffectParameterFactory::LoadFromXML(FCDocument* document, xmlNode* parameterNode, FUStatus* status)
{
	// Look for the type of the parameter.
	FCDEffectParameter* parameter = NULL;
	for (xmlNode* child = parameterNode->children; child != NULL && parameter == NULL; child = child->next)
	{
		if (child->type != XML_ELEMENT_NODE) continue;

		if (IsEquivalent(child->name, DAE_FXCMN_BOOL_ELEMENT)) parameter = new FCDEffectParameterBool(document);
		else if (IsEquivalent(child->name, DAE_FXCMN_FLOAT_ELEMENT)) parameter = new FCDEffectParameterFloat(document);
		else if (IsEquivalent(child->name, DAE_FXCMN_FLOAT2_ELEMENT)) parameter = new FCDEffectParameterFloat2(document);
		else if (IsEquivalent(child->name, DAE_FXCMN_FLOAT3_ELEMENT)) parameter = new FCDEffectParameterFloat3(document);
		else if (IsEquivalent(child->name, DAE_FXCMN_FLOAT4_ELEMENT)) parameter = new FCDEffectParameterVector(document);
		else if (IsEquivalent(child->name, DAE_FXCMN_FLOAT4X4_ELEMENT)) parameter = new FCDEffectParameterMatrix(document);
		else if (IsEquivalent(child->name, DAE_FXCMN_HALF_ELEMENT)) parameter = new FCDEffectParameterFloat(document);
		else if (IsEquivalent(child->name, DAE_FXCMN_HALF2_ELEMENT)) parameter = new FCDEffectParameterFloat2(document);
		else if (IsEquivalent(child->name, DAE_FXCMN_HALF3_ELEMENT)) parameter = new FCDEffectParameterFloat3(document);
		else if (IsEquivalent(child->name, DAE_FXCMN_HALF4_ELEMENT)) parameter = new FCDEffectParameterVector(document);
		else if (IsEquivalent(child->name, DAE_FXCMN_HALF4X4_ELEMENT)) parameter = new FCDEffectParameterMatrix(document);
		else if (IsEquivalent(child->name, DAE_FXCMN_INT_ELEMENT)) parameter = new FCDEffectParameterInt(document);
		else if (IsEquivalent(child->name, DAE_FXCMN_SAMPLER1D_ELEMENT)) parameter = new FCDEffectParameterSampler(document);
		else if (IsEquivalent(child->name, DAE_FXCMN_SAMPLER2D_ELEMENT)) parameter = new FCDEffectParameterSampler(document);
		else if (IsEquivalent(child->name, DAE_FXCMN_SAMPLER3D_ELEMENT)) parameter = new FCDEffectParameterSampler(document);
		else if (IsEquivalent(child->name, DAE_FXCMN_SAMPLERCUBE_ELEMENT)) parameter = new FCDEffectParameterSampler(document);
		else if (IsEquivalent(child->name, DAE_FXCMN_SURFACE_ELEMENT)) parameter = new FCDEffectParameterSurface(document);
		else if (IsEquivalent(child->name, DAE_FXCMN_STRING_ELEMENT)) parameter = new FCDEffectParameterString(document);
	}

	if (parameter != NULL)
	{
		FUStatus s = parameter->LoadFromXML(parameterNode);
		if (status != NULL) status->AppendStatus(s);
	}
	return parameter;
}
