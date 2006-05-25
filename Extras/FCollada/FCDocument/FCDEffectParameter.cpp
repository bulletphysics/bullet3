/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDAnimated.h"
#include "FCDocument/FCDEffectPass.h"
#include "FCDocument/FCDEffectProfile.h"
#include "FCDocument/FCDEffectTechnique.h"
#include "FCDocument/FCDEffectParameter.h"
#include "FCDocument/FCDImage.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDEffectParameter::FCDEffectParameter(FCDocument* document) : FCDObject(document, "FCDEffectParameter")
{
	isGenerator = true;
	isFragment = false;
	reference = "";
	semantic = "";
	bindSymbol = "";
}

FCDEffectParameter::~FCDEffectParameter()
{
}

FUStatus FCDEffectParameter::LoadFromXML(xmlNode* parameterNode)
{
	FUStatus status;
	
	// This parameter is a generator if this is a <newparam> element. Otherwise, it modifies
	// an existing parameter (<bind>, <bind_semantic> or <setparam>.
	isGenerator = IsEquivalent(parameterNode->name, DAE_FXCMN_NEWPARAM_ELEMENT);
	if (isGenerator)
	{
		reference = ReadNodeProperty(parameterNode, DAE_SID_ATTRIBUTE);
		if (reference.empty())
		{
			return status.Warning(FS("No reference attribute on generator parameter."), parameterNode->line);
		}
	}
	else
	{
		reference = ReadNodeProperty(parameterNode, DAE_REF_ATTRIBUTE);
		if (reference.empty())
		{
			return status.Warning(FS("No reference attribute on modifier parameter."), parameterNode->line);
		}
	}
	
	xmlNode* valueNode = FindChildByType(parameterNode, DAE_FXCMN_SEMANTIC_ELEMENT);
	if (valueNode != NULL)
	{
		semantic = ReadNodeContentDirect(valueNode);
	}

	return status;
}

// Write out this ColladaFX parameter to the xml node tree
xmlNode* FCDEffectParameter::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* parameterNode;
	if (isGenerator)
	{
		parameterNode = AddChild(parentNode, DAE_FXCMN_NEWPARAM_ELEMENT);
		if (!reference.empty()) AddAttribute(parameterNode, DAE_SID_ATTRIBUTE, reference);
		if (!semantic.empty()) AddChild(parameterNode, DAE_FXCMN_SEMANTIC_ELEMENT, semantic);
	}
	else
	{
		parameterNode = AddChild(parentNode, DAE_FXCMN_SETPARAM_ELEMENT);
		if (!reference.empty()) AddAttribute(parameterNode, DAE_REF_ATTRIBUTE, reference);
	}
	return parameterNode;
}

void FCDEffectParameter::Clone(FCDEffectParameter* clone)
{
	clone->bindSymbol = bindSymbol;
	clone->reference = reference;
	clone->semantic = semantic;
	clone->isFragment = isFragment;
	clone->isGenerator = isGenerator;
}

// Flattening: overwrite the target parameter with this parameter
void FCDEffectParameter::Overwrite(FCDEffectParameter* UNUSED(target))
{
	// Do nothing on the base class, only values and animations should be overwritten
}

FCDEffectParameterSampler::FCDEffectParameterSampler(FCDocument* document) : FCDEffectParameter(document)
{
	samplerType = SAMPLER2D;
}

FCDEffectParameterSampler::~FCDEffectParameterSampler()
{
}

// Clone
FCDEffectParameter* FCDEffectParameterSampler::Clone()
{
	FCDEffectParameterSampler* clone = new FCDEffectParameterSampler(GetDocument());
	FCDEffectParameter::Clone(clone);
	clone->surfaceSid = surfaceSid;
	clone->samplerType = samplerType;
	return clone;
}

// Flattening: overwrite the target parameter with this parameter
void FCDEffectParameterSampler::Overwrite(FCDEffectParameter* target)
{
	if (target->GetType() == SAMPLER)
	{
		FCDEffectParameterSampler* s = (FCDEffectParameterSampler*) target;
		if (samplerType == s->samplerType)
		{
			s->surfaceSid = surfaceSid;
		}
	}
}

FUStatus FCDEffectParameterSampler::LoadFromXML(xmlNode* parameterNode)
{
	FUStatus status = FCDEffectParameter::LoadFromXML(parameterNode);

	// Find the sampler node
	xmlNode* samplerNode = NULL;
	for (xmlNode* child = parameterNode->children; child != NULL; child = child->next)
	{
		if (child->type != XML_ELEMENT_NODE) continue;

		if (IsEquivalent(child->name, DAE_FXCMN_SAMPLER1D_ELEMENT)) { samplerType = SAMPLER1D; samplerNode = child; break; }
		else if (IsEquivalent(child->name, DAE_FXCMN_SAMPLER2D_ELEMENT)) { samplerType = SAMPLER2D; samplerNode = child; break; }
		else if (IsEquivalent(child->name, DAE_FXCMN_SAMPLER3D_ELEMENT)) { samplerType = SAMPLER3D; samplerNode = child; break; }
		else if (IsEquivalent(child->name, DAE_FXCMN_SAMPLERCUBE_ELEMENT)) { samplerType = SAMPLERCUBE; samplerNode = child; break; }
	}

	if (samplerNode == NULL)
	{
		return status.Warning(FS("Unable to find sampler node for sampler parameter: ") + TO_FSTRING(GetReference()), parameterNode->line);
	}

	// Parse the source node
	xmlNode* sourceNode = FindChildByType(samplerNode, DAE_SOURCE_ELEMENT);
	surfaceSid = ReadNodeContentDirect(sourceNode);
	if (surfaceSid.empty())
	{
		return status.Fail(FS("Empty surface source value for sampler parameter: ") + TO_FSTRING(GetReference()), parameterNode->line);
	}
	
	return status;
}

xmlNode* FCDEffectParameterSampler::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* parameterNode = FCDEffectParameter::WriteToXML(parentNode);
	const char* samplerName;
	switch(samplerType)
	{
	case SAMPLER1D: samplerName = DAE_FXCMN_SAMPLER1D_ELEMENT; break;
	case SAMPLER2D: samplerName = DAE_FXCMN_SAMPLER2D_ELEMENT; break;
	case SAMPLER3D: samplerName = DAE_FXCMN_SAMPLER3D_ELEMENT; break;
	case SAMPLERCUBE: samplerName = DAE_FXCMN_SAMPLERCUBE_ELEMENT; break;
	default: samplerName = DAEERR_UNKNOWN_ELEMENT; break;
	}
	xmlNode* samplerNode = AddChild(parameterNode, samplerName);
	if (!surfaceSid.empty()) AddChild(samplerNode, DAE_SOURCE_ELEMENT, surfaceSid);
	return parameterNode;
}

FCDEffectParameterInt::FCDEffectParameterInt(FCDocument* document) : FCDEffectParameter(document) { value = 0; }
FCDEffectParameterInt::~FCDEffectParameterInt()
{
	value = 0;
}

// Clone
FCDEffectParameter* FCDEffectParameterInt::Clone()
{
	FCDEffectParameterInt* clone = new FCDEffectParameterInt(GetDocument());
	FCDEffectParameter::Clone(clone);
	clone->value = value;
	return clone;
}

// Flattening: overwrite the target parameter with this parameter
void FCDEffectParameterInt::Overwrite(FCDEffectParameter* target)
{
	if (target->GetType() == INTEGER)
	{
		FCDEffectParameterInt* s = (FCDEffectParameterInt*) target;
		s->value = value;
	}
}

// Parse in this ColladaFX integer from the document's XML node
FUStatus FCDEffectParameterInt::LoadFromXML(xmlNode* parameterNode)
{
	FUStatus status = FCDEffectParameter::LoadFromXML(parameterNode);
	xmlNode* valueNode = FindChildByType(parameterNode, DAE_FXCMN_INT_ELEMENT);
	const char* valueString = ReadNodeContentDirect(valueNode);
	if (valueString == NULL || *valueString == 0)
	{
		return status.Fail(FS("Bad value for float parameter in integer parameter: ") + TO_FSTRING(GetReference()), parameterNode->line);
	}
	value = FUStringConversion::ToInt32(valueString);
	return status;
}

xmlNode* FCDEffectParameterInt::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* parameterNode = FCDEffectParameter::WriteToXML(parentNode);
	AddChild(parameterNode, DAE_FXCMN_INT_ELEMENT, value);
	return parameterNode;
}

// boolean type parameter
FCDEffectParameterBool::FCDEffectParameterBool(FCDocument* document) : FCDEffectParameter(document)
{
	value = 0;
}

FCDEffectParameterBool::~FCDEffectParameterBool() {}

// Clone
FCDEffectParameter* FCDEffectParameterBool::Clone()
{
	FCDEffectParameterBool* clone = new FCDEffectParameterBool(GetDocument());
	FCDEffectParameter::Clone(clone);
	clone->value = value;
	return clone;
}

// Flattening: overwrite the target parameter with this parameter
void FCDEffectParameterBool::Overwrite(FCDEffectParameter* target)
{
	if (target->GetType() == BOOLEAN)
	{
		FCDEffectParameterBool* s = (FCDEffectParameterBool*) target;
		s->value = value;
	}
}

FUStatus FCDEffectParameterBool::LoadFromXML(xmlNode* parameterNode)
{
	FUStatus status = FCDEffectParameter::LoadFromXML(parameterNode);
	xmlNode* valueNode = FindChildByType(parameterNode, DAE_FXCMN_BOOL_ELEMENT);
	const char* valueString = ReadNodeContentDirect(valueNode);
	if (valueString == NULL || *valueString == 0)
	{
		return status.Fail(FS("Bad value for boolean parameter in effect: ") + TO_FSTRING(GetReference()), parameterNode->line);
	}
	value = FUStringConversion::ToBoolean(valueString);
	return status;
}

xmlNode* FCDEffectParameterBool::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* parameterNode = FCDEffectParameter::WriteToXML(parentNode);
	AddChild(parameterNode, DAE_FXCMN_BOOL_ELEMENT, value);
	return parameterNode;
}

// string type parameter
FCDEffectParameterString::FCDEffectParameterString(FCDocument* document) : FCDEffectParameter(document)
{
	value = "";
}

FCDEffectParameterString::~FCDEffectParameterString()
{
}

// Clone
FCDEffectParameter* FCDEffectParameterString::Clone()
{
	FCDEffectParameterString* clone = new FCDEffectParameterString(GetDocument());
	FCDEffectParameter::Clone(clone);
	clone->value = value;
	return clone;
}

// Flattening: overwrite the target parameter with this parameter
void FCDEffectParameterString::Overwrite(FCDEffectParameter* target)
{
	if (target->GetType() == STRING)
	{
		FCDEffectParameterString* s = (FCDEffectParameterString*) target;
		s->value = value;
	}
}

FUStatus FCDEffectParameterString::LoadFromXML(xmlNode* parameterNode)
{
	FUStatus status = FCDEffectParameter::LoadFromXML(parameterNode);
	xmlNode* valueNode = FindChildByType(parameterNode, DAE_FXCMN_STRING_ELEMENT);
	const char* valueString = ReadNodeContentDirect(valueNode);
	value = valueString;
	return status;
}

xmlNode* FCDEffectParameterString::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* parameterNode = FCDEffectParameter::WriteToXML(parentNode);
	AddChild(parameterNode, DAE_FXCMN_STRING_ELEMENT, value);
	return parameterNode;
}

// float type parameter
FCDEffectParameterFloat::FCDEffectParameterFloat(FCDocument* document) : FCDEffectParameter(document)
{
	floatType = FLOAT;
	value = 0.0f;
	min = 0.0f;
	max = 1.0f;
}

FCDEffectParameterFloat::~FCDEffectParameterFloat()
{
}

// Clone
FCDEffectParameter* FCDEffectParameterFloat::Clone()
{
	FCDEffectParameterFloat* clone = new FCDEffectParameterFloat(GetDocument());
	FCDEffectParameter::Clone(clone);
	clone->floatType = floatType;
	clone->value = value;
	clone->min = min;
	clone->max = max;
	return clone;
}

// Flattening: overwrite the target parameter with this parameter
void FCDEffectParameterFloat::Overwrite(FCDEffectParameter* target)
{
	if (target->GetType() == FCDEffectParameter::FLOAT)
	{
		FCDEffectParameterFloat* s = (FCDEffectParameterFloat*) target;
		if (s->floatType == floatType) s->value = value;
	}
}

FUStatus FCDEffectParameterFloat::LoadFromXML(xmlNode* parameterNode)
{
	FUStatus status = FCDEffectParameter::LoadFromXML(parameterNode);
	xmlNode* valueNode = FindChildByType(parameterNode, DAE_FXCMN_FLOAT_ELEMENT);
	if (valueNode == NULL)
	{
		valueNode = FindChildByType(parameterNode, DAE_FXCMN_HALF_ELEMENT);
		floatType = HALF;
	}
	else floatType = FLOAT;
		
	const char* valueString = ReadNodeContentDirect(valueNode);
	if (valueString == NULL || *valueString == 0)
	{
		return status.Fail(FS("Bad float value for float parameter: ") + TO_FSTRING(GetReference()), parameterNode->line);
	}
	value = FUStringConversion::ToFloat(valueString);
	
	FCDAnimatedFloat::Create(GetDocument(), parameterNode, &value);
	
	xmlNode* minannNode = FindChildByName(parameterNode, "UIMin");
	if(minannNode) {
		xmlNode* minNode = FindChildByType(minannNode, DAE_FXCMN_FLOAT_ELEMENT);
		valueString = ReadNodeContentDirect(minNode);
		if (valueString != NULL) min = FUStringConversion::ToFloat(valueString);
	}
	
	xmlNode* maxannNode = FindChildByName(parameterNode, "UIMax");
	if(maxannNode) {
		xmlNode* maxNode = FindChildByType(maxannNode, DAE_FXCMN_FLOAT_ELEMENT);
		valueString = ReadNodeContentDirect(maxNode);
		if (valueString != NULL) max = FUStringConversion::ToFloat(valueString);
	}
	
	return status;
}

xmlNode* FCDEffectParameterFloat::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* parameterNode = FCDEffectParameter::WriteToXML(parentNode);
	AddChild(parameterNode, (floatType == FLOAT) ? DAE_FXCMN_FLOAT_ELEMENT : DAE_FXCMN_HALF_ELEMENT, value);
	return parameterNode;
}

// float2 type parameter
FCDEffectParameterFloat2::FCDEffectParameterFloat2(FCDocument* document) : FCDEffectParameter(document)
{
	floatType = FLOAT;
	value_x = 0.0f;
	value_y = 0.0f;
}

FCDEffectParameterFloat2::~FCDEffectParameterFloat2()
{
}

// Clone
FCDEffectParameter* FCDEffectParameterFloat2::Clone()
{
	FCDEffectParameterFloat2* clone = new FCDEffectParameterFloat2(GetDocument());
	FCDEffectParameter::Clone(clone);
	clone->floatType = floatType;
	clone->value_x = value_x;
	clone->value_y = value_y;
	return clone;
}

// Flattening: overwrite the target parameter with this parameter
void FCDEffectParameterFloat2::Overwrite(FCDEffectParameter* target)
{
	if (target->GetType() == FLOAT2)
	{
		FCDEffectParameterFloat2* s = (FCDEffectParameterFloat2*) target;
		if (s->floatType == floatType)
		{
			s->value_x = value_x;
			s->value_y = value_y;
		}
	}
}

FUStatus FCDEffectParameterFloat2::LoadFromXML(xmlNode* parameterNode)
{
	FUStatus status = FCDEffectParameter::LoadFromXML(parameterNode);
	xmlNode* valueNode = FindChildByType(parameterNode, DAE_FXCMN_FLOAT2_ELEMENT);
	if (valueNode == NULL)
	{
		valueNode = FindChildByType(parameterNode, DAE_FXCMN_HALF2_ELEMENT);
		floatType = HALF;
	}
	else floatType = FLOAT;
		
	const char* valueString = ReadNodeContentDirect(valueNode);
	if (valueString == NULL || *valueString == 0)
	{
		return status.Fail(FS("Bad value for float2 parameter: ") + TO_FSTRING(GetReference()), parameterNode->line);
	}
	value_x = FUStringConversion::ToFloat(&valueString);
	value_y = FUStringConversion::ToFloat(&valueString);
	return status;
}

xmlNode* FCDEffectParameterFloat2::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* parameterNode = FCDEffectParameter::WriteToXML(parentNode);
	globalSBuilder.set(value_x); globalSBuilder.append(' '); globalSBuilder.append(value_y);
	AddChild(parameterNode, (floatType == FLOAT) ? DAE_FXCMN_FLOAT2_ELEMENT : DAE_FXCMN_HALF2_ELEMENT, globalSBuilder);
	return parameterNode;
}

// float3 type parameter
FCDEffectParameterFloat3::FCDEffectParameterFloat3(FCDocument* document) : FCDEffectParameter(document)
{
	floatType = FLOAT;
	value = FMVector3(0.0f, 0.0f, 0.0f);
}

FCDEffectParameterFloat3::~FCDEffectParameterFloat3()
{
}

// Clone
FCDEffectParameter* FCDEffectParameterFloat3::Clone()
{
	FCDEffectParameterFloat3* clone = new FCDEffectParameterFloat3(GetDocument());
	FCDEffectParameter::Clone(clone);
	clone->floatType = floatType;
	clone->value = value;
	return clone;
}

// Flattening: overwrite the target parameter with this parameter
void FCDEffectParameterFloat3::Overwrite(FCDEffectParameter* target)
{
	if (target->GetType() == FLOAT3)
	{
		FCDEffectParameterFloat3* s = (FCDEffectParameterFloat3*) target;
		if (s->floatType == floatType)
		{
			s->value = value;
		}
	}
}

FUStatus FCDEffectParameterFloat3::LoadFromXML(xmlNode* parameterNode)
{
	FUStatus status = FCDEffectParameter::LoadFromXML(parameterNode);
	xmlNode* valueNode = FindChildByType(parameterNode, DAE_FXCMN_FLOAT3_ELEMENT);
	if (valueNode == NULL)
	{
		valueNode = FindChildByType(parameterNode, DAE_FXCMN_HALF3_ELEMENT);
		floatType = HALF;
	}
	else floatType = FLOAT;
		
	const char* valueString = ReadNodeContentDirect(valueNode);
	if (valueString == NULL || *valueString == 0)
	{
		return status.Fail(FS("Bad value for float3 parameter: ") + TO_FSTRING(GetReference()), parameterNode->line);
	}
	value = FUStringConversion::ToPoint(valueString);
	FCDAnimatedColor::Create(GetDocument(), parameterNode, &value);
	
	return status;
}

xmlNode* FCDEffectParameterFloat3::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* parameterNode = FCDEffectParameter::WriteToXML(parentNode);
	string s = FUStringConversion::ToString(value);
	AddChild(parameterNode, (floatType == FLOAT) ? DAE_FXCMN_FLOAT3_ELEMENT : DAE_FXCMN_HALF3_ELEMENT, s);
	return parameterNode;
}

FCDEffectParameterVector::FCDEffectParameterVector(FCDocument* document) : FCDEffectParameter(document)
{
	floatType = FLOAT;
	vector[0] = vector[1] = vector[2] = vector[3] = 0.0f;
}

FCDEffectParameterVector::~FCDEffectParameterVector() {}

// Clone
FCDEffectParameter* FCDEffectParameterVector::Clone()
{
	FCDEffectParameterVector* clone = new FCDEffectParameterVector(GetDocument());
	FCDEffectParameter::Clone(clone);
	clone->floatType = floatType;
	memcpy(clone->vector, vector, sizeof(float) * 4);
	return clone;
}

// Flattening: overwrite the target parameter with this parameter
void FCDEffectParameterVector::Overwrite(FCDEffectParameter* target)
{
	if (target->GetType() == VECTOR)
	{
		FCDEffectParameterVector* s = (FCDEffectParameterVector*) target;
		if (s->floatType == floatType)
		{
			memcpy(s->vector, vector, sizeof(float) * 4);
		}
	}
}

FUStatus FCDEffectParameterVector::LoadFromXML(xmlNode* parameterNode)
{
	FUStatus status = FCDEffectParameter::LoadFromXML(parameterNode);
	xmlNode* valueNode = FindChildByType(parameterNode, DAE_FXCMN_FLOAT4_ELEMENT);
	if (valueNode == NULL)
	{
		valueNode = FindChildByType(parameterNode, DAE_FXCMN_HALF4_ELEMENT);
		floatType = HALF;
	}
	else floatType = FLOAT;
		
	const char* valueString = ReadNodeContentDirect(valueNode);
	if (valueString == NULL || *valueString == 0)
	{
		return status.Fail(FS("Bad value for float4 parameter: ") + TO_FSTRING(GetReference()), parameterNode->line);
	}
	vector[0] = FUStringConversion::ToFloat(&valueString);
	vector[1] = FUStringConversion::ToFloat(&valueString);
	vector[2] = FUStringConversion::ToFloat(&valueString);
	vector[3] = FUStringConversion::ToFloat(&valueString);

	return status;
}

xmlNode* FCDEffectParameterVector::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* parameterNode = FCDEffectParameter::WriteToXML(parentNode);
	globalSBuilder.set(vector[0]); globalSBuilder.append(' '); globalSBuilder.append(vector[1]); globalSBuilder.append(' ');
	globalSBuilder.append(vector[2]); globalSBuilder.append(' '); globalSBuilder.append(vector[3]);
	AddChild(parameterNode, (floatType == FLOAT) ? DAE_FXCMN_FLOAT4_ELEMENT : DAE_FXCMN_HALF4_ELEMENT, globalSBuilder);
	return parameterNode;
}

FCDEffectParameterMatrix::FCDEffectParameterMatrix(FCDocument* document) : FCDEffectParameter(document)
{
	floatType = FLOAT;
	matrix = FMMatrix44::Identity;
}

FCDEffectParameterMatrix::~FCDEffectParameterMatrix()
{
}

// Clone
FCDEffectParameter* FCDEffectParameterMatrix::Clone()
{
	FCDEffectParameterMatrix* clone = new FCDEffectParameterMatrix(GetDocument());
	FCDEffectParameter::Clone(clone);
	clone->floatType = floatType;
	clone->matrix = matrix;
	return clone;
}

// Flattening: overwrite the target parameter with this parameter
void FCDEffectParameterMatrix::Overwrite(FCDEffectParameter* target)
{
	if (target->GetType() == MATRIX)
	{
		FCDEffectParameterMatrix* s = (FCDEffectParameterMatrix*) target;
		s->matrix = matrix;
	}
}

FUStatus FCDEffectParameterMatrix::LoadFromXML(xmlNode* parameterNode)
{
	FUStatus status = FCDEffectParameter::LoadFromXML(parameterNode);
	xmlNode* valueNode = FindChildByType(parameterNode, DAE_FXCMN_FLOAT4X4_ELEMENT);
	if (valueNode == NULL)
	{
		valueNode = FindChildByType(parameterNode, DAE_FXCMN_HALF4X4_ELEMENT);
		floatType = HALF;
	}
	else floatType = FLOAT;
		
	const char* valueString = ReadNodeContentDirect(valueNode);
	if (valueString == NULL || *valueString == 0)
	{
		return status.Fail(FS("Bad value for matrix parameter: ") + TO_FSTRING(GetReference()), parameterNode->line);
	}
	FUStringConversion::ToMatrix(valueString, matrix, GetDocument()->GetLengthUnitConversion());
	return status;
}

xmlNode* FCDEffectParameterMatrix::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* parameterNode = FCDEffectParameter::WriteToXML(parentNode);
	string s = FUStringConversion::ToString(matrix); 
	AddChild(parameterNode, (floatType == FLOAT) ? DAE_FXCMN_FLOAT4X4_ELEMENT : DAE_FXCMN_HALF4X4_ELEMENT, s);
	return parameterNode;
}
