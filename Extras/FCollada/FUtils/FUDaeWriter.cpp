/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;

#define FLOAT_STR_ESTIMATE 12

namespace FUDaeWriter
{
	// List of common accessor types
	const char* FUDaeAccessor::XYZW[] = { "X", "Y", "Z", "W", 0 };
	const char* FUDaeAccessor::RGBA[] = { "R", "G", "B", "A", 0 };
	const char* FUDaeAccessor::STPQ[] = { "S", "T", "P", "Q", 0 };

	// Write out the <extra><technique> element unto the given parent xml tree node.
	// Check for only one <extra> element for this profile.
	xmlNode* AddExtraTechniqueChild(xmlNode* parent, const char* profile)
	{
		xmlNode* techniqueNode = NULL;
		if (parent != NULL)
		{
			xmlNode* extraNode = AddChildOnce(parent, DAE_EXTRA_ELEMENT);
			techniqueNode = AddTechniqueChild(extraNode, profile);
		}
		return techniqueNode;
	}

	// Write out the <technique> element unto the given parent xml tree node.
	// Check for only one <technique> element for this profile.
	xmlNode* AddTechniqueChild(xmlNode* parent, const char* profile)
	{
		xmlNode* techniqueNode = NULL;
		if (parent != NULL)
		{
			techniqueNode = FindTechnique(parent, profile);
			if (techniqueNode == NULL)
			{
				techniqueNode = AddChild(parent, DAE_TECHNIQUE_ELEMENT);
				AddAttribute(techniqueNode, DAE_PROFILE_ATTRIBUTE, profile);
			}
		}
		return techniqueNode;
	}

	xmlNode* AddParameter(xmlNode* parent, const char* name, const char* type)
	{
		xmlNode* parameterNode = AddChild(parent, DAE_PARAMETER_ELEMENT);
		if (name != NULL) AddAttribute(parameterNode, DAE_NAME_ATTRIBUTE, name);
		if (type == NULL) type = DAE_FLOAT_TYPE;
		AddAttribute(parameterNode, DAE_TYPE_ATTRIBUTE, type);
		return parameterNode;
	}

	xmlNode* AddInput(xmlNode* parent, const char* sourceId, const char* semantic, int32 offset, int32 set)
	{
		if (sourceId == NULL || *sourceId == 0 || semantic == NULL || *semantic == 0) return NULL;
		xmlNode* inputNode = AddChild(parent, DAE_INPUT_ELEMENT);
		AddAttribute(inputNode, DAE_SEMANTIC_ATTRIBUTE, semantic);
		AddAttribute(inputNode, DAE_SOURCE_ATTRIBUTE, string("#") + sourceId);
		if (offset >= 0) AddAttribute(inputNode, DAE_OFFSET_ATTRIBUTE, offset);
		if (set >= 0) AddAttribute(inputNode, DAE_SET_ATTRIBUTE, set);
		return inputNode;
	}

	xmlNode* AddArray(xmlNode* parent, const char* id, const char* arrayType, const char* content, size_t count)
	{
		xmlNode* arrayNode = AddChild(parent, arrayType, content);
		AddAttribute(arrayNode, DAE_ID_ATTRIBUTE, id);
		AddAttribute(arrayNode, DAE_COUNT_ATTRIBUTE, count);
		return arrayNode;
	}

	xmlNode* AddArray(xmlNode* parent, const char* id, const FMVector3List& values, float lengthFactor)
	{
		// Reserve the necessary space within the string builder
		globalSBuilder.clear();
		size_t valueCount = values.size();
		globalSBuilder.reserve(valueCount * 3 * FLOAT_STR_ESTIMATE);
		if (valueCount > 0)
		{
			// Write out the values
			FMVector3List::const_iterator itP = values.begin();
			FUStringConversion::ToString(globalSBuilder, *itP, lengthFactor);
			for (++itP; itP != values.end(); ++itP) { globalSBuilder.append(' '); FUStringConversion::ToString(globalSBuilder, *itP, lengthFactor); }
		}

		// Create the typed array node.
		return AddArray(parent, id, DAE_FLOAT_ARRAY_ELEMENT, globalSBuilder.ToCharPtr(), valueCount * 3);
	}

	xmlNode* AddArray(xmlNode* parent, const char* id, const FMMatrix44List& values, float lengthFactor)
	{
		globalSBuilder.clear();
		size_t valueCount = values.size();
		globalSBuilder.reserve(valueCount * 16 * FLOAT_STR_ESTIMATE);
		if (valueCount > 0)
		{
			FMMatrix44List::const_iterator itM = values.begin();
			FUStringConversion::ToString(globalSBuilder, *itM, lengthFactor);
			for (++itM; itM != values.end(); ++itM) { globalSBuilder.append(' '); FUStringConversion::ToString(globalSBuilder, *itM, lengthFactor); }
		}
		return AddArray(parent, id, DAE_FLOAT_ARRAY_ELEMENT, globalSBuilder.ToCharPtr(), valueCount * 16);
	}

	xmlNode* AddArray(xmlNode* parent, const char* id, const FloatList& values, float lengthFactor)
	{
		size_t valueCount = values.size();
		globalSBuilder.clear();
		globalSBuilder.reserve(valueCount * FLOAT_STR_ESTIMATE);
		FUStringConversion::ToString(globalSBuilder, values, lengthFactor);
		return AddArray(parent, id, DAE_FLOAT_ARRAY_ELEMENT, globalSBuilder.ToCharPtr(), valueCount);
	}

	xmlNode* AddArray(xmlNode* parent, const char* id, const StringList& values, const char* arrayType)
	{
		size_t valueCount = values.size();
		globalSBuilder.reserve(valueCount * 18); // Pulled out of a hat
		globalSBuilder.clear();
		if (valueCount > 0)
		{
			StringList::const_iterator itV = values.begin();
			globalSBuilder.set(*itV);
			for (++itV; itV != values.end(); ++itV) { globalSBuilder.append(' '); globalSBuilder.append(*itV); }
		}
		return AddArray(parent, id, arrayType, globalSBuilder.ToCharPtr(), valueCount);
	}

	xmlNode* AddAccessor(xmlNode* parent, const char* arrayId, size_t count, size_t stride, const char** parameters, const char* type)
	{
		// Create the accessor element and fill its basic properties
		xmlNode* accessorNode = AddChild(parent, DAE_ACCESSOR_ELEMENT);
		AddAttribute(accessorNode, DAE_SOURCE_ATTRIBUTE, string("#") + arrayId);
		AddAttribute(accessorNode, DAE_COUNT_ATTRIBUTE, count);
		AddAttribute(accessorNode, DAE_STRIDE_ATTRIBUTE, stride);

		// Create the stride parameters
		if (type == NULL) type = DAE_FLOAT_TYPE;
		if (!IsEquivalent(type, DAE_MATRIX_TYPE))
		{
			size_t p = 0;
			for (size_t i = 0; i < stride; ++i)
			{
				const char* parameter = NULL;
				if (parameters != NULL)
				{
					parameter = parameters[p++];
					if (parameter == NULL) { parameter = parameters[0]; p = 0; }
				}
				AddParameter(accessorNode, parameter, type);
			}
		}
		else
		{
			const char* parameter = (parameters != NULL) ? *parameters : NULL;
			AddParameter(accessorNode, parameter, type);
		}
		return accessorNode;
	}

	xmlNode* AddSourceMatrix(xmlNode* parent, const char* id, const FMMatrix44List& values, float lengthFactor)
	{
		xmlNode* sourceNode = AddChild(parent, DAE_SOURCE_ELEMENT);
		AddAttribute(sourceNode, DAE_ID_ATTRIBUTE, id);
		FUSStringBuilder arrayId(id); arrayId.append("-array");
		AddArray(sourceNode, arrayId.ToCharPtr(), values, lengthFactor);
		xmlNode* techniqueCommonNode = AddChild(sourceNode, DAE_TECHNIQUE_COMMON_ELEMENT);
		AddAccessor(techniqueCommonNode, arrayId.ToCharPtr(), values.size(), 16, NULL, DAE_MATRIX_TYPE);
		return sourceNode;
	}

	xmlNode* AddSourceColor(xmlNode* parent, const char* id, const FMVector3List& values)
	{
		xmlNode* sourceNode = AddChild(parent, DAE_SOURCE_ELEMENT);
		AddAttribute(sourceNode, DAE_ID_ATTRIBUTE, id);
		FUSStringBuilder arrayId(id); arrayId.append("-array");
		AddArray(sourceNode, arrayId.ToCharPtr(), values);
		xmlNode* techniqueCommonNode = AddChild(sourceNode, DAE_TECHNIQUE_COMMON_ELEMENT);
		AddAccessor(techniqueCommonNode, arrayId.ToCharPtr(), values.size(), 3, FUDaeAccessor::RGBA, DAE_FLOAT_TYPE);
		return sourceNode;
	}

	xmlNode* AddSourceTexcoord(xmlNode* parent, const char* id, const FMVector3List& values)
	{
		xmlNode* sourceNode = AddChild(parent, DAE_SOURCE_ELEMENT);
		AddAttribute(sourceNode, DAE_ID_ATTRIBUTE, id);
		FUSStringBuilder arrayId(id); arrayId.append("-array");
		AddArray(sourceNode, arrayId.ToCharPtr(), values);
		xmlNode* techniqueCommonNode = AddChild(sourceNode, DAE_TECHNIQUE_COMMON_ELEMENT);
		AddAccessor(techniqueCommonNode, arrayId.ToCharPtr(), values.size(), 3, FUDaeAccessor::STPQ, DAE_FLOAT_TYPE);
		return sourceNode;
	}

	xmlNode* AddSourcePosition(xmlNode* parent, const char* id, const FMVector3List& values, float lengthFactor)
	{
		xmlNode* sourceNode = AddChild(parent, DAE_SOURCE_ELEMENT);
		AddAttribute(sourceNode, DAE_ID_ATTRIBUTE, id);
		FUSStringBuilder arrayId(id); arrayId.append("-array");
		AddArray(sourceNode, arrayId.ToCharPtr(), values, lengthFactor);
		xmlNode* techniqueCommonNode = AddChild(sourceNode, DAE_TECHNIQUE_COMMON_ELEMENT);
		AddAccessor(techniqueCommonNode, arrayId.ToCharPtr(), values.size(), 3, FUDaeAccessor::XYZW, DAE_FLOAT_TYPE);
		return sourceNode;
	}

	xmlNode* AddSourceFloat(xmlNode* parent, const char* id, const FloatList& values, const char* parameter, float lengthFactor)
	{ return AddSourceFloat(parent, id, values, 1, &parameter, lengthFactor); }
	xmlNode* AddSourceFloat(xmlNode* parent, const char* id, const FloatList& values, size_t stride, const char** parameters, float lengthFactor)
	{
		xmlNode* sourceNode = AddChild(parent, DAE_SOURCE_ELEMENT);
		AddAttribute(sourceNode, DAE_ID_ATTRIBUTE, id);
		FUSStringBuilder arrayId(id); arrayId.append("-array");
		AddArray(sourceNode, arrayId.ToCharPtr(), values, lengthFactor);
		xmlNode* techniqueCommonNode = AddChild(sourceNode, DAE_TECHNIQUE_COMMON_ELEMENT);
		if (stride == 0) stride = 1;
		AddAccessor(techniqueCommonNode, arrayId.ToCharPtr(), values.size() / stride, stride, parameters, DAE_FLOAT_TYPE);
		return sourceNode;
	}

	xmlNode* AddSourceString(xmlNode* parent, const char* id, const StringList& values, const char* parameter)
	{
		xmlNode* sourceNode = AddChild(parent, DAE_SOURCE_ELEMENT);
		AddAttribute(sourceNode, DAE_ID_ATTRIBUTE, id);
		FUSStringBuilder arrayId(id); arrayId.append("-array");
		AddArray(sourceNode, arrayId.ToCharPtr(), values, DAE_NAME_ARRAY_ELEMENT);
		xmlNode* techniqueCommonNode = AddChild(sourceNode, DAE_TECHNIQUE_COMMON_ELEMENT);
		AddAccessor(techniqueCommonNode, arrayId.ToCharPtr(), values.size(), 1, &parameter, DAE_NAME_TYPE);
		return sourceNode;
	}
	
	xmlNode* AddSourceIDRef(xmlNode* parent, const char* id, const StringList& values, const char* parameter)
	{
		xmlNode* sourceNode = AddChild(parent, DAE_SOURCE_ELEMENT);
		AddAttribute(sourceNode, DAE_ID_ATTRIBUTE, id);
		FUSStringBuilder arrayId(id); arrayId.append("-array");
		AddArray(sourceNode, arrayId.ToCharPtr(), values, DAE_IDREF_ARRAY_ELEMENT);
		xmlNode* techniqueCommonNode = AddChild(sourceNode, DAE_TECHNIQUE_COMMON_ELEMENT);
		AddAccessor(techniqueCommonNode, arrayId.ToCharPtr(), values.size(), 1, &parameter, DAE_IDREF_TYPE);
		return sourceNode;
	}

	xmlNode* AddSourceInterpolation(xmlNode* parent, const char* id, const FUDaeInterpolationList& interpolations)
	{
		xmlNode* sourceNode = AddChild(parent, DAE_SOURCE_ELEMENT);
		AddAttribute(sourceNode, DAE_ID_ATTRIBUTE, id);
		FUSStringBuilder arrayId(id); arrayId.append("-array");

		globalSBuilder.clear();
		size_t valueCount = interpolations.size();
		if (valueCount > 0)
		{
			FUDaeInterpolationList::const_iterator itI = interpolations.begin();
			globalSBuilder.append(FUDaeInterpolation::ToString(*itI));
			for (++itI; itI != interpolations.end(); ++itI)
			{
				globalSBuilder.append(' '); globalSBuilder.append(FUDaeInterpolation::ToString(*itI));
			}
		}
		AddArray(sourceNode, arrayId.ToCharPtr(), DAE_NAME_ARRAY_ELEMENT, globalSBuilder.ToCharPtr(), valueCount);
		xmlNode* techniqueCommonNode = AddChild(sourceNode, DAE_TECHNIQUE_COMMON_ELEMENT);
		const char* parameter = "INTERPOLATION";
		AddAccessor(techniqueCommonNode, arrayId.ToCharPtr(), valueCount, 1, &parameter, DAE_NAME_TYPE);
		return sourceNode;
	}

	// Clean-up the id and names to match the schema definitions of 'IDref' and 'Name'.
	string CleanId(const char* c)
	{
		globalSBuilder.clear();
		if (*c != 0)
		{
			// First character: alphabetic or '_'.
			if ((*c >= 'a' && *c <= 'z') || (*c >= 'A' && *c <= 'Z') || *c == '_') globalSBuilder += *c;
			else globalSBuilder += '_';

			// Other characters: alphabetic, numeric or '_'.
			// Otherwise, use HTML extended character write-up: &#<num>;
			for (++c; *c != 0; ++c)
			{
				if ((*c >= 'a' && *c <= 'z') || (*c >= 'A' && *c <= 'Z') || (*c >= '0' && *c <= '9') || *c == '_' || *c == '-') globalSBuilder += *c;
				else globalSBuilder += '_';
			}
		}
		return globalSBuilder.ToString();
	}

	fstring CleanName(const fchar* c)
	{
		globalBuilder.clear();
		if (*c != 0)
		{
			// First character: alphabetic, '_' or ':'.
			if ((*c >= 'a' && *c <= 'z') || (*c >= 'A' && *c <= 'Z') || *c == '_' || *c == ':') globalBuilder += *c;
			else globalBuilder += '_';

			// Other characters: alphabetic, numeric, '_', ':', '-' or '.'.
			// Otherwise, use HTML extended character write-up: &#<num>;
			for (++c; *c != 0; ++c)
			{
				if ((*c >= 'a' && *c <= 'z') || (*c >= 'A' && *c <= 'Z') || (*c >= '0' && *c <= '9') || *c == '_' || *c == ':' || *c == '-' || *c == '.') globalBuilder += *c;
				else { globalBuilder += '%'; globalBuilder += (uint32) *c; }
			}
		}
		return globalBuilder.ToString();
	}

	// Add an 'sid' attribute to the given xml node, ensuring unicity. Returns the final 'sid' value.
	FCOLLADA_EXPORT string AddNodeSid(xmlNode* node, const char* wantedSid)
	{
		// Find the first parent node with an id or sid. If this node has an id, return right away.
		xmlNode* parentNode = node;
		for (parentNode = node; parentNode != NULL; parentNode = parentNode->parent)
		{
			if (HasNodeProperty(parentNode, DAE_ID_ATTRIBUTE) || HasNodeProperty(parentNode, DAE_SID_ATTRIBUTE)) break;
		}
		if (parentNode == node)
		{
			if (!HasNodeProperty(parentNode, DAE_SID_ATTRIBUTE)) AddAttribute(node, DAE_SID_ATTRIBUTE, wantedSid);
			return wantedSid;
		}
		if (parentNode == NULL)
		{
			// Retrieve the last parent node available
			for (parentNode = node; parentNode->parent != NULL; parentNode = parentNode->parent) {}
		}

		// Check the wanted sid for unicity
		xmlNode* existingNode = FindHierarchyChildBySid(parentNode, wantedSid);
		if (existingNode == NULL)
		{
			AddAttribute(node, DAE_SID_ATTRIBUTE, wantedSid);
			return wantedSid;
		}

		// Generate new sids with an incremental counter.
		for (uint32 counter = 2; counter < 100; ++counter)
		{
			globalSBuilder.set(wantedSid); globalSBuilder.append(counter);
			existingNode = FindHierarchyChildBySid(parentNode, globalSBuilder.ToCharPtr());
			if (existingNode == NULL)
			{
				AddAttribute(node, DAE_SID_ATTRIBUTE, globalSBuilder);
				return globalSBuilder.ToString();
			}
		}
		return string("");
	}
};
