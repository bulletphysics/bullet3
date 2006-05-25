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
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeEnum.h"
#include "FUtils/FUStringConversion.h"
#include "FUtils/FUXmlNodeIdPair.h"

namespace FUDaeParser
{
	// Returns the first child node with a given id
	xmlNode* FindChildById(xmlNode* parent, const string& id)
	{
		if (parent != NULL && !id.empty())
		{
			const char* localId = id.c_str();
			if (localId[0] == '#') ++localId;
			for (xmlNode* child = parent->children; child != NULL; child = child->next)
			{
				if (child->type == XML_ELEMENT_NODE)
				{
					string nodeId = ReadNodeId(child);
					if (nodeId == localId) return child;
				}
			}
		}
		return NULL;
	}

	// Returns the first child node of a given "ref" property
	xmlNode* FindChildByRef(xmlNode* parent, const char* ref)
	{
		return FindChildByProperty(parent, DAE_REF_ATTRIBUTE, ref);
	}

	// Returns the first child with the given 'sid' value within a given xml hierarchy 
	xmlNode* FindHierarchyChildBySid(xmlNode* hierarchyRoot, const char* sid)
	{
		xmlNode* found = NULL;
		for (xmlNode* child = hierarchyRoot->children; child != NULL && found == NULL; child = child->next)
		{
			if (child->type != XML_ELEMENT_NODE) continue;
			if (ReadNodeProperty(child, DAE_SID_ATTRIBUTE) == sid) return child;
			found = FindHierarchyChildBySid(child, sid);
		}
		return found;
	}

	// Returns the first technique node with a given profile
	xmlNode* FindTechnique(xmlNode* parent, const char* profile)
	{
		if (parent != NULL)
		{
			if (IsEquivalent(profile, DAE_COMMON_PROFILE))
			{
				// COLLADA 1.4: Look for the <technique_common> element
				xmlNode* techniqueNode = FindChildByType(parent, DAE_TECHNIQUE_COMMON_ELEMENT);
				if (techniqueNode != NULL) return techniqueNode;
			}

			xmlNodeList techniqueNodes;
			FindChildrenByType(parent, DAE_TECHNIQUE_ELEMENT, techniqueNodes);
			size_t techniqueNodeCount = techniqueNodes.size();
			for (size_t i = 0; i < techniqueNodeCount; ++i)
			{
				xmlNode* techniqueNode = techniqueNodes[i];
				string techniqueProfile = ReadNodeProperty(techniqueNode, DAE_PROFILE_ATTRIBUTE);
				if (techniqueProfile == profile) return techniqueNode;
			}
		}
		return NULL;
	}

	// Returns the accessor node for a given source node
	xmlNode* FindTechniqueAccessor(xmlNode* parent)
	{
		xmlNode* techniqueNode = FindTechnique(parent, DAE_COMMON_PROFILE);
		return FindChildByType(techniqueNode, DAE_ACCESSOR_ELEMENT);
	}

	// Returns the array node from a given array type
	xmlNode* FindArray(xmlNode* parent, const char* arrayType)
	{
		xmlNode* stronglyTypedArrayNode = FindChildByType(parent, arrayType);
		if (stronglyTypedArrayNode != NULL) return stronglyTypedArrayNode;

		xmlNode* weaklyTypedArrayNode = FindChildByType(parent, DAE_ARRAY_ELEMENT);
		if (weaklyTypedArrayNode != NULL) return weaklyTypedArrayNode;

		return NULL;
	}

	// Returns a list of parameter names and nodes held by a given XML node
	// Useful for COLLADA 1.3 backward-compatibility: the names are taken from the <param> node's 'name' attribute
	void FindParameters(xmlNode* parent, StringList& parameterNames, xmlNodeList& parameterNodes)
	{
		if (parent == NULL || parameterNames.size() != parameterNodes.size()) return;

		size_t originalCount = parameterNodes.size();
		for (xmlNode* child = parent->children; child != NULL; child = child->next)
		{
			if (child->type != XML_ELEMENT_NODE) continue;

			// Drop the technique and exta elements that may be found
			if (IsEquivalent(child->name, DAE_TECHNIQUE_ELEMENT) ||
				IsEquivalent(child->name, DAE_EXTRA_ELEMENT)) continue;

			// Buffer this parameter node
			parameterNodes.push_back(child);
		}

		// Retrieve all the parameter's names
		size_t parameterNodeCount = parameterNodes.size();
		parameterNames.resize(parameterNodeCount);
		for (size_t i = originalCount; i < parameterNodeCount; ++i)
		{
			xmlNode* node = parameterNodes[i];
			if (IsEquivalent(node->name, DAE_PARAMETER_ELEMENT)) parameterNames[i] = ReadNodeName(node);
			else parameterNames[i] = (const char*) node->name;
		}
	}

	// Retrieves a list of floats from a source node
	// Returns the data's stride.
	uint32 ReadSource(xmlNode* sourceNode, FloatList& array)
	{
		uint32 stride = 0;
		if (sourceNode != NULL)
		{
			// Get the accessor's count
			xmlNode* accessorNode = FindTechniqueAccessor(sourceNode);
			stride = ReadNodeStride(accessorNode);
			array.resize(ReadNodeCount(accessorNode) * stride);

			xmlNode* arrayNode = FindArray(sourceNode, DAE_FLOAT_ARRAY_ELEMENT);
			const char* arrayContent = ReadNodeContentDirect(arrayNode);
			FUStringConversion::ToFloatList(arrayContent, array);
		}
		return stride;
	}

	// Retrieves a list of signed integers from a source node
	void ReadSource(xmlNode* sourceNode, Int32List& array)
	{
		if (sourceNode != NULL)
		{
			// Get the accessor's count
			xmlNode* accessorNode = FindTechniqueAccessor(sourceNode);
			array.resize(ReadNodeCount(accessorNode));

			xmlNode* arrayNode = FindArray(sourceNode, DAE_FLOAT_ARRAY_ELEMENT);
			const char* arrayContent = ReadNodeContentDirect(arrayNode);
			FUStringConversion::ToInt32List(arrayContent, array);
		}
	}

	// Retrieves a list of strings from a source node
	void ReadSource(xmlNode* sourceNode, StringList& array)
	{
		if (sourceNode != NULL)
		{
			// Get the accessor's count
			xmlNode* accessorNode = FindTechniqueAccessor(sourceNode);
			array.resize(ReadNodeCount(accessorNode));

			xmlNode* arrayNode = FindArray(sourceNode, DAE_NAME_ARRAY_ELEMENT);
			if (arrayNode == NULL) arrayNode = FindArray(sourceNode, DAE_IDREF_ARRAY_ELEMENT);
			const char* arrayContent = ReadNodeContentDirect(arrayNode);
			FUStringConversion::ToStringList(arrayContent, array);
		}
	}

	// Retrieves a list of points from a source node
	void ReadSource(xmlNode* sourceNode, FMVector3List& array, float lengthFactor)
	{
		if (sourceNode != NULL)
		{
			// Get the accessor's count
			xmlNode* accessorNode = FindTechniqueAccessor(sourceNode);
			array.resize(ReadNodeCount(accessorNode));

			xmlNode* arrayNode = FindArray(sourceNode, DAE_FLOAT_ARRAY_ELEMENT);
			const char* arrayContent = ReadNodeContentDirect(arrayNode);
			FUStringConversion::ToPointList(arrayContent, array, lengthFactor);
		}
	}

	// Retrieves a list of matrices from a source node
	void ReadSource(xmlNode* sourceNode, FMMatrix44List& array, float lengthFactor)
	{
		if (sourceNode != NULL)
		{
			// Get the accessor's count
			xmlNode* accessorNode = FindTechniqueAccessor(sourceNode);
			array.resize(ReadNodeCount(accessorNode));

			xmlNode* arrayNode = FindArray(sourceNode, DAE_FLOAT_ARRAY_ELEMENT);
			const char* arrayContent = ReadNodeContentDirect(arrayNode);
			FUStringConversion::ToMatrixList(arrayContent, array, lengthFactor);
		}
	}

	// Retrieves a series of interleaved floats from a source node
	void ReadSourceInterleaved(xmlNode* sourceNode, vector<FloatList*>& arrays)
	{
		if (sourceNode != NULL)
		{
			// Get the accessor's count
			xmlNode* accessorNode = FindTechniqueAccessor(sourceNode);
			uint32 count = ReadNodeCount(accessorNode);
			for (vector<FloatList*>::iterator it = arrays.begin(); it != arrays.end(); ++it)
			{
				(*it)->resize(count);
			}

			// Use the stride to pad the interleaved float lists or remove extra elements
			uint32 stride = ReadNodeStride(accessorNode);
			while (stride < arrays.size()) arrays.pop_back();
			while (stride > arrays.size()) arrays.push_back(NULL);

			// Read and parse the float array
   			xmlNode* arrayNode = FindArray(sourceNode, DAE_FLOAT_ARRAY_ELEMENT);
			const char* arrayContent = ReadNodeContentDirect(arrayNode);
			FUStringConversion::ToInterleavedFloatList(arrayContent, arrays);
		}
	}

	// Retrieves a series of interpolation values from a source node
	void ReadSourceInterpolation(xmlNode* sourceNode, UInt32List& array)
	{
		if (sourceNode != NULL)
		{
			// Get the accessor's count
			xmlNode* accessorNode = FindTechniqueAccessor(sourceNode);
			uint32 count = ReadNodeCount(accessorNode);
			array.resize(count);

			StringList stringArray(count);
			xmlNode* arrayNode = FindArray(sourceNode, DAE_NAME_ARRAY_ELEMENT);
			const char* arrayContent = ReadNodeContentDirect(arrayNode);
			FUStringConversion::ToStringList(arrayContent, stringArray);
			for (uint32 i = 0; i < count; ++i)
			{
				array[i] = (uint32) FUDaeInterpolation::FromString(stringArray[i]);
			}
		}
	}

	// Retrieves a series of interpolation values from a source node
	void ReadSourceInterpolationInterleaved(xmlNode* sourceNode, vector<UInt32List*>& arrays)
	{
		if (sourceNode != NULL)
		{
			// Get the accessor's count
			xmlNode* accessorNode = FindTechniqueAccessor(sourceNode);
			uint32 count = ReadNodeCount(accessorNode);
			for (vector<UInt32List*>::iterator it = arrays.begin(); it != arrays.end(); ++it)
			{
				(*it)->resize(count);
			}

			// Get the source node's stride from the accessor node
			uint32 stride = ReadNodeStride(accessorNode);
			while (stride < arrays.size()) arrays.pop_back();

			StringList stringArray(count * stride);
			xmlNode* arrayNode = FindArray(sourceNode, DAE_NAME_ARRAY_ELEMENT);
			const char* arrayContent = ReadNodeContentDirect(arrayNode);
			FUStringConversion::ToStringList(arrayContent, stringArray);
			for (uint32 i = 0; i < count; ++i)
			{
				for (size_t j = 0; j < arrays.size(); ++j)
				{
					arrays[j]->at(i) = (uint32) FUDaeInterpolation::FromString(stringArray[i * stride + j]);
				}
			}
		}
	}

	// Retrieves the target property of a targeting node, split into its pointer and its qualifier(s)
	void ReadNodeTargetProperty(xmlNode* targetingNode, string& pointer, string& qualifier)
	{
		string target = ReadNodeProperty(targetingNode, DAE_TARGET_ATTRIBUTE);
		SplitTarget(target, pointer, qualifier);
	}

	// Split the target string into its pointer and its qualifier(s)
	void SplitTarget(const string& target, string& pointer, string& qualifier)
	{
		uint32 splitIndex = (uint32) target.find_first_of("([.");
		if (splitIndex != string::npos)
		{
			pointer = target.substr(0, splitIndex);
			qualifier = target.substr(splitIndex);
		}
		else
		{
			pointer = target;
			qualifier.clear();
		}
	}

	// Calculate the target pointer for a targetable node
	void CalculateNodeTargetPointer(xmlNode* target, string& pointer)
	{
		if (target != NULL)
		{
			// The target node should have either a subid or an id
			if (HasNodeProperty(target, DAE_ID_ATTRIBUTE))
			{
				pointer = ReadNodeId(target);
				return;
			}
			else if (!HasNodeProperty(target, DAE_SID_ATTRIBUTE))
			{
				pointer.clear();
				return;
			}
	
			// Generate a list of parent nodes up to the first properly identified parent
			xmlNodeList traversal;
			traversal.push_back(target);
			xmlNode* current = target->parent;
			while (current != NULL)
			{
				traversal.push_back(current);
				if (HasNodeProperty(current, DAE_ID_ATTRIBUTE)) break;
				current = current->parent;
			}
	
			// The top parent should have the ID property
			FUSStringBuilder builder;
			intptr_t nodeCount = (intptr_t) traversal.size();
			builder.append(ReadNodeId(traversal[nodeCount - 1]));
			if (builder.empty()) { pointer.clear(); return; }
	
			// Build up the target string
			for (intptr_t i = nodeCount - 2; i >= 0; --i)
			{
				xmlNode* node = traversal[i];
				string subId = ReadNodeProperty(node, DAE_SID_ATTRIBUTE);
				if (!subId.empty())
				{
					builder.append('/');
					builder.append(subId);
				}
			}
	
			pointer = builder.ToString();
		}
		else pointer.clear();
	}

	// Parse the target elements of the matrix
	int32 ReadTargetMatrixElement(string& qualifier)
	{
		int32 returnValue = -1;
		const char* c = qualifier.c_str();
		while(*c == '(' || *c == '[')
		{
			const char* number = ++c;
			while (*c >= '0' && *c <= '9') ++c;
			if (*c == ')' || *c == ']')
			{
				returnValue = FUStringConversion::ToInt32(number);
				string temp = c + 1;
				qualifier = temp;
				break;
			}
		}
		return returnValue;
	}

	// Parse the Url attribute off a node
	FUUri ReadNodeUrl(xmlNode* node, const char* attribute)
	{
		FUUri out;
		string uriString = ReadNodeProperty(node, attribute);
		uint32 suffixStart = (uint32) uriString.find('#');
		if (suffixStart == string::npos) out.prefix = TO_FSTRING(uriString);
		else
		{
			out.prefix = TO_FSTRING(uriString.substr(0, suffixStart));
			out.suffix = uriString.substr(suffixStart + 1);
		}
		return out;
	}

	// Parse the count attribute off a node
	uint32 ReadNodeCount(xmlNode* node)
	{
		string countString = ReadNodeProperty(node, DAE_COUNT_ATTRIBUTE);
		return FUStringConversion::ToUInt32(countString);
	}

	// Parse the stride attribute off a node
	uint32 ReadNodeStride(xmlNode* node)
	{
		string strideString = ReadNodeProperty(node, DAE_STRIDE_ATTRIBUTE);
		uint32 stride = FUStringConversion::ToUInt32(strideString);
		if (stride == 0) stride = 1;
		return stride;
	}

	// Pre-buffer the children of a node, with their ids for performance optimization
	void ReadChildrenIds(xmlNode* node, FUXmlNodeIdPairList& pairs)
	{
		// To avoid unnecessary memory copies:
		// Start with calculating the maximum child count
		uint32 nodeCount = 0;
		for (xmlNode* child = node->children; child != NULL; child = child->next)
		{
			if (child->type == XML_ELEMENT_NODE) ++nodeCount;
		}

		// Now, buffer the child nodes and their ids
		pairs.reserve(nodeCount);
		for (xmlNode* child = node->children; child != NULL; child = child->next)
		{
			if (child->type == XML_ELEMENT_NODE)
			{
				FUXmlNodeIdPairList::iterator it = pairs.insert(pairs.end(), FUXmlNodeIdPair());
				(*it).node = child;
				(*it).id = ReadNodePropertyCRC(child, DAE_ID_ATTRIBUTE);
			}
		}
	}

	// Skip the pound(#) character from a COLLADA id string
	const char* SkipPound(const string& id)
	{
		const char* s = id.c_str();
		if (s == NULL) return NULL;
		else if (*s == '#') ++s;
		return s;
	}
}
