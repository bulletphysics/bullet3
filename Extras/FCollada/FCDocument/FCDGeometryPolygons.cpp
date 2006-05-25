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
#include "FCDocument/FCDGeometryMesh.h"
#include "FCDocument/FCDGeometryPolygons.h"
#include "FCDocument/FCDGeometrySource.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
#include "FUtils/FUStringConversion.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

typedef vector<UInt32List> UniqueVerticesTable;

FCDGeometryPolygons::FCDGeometryPolygons(FCDocument* document, FCDGeometryMesh* _parent) : FCDObject(document, "FCDGeometryPolygons")
{
	parent = _parent;
	faceVertexCount = 0;
	faceOffset = faceVertexOffset = 0;
}

FCDGeometryPolygons::~FCDGeometryPolygons()
{
	CLEAR_POINTER_VECTOR(inputs);
	idxOwners.clear();
	holeFaces.clear();
	parent = NULL;
}


// Creates a new face.
void FCDGeometryPolygons::AddFace(uint32 degree)
{
	faceVertexCounts.push_back(degree);

	// Inserts empty indices
	for (FCDGeometryPolygonsInputList::iterator it = idxOwners.begin(); it != idxOwners.end(); ++it)
	{
		FCDGeometryPolygonsInput* input = (*it);
		input->indices.resize(input->indices.size() + degree, 0);
	}

	parent->Recalculate();
}

// Removes a face
void FCDGeometryPolygons::RemoveFace(size_t index)
{
	FUAssert(index < GetFaceCount(), return);

	// Remove the associated indices, if they exist.
	size_t offset = GetFaceVertexOffset(index);
	size_t indexCount = GetFaceVertexCount(index);
	for (FCDGeometryPolygonsInputList::iterator it = idxOwners.begin(); it != idxOwners.end(); ++it)
	{
		FCDGeometryPolygonsInput* input = (*it);
		size_t inputIndexCount = input->indices.size();
		if (offset < inputIndexCount)
		{
			UInt32List::iterator end, begin = input->indices.begin() + offset;
			if (offset + indexCount < inputIndexCount) end = begin + indexCount;
			else end = input->indices.end();
			input->indices.erase(begin, end);
		}
	}

	// Remove the face and its holes
	size_t holeBefore = GetHoleCountBefore(index);
	UInt32List::iterator itFV = faceVertexCounts.begin() + index + holeBefore;
	size_t holeCount = GetHoleCount(index);
	faceVertexCounts.erase(itFV, itFV + holeCount + 1); // +1 in order to remove the polygon as well as the holes.

	parent->Recalculate();
}

// Calculates the offset of face-vertex pairs before the given face index within the polygon set.
size_t FCDGeometryPolygons::GetFaceVertexOffset(size_t index) const
{
	size_t offset = 0;

	// We'll need to skip over the holes
	size_t holeCount = GetHoleCountBefore(index);
	if (index + holeCount < faceVertexCounts.size())
	{
		// Sum up the wanted offset
		UInt32List::const_iterator end = faceVertexCounts.begin() + index + holeCount;
		for (UInt32List::const_iterator it = faceVertexCounts.begin(); it != end; ++it)
		{
			offset += (*it);
		}
	}
	return offset;
}

// Calculates the number of holes within the polygon set that appear before the given face index.
size_t FCDGeometryPolygons::GetHoleCountBefore(size_t index) const
{
	size_t holeCount = 0;
	for (UInt32List::const_iterator it = holeFaces.begin(); it != holeFaces.end(); ++it)
	{
		if ((*it) < index) ++holeCount;
	}
	return holeCount;
}

// Retrieves the number of holes within a given face.
size_t FCDGeometryPolygons::GetHoleCount(size_t index) const
{
	size_t holeCount = 0;
	for (size_t i = GetFaceVertexOffset(index) + 1; i < faceVertexCounts.size(); ++i)
	{
		bool isHoled = std::find(holeFaces.begin(), holeFaces.end(), i) != holeFaces.end();
		if (!isHoled) break;
		else ++holeCount;
	}
	return holeCount;
}

// The number of face-vertex pairs for a given face.
size_t FCDGeometryPolygons::GetFaceVertexCount(size_t index) const
{
	size_t count = 0;
	if (index < GetFaceCount())
	{
		size_t holeCount = GetHoleCount(index);
		UInt32List::const_iterator it = faceVertexCounts.begin() + index + GetHoleCountBefore(index);
		UInt32List::const_iterator end = it + holeCount + 1; // +1 in order to sum the face-vertex pairs of the polygon as its holes.
		for (; it != end; ++it) count += (*it);
	}
	return count;
}

FCDGeometryPolygonsInput* FCDGeometryPolygons::AddInput(FCDGeometrySource* source, uint32 offset)
{
	FCDGeometryPolygonsInput* input = new FCDGeometryPolygonsInput();
	input->source = source;
	input->idx = offset;
	input->semantic = source->GetSourceType();

	// Check if the offset is new
	input->ownsIdx = true;
	for (FCDGeometryPolygonsInputList::iterator it = inputs.begin(); it != inputs.end(); ++it)
	{
		if ((*it)->idx == input->idx)
		{
			input->ownsIdx = false;
			break;
		}
	}

	inputs.push_back(input);
	if (input->ownsIdx) idxOwners.push_back(input);
	return input;
}

void FCDGeometryPolygons::ReleaseInput(FCDGeometryPolygonsInput* input)
{
	FCDGeometryPolygonsInputList::iterator itP = std::find(inputs.begin(), inputs.end(), input);
	if (itP != inputs.end())
	{
		// Before releasing the polygon set input, verify that shared indices are not lost
		if (input->ownsIdx)
		{
			for (FCDGeometryPolygonsInputList::iterator it = inputs.begin(); it != inputs.end(); ++it)
			{
				if ((*it)->idx == input->idx && !(*it)->ownsIdx)
				{
					(*it)->indices = input->indices;
					(*it)->ownsIdx = true;
					idxOwners.push_back(*it);
					break;
				}
			}

			FCDGeometryPolygonsInputList::iterator itO = std::find(idxOwners.begin(), idxOwners.end(), input);
			if (itO != idxOwners.end()) idxOwners.erase(itO);
			input->indices.clear();
			input->ownsIdx = false;
		}

		// Release the polygon set input
		SAFE_DELETE(input);
		inputs.erase(itP);
	}
}

FCDGeometryPolygonsInput* FCDGeometryPolygons::FindInput(FUDaeGeometryInput::Semantic semantic)
{
	for (FCDGeometryPolygonsInputList::iterator it = inputs.begin(); it != inputs.end(); ++it)
	{
		if ((*it)->semantic == semantic) return (*it);
	}
	return NULL;
}

const FCDGeometryPolygonsInput* FCDGeometryPolygons::FindInput(FUDaeGeometryInput::Semantic semantic) const
{
	for (FCDGeometryPolygonsInputList::const_iterator it = inputs.begin(); it != inputs.end(); ++it)
	{
		if ((*it)->semantic == semantic) return (*it);
	}
	return NULL;
}

FCDGeometryPolygonsInput* FCDGeometryPolygons::FindInput(FCDGeometrySource* source)
{
	for (FCDGeometryPolygonsInputList::iterator it = inputs.begin(); it != inputs.end(); ++it)
	{
		if ((*it)->source == source) return (*it);
	}
	return NULL;
}

const FCDGeometryPolygonsInput* FCDGeometryPolygons::FindInput(const FCDGeometrySource* source) const
{
	for (FCDGeometryPolygonsInputList::const_iterator it = inputs.begin(); it != inputs.end(); ++it)
	{
		if ((*it)->source == source) return (*it);
	}
	return NULL;
}

FCDGeometryPolygonsInput* FCDGeometryPolygons::FindInput(const string& sourceId)
{
	const char* s = sourceId.c_str();
	if (*s == '#') ++s;
	for (FCDGeometryPolygonsInputList::iterator it = inputs.begin(); it != inputs.end(); ++it)
	{
		if ((*it)->source->GetDaeId() == s) return (*it);
	}
	return NULL;
}

void FCDGeometryPolygons::FindInputs(FUDaeGeometryInput::Semantic semantic, FCDGeometryPolygonsInputList& _inputs)
{
	for (FCDGeometryPolygonsInputList::iterator it = inputs.begin(); it != inputs.end(); ++it)
	{
		if ((*it)->semantic == semantic) _inputs.push_back(*it);
	}
}

UInt32List* FCDGeometryPolygons::FindIndicesForIdx(uint32 idx)
{
	for (FCDGeometryPolygonsInputList::iterator it = idxOwners.begin(); it != idxOwners.end(); ++it)
	{
		if ((*it)->idx == idx) return &(*it)->indices;
	}
	return NULL;
}

const UInt32List* FCDGeometryPolygons::FindIndicesForIdx(uint32 idx) const
{
	for (FCDGeometryPolygonsInputList::const_iterator cit = idxOwners.begin(); cit != idxOwners.end(); ++cit)
	{
		if ((*cit)->idx == idx) return &(*cit)->indices;
	}
	return NULL;
}

UInt32List* FCDGeometryPolygons::FindIndices(FCDGeometrySource* source)
{
	FCDGeometryPolygonsInput* input = FindInput(source);
	return (input != NULL) ? FindIndices(input) : NULL;
}

const UInt32List* FCDGeometryPolygons::FindIndices(const FCDGeometrySource* source) const
{
	const FCDGeometryPolygonsInput* input = FindInput(source);
	return (input != NULL) ? FindIndices(input) : NULL;
}

UInt32List* FCDGeometryPolygons::FindIndices(FCDGeometryPolygonsInput* input)
{
	FCDGeometryPolygonsInputList::iterator itP = std::find(inputs.begin(), inputs.end(), input);
	return itP != inputs.end() ? FindIndicesForIdx(input->idx) : NULL;
}

const UInt32List* FCDGeometryPolygons::FindIndices(const FCDGeometryPolygonsInput* input) const
{
	FCDGeometryPolygonsInputList::const_iterator itP = std::find(inputs.begin(), inputs.end(), input);
	return itP != inputs.end() ? FindIndicesForIdx(input->idx) : NULL;
}

// Forces the triangulation of the polygons
void FCDGeometryPolygons::Triangulate()
{
	// Pre-allocate and ready the end index/count buffers
	UInt32List oldFaceVertexCounts = faceVertexCounts;
	faceVertexCounts.clear();
	vector<UInt32List*> dataIndices;
	vector<UInt32List> oldDataIndices;
	for (FCDGeometryPolygonsInputList::iterator itI = idxOwners.begin(); itI != idxOwners.end(); ++itI)
	{
		UInt32List* indices = &(*itI)->indices;
		oldDataIndices.push_back(*indices);
		dataIndices.push_back(indices);
		indices->clear();
	}
	size_t dataIndicesCount = oldDataIndices.size();

	// Rebuild the index/count buffers through simple fan-triangulation.
	// Drop holes and polygons with less than two vertices. 
	size_t oldOffset = 0, oldFaceCount = oldFaceVertexCounts.size();
	for (size_t oldFaceIndex = 0; oldFaceIndex < oldFaceCount; ++oldFaceIndex)
	{
		size_t oldFaceVertexCount = oldFaceVertexCounts[oldFaceIndex];
		bool isHole = std::find(holeFaces.begin(), holeFaces.end(), oldFaceIndex) != holeFaces.end();
		if (!isHole && oldFaceVertexCount >= 3)
		{
			// Fan-triangulation: works well on convex polygons.
			size_t triangleCount = oldFaceVertexCount - 2;
			for (size_t triangleIndex = 0; triangleIndex < triangleCount; ++triangleIndex)
			{
				for (size_t j = 0; j < dataIndicesCount; ++j)
				{
					UInt32List& oldData = oldDataIndices[j];
					UInt32List* newData = dataIndices[j];
					newData->push_back(oldData[oldOffset]);
					newData->push_back(oldData[oldOffset + triangleIndex + 1]);
					newData->push_back(oldData[oldOffset + triangleIndex + 2]);
				}
				faceVertexCounts.push_back(3);
			}
		}
		oldOffset += oldFaceVertexCount;
	}

	holeFaces.clear();
}

// Recalculates the face-vertex count within the polygons
void FCDGeometryPolygons::Recalculate()
{
	faceVertexCount = 0;
	for (UInt32List::iterator itC = faceVertexCounts.begin(); itC != faceVertexCounts.end(); ++itC) faceVertexCount += (*itC);
}

FUStatus FCDGeometryPolygons::LoadFromXML(xmlNode* baseNode)
{
	FUStatus status;

	// Retrieve the expected face count from the base node's 'count' attribute
	size_t expectedFaceCount = ReadNodeCount(baseNode);

	// Check the node's name to know whether to expect a <vcount> element
	size_t expectedVertexCount; bool isPolygons = false, isTriangles = false, isPolylist = false;
	if (IsEquivalent(baseNode->name, DAE_POLYGONS_ELEMENT)) { expectedVertexCount = 4; isPolygons = true; }
	else if (IsEquivalent(baseNode->name, DAE_TRIANGLES_ELEMENT)) { expectedVertexCount = 3 * expectedFaceCount; isTriangles = true; }
	else if (IsEquivalent(baseNode->name, DAE_POLYLIST_ELEMENT)) { expectedVertexCount = 0; isPolylist = true; }
	else
	{
		return status.Fail(FS("Unknown polygons element in geometry: ") + TO_FSTRING(parent->GetDaeId()), baseNode->line);
	}

	// Retrieve the material symbol used by these polygons
	materialSemantic = TO_FSTRING(ReadNodeProperty(baseNode, DAE_MATERIAL_ATTRIBUTE));
	if (materialSemantic.empty())
	{
		status.Warning(FS("Unknown or missing polygonal material symbol in geometry: ") + TO_FSTRING(parent->GetDaeId()), baseNode->line);
	}

	// Read in the per-face, per-vertex inputs
	uint32 idxCount = 1, tableIdx = 0;
	xmlNode* itNode = NULL;
	for (itNode = baseNode->children; itNode != NULL; itNode = itNode->next)
	{
		if (itNode->type != XML_ELEMENT_NODE) continue;
		if (IsEquivalent(itNode->name, DAE_INPUT_ELEMENT))
		{
			FCDGeometryPolygonsInput* input = new FCDGeometryPolygonsInput();
			string sourceId = ReadNodeSource(itNode);
			if (sourceId[0] == '#') sourceId.erase(0, 1);

			// Parse input idx
			string idx = ReadNodeProperty(itNode, DAE_OFFSET_ATTRIBUTE);
			if (idx.empty()) idx = ReadNodeProperty(itNode, DAE_IDX_ATTRIBUTE); // COLLADA 1.3 Backward-compatibility
			input->idx = (!idx.empty()) ? FUStringConversion::ToUInt32(idx) : idxCount;
			idxCount = max(input->idx + 1, idxCount);

			// Parse input set
			string setString = ReadNodeProperty(itNode, DAE_SET_ATTRIBUTE);
			input->set = setString.empty() ? -1 : FUStringConversion::ToInt32(setString);

			// Parse input semantic
			string semanticString = ReadNodeSemantic(itNode);
			input->semantic = FUDaeGeometryInput::FromString(semanticString);
			if (input->semantic == FUDaeGeometryInput::UNKNOWN)
			{
				// Unknown input type
				SAFE_DELETE(input);
				continue;
			}
			else if (input->semantic == FUDaeGeometryInput::VERTEX)
			{
				tableIdx = input->idx;
			}
			else
			{
				// Retrieve the source for this input
				input->source = parent->FindSourceById(sourceId);
				if (input->source == NULL)
				{
					status.Warning(FS("Unknown polygons set input with id: '") + TO_FSTRING(sourceId) + FS("' in geometry: ") + TO_FSTRING(parent->GetDaeId()), itNode->line);
					SAFE_DELETE(input);
					continue;
				}
			}

			// Check uniqueness of idx
			input->ownsIdx = true;
			for (FCDGeometryPolygonsInputList::iterator it = inputs.begin(); it != inputs.end(); ++it)
			{
				if ((*it)->idx == input->idx)
				{
					input->ownsIdx = false;
					break;
				}
			}

			// Add to our lists
			if (input->ownsIdx) idxOwners.push_back(input);
			inputs.push_back(input);
		}
		else if (IsEquivalent(itNode->name, DAE_POLYGON_ELEMENT)
			|| IsEquivalent(itNode->name, DAE_VERTEXCOUNT_ELEMENT)
			|| IsEquivalent(itNode->name, DAE_POLYGONHOLED_ELEMENT))
		{
			break;
		}

		// COLLADA 1.3 backward compatibility: <param> is a valid node, but unused
		else if (IsEquivalent(itNode->name, DAE_PARAMETER_ELEMENT)) {} 
		else
		{
			status.Warning(FS("Unknown polygon child element in geometry: ") + TO_FSTRING(parent->GetDaeId()), itNode->line);
		}
	}
	if (itNode == NULL)
	{
		return status.Fail(FS("No polygon <p>/<vcount> element found in geometry: ") + TO_FSTRING(parent->GetDaeId()), baseNode->line);
	}

	// Look for the <vcount> element and parse it in
	xmlNode* vCountNode = FindChildByType(baseNode, DAE_VERTEXCOUNT_ELEMENT);
	const char* vCountDataString = ReadNodeContentDirect(vCountNode);
	if (vCountDataString != NULL) FUStringConversion::ToUInt32List(vCountDataString, faceVertexCounts);
	bool hasVertexCounts = !faceVertexCounts.empty();
	if (isPolylist && !hasVertexCounts)
	{
		return status.Fail(FS("No or empty <vcount> element found in geometry: ") + TO_FSTRING(parent->GetDaeId()), baseNode->line);
	}
	else if (!isPolylist && hasVertexCounts)
	{
		return status.Fail(FS("<vcount> is only expected with the <polylist> element in geometry: ") + TO_FSTRING(parent->GetDaeId()), baseNode->line);
	}
	else if (isPolylist)
	{
		// Count the total number of face-vertices expected, to pre-buffer the index lists
		expectedVertexCount = 0;
		for (UInt32List::iterator itC = faceVertexCounts.begin(); itC != faceVertexCounts.end(); ++itC)
		{
			expectedVertexCount += *itC;
		}
	}

	string holeBuffer;
	UInt32List allIndices;
	// First pass, allocation for tessellation
	xmlNode* savedNode = itNode; // two pass process for performance
	uint32 indicesSize = 0;
	faceVertexCount = 0;
	allIndices.clear();
	allIndices.reserve(expectedVertexCount * idxCount);
	for (; itNode != NULL; itNode = itNode->next)
	{
		uint32 localFaceVertexCount;
		const char* content = NULL;
		xmlNode* holeNode = NULL;
		bool failed = false;
		if (!InitTessellation(itNode, &localFaceVertexCount, allIndices, content, holeNode, idxCount, &failed)) continue;

		if (failed)
		{
			return status.Fail(FS("Unknown element found in <ph> element for geometry: ") + TO_FSTRING(parent->GetDaeId()), itNode->line);
		}

		indicesSize += localFaceVertexCount;
	}
	for (FCDGeometryPolygonsInputList::iterator it = idxOwners.begin(); it != idxOwners.end(); ++it)
	{
		size_t currentIndexCount = (*it)->indices.size();
		(*it)->indices.reserve(currentIndexCount + indicesSize);
	}

	// Second pass, saving the tessellation
	faceVertexCount = 0;
	allIndices.clear();
	allIndices.reserve(expectedVertexCount * idxCount);
	itNode = savedNode;
	for (; itNode != NULL; itNode = itNode->next)
	{
		uint32 localFaceVertexCount;
		const char* content = NULL;
		xmlNode* holeNode = NULL;
		bool failed = false;
		if (!InitTessellation(itNode, &localFaceVertexCount, allIndices, content, holeNode, idxCount, &failed)) continue;

		if (failed)
		{
			return status.Fail(FS("Unknown element found in <ph> element for geometry: ") + TO_FSTRING(parent->GetDaeId()), itNode->line);
		}

		if (isTriangles) for (uint32 i = 0; i < localFaceVertexCount / 3; ++i) faceVertexCounts.push_back(3);
		else if (isPolygons) faceVertexCounts.push_back(localFaceVertexCount);
		faceVertexCount += localFaceVertexCount;

		// Append any hole indices found
		for (; holeNode != NULL; holeNode = holeNode->next)
		{
			if (holeNode->type != XML_ELEMENT_NODE) continue;

			// Read in the hole indices and push them on top of the other indices
			UInt32List holeIndices; holeIndices.reserve(expectedVertexCount * idxCount);
			content = ReadNodeContentDirect(holeNode);
			FUStringConversion::ToUInt32List(content, holeIndices);
			allIndices.insert(allIndices.end(), holeIndices.begin(), holeIndices.end());

			// Create the hole face and record its index
			size_t holeVertexCount = holeIndices.size() / idxCount;
			holeFaces.push_back((uint32) faceVertexCounts.size());
			faceVertexCounts.push_back((uint32) holeVertexCount);
			faceVertexCount += holeVertexCount;
		}

		// Create a new entry for the vertex buffer
		for (size_t offset = 0; offset < allIndices.size(); offset += idxCount)
		{
			for (FCDGeometryPolygonsInputList::iterator it = idxOwners.begin(); it != idxOwners.end(); ++it)
			{
				(*it)->indices.push_back(allIndices[offset + (*it)->idx]);
			}
		}
	}

	// Check the actual face count
	if (expectedFaceCount != faceVertexCounts.size() - holeFaces.size())
	{
		return status.Fail(FS("Face count for polygons node doesn't match actual number of faces found in <p> element(s) in geometry: ") + TO_FSTRING(parent->GetDaeId()), baseNode->line);
	}

	// Merge the vertex input with the vertices node information
	FCDGeometryPolygonsInputList::iterator itVertex;
	for (itVertex = inputs.begin(); itVertex != inputs.end(); ++itVertex)
	{
		if ((*itVertex)->semantic == FUDaeGeometryInput::VERTEX) break;
	}
	if (itVertex == inputs.end())
	{
		return status.Fail(FS("Cannot find VERTEX polygons' input within geometry: ") + TO_FSTRING(parent->GetDaeId()), baseNode->line);
	}

	FCDGeometrySourceList& vertexSources = parent->GetVertexSources();
	size_t vertexMergeCount = vertexSources.size();
	if (vertexMergeCount == 0)
	{
		return status.Fail(FS("Empty <vertices> element in geometry: ") + TO_FSTRING(parent->GetDaeId()), baseNode->line);
	}
	(*itVertex)->semantic = vertexSources.front()->GetSourceType();
	(*itVertex)->source = vertexSources.front();
	uint32 vertexIdx = (*itVertex)->idx;
	uint32 vertexPosition = (uint32) (itVertex - inputs.begin());
	for (uint32 i = 1; i < vertexMergeCount; ++i)
	{
		FCDGeometryPolygonsInput* perVertexInput = new FCDGeometryPolygonsInput();
		perVertexInput->source = vertexSources[i];
		perVertexInput->semantic = vertexSources[i]->GetSourceType();
		perVertexInput->ownsIdx = false;
		perVertexInput->idx = vertexIdx;
		perVertexInput->set = 0;
		inputs.insert(inputs.begin() + vertexPosition, perVertexInput);
	}

	// Read in the polygons source animations
	for (FCDGeometryPolygonsInputList::iterator it = inputs.begin(); it != inputs.end(); ++it)
	{
		(*it)->source->SetSourceType((*it)->semantic);
	}

	return status;
}

bool FCDGeometryPolygons::InitTessellation(xmlNode* itNode, 
		uint32* localFaceVertexCount, UInt32List& allIndices, 
		const char* content, xmlNode*& holeNode, uint32 idxCount, 
		bool* failed)
{
	if (itNode->type != XML_ELEMENT_NODE) return false;
	if (!IsEquivalent(itNode->name, DAE_POLYGON_ELEMENT) 
		&& !IsEquivalent(itNode->name, DAE_POLYGONHOLED_ELEMENT)) return false;

	// Retrieve the indices
	content = NULL;
	holeNode = NULL;
	if (!IsEquivalent(itNode->name, DAE_POLYGONHOLED_ELEMENT)) 
	{
		content = ReadNodeContentDirect(itNode);
	} 
	else 
	{
		// Holed face found
		for (xmlNode* child = itNode->children; child != NULL; child = child->next)
		{
			if (child->type != XML_ELEMENT_NODE) continue;
			if (IsEquivalent(child->name, DAE_POLYGON_ELEMENT)) 
			{
				content = ReadNodeContentDirect(child);
			}
			else if (IsEquivalent(child->name, DAE_HOLE_ELEMENT)) 
			{ 
				holeNode = child; break; 
			}
			else 
			{
				*failed = true;
				return true;
			}
		}
	}

	// Parse the indices
	allIndices.clear();
	FUStringConversion::ToUInt32List(content, allIndices);
	*localFaceVertexCount = (uint32) allIndices.size() / idxCount;
	return true;
}

// Write out the polygons structure to the COLLADA xml tree
xmlNode* FCDGeometryPolygons::WriteToXML(xmlNode* parentNode) const
{
	// Are there holes? Then, export a <polygons> element.
	// Are there only non-triangles within the list? Then, export a <polylist> element.
	// Otherwise, you only have triangles: export a <triangles> element.
	bool hasHoles = !holeFaces.empty(), hasNPolys = true;
	if (!hasHoles)
	{
		UInt32List::const_iterator itC;
		for (itC = faceVertexCounts.begin(); itC != faceVertexCounts.end() && (*itC) == 3; ++itC) {}
		hasNPolys = (itC != faceVertexCounts.end());
	}

	// Create the base node for these polygons
	const char* polygonNodeType;
	if (hasHoles) polygonNodeType = DAE_POLYGONS_ELEMENT;
	else if (hasNPolys) polygonNodeType = DAE_POLYLIST_ELEMENT;
	else polygonNodeType = DAE_TRIANGLES_ELEMENT;
	xmlNode* polygonsNode = AddChild(parentNode, polygonNodeType);

	// Add the inputs
	// Find which input owner belongs to the <vertices> element. Replace the semantic and the source id accordingly.
	// Make sure to add that 'vertex' input only once.
	FUSStringBuilder verticesNodeId(parent->GetDaeId()); verticesNodeId.append("-vertices");
	const FCDGeometrySourceList& vertexSources = parent->GetVertexSources();
	bool isVertexInputFound = false;
	for (FCDGeometryPolygonsInputList::const_iterator itI = inputs.begin(); itI != inputs.end(); ++itI)
	{
		FCDGeometryPolygonsInput* input = *itI;
		if (std::find(vertexSources.begin(), vertexSources.end(), input->source) == vertexSources.end())
		{
			const char* semantic = FUDaeGeometryInput::ToString(input->semantic);
			FUDaeWriter::AddInput(polygonsNode, input->source->GetDaeId(), semantic, input->idx, input->set);
		}
		else if (!isVertexInputFound)
		{
			FUDaeWriter::AddInput(polygonsNode, verticesNodeId.ToCharPtr(), DAE_VERTEX_INPUT, input->idx);
			isVertexInputFound = true;
		}
	}

	FUSStringBuilder builder;
	builder.reserve(1024);

	// For the poly-list case, export the list of vertex counts
	if (!hasHoles && hasNPolys)
	{
		FUStringConversion::ToString(builder, faceVertexCounts);
		AddChild(polygonsNode, DAE_VERTEXCOUNT_ELEMENT, builder.ToCharPtr());
		builder.clear();
	}

	// For the non-holes cases, open only one <p> element for all the data indices
	xmlNode* pNode = NULL,* phNode = NULL;
	if (!hasHoles) pNode = AddChild(polygonsNode, DAE_POLYGON_ELEMENT);

	// Export the data indices (tessellation information)
	size_t faceCount = faceVertexCounts.size();
	uint32 faceVertexOffset = 0;
	for (size_t faceIndex = 0; faceIndex < faceCount; ++faceIndex)
	{
		// For the holes cases, verify whether this face or the next one(s) are holes. We may need to open a new <ph>/<p> element
		bool isHole = false, isHoleNext = false;
		if (hasHoles)
		{
			for (UInt32List::const_iterator itH = holeFaces.begin(); itH != holeFaces.end(); ++itH)
			{
				isHole |= (*itH) == (uint32) faceIndex;
				isHoleNext |= (*itH) + 1 == (uint32) faceIndex;
			}

			if (!isHole)
			{
				// Just open a <p> element: this is the most common case
				if (!isHoleNext) pNode = AddChild(polygonsNode, DAE_POLYGONHOLED_ELEMENT);
				else
				{
					// Open up a new <ph> element and its <p> element
					phNode = AddChild(polygonsNode, DAE_POLYGONHOLED_ELEMENT);
					pNode = AddChild(phNode, DAE_POLYGON_ELEMENT);
				}
			}
			else
			{
				// Open up a <h> element
				pNode = AddChild(phNode, DAE_HOLE_ELEMENT);
			}
		}

		// Write out the tessellation information for all the vertices of this face
		uint32 faceVertexCount = faceVertexCounts[faceIndex];
		for (uint32 faceVertexIndex = faceVertexOffset; faceVertexIndex < faceVertexOffset + faceVertexCount; ++faceVertexIndex)
		{
			for (FCDGeometryPolygonsInputList::const_iterator itI = idxOwners.begin(); itI != idxOwners.end(); ++itI)
			{
				UInt32List& indices = (*itI)->indices;
				builder.append(indices[faceVertexIndex]);
				builder.append(' ');
			}
		}

		// For the holes cases: write out the indices for every polygon element
		if (hasHoles)
		{
			if (!builder.empty()) builder.pop_back(); // take out the last space
			AddContent(pNode, builder);
		}
		faceVertexOffset += faceVertexCount;
	}

	// For the non-holes cases: write out the indices at the very end, for the single <p> element
	if (!hasHoles)
	{
		if (!builder.empty()) builder.pop_back(); // take out the last space
		AddContent(pNode, builder);
	}

	// Write out the material semantic and the number of polygons
	if (!materialSemantic.empty())
	{
		AddAttribute(polygonsNode, DAE_MATERIAL_ATTRIBUTE, materialSemantic);
	}
	AddAttribute(polygonsNode, DAE_COUNT_ATTRIBUTE, GetFaceCount() - GetHoleCount());

	return polygonsNode;
}

// Clone this list of polygons
FCDGeometryPolygons* FCDGeometryPolygons::Clone(FCDGeometryMesh* cloneParent)
{
	FCDGeometryPolygons* clone = new FCDGeometryPolygons(GetDocument(), cloneParent);
	clone->materialSemantic = materialSemantic;
	clone->faceVertexCounts = faceVertexCounts;
	clone->faceOffset = faceOffset;
	clone->faceVertexCount = faceVertexCount;
	clone->faceVertexOffset = faceVertexOffset;
	
	// Clone the geometry inputs
	uint32 inputCount = (uint32) inputs.size();
	clone->inputs.resize(inputCount);
	for (uint32 i = 0; i < inputCount; ++i)
	{
		clone->inputs[i] = new FCDGeometryPolygonsInput();
		clone->inputs[i]->source = cloneParent->FindSourceById(inputs[i]->source->GetDaeId());
		clone->inputs[i]->idx = inputs[i]->idx;
		clone->inputs[i]->indices = inputs[i]->indices;
		clone->inputs[i]->ownsIdx = inputs[i]->ownsIdx;
		clone->inputs[i]->semantic = inputs[i]->semantic;
		clone->inputs[i]->set = inputs[i]->set;

		// Regenerate the idxOwners list with the new inputs
		if (clone->inputs[i]->ownsIdx) clone->idxOwners.push_back(clone->inputs[i]);
	}
	return clone;
}

FCDGeometryPolygonsInput::FCDGeometryPolygonsInput()
{
	idx = 0;
	ownsIdx = false;
	semantic = FUDaeGeometryInput::UNKNOWN;
	set = -1;
	source = NULL;
}
