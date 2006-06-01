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
#include "FCDocument/FCDGeometry.h"
#include "FCDocument/FCDGeometryMesh.h"
#include "FCDocument/FCDGeometryPolygons.h"
#include "FCDocument/FCDGeometrySource.h"
#include "FUtils/FUStringConversion.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDGeometryMesh::FCDGeometryMesh(FCDocument* document, FCDGeometry* _parent) : FCDObject(document, "FCDGeometryMesh")
{
	m_convex = false;
	parent = _parent;
	faceVertexCount = faceCount = holeCount = 0;
	isDoubleSided = true;
}

FCDGeometryMesh::~FCDGeometryMesh()
{
	CLEAR_POINTER_VECTOR(sources);
	CLEAR_POINTER_VECTOR(polygons);
	faceVertexCount = faceCount = 0;
	parent = NULL;
}

// Retrieve the parent's id
const string& FCDGeometryMesh::GetDaeId() const
{
	return parent->GetDaeId();
}

// Search for a data source in the geometry node
FCDGeometrySource* FCDGeometryMesh::FindSourceById(const string& id)
{
	const char* localId = id.c_str();
	if (localId[0] == '#') ++localId;
	for (FCDGeometrySourceList::iterator it = sources.begin(); it != sources.end(); ++it)
	{
		if ((*it)->GetSourceId() == localId) return (*it);
	}
	return NULL;
}

// Search for a data source in the geometry node
const FCDGeometrySource* FCDGeometryMesh::FindSourceById(const string& id) const
{
	const char* localId = id.c_str();
	if (localId[0] == '#') ++localId;
	for (FCDGeometrySourceList::const_iterator it = sources.begin(); it != sources.end(); ++it)
	{
		if ((*it)->GetSourceId() == localId) return (*it);
	}
	return NULL;
}

// Retrieve the source for the vertex position
FCDGeometrySource* FCDGeometryMesh::GetPositionSource()
{
	string vertexSourceId;
	for (FCDGeometrySourceList::iterator itS = vertexSources.begin(); itS != vertexSources.end(); ++itS)
	{
		if ((*itS)->GetSourceType() == FUDaeGeometryInput::POSITION) return (*itS);
	}
	return NULL;
}

// Retrieve the source of the vertex position
const FCDGeometrySource* FCDGeometryMesh::GetPositionSource() const
{
	string vertexSourceId;
	for (FCDGeometrySourceList::const_iterator itS = vertexSources.begin(); itS != vertexSources.end(); ++itS)
	{
		if ((*itS)->GetSourceType() == FUDaeGeometryInput::POSITION) return (*itS);
	}
	return NULL;
}

// Creates a new polygon group.
FCDGeometryPolygons* FCDGeometryMesh::AddPolygons()
{
	FCDGeometryPolygons* polys = new FCDGeometryPolygons(GetDocument(), this);
	polygons.push_back(polys);

	// Add to this new polygons all the per-vertex sources.
	for (FCDGeometrySourceList::iterator itS = vertexSources.begin(); itS != vertexSources.end(); ++itS)
	{
		polys->AddInput(*itS, 0);
	}

	return polys;
}

// Creates a new per-vertex data source
FCDGeometrySource* FCDGeometryMesh::AddVertexSource()
{
	FCDGeometrySource* vertexSource = AddSource();
	vertexSources.push_back(vertexSource);

	// Add this new per-vertex data source to all the existing polygon groups, at offset 0.
	for (FCDGeometryPolygonsList::iterator itP = polygons.begin(); itP != polygons.end(); ++itP)
	{
		(*itP)->AddInput(vertexSource, 0);
	}
	return vertexSource;
}

// Creates a new data source
FCDGeometrySource* FCDGeometryMesh::AddSource()
{
	FCDGeometrySource* source = new FCDGeometrySource(GetDocument());
	sources.push_back(source);
	return source;
}

// Forces the triangulation of the mesh polygons
void FCDGeometryMesh::Triangulate()
{
	for (FCDGeometryPolygonsList::iterator itP = polygons.begin(); itP != polygons.end(); ++itP)
	{
		(*itP)->Triangulate();
	}

	// Recalculate the mesh/polygons statistics
	Recalculate();
}

// Recalculates all the hole/vertex/face-vertex counts and offsets within the mesh and its polygons
void FCDGeometryMesh::Recalculate()
{
	faceCount = holeCount = faceVertexCount = 0;
	for (FCDGeometryPolygonsList::iterator itP = polygons.begin(); itP != polygons.end(); ++itP)
	{
		FCDGeometryPolygons* polygons = *itP;
		polygons->Recalculate();

		polygons->SetFaceOffset(faceCount);
		polygons->SetHoleOffset(holeCount);
		polygons->SetFaceVertexOffset(faceVertexCount);
		faceCount += polygons->GetFaceCount();
		holeCount += polygons->GetHoleCount();
		faceVertexCount += polygons->GetFaceVertexCount();
	}
}

// Create a copy of this geometry, with the vertices overwritten
FCDGeometryMesh* FCDGeometryMesh::Clone(FloatList& newPositions, uint32 newPositionsStride, FloatList& newNormals, uint32 newNormalsStride)
{
	// Create the clone and fill it with the known information
	FCDGeometryMesh* clone = new FCDGeometryMesh(GetDocument(), NULL);
	clone->faceCount = faceCount;

	// Clone the source data
	size_t sourceCount = sources.size();
	clone->sources.reserve(sourceCount);
	for (FCDGeometrySourceList::const_iterator itS = sources.begin(); itS != sources.end(); ++itS)
	{
		clone->sources.push_back((*itS)->Clone());
		if (std::find(vertexSources.begin(), vertexSources.end(), (*itS)) != vertexSources.end())
		{
			clone->vertexSources.push_back(clone->sources.back());
		}
	}

	// Clone the polygons data
	// Gather up the position and normal data sources
	FCDGeometrySourceList positionSources, normalSources;
	size_t polygonsCount = polygons.size();
	clone->polygons.resize(polygonsCount);
	for (size_t i = 0; i < polygonsCount; ++i)
	{
		clone->polygons[i] = polygons[i]->Clone(clone);

		// Retrieve the position data source
		FCDGeometryPolygonsInput* positionInput = clone->polygons[i]->FindInput(FUDaeGeometryInput::POSITION);
		if (positionInput != NULL)
		{
			FCDGeometrySource* dataSource = positionInput->source;
			FUAssert(dataSource != NULL, continue);
			if (std::find(positionSources.begin(), positionSources.end(), dataSource) == positionSources.end())
			{
				positionSources.push_back(dataSource);
			}
		}

		// Retrieve the normal data source
		FCDGeometryPolygonsInput* normalInput = clone->polygons[i]->FindInput(FUDaeGeometryInput::NORMAL);
		if (normalInput != NULL)
		{
			FCDGeometrySource* dataSource = normalInput->source;
			FUAssert(dataSource != NULL, continue);
			if (std::find(normalSources.begin(), normalSources.end(), dataSource) == normalSources.end())
			{
				normalSources.push_back(dataSource);
			}
		}
	}

	// Override the position and normal data sources with the given data (from the controller's bind shape)
#	define OVERWRITE_SOURCES(cloneSources, newSourceData, newSourceStride) { \
		size_t dataSourceCount = min(cloneSources.size(), newSourceData.size()), offset = 0; \
		for (size_t i = 0; i < dataSourceCount; ++i) { \
			FCDGeometrySource* dataSource = cloneSources[i]; \
			size_t dataCount = dataSource->GetSourceData().size() / dataSource->GetSourceStride(); \
			if (offset + dataCount > newSourceData.size() / newSourceStride) dataCount = newSourceData.size() / newSourceStride - offset; \
			if (dataCount == 0) break; \
			/* Insert the relevant data in this source */ \
			dataSource->SetSourceData(newSourceData, newSourceStride, offset * newSourceStride, (offset + dataCount) * newSourceStride); \
			offset += dataCount; \
		} } 

	OVERWRITE_SOURCES(positionSources, newPositions, newPositionsStride);
	OVERWRITE_SOURCES(normalSources, newNormals, newNormalsStride);
#	undef OVERWRITE_SOURCES

	return clone;
}

// Read in the <mesh> node of the COLLADA document
FUStatus FCDGeometryMesh::LoadFromXML(xmlNode* meshNode)
{
	FUStatus status;
	
	// Read in the data sources
	xmlNodeList sourceDataNodes;
	FindChildrenByType(meshNode, DAE_SOURCE_ELEMENT, sourceDataNodes);
	for (xmlNodeList::iterator it = sourceDataNodes.begin(); it != sourceDataNodes.end(); ++it)
	{
		FCDGeometrySource* source = AddSource();
		status.AppendStatus(source->LoadFromXML(*it));
		if (source->GetSourceStride() < 3) continue;

		// COLLADA 1.3 backward compatibility
		// Maya-specific: Look for the double-sided flag in the normals source
		StringList parameterNames; xmlNodeList parameterNodes;
		xmlNode* mayaTechniqueNode = FindTechnique((*it), DAEMAYA_MAYA_PROFILE);
		FindParameters(mayaTechniqueNode, parameterNames, parameterNodes);
		size_t parameterCount = parameterNodes.size();
		for (size_t i = 0; i < parameterCount; ++i)
		{
			const char* content = ReadNodeContentDirect(parameterNodes[i]);
			if (parameterNames[i] == DAEMAYA_DOUBLE_SIDED_PARAMETER) isDoubleSided = FUStringConversion::ToBoolean(content);
		}
	}

	// Retrieve the <vertices> node
	xmlNode* verticesNode = FindChildByType(meshNode, DAE_VERTICES_ELEMENT);
	if (verticesNode == NULL)
	{
		return status.Warning(FS("No <vertices> element in mesh: ") + TO_FSTRING(parent->GetDaeId()), meshNode->line);
	}

	// Read in the per-vertex inputs
	bool hasPositions = false;

	xmlNodeList vertexInputNodes;
	FindChildrenByType(verticesNode, DAE_INPUT_ELEMENT, vertexInputNodes);
	for (xmlNodeList::iterator it = vertexInputNodes.begin(); it < vertexInputNodes.end(); ++it)
	{
		xmlNode* vertexInputNode = *it;
		string inputSemantic = ReadNodeSemantic(vertexInputNode);
		FUDaeGeometryInput::Semantic semantic = FUDaeGeometryInput::FromString(inputSemantic);
		if (semantic == FUDaeGeometryInput::POSITION || semantic == FUDaeGeometryInput::NORMAL
			|| semantic == FUDaeGeometryInput::COLOR || semantic == FUDaeGeometryInput::TEXCOORD)
		{
			string sourceId = ReadNodeSource(vertexInputNode);
			FCDGeometrySource* source = FindSourceById(sourceId);
			if (source == NULL)
			{
				return status.Fail(FS("Mesh has source with an unknown id: ") + TO_FSTRING(parent->GetDaeId()), vertexInputNode->line);
			}
			source->SetSourceType(semantic);
			if (semantic == FUDaeGeometryInput::POSITION) hasPositions = true;
			vertexSources.push_back(source);
		}
	}
	if (!hasPositions)
	{
		return status.Warning(FS("No vertex position input node in mesh: ") + TO_FSTRING(parent->GetDaeId()), verticesNode->line);
	}

	// Create our rendering object and read in the tessellation
	xmlNodeList polygonsNodes;
	FindChildrenByType(meshNode, DAE_POLYGONS_ELEMENT, polygonsNodes);
	FindChildrenByType(meshNode, DAE_TRIANGLES_ELEMENT, polygonsNodes);
	FindChildrenByType(meshNode, DAE_POLYLIST_ELEMENT, polygonsNodes);
	if (polygonsNodes.empty())
	{
		return status.Warning(FS("No tessellation found for mesh: ") + TO_FSTRING(parent->GetDaeId()), meshNode->line);
	}
	for (xmlNodeList::iterator it = polygonsNodes.begin(); it != polygonsNodes.end(); ++it)
	{
		// Create the geometry polygons object
		xmlNode* polygonsNode = *it;
		FCDGeometryPolygons* polygon = new FCDGeometryPolygons(GetDocument(), this);
		polygons.push_back(polygon);
		status = polygon->LoadFromXML(polygonsNode);
		if (!status) return status;
	}

	// Calculate the important statistics/offsets/counts
	Recalculate();

	// Apply the length factor on the vertex positions
	float lengthFactor = GetDocument()->GetLengthUnitConversion();
	FCDGeometrySource* positionSource = GetPositionSource();
	if (positionSource == NULL)
	{
		return status.Fail(FS("Cannot process the vertex POSITION source for mesh: ") + TO_FSTRING(parent->GetDaeId()), meshNode->line);
	}
	FloatList& positionData = positionSource->GetSourceData();
	for (FloatList::iterator it = positionData.begin(); it != positionData.end(); ++it) (*it) *= lengthFactor;

	return status;
}

// Write out the <mesh> node to the COLLADA xml tree
xmlNode* FCDGeometryMesh::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* meshNode = AddChild(parentNode, DAE_MESH_ELEMENT);

	// Write out the sources
	for (FCDGeometrySourceList::const_iterator itS = sources.begin(); itS != sources.end(); ++itS)
	{
		(*itS)->WriteToXML(meshNode);
	}

	// Write out the <vertices> element
	xmlNode* verticesNode = AddChild(meshNode, DAE_VERTICES_ELEMENT);
	for (FCDGeometrySourceList::const_iterator itS = vertexSources.begin(); itS != vertexSources.end(); ++itS)
	{
		const char* semantic = FUDaeGeometryInput::ToString((*itS)->GetSourceType());
		AddInput(verticesNode, (*itS)->GetSourceId(), semantic);
	}
	FUSStringBuilder verticesNodeId(GetDaeId()); verticesNodeId.append("-vertices");
	AddAttribute(verticesNode, DAE_ID_ATTRIBUTE, verticesNodeId);

	// Write out the polygons
	for (FCDGeometryPolygonsList::const_iterator itP = polygons.begin(); itP != polygons.end(); ++itP)
	{
		(*itP)->WriteToXML(meshNode);
	}

	return meshNode;
}
