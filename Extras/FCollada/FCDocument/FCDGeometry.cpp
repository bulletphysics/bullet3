/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDAnimated.h"
#include "FCDocument/FCDGeometry.h"
#include "FCDocument/FCDGeometryMesh.h"
#include "FCDocument/FCDGeometrySpline.h"
#include "FUtils/FUStringConversion.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDGeometry::FCDGeometry(FCDocument* document) : FCDEntity(document, "Geometry")
{
	spline = NULL;
	mesh = NULL;
}

FCDGeometry::~FCDGeometry()
{
	SAFE_DELETE(spline);
	SAFE_DELETE(mesh);
}

// Sets the type of this geometry to mesh and creates an empty mesh structure.
FCDGeometryMesh* FCDGeometry::CreateMesh()
{
	SAFE_DELETE(spline);
	SAFE_DELETE(mesh);

	mesh = new FCDGeometryMesh(GetDocument(), this);
	return mesh;
}

// Sets the type of this geometry to spline and creates an empty spline structure.
FCDGeometrySpline* FCDGeometry::CreateSpline()
{
	SAFE_DELETE(spline);
	SAFE_DELETE(mesh);

	spline = new FCDGeometrySpline(GetDocument(), this);
	return spline;
}

// Create a copy of this geometry, with the vertices overwritten
FCDGeometry* FCDGeometry::Clone(FloatList& newPositions, uint32 newPositionsStride, FloatList& newNormals, uint32 newNormalsStride)
{
	// Clone only mesh geometries. This function is necessary for COLLADA 1.3 backward compatibility and should not be used
	// in some other way (yet)
	if (!IsMesh()) return NULL;
	
	FCDGeometry* clone = new FCDGeometry(GetDocument());
	clone->mesh = mesh->Clone(newPositions, newPositionsStride, newNormals, newNormalsStride);
	return clone;
}

// Load from a XML node the given geometry
FUStatus FCDGeometry::LoadFromXML(xmlNode* geometryNode)
{
	SAFE_DELETE(mesh);
	SAFE_DELETE(spline);

	FUStatus status = FCDEntity::LoadFromXML(geometryNode);
	if (!status) return status;
	if (!IsEquivalent(geometryNode->name, DAE_GEOMETRY_ELEMENT))
	{
		return status.Warning(FS("Geometry library contains unknown element."), geometryNode->line);
	}

	// Read in the first valid child element found
	for (xmlNode* child = geometryNode->children; child != NULL; child = child->next)
	{
		if (child->type != XML_ELEMENT_NODE) continue;

		if (IsEquivalent(child->name, DAE_MESH_ELEMENT))
		{
			// Create a new mesh
			FCDGeometryMesh* m = CreateMesh();
			status.AppendStatus(m->LoadFromXML(child));
			break;
		}
		else if (IsEquivalent(child->name, DAE_SPLINE_ELEMENT))
		{
			// Create a new spline
			FCDGeometrySpline* s = CreateSpline();
			status.AppendStatus(s->LoadFromXML(child));
			break;
		}
		else if (IsEquivalent(child->name, DAE_CONVEX_MESH_ELEMENT))
		{
			//several cases can exist here...
			//wait for fixed version of FCollada?
			//assume <convex_hull_of> and do instantiate?

			// Create a new mesh
			FCDGeometryMesh* m = CreateMesh();
			m->m_convex = true;
			FUUri url;
			char hullname[] = "convex_hull_of";

			url = ReadNodeUrl(child,hullname);
			
			if (!url.prefix.empty())
			{
				
				FCDGeometry* entity = GetDocument()->FindGeometry(url.prefix);
				if (entity)
				{
					printf("found convex_mesh%s\n",url.prefix);
					//quick hack
					this->mesh = entity->GetMesh();
					mesh->m_convex = true;

				}
				
			}
		
			
			//status.AppendStatus(m->LoadFromXML(child));

			break;
		}
		else
		{
			return status.Fail(FS("Unknown child in <geometry> with id: ") + TO_FSTRING(GetDaeId()), child->line);
		}
	}

	if (mesh == NULL && spline == NULL)
	{
		status.Warning(FS("No mesh or spline found within geometry: ") + TO_FSTRING(GetDaeId()), geometryNode->line);
	}
	return status;
}

// Write out the <geometry> node
xmlNode* FCDGeometry::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* geometryNode = WriteToEntityXML(parentNode, DAE_GEOMETRY_ELEMENT);

	if (mesh != NULL) mesh->WriteToXML(geometryNode);
	else if (spline != NULL) spline->WriteToXML(geometryNode);

	FCDEntity::WriteToExtraXML(geometryNode);
	return geometryNode;
}
