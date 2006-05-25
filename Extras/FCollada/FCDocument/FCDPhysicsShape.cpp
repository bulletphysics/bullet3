/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDPhysicsShape.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDTransform.h"
#include "FCDocument/FCDGeometryInstance.h"
#include "FCDocument/FCDPhysicsRigidBody.h"
#include "FCDocument/FCDPhysicsMaterial.h"
#include "FCDocument/FCDPhysicsAnalyticalGeometry.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDPhysicsShape::FCDPhysicsShape(FCDocument* document) : FCDEntity(document, "PhysicsShape")
{
	hollow= true;
	physicsMaterial = NULL;
	ownsPhysicsMaterial = true;
	ownsGeometryInstance = true;
	geometry = NULL;
	analGeom = NULL;
	mass = 1.0f; //FIXME: should be derived from density x shape volume
	density = 1.0f;
}

FCDPhysicsShape::~FCDPhysicsShape()
{
	if(ownsPhysicsMaterial)
	{
		SAFE_DELETE(physicsMaterial);
	}
	else
	{
		physicsMaterial = NULL;
	}

	if(ownsGeometryInstance)
	{
		SAFE_DELETE(geometry);
	}
	else
	{
		geometry = NULL;
	}

	SAFE_DELETE(analGeom);
	CLEAR_POINTER_VECTOR(transforms);
}

// Create a copy of this shape
// Note: geometries are just shallow-copied
FCDPhysicsShape* FCDPhysicsShape::Clone()
{
	FCDPhysicsShape* clone = new FCDPhysicsShape(GetDocument());
	FCDEntity::Clone(clone);
	clone->SetMass(mass);
	clone->SetDensity(density);
	if(analGeom)
		clone->SetAnalyticalGeometry(analGeom->Clone());
	else
		clone->SetAnalyticalGeometry(NULL);
	clone->SetGeometryInstance(geometry); //FIXME: should do Clone?!
	clone->SetGeometryInstanceOwnership(false);
	if(physicsMaterial)
		clone->SetPhysicsMaterial(physicsMaterial->Clone());
	else
		clone->SetPhysicsMaterial(NULL);
	clone->SetOwnsPhysicsMaterial(true);
	clone->SetHollow(hollow);
	for(size_t i = 0; i < transforms.size(); ++i)
	{
		clone->AddTransform(transforms[i]->Clone(NULL));
	}

	return clone;
}


// Load from a XML node the given physicsShape
FUStatus FCDPhysicsShape::LoadFromXML(xmlNode* physicsShapeNode)
{
	FUStatus status;
	if (!IsEquivalent(physicsShapeNode->name, DAE_SHAPE_ELEMENT))
	{
		return status.Warning(FS("PhysicsShape library contains unknown element."), physicsShapeNode->line);
	}

	// Read in the first valid child element found
	for (xmlNode* child = physicsShapeNode->children; child != NULL; child = child->next)
	{
		if (child->type != XML_ELEMENT_NODE) continue;

		if(IsEquivalent(child->name, DAE_HOLLOW_ELEMENT))
		{
			hollow = FUStringConversion::ToBoolean(ReadNodeContentDirect(child));
		}
		else if(IsEquivalent(child->name, DAE_MASS_ELEMENT))
		{
			mass = FUStringConversion::ToFloat(ReadNodeContentDirect(child));
		}
		else if(IsEquivalent(child->name, DAE_DENSITY_ELEMENT))
		{
			density = FUStringConversion::ToFloat(ReadNodeContentDirect(child));
		}
		else if(IsEquivalent(child->name, DAE_PHYSICS_MATERIAL_ELEMENT))
		{
			if(!HasNodeProperty(child, DAE_URL_ATTRIBUTE))
			{
				//inline definition of physics_material
				physicsMaterial = new FCDPhysicsMaterial(GetDocument());
				physicsMaterial->LoadFromXML(child);
				ownsPhysicsMaterial = true;
			}
			else
			{
				FUUri url = ReadNodeUrl(child);
				if (url.prefix.empty())
				{
					physicsMaterial = GetDocument()->FindPhysicsMaterial(url.suffix);
					ownsPhysicsMaterial = false;
				}
			}
		}
		else if(IsEquivalent(child->name, DAE_INSTANCE_GEOMETRY_ELEMENT))
		{
			FUUri url = ReadNodeUrl(child);
			if (url.prefix.empty())
			{
				FCDGeometry* entity = GetDocument()->FindGeometry(url.suffix);
				if (entity != NULL)
				{
					geometry = new FCDGeometryInstance(GetDocument(), (FCDEntity*)entity);
					if(analGeom) SAFE_DELETE(analGeom);
					status.AppendStatus(geometry->LoadFromXML(child));
					continue; 
				}
			}
			status.Warning(FS("Unable to retrieve FCDGeometry instance for scene node: ") + TO_FSTRING(GetDaeId()), child->line);
		}

#define PARSE_ANALYTICAL_SHAPE(className, nodeName) \
		else if (IsEquivalent(child->name, nodeName)) { \
			if(analGeom) SAFE_DELETE(analGeom); \
			analGeom = new className(GetDocument()); \
			if(geometry) SAFE_DELETE(geometry); \
			status = analGeom->LoadFromXML(child);}\
			if (!status) { status.Warning(fstring(FC("Invalid <") FC(nodeName) FC("> shape in node: ")) + TO_FSTRING(GetDaeId()), child->line); break; }

		PARSE_ANALYTICAL_SHAPE(FCDPASBox, DAE_BOX_ELEMENT)
		PARSE_ANALYTICAL_SHAPE(FCDPASPlane, DAE_PLANE_ELEMENT)
		PARSE_ANALYTICAL_SHAPE(FCDPASSphere, DAE_SPHERE_ELEMENT)
		PARSE_ANALYTICAL_SHAPE(FCDPASCylinder, DAE_CYLINDER_ELEMENT)
		PARSE_ANALYTICAL_SHAPE(FCDPASCapsule, DAE_CAPSULE_ELEMENT)
		PARSE_ANALYTICAL_SHAPE(FCDPASTaperedCapsule, DAE_TAPERED_CAPSULE_ELEMENT)
		PARSE_ANALYTICAL_SHAPE(FCDPASTaperedCylinder, DAE_TAPERED_CYLINDER_ELEMENT)
#undef PARSE_ANALYTICAL_SHAPE


		// Parse the physics shape transforms <rotate>, <scale>, <translate> are supported.
		else if(IsEquivalent(child->name, DAE_ASSET_ELEMENT)) {}
		else if(IsEquivalent(child->name, DAE_EXTRA_ELEMENT)) {}
		else
		{
			FCDTransform* transform = FCDTFactory::CreateTransform(GetDocument(), NULL, child);
			if (transform != NULL && (transform->GetType() != FCDTransform::TRANSLATION
				|| transform->GetType() != FCDTransform::ROTATION || transform->GetType() != FCDTransform::SCALE))
			{
				SAFE_DELETE(transform);
			}
			else
			{
				transforms.push_back(transform);
				status.AppendStatus(transform->LoadFromXML(child));
			}
		}
	}

	return status;
}

// Write out the <physicsShape> node
xmlNode* FCDPhysicsShape::WriteToXML(xmlNode* UNUSED(parentNode)) const
{
	xmlNode* physicsShapeNode = NULL;

	//TODO:

	return physicsShapeNode;
}
