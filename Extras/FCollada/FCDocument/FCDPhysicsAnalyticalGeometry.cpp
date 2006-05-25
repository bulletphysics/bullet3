/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDPhysicsAnalyticalGeometry.h"
#include "FCDocument/FCDPhysicsParameterGeneric.h"
#include "FCDocument/FCDocument.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDPhysicsAnalyticalGeometry::FCDPhysicsAnalyticalGeometry(FCDocument* document) : FCDEntity(document, "AnalyticalGeometry")
{
}

FCDPhysicsAnalyticalGeometry::~FCDPhysicsAnalyticalGeometry()
{
}


// Load from a XML node the given physicsAnalyticalGeometry
FUStatus FCDPhysicsAnalyticalGeometry::LoadFromXML(xmlNode* node)
{
	FUStatus status = FCDEntity::LoadFromXML(node);
	return status;
}

FCDPASBox::FCDPASBox(FCDocument* document) : FCDPhysicsAnalyticalGeometry(document)
{
	halfExtents.x = halfExtents.y = halfExtents.z = 0.f;
}

FCDPhysicsAnalyticalGeometry* FCDPASBox::Clone()
{
	FCDPASBox* clone = new FCDPASBox(GetDocument());
	FCDEntity::Clone(clone);
	clone->halfExtents = halfExtents;
	return clone;
}

FUStatus FCDPASBox::LoadFromXML(xmlNode* node)
{
	FUStatus status;

	if (!IsEquivalent(node->name, DAE_BOX_ELEMENT))
	{
		return status.Warning(FS("Box is not of the right type."), node->line);
	}

	for (xmlNode* child = node->children; child != NULL; child = child->next)
	{
		if (child->type != XML_ELEMENT_NODE) continue;

		if(IsEquivalent(child->name, DAE_HALF_EXTENTS_ELEMENT))
		{
			const char* halfExt = ReadNodeContentDirect(child);
			halfExtents.x = FUStringConversion::ToFloat(&halfExt);
			halfExtents.y = FUStringConversion::ToFloat(&halfExt);
			halfExtents.z = FUStringConversion::ToFloat(&halfExt);
		}
	}
	return status;
}

xmlNode* FCDPASBox::WriteToXML(xmlNode* node) const
{
	xmlNode* geomNode = AddChild(node, DAE_BOX_ELEMENT);
	string s = FUStringConversion::ToString(halfExtents);
	AddChild(geomNode, DAE_HALF_EXTENTS_ELEMENT, s);
	return geomNode;
}


FCDPASPlane::FCDPASPlane(FCDocument* document) : FCDPhysicsAnalyticalGeometry(document)
{
	normal.x = normal.y = normal.z = d = 0.f;
}

FCDPhysicsAnalyticalGeometry* FCDPASPlane::Clone()
{
	FCDPASPlane* clone = new FCDPASPlane(GetDocument());
	FCDEntity::Clone(clone);
	clone->normal = normal;
	return clone;
}

FUStatus FCDPASPlane::LoadFromXML(xmlNode* node)
{
	FUStatus status;

	if (!IsEquivalent(node->name, DAE_PLANE_ELEMENT))
	{
		return status.Warning(FS("Plane is not of the right type."), node->line);
	}

	for (xmlNode* child = node->children; child != NULL; child = child->next)
	{
		if (child->type != XML_ELEMENT_NODE) continue;

		if(IsEquivalent(child->name, DAE_EQUATION_ELEMENT))
		{
			const char* eq = ReadNodeContentDirect(child);
			normal.x = FUStringConversion::ToFloat(&eq);
			normal.y = FUStringConversion::ToFloat(&eq);
			normal.z = FUStringConversion::ToFloat(&eq);
			d = FUStringConversion::ToFloat(&eq);
		}
	}
	return status;
}

xmlNode* FCDPASPlane::WriteToXML(xmlNode* node) const
{
	xmlNode* geomNode = AddChild(node, DAE_PLANE_ELEMENT);
	FMVector4 equation;
	equation.w = normal.x; equation.x = normal.y; equation.y = normal.z; equation.z = d;
	string s = FUStringConversion::ToString(equation);
	AddChild(geomNode, DAE_EQUATION_ELEMENT, s);
	return geomNode;
}


FCDPASSphere::FCDPASSphere(FCDocument* document) : FCDPhysicsAnalyticalGeometry(document)
{
	radius = 0.f;
}

FCDPhysicsAnalyticalGeometry* FCDPASSphere::Clone()
{
	FCDPASSphere* clone = new FCDPASSphere(GetDocument());
	FCDEntity::Clone(clone);
	clone->radius = radius;
	return clone;
}

FUStatus FCDPASSphere::LoadFromXML(xmlNode* node)
{
	FUStatus status;

	if (!IsEquivalent(node->name, DAE_SPHERE_ELEMENT))
	{
		return status.Warning(FS("Sphere is not of the right type."), node->line);
	}

	for (xmlNode* child = node->children; child != NULL; child = child->next)
	{
		if (child->type != XML_ELEMENT_NODE) continue;

		if(IsEquivalent(child->name, DAE_RADIUS_ELEMENT))
		{
			radius = FUStringConversion::ToFloat(ReadNodeContentDirect(child));
		}
	}
	return status;
}

xmlNode* FCDPASSphere::WriteToXML(xmlNode* node) const
{
	xmlNode* geomNode = AddChild(node, DAE_SPHERE_ELEMENT);
	AddChild(geomNode, DAE_RADIUS_ELEMENT, radius);
	return geomNode;
}

FCDPASCylinder::FCDPASCylinder(FCDocument* document) : FCDPhysicsAnalyticalGeometry(document)
{
	height = 0.f;
	radius = 0.f;
}

FCDPhysicsAnalyticalGeometry* FCDPASCylinder::Clone()
{
	FCDPASCylinder* clone = new FCDPASCylinder(GetDocument());
	FCDEntity::Clone(clone);
	clone->radius = radius;
	clone->height = height;
	return clone;
}

FUStatus FCDPASCylinder::LoadFromXML(xmlNode* node)
{
	FUStatus status;

	if (!IsEquivalent(node->name, DAE_CYLINDER_ELEMENT))
	{
		return status.Warning(FS("Sphere is not of the right type."), node->line);
	}

	for (xmlNode* child = node->children; child != NULL; child = child->next)
	{
		if (child->type != XML_ELEMENT_NODE) continue;

		if(IsEquivalent(child->name, DAE_HEIGHT_ELEMENT))
		{
			height = FUStringConversion::ToFloat(ReadNodeContentDirect(child));
		}
		else if(IsEquivalent(child->name, DAE_RADIUS_ELEMENT))
		{
			radius = FUStringConversion::ToFloat(ReadNodeContentDirect(child));
		}
	}
	return status;
}

xmlNode* FCDPASCylinder::WriteToXML(xmlNode* node) const
{
	xmlNode* geomNode = AddChild(node, DAE_CYLINDER_ELEMENT);
	AddChild(geomNode, DAE_HEIGHT_ELEMENT, height);
	AddChild(geomNode, DAE_RADIUS_ELEMENT, radius);
	return geomNode;
}

FCDPASCapsule::FCDPASCapsule(FCDocument* document) : FCDPhysicsAnalyticalGeometry(document)
{
	height = 0.f;
	radius = 0.f;
}

FCDPhysicsAnalyticalGeometry* FCDPASCapsule::Clone()
{
	FCDPASCapsule* clone = new FCDPASCapsule(GetDocument());
	FCDEntity::Clone(clone);
	clone->radius = radius;
	clone->height = height;
	return clone;
}

FUStatus FCDPASCapsule::LoadFromXML(xmlNode* node)
{
	FUStatus status;

	if (!IsEquivalent(node->name, DAE_CAPSULE_ELEMENT))
	{
		return status.Warning(FS("Capsule is not of the right type."), node->line);
	}

	for (xmlNode* child = node->children; child != NULL; child = child->next)
	{
		if (child->type != XML_ELEMENT_NODE) continue;

		if(IsEquivalent(child->name, DAE_HEIGHT_ELEMENT))
		{
			height = FUStringConversion::ToFloat(ReadNodeContentDirect(child));
		}
		else if(IsEquivalent(child->name, DAE_RADIUS_ELEMENT))
		{
			radius = FUStringConversion::ToFloat(ReadNodeContentDirect(child));
		}
	}
	return status;
}

xmlNode* FCDPASCapsule::WriteToXML(xmlNode* node) const
{
	xmlNode* geomNode = AddChild(node, DAE_CAPSULE_ELEMENT);
	AddChild(geomNode, DAE_HEIGHT_ELEMENT, height);
	AddChild(geomNode, DAE_RADIUS_ELEMENT, radius);
	return geomNode;
}

FCDPASTaperedCapsule::FCDPASTaperedCapsule(FCDocument* document) : FCDPASCapsule(document)
{
	radius2 = 0.f;
}

FCDPhysicsAnalyticalGeometry* FCDPASTaperedCapsule::Clone()
{
	FCDPASTaperedCapsule* clone = new FCDPASTaperedCapsule(GetDocument());
	FCDEntity::Clone(clone);
	clone->radius = radius;
	return clone;
}

FUStatus FCDPASTaperedCapsule::LoadFromXML(xmlNode* node)
{
	FUStatus status;

	if (!IsEquivalent(node->name, DAE_TAPERED_CAPSULE_ELEMENT))
	{
		return status.Warning(FS("Tapered Capsule is not of the right type."), node->line);
	}

	for (xmlNode* child = node->children; child != NULL; child = child->next)
	{
		if (child->type != XML_ELEMENT_NODE) continue;

		if(IsEquivalent(child->name, DAE_HEIGHT_ELEMENT))
		{
			const char* h = ReadNodeContentDirect(child);
			height = FUStringConversion::ToFloat(&h);
		}
		else if(IsEquivalent(child->name, DAE_RADIUS1_ELEMENT))
		{
			const char* rad = ReadNodeContentDirect(child);
			radius = FUStringConversion::ToFloat(&rad);
		}
		else if(IsEquivalent(child->name, DAE_RADIUS2_ELEMENT))
		{
			const char* rad = ReadNodeContentDirect(child);
			radius2 = FUStringConversion::ToFloat(&rad);
		}
	}
	return status;
}

xmlNode* FCDPASTaperedCapsule::WriteToXML(xmlNode* node) const
{
	xmlNode* geomNode = AddChild(node, DAE_TAPERED_CAPSULE_ELEMENT);
	AddChild(geomNode, DAE_HEIGHT_ELEMENT, height);
	AddChild(geomNode, DAE_RADIUS1_ELEMENT, radius);
	AddChild(geomNode, DAE_RADIUS2_ELEMENT, radius2);
	return geomNode;
}

FCDPASTaperedCylinder::FCDPASTaperedCylinder(FCDocument* document) : FCDPASCylinder(document)
{
	 radius2 = 0.f;
}

FCDPhysicsAnalyticalGeometry* FCDPASTaperedCylinder::Clone()
{
	FCDPASTaperedCylinder* clone = new FCDPASTaperedCylinder(GetDocument());
	FCDEntity::Clone(clone);
	clone->radius = radius;
	return clone;
}

FUStatus FCDPASTaperedCylinder::LoadFromXML(xmlNode* node)
{
	FUStatus status;

	if (!IsEquivalent(node->name, DAE_TAPERED_CYLINDER_ELEMENT))
	{
		return status.Warning(FS("Tapered cylinder is not of the right type."), node->line);
	}

	for (xmlNode* child = node->children; child != NULL; child = child->next)
	{
		if (child->type != XML_ELEMENT_NODE) continue;
		
		if(IsEquivalent(child->name, DAE_HEIGHT_ELEMENT))
		{
			const char* h = ReadNodeContentDirect(child);
			height = FUStringConversion::ToFloat(&h);
		}
		else if(IsEquivalent(child->name, DAE_RADIUS1_ELEMENT))
		{
			const char* rad = ReadNodeContentDirect(child);
			radius = FUStringConversion::ToFloat(&rad);
		}
		else if(IsEquivalent(child->name, DAE_RADIUS2_ELEMENT))
		{
			const char* rad = ReadNodeContentDirect(child);
			radius2 = FUStringConversion::ToFloat(&rad);
		}
	}
	return status;
}

xmlNode* FCDPASTaperedCylinder::WriteToXML(xmlNode* node) const
{
	xmlNode* geomNode = AddChild(node, DAE_TAPERED_CYLINDER_ELEMENT);
	AddChild(geomNode, DAE_HEIGHT_ELEMENT, height);
	AddChild(geomNode, DAE_RADIUS1_ELEMENT, radius);
	AddChild(geomNode, DAE_RADIUS2_ELEMENT, radius2);
	return geomNode;
}
