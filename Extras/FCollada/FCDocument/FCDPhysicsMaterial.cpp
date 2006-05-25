/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDPhysicsMaterial.h"
#include "FUtils/FUStringConversion.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDPhysicsMaterial::FCDPhysicsMaterial(FCDocument* document) : FCDEntity(document, "PhysicsMaterial")
{
	staticFriction = 0.f;
	dynamicFriction = 0.f;
	restitution = 0.f;
}

FCDPhysicsMaterial::~FCDPhysicsMaterial()
{
}

// Cloning
FCDPhysicsMaterial* FCDPhysicsMaterial::Clone()
{
	FCDPhysicsMaterial* clone = new FCDPhysicsMaterial(GetDocument());
	FCDEntity::Clone(clone);
	clone->SetStaticFriction(staticFriction);
	clone->SetDynamicFriction(dynamicFriction);
	clone->SetRestitution(restitution);
	return clone;
}

// Parse COLLADA document's <physics_material> element
FUStatus FCDPhysicsMaterial::LoadFromXML(xmlNode* physicsMaterialNode)
{
	FUStatus status = FCDEntity::LoadFromXML(physicsMaterialNode);
	if (!status) return status;
	if (!IsEquivalent(physicsMaterialNode->name, DAE_PHYSICS_MATERIAL_ELEMENT))
	{
		return status.Warning(FS("Unknown element in physics material library."), physicsMaterialNode->line);
	}

	//read in the <technique_common> element
	xmlNode* commonTechniqueNode = FindTechnique(physicsMaterialNode, DAE_COMMON_PROFILE);
	if (commonTechniqueNode == NULL)
	{
		return status.Fail(FS("Unable to find common technique for physics material: ") + TO_FSTRING(GetDaeId()), physicsMaterialNode->line);
	}

	xmlNode* paramNode = FindChildByType(commonTechniqueNode, DAE_PHYSICS_STATIC_FRICTION);
	if (paramNode != NULL) 
	{
		const char* content = ReadNodeContentDirect(paramNode);
		staticFriction = FUStringConversion::ToFloat(content);
	}

	paramNode = FindChildByType(commonTechniqueNode, DAE_PHYSICS_DYNAMIC_FRICTION);
	if (paramNode != NULL) 
	{
		const char* content = ReadNodeContentDirect(paramNode);
		dynamicFriction = FUStringConversion::ToFloat(content);
	}

	paramNode = FindChildByType(commonTechniqueNode, DAE_PHYSICS_RESTITUTION);
	if (paramNode != NULL)
	{
		const char* content = ReadNodeContentDirect(paramNode);
		restitution = FUStringConversion::ToFloat(content);
	}

	return status;
}

// Write out the <physics_material> element to the COLLADA xml tree
xmlNode* FCDPhysicsMaterial::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* physicsMaterialNode = WriteToEntityXML(parentNode, DAE_PHYSICS_MATERIAL_ELEMENT);
	AddChild(physicsMaterialNode, DAE_PHYSICS_STATIC_FRICTION, staticFriction);
	AddChild(physicsMaterialNode, DAE_PHYSICS_DYNAMIC_FRICTION, dynamicFriction);
	AddChild(physicsMaterialNode, DAE_PHYSICS_RESTITUTION, restitution);

	FCDEntity::WriteToExtraXML(physicsMaterialNode);
	return physicsMaterialNode;
}
