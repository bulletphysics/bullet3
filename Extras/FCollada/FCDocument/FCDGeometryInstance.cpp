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
#include "FCDocument/FCDController.h"
#include "FCDocument/FCDEntity.h"
#include "FCDocument/FCDGeometry.h"
#include "FCDocument/FCDGeometryInstance.h"
#include "FCDocument/FCDGeometryMesh.h"
#include "FCDocument/FCDGeometryPolygons.h"
#include "FCDocument/FCDMaterialInstance.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

// Parasitic: Write out the instantiation information to the xml node tree
xmlNode* FCDEntityInstance::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* instanceNode = NULL;
	if (entity != NULL)
	{
		const char* instanceEntityName;
		switch (entity->GetType())
		{
		case FCDEntity::ANIMATION: instanceEntityName = DAE_INSTANCE_ANIMATION_ELEMENT; break;
		case FCDEntity::CAMERA: instanceEntityName = DAE_INSTANCE_CAMERA_ELEMENT; break;
		case FCDEntity::CONTROLLER: instanceEntityName = DAE_INSTANCE_CONTROLLER_ELEMENT; break;
		case FCDEntity::EFFECT: instanceEntityName = DAE_INSTANCE_EFFECT_ELEMENT; break;
		case FCDEntity::GEOMETRY: instanceEntityName = DAE_INSTANCE_GEOMETRY_ELEMENT; break;
		case FCDEntity::LIGHT: instanceEntityName = DAE_INSTANCE_LIGHT_ELEMENT; break;
		case FCDEntity::MATERIAL: instanceEntityName = DAE_INSTANCE_MATERIAL_ELEMENT; break;
		case FCDEntity::PHYSICS_MODEL: instanceEntityName = DAE_INSTANCE_PHYSICS_MODEL_ELEMENT; break;
		case FCDEntity::PHYSICS_RIGID_BODY: instanceEntityName = DAE_INSTANCE_RIGID_BODY_ELEMENT; break;
		case FCDEntity::PHYSICS_RIGID_CONSTRAINT: instanceEntityName = DAE_INSTANCE_RIGID_CONSTRAINT_ELEMENT; break;
		case FCDEntity::SCENE_NODE: instanceEntityName = DAE_INSTANCE_NODE_ELEMENT; break;

		case FCDEntity::ANIMATION_CLIP:
		case FCDEntity::ENTITY:
		case FCDEntity::IMAGE:
		case FCDEntity::TEXTURE:
		default: instanceEntityName = DAEERR_UNKNOWN_ELEMENT;
		}

		instanceNode = AddChild(parentNode, instanceEntityName);
		AddAttribute(instanceNode, DAE_URL_ATTRIBUTE, string("#") + entity->GetDaeId());
	}
	return instanceNode;
}

FCDGeometryInstance::FCDGeometryInstance(FCDocument* document, FCDEntity* entity) : FCDEntityInstance(document, entity)
{
}

FCDGeometryInstance::~FCDGeometryInstance()
{
	CLEAR_POINTER_VECTOR(materials);
}

// Access Bound Materials
FCDMaterialInstance* FCDGeometryInstance::FindMaterialInstance(const fstring& semantic)
{
	for (FCDMaterialInstanceList::iterator itB = materials.begin(); itB != materials.end(); ++itB)
	{
		if ((*itB)->GetSemantic() == semantic) return (*itB);
	}
	return NULL;
}

const FCDMaterialInstance* FCDGeometryInstance::FindMaterialInstance(const fstring& semantic) const
{
	for (FCDMaterialInstanceList::const_iterator itB = materials.begin(); itB != materials.end(); ++itB)
	{
		if ((*itB)->GetSemantic() == semantic) return (*itB);
	}
	return NULL;
}

// Load the geometry instance from the COLLADA document
FUStatus FCDGeometryInstance::LoadFromXML(xmlNode* instanceNode)
{
	FUStatus status = FCDEntityInstance::LoadFromXML(instanceNode);
	if (!status) return status;

	if (entity == NULL)
	{
		return status.Fail(FS("Trying to instantiate non-valid geometric entity."), instanceNode->line);
	}

	// Check for the expected instantiation node type
	if (!IsEquivalent(instanceNode->name, DAE_INSTANCE_GEOMETRY_ELEMENT) && !IsEquivalent(instanceNode->name, DAE_INSTANCE_CONTROLLER_ELEMENT)
		&& !IsEquivalent(instanceNode->name, DAE_INSTANCE_ELEMENT))
	{
		return status.Fail(FS("Unknown element for instantiation of entity: ") + TO_FSTRING(entity->GetDaeId()), instanceNode->line);
	}

	// Look for the <bind_material> element. The others are discarded for now.
	xmlNode* bindMaterialNode = FindChildByType(instanceNode, DAE_BINDMATERIAL_ELEMENT);
	if (bindMaterialNode != NULL)
	{
		// Retrieve the list of the <technique_common><instance_material> elements.
		xmlNode* techniqueNode = FindChildByType(bindMaterialNode, DAE_TECHNIQUE_COMMON_ELEMENT);
		xmlNodeList materialNodes;
		FindChildrenByType(techniqueNode, DAE_INSTANCE_MATERIAL_ELEMENT, materialNodes);
		for (xmlNodeList::iterator itM = materialNodes.begin(); itM != materialNodes.end(); ++itM)
		{
			FCDMaterialInstance* material = new FCDMaterialInstance(GetDocument(), this);
			status.AppendStatus(material->LoadFromXML(*itM));
			materials.push_back(material);
		}
	}

	if (materials.empty())
	{
		// COLLADA 1.3 backward compatibility: Create blank material instances for all the geometry's
		// polygons that have a valid material semantic
		FCDEntity* itE = entity;
		while (itE != NULL && itE->GetType() == FCDEntity::CONTROLLER) itE = ((FCDController*) itE)->GetBaseTarget();
		if (itE != NULL) 
		{
			FCDGeometry* geometry = (FCDGeometry*) itE;
			if (geometry->IsMesh())
			{
				FCDGeometryMesh* mesh = geometry->GetMesh();
				size_t polygonsCount = mesh->GetPolygonsCount();
				for (size_t i = 0; i < polygonsCount; ++i)
				{
					FCDGeometryPolygons* polygons = mesh->GetPolygons(i);
					const fstring& materialSemantic = polygons->GetMaterialSemantic();
					
					if (!materialSemantic.empty())
					{
						FCDMaterialInstance* material = new FCDMaterialInstance(GetDocument(), this);
						status.AppendStatus(material->LoadFromId(FUStringConversion::ToString(materialSemantic)));
						materials.push_back(material);
					}
				}
			}
		}
	}

	return status;
}

// Write out the instantiation information to the xml node tree
xmlNode* FCDGeometryInstance::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* instanceNode = FCDEntityInstance::WriteToXML(parentNode);
	if (!materials.empty())
	{
		xmlNode* bindMaterialNode = AddChild(instanceNode, DAE_BINDMATERIAL_ELEMENT);
		xmlNode* techniqueCommonNode = AddChild(bindMaterialNode, DAE_TECHNIQUE_COMMON_ELEMENT);
		for (FCDMaterialInstanceList::const_iterator itM = materials.begin(); itM != materials.end(); ++itM)
		{
			(*itM)->WriteToXML(techniqueCommonNode);
		}
	}
	return instanceNode;
}
