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
#include "FCDocument/FCDEffectParameter.h"
#include "FCDocument/FCDEffectParameterFactory.h"
#include "FCDocument/FCDEffectParameterList.h"
#include "FCDocument/FCDGeometry.h"
#include "FCDocument/FCDController.h"
#include "FCDocument/FCDGeometryInstance.h"
#include "FCDocument/FCDGeometryMesh.h"
#include "FCDocument/FCDGeometryPolygons.h"
#include "FCDocument/FCDMaterial.h"
#include "FCDocument/FCDMaterialInstance.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDMaterialInstance::FCDMaterialInstance(FCDocument* document, FCDGeometryInstance* _parent) : FCDEntityInstance(document, NULL)
{
	material = NULL;
	parent = _parent;
}

FCDMaterialInstance::~FCDMaterialInstance()
{
	bindings.clear();
	parent = NULL;
	material = NULL;
}

// Create a flattened version of the instantiated material: this is the
// prefered way to generate DCC/viewer materials from a COLLADA document
FCDMaterial* FCDMaterialInstance::FlattenMaterial()
{
	FCDGeometry* geometry = NULL;

	// Retrieve the necessary geometry and material information
	if(parent->GetEntity()->GetType() == FCDEntity::CONTROLLER)
	{
		FCDController* controller = (FCDController*) parent->GetEntity();
		FCDEntity* baseTarget = controller->GetBaseTarget();
		if(baseTarget->GetType() == FCDEntity::GEOMETRY)
			geometry = (FCDGeometry*) baseTarget;
	}
	else if(parent->GetEntity()->GetType() == FCDEntity::GEOMETRY)
	{
		geometry = (FCDGeometry*) parent->GetEntity();
	}

	if (material == NULL || geometry == NULL || !geometry->IsMesh()) return NULL;

	// Retrieve the correct polygons for this material semantic
	FCDGeometryMesh* mesh = geometry->GetMesh();
	size_t polygonsCount = mesh->GetPolygonsCount();
	FCDGeometryPolygons* polygons = NULL;
	for (size_t i = 0; i < polygonsCount; ++i)
	{
		FCDGeometryPolygons* p = mesh->GetPolygons(i);
		if (semantic == p->GetMaterialSemantic()) { polygons = p; break; }
	}
	if (polygons == NULL) return NULL;

	FCDMaterial* clone = material->Clone();
	clone->Flatten();

	// Flatten: Apply the bindings to the cloned material
	for (FCDMaterialInstanceBindList::iterator itB = bindings.begin(); itB != bindings.end(); ++itB)
	{
		FCDEffectParameterList parameters;
		clone->FindParametersBySemantic((*itB).semantic, parameters);
		for (FCDEffectParameterList::iterator itP = parameters.begin(); itP != parameters.end(); ++itP)
		{
			FCDEffectParameter* param = (*itP);
			if (param->GetType() == FCDEffectParameter::INTEGER)
			{
				FCDEffectParameterInt* intParam = (FCDEffectParameterInt*) param;

				// Fairly hacky: only supported bind type right now is the texture-texture coordinate sets, which are never animated

				// Resolve the target as a geometry source
				FCDGeometryPolygonsInput* input = polygons->FindInput((*itB).target);
				if (input != NULL) intParam->SetValue(input->set);
			}
		}
	}
	return clone;
}

// Read in the <instance_material> element from the COLLADA document
FUStatus FCDMaterialInstance::LoadFromXML(xmlNode* instanceNode)
{
	FUStatus status = FCDEntityInstance::LoadFromXML(instanceNode);
	if (!status) return status;
	bindings.clear();

	semantic = TO_FSTRING(ReadNodeProperty(instanceNode, DAE_SYMBOL_ATTRIBUTE));
	string materialId = ReadNodeProperty(instanceNode, DAE_TARGET_ATTRIBUTE);
	entity = material = GetDocument()->FindMaterial(materialId);
	if (material == NULL)
	{
		return status.Warning(FS("Invalid material binding in geometry instantiation."), instanceNode->line);
	}

	// Read in the ColladaFX bindings
	xmlNodeList bindNodes;
	FindChildrenByType(instanceNode, DAE_BIND_ELEMENT, bindNodes);
	for (xmlNodeList::iterator itB = bindNodes.begin(); itB != bindNodes.end(); ++itB)
	{
		FCDMaterialInstanceBind& bind = (*bindings.insert(bindings.end(), FCDMaterialInstanceBind()));
		bind.semantic = ReadNodeProperty(*itB, DAE_SEMANTIC_ATTRIBUTE);
		bind.target = ReadNodeProperty(*itB, DAE_TARGET_ATTRIBUTE);
	}

	return status;
}

FUStatus FCDMaterialInstance::LoadFromId(const string& materialId)
{
	FUStatus status;
	bindings.clear();

	// Copy the semantic over
	semantic = TO_FSTRING(materialId);

	// Find the material associated with this Id and clone it.
	entity = material = GetDocument()->FindMaterial(materialId);
	if (material == NULL)
	{
		return status.Warning(FS("Unknown material id or semantic: ") + TO_FSTRING(materialId));
	}
	
	return status;
}

// Write out the instantiation information to the xml node tree
xmlNode* FCDMaterialInstance::WriteToXML(xmlNode* parentNode) const
{
	// Intentionally skip the parent WriteToXML class
	xmlNode* instanceNode = AddChild(parentNode, DAE_INSTANCE_MATERIAL_ELEMENT);
	const FCDMaterial* material = GetMaterial();
	if (material != NULL)
	{
		AddAttribute(instanceNode, DAE_SYMBOL_ATTRIBUTE, semantic);
		AddAttribute(instanceNode, DAE_TARGET_ATTRIBUTE, material->GetDaeId());
	}
	return instanceNode;
}
