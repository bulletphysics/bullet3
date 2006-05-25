/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDController.h"
#include "FCDocument/FCDEntity.h"
#include "FCDocument/FCDPhysicsModel.h"
#include "FCDocument/FCDPhysicsModelInstance.h"
#include "FCDocument/FCDPhysicsRigidBodyInstance.h"
#include "FCDocument/FCDPhysicsRigidConstraintInstance.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDPhysicsModelInstance::FCDPhysicsModelInstance(FCDocument* document, FCDEntity* entity) : FCDEntityInstance(document, entity)
{
	SetClassName("FCDPhysicsModelInstance");
}

FCDPhysicsModelInstance::~FCDPhysicsModelInstance()
{
	CLEAR_POINTER_VECTOR(instances);
}

// Load the geometry instance from the COLLADA document
FUStatus FCDPhysicsModelInstance::LoadFromXML(xmlNode* instanceNode)
{
	FUStatus status = FCDEntityInstance::LoadFromXML(instanceNode);
	if (!status) return status;

	if (entity == NULL)
	{
		return status.Fail(FS("Trying to instantiate non-valid physics entity."), instanceNode->line);
	}

	// Check for the expected instantiation node type
	if (!IsEquivalent(instanceNode->name, DAE_INSTANCE_PHYSICS_MODEL_ELEMENT))
	{
		return status.Fail(FS("Unknown element for instantiation of entity: ") + TO_FSTRING(entity->GetDaeId()), instanceNode->line);
	}

	//this is already done in the FCDSceneNode
//	string physicsModelId = ReadNodeProperty(instanceNode, DAE_TARGET_ATTRIBUTE);
//	entity = GetDocument()->FindPhysicsModel(physicsModelId);
//	if(!entity)	return status.Fail(FS("Couldn't find physics model for instantiation"), instanceNode->line);

	xmlNodeList rigidBodyNodes;
	FindChildrenByType(instanceNode, DAE_INSTANCE_RIGID_BODY_ELEMENT, rigidBodyNodes);
	for(xmlNodeList::iterator itB = rigidBodyNodes.begin(); itB != rigidBodyNodes.end(); ++itB)
	{
		FCDPhysicsRigidBodyInstance* instance = new FCDPhysicsRigidBodyInstance(GetDocument(), entity);
		status.AppendStatus(instance->LoadFromXML(*itB));
		instances.push_back(instance);
	}

	xmlNodeList rigidConstraintNodes;
	FindChildrenByType(instanceNode, DAE_INSTANCE_RIGID_CONSTRAINT_ELEMENT, rigidConstraintNodes);
	for(xmlNodeList::iterator itC = rigidConstraintNodes.begin(); itC != rigidConstraintNodes.end(); ++itC)
	{
		FCDPhysicsRigidConstraintInstance* instance = new FCDPhysicsRigidConstraintInstance(GetDocument(), entity);
		status.AppendStatus(instance->LoadFromXML(*itC));
		instances.push_back(instance);
	}

	//FIXME: create a FCDPhysicsForceFieldInstance class
	xmlNodeList forceFieldNodes;
	FindChildrenByType(instanceNode, DAE_INSTANCE_FORCE_FIELD_ELEMENT, forceFieldNodes);
	for(xmlNodeList::iterator itN = forceFieldNodes.begin(); itN != forceFieldNodes.end(); ++itN)
	{
		FCDEntityInstance* instance = new FCDEntityInstance(GetDocument(), NULL);
		status.AppendStatus(instance->LoadFromXML(*itN));
		instances.push_back(instance);
	}

	return status;
}

// Write out the instantiation information to the xml node tree
xmlNode* FCDPhysicsModelInstance::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* instanceNode = FCDEntityInstance::WriteToXML(parentNode);
	for(FCDEntityInstanceList::const_iterator it = instances.begin(); it != instances.end(); ++it)
	{
		(*it)->WriteToXML(instanceNode);
	}
	return instanceNode;
}
