/*
	Copyright (C) 2005-2006 Feeling Software Inc.
	MIT License: http://www.opensource.org/licenses/mit-license.php
*/

#include "StdAfx.h"
#include "FCDocument/FCDocument.h"
#include "FCDocument/FCDEntity.h"
#include "FCDocument/FCDPhysicsModel.h"
#include "FCDocument/FCDPhysicsModelInstance.h"
#include "FCDocument/FCDPhysicsRigidConstraint.h"
#include "FCDocument/FCDPhysicsRigidConstraintInstance.h"
#include "FUtils/FUDaeParser.h"
#include "FUtils/FUDaeWriter.h"
using namespace FUDaeParser;
using namespace FUDaeWriter;

FCDPhysicsRigidConstraintInstance::FCDPhysicsRigidConstraintInstance(FCDocument* document, FCDEntity* _parent) : FCDEntityInstance(document, NULL)
{
	if (_parent->GetType() == FCDEntity::PHYSICS_MODEL)
		parent = (FCDPhysicsModel*)_parent;

	rigidConstraint = NULL;
	SetClassName("FCDPhysicsRigidConstraintInstance");
}

FCDPhysicsRigidConstraintInstance::~FCDPhysicsRigidConstraintInstance()
{
	parent = NULL;
	rigidConstraint = NULL;
}


// Load the geometry instance from the COLLADA document
FUStatus FCDPhysicsRigidConstraintInstance::LoadFromXML(xmlNode* instanceNode)
{
	FUStatus status = FCDEntityInstance::LoadFromXML(instanceNode);
	if (!status) return status;

	// Check for the expected instantiation node type
	if (!IsEquivalent(instanceNode->name, DAE_INSTANCE_RIGID_CONSTRAINT_ELEMENT))
	{
		return status.Fail(FS("Unknown element for instantiation of entity: ") + TO_FSTRING(entity->GetDaeId()), instanceNode->line);
	}
	if (parent == NULL)
	{
		return status.Fail(FS("No Physics Model parent for rigid constraint instantiation"), instanceNode->line);
	}

	string physicsRigidConstraintSid = ReadNodeProperty(instanceNode, DAE_CONSTRAINT_ATTRIBUTE);
	entity = rigidConstraint = parent->FindRigidConstraintFromSid(physicsRigidConstraintSid);
	if(!rigidConstraint)
	{
		return status.Fail(FS("Couldn't find rigid constraint for instantiation"), instanceNode->line);
	}

	return status;
}


// Write out the instantiation information to the xml node tree
xmlNode* FCDPhysicsRigidConstraintInstance::WriteToXML(xmlNode* parentNode) const
{
	xmlNode* instanceNode = FCDEntityInstance::WriteToXML(parentNode);

	//TODO

	return instanceNode;
}
